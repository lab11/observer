#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../ioctl.h"
#include <unistd.h>

static void parse_data(char* buf);

int main(char ** argv, int argc)
{	//setvbuf(stdout, NULL, _IOLBF, 0);
    //setlinebuf(stdout);

    int result = 0;
    //printf("Testing cc2520 driver...\n");
    int file_desc;
    file_desc = open("/dev/cc2520_0", O_RDWR);	

    //printf("Setting channel\n");
    struct cc2520_set_channel_data chan_data;
    chan_data.channel = 22;
    ioctl(file_desc, CC2520_IO_RADIO_SET_CHANNEL, &chan_data);

    //printf("Setting address\n");
    struct cc2520_set_address_data addr_data;
    addr_data.short_addr = 0x0001;
    addr_data.extended_addr = 0x0000000000000001;
    addr_data.pan_id = 0x22;
    ioctl(file_desc, CC2520_IO_RADIO_SET_ADDRESS, &addr_data);

    //printf("Setting tx power\n");
    struct cc2520_set_txpower_data txpower_data;
    txpower_data.txpower = CC2520_TXPOWER_0DBM;
    ioctl(file_desc, CC2520_IO_RADIO_SET_TXPOWER, &txpower_data);

    //printf("Turning on the radio...\n");
    ioctl(file_desc, CC2520_IO_RADIO_INIT, NULL);
    ioctl(file_desc, CC2520_IO_RADIO_ON, NULL);

    fflush(stdout);

    int i = 0;

    char buf[256];
    char pbuf[1024];
    char *buf_ptr = NULL;

    char databuf[26];
    bool correct_device;

    for (;;) {
        //printf("Receiving a test message...\n");
        result = read(file_desc, buf, 127);
        correct_device = false;

        //printf("result %d\n", result);
        if (result > 0 && result == 42) {
	    if (buf[41] & 0x80) { // CRC result is 1
            	buf_ptr = pbuf;
            	for (i = 0; i < result; i++)
            	{ 
                	if (i >= 14 && i < 40) {
                    	buf_ptr += sprintf(buf_ptr, " 0x%02X", buf[i]);
                    	databuf[i-14] = buf[i];
                	}
            	}
            	*(buf_ptr) = '\0';
		//printf("%s\n", buf_ptr);
            	//printf("%s\n", pbuf);	
            	//fflush(stdout);

            	parse_data(databuf);
	    }
        }
    }

    printf("Turning off the radio...\n");
    //fflush(stdout);
    ioctl(file_desc, CC2520_IO_RADIO_OFF, NULL);

    close(file_desc);
}

static void parse_data(char* buf) {
    uint16_t temp = (buf[2] << 8) | buf[1];
    float temp_f = ((float)(temp * 175.72)) / 65536 - 46.85;


    uint16_t humid = (buf[4] << 8) | buf[3];
    float humid_f = ((float)(humid * 125)) / 65536 - 6;


	uint16_t light = (buf[6] << 8) | buf[5];
	float light_f = ((float)(light - 256) / .146);
	if(light_f < 0) light_f = 0;


    uint32_t press = (buf[9] << 16) | (buf[8] << 8) | buf[7];
    float press_f = ((float)press)/4096;


    uint8_t pir_motion = buf[10];


    float x_mag_adj = (float)((uint8_t)buf[17] - 128)/256 + 1;
    float y_mag_adj = (float)((uint8_t)buf[18] - 128)/256 + 1;
    float z_mag_adj = (float)((uint8_t)buf[19] - 128)/256 + 1;

    float x_mag = (float)(int16_t)((buf[12] << 8)|buf[11]) * x_mag_adj;
    float y_mag = (float)(int16_t)((buf[14] << 8)|buf[13]) * x_mag_adj;
    float z_mag = (float)(int16_t)((buf[16] << 8)|buf[15]) * z_mag_adj;

    int16_t x_accel = (buf[21] << 8)|buf[20];
    int16_t y_accel = (buf[23] << 8)|buf[22];
    int16_t z_accel = (buf[25] << 8)|buf[24];

   // printf("%s\n", buf);
    printf("%i %.3f %.3f %.3f %.3f %u %.3f %.3f %.3f %i %i %i\n", buf[0], temp_f, humid_f, press_f, light_f, pir_motion, x_mag, y_mag, z_mag, x_accel, y_accel, z_accel);

    fflush(stdout);

    return; 
}
