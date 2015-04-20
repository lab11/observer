#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../ioctl.h"
#include <unistd.h>
#include "checksum.h"

static void parse_data(char* buf);

int main(char ** argv, int argc)
{	setvbuf(stdout, NULL, _IOLBF, 0);
	//setlinebuf(stdout);

	int result = 0;
	//printf("Testing cc2520 driver...\n");
	int file_desc;
	file_desc = open("/dev/cc2520_0", O_RDWR);	

	//printf("Setting channel\n");
	struct cc2520_set_channel_data chan_data;
	chan_data.channel = 24;
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
	bool correct_device;

	for (i = 0; i < 100; i++) {
		//printf("Receiving a test message...\n");
		result = read(file_desc, buf, 127);
		correct_device = false;
	
		//printf("result %d\n", result);
		if (result > 0) {
			buf_ptr = pbuf;
			for (i = 0; i < result; i++)
			{
				if (i < 53 || i > 72) {}
				else {
					buf_ptr += sprintf(buf_ptr, " 0x%02X", buf[i]);
				}
				if (i == 52) {
					if (buf[i] == 0x02) {
						correct_device = true;
					}
				}
			}
			*(buf_ptr) = '\0';
			//printf("read %s\n", pbuf);
			if (correct_device) {
				if (validation(buf[53], buf[54], buf[55], buf[56], buf[57],
						buf[58], buf[59], buf[60], buf[61], buf[62],
						buf[63], buf[64], buf[65], buf[66],
						buf[67], buf[68], buf[69], buf[72],
						buf[71])) {
					parse_data(buf);
					//printf("%s\n", pbuf);
					//fflush(stdout);
				}
			}
		}
	}

	//printf("Turning off the radio...\n");
	fflush(stdout);
	ioctl(file_desc, CC2520_IO_RADIO_OFF, NULL);

	close(file_desc);
}

static void parse_data(char* buf) {
	int16_t accelx = (buf[54] << 8) | buf[53];
	int16_t accely = (buf[56] << 8) | buf[55];
	int16_t accelz = (buf[58] << 8) | buf[57];

	uint16_t humid = (buf[60] << 8) | buf[59];
	humid = (humid * 125) / 65536 - 6;	

	uint16_t light = (buf[62] << 8) | buf[61];

	uint16_t mic = (buf[64] << 8) | buf[63];

	uint32_t press = (buf[67] << 16) | (buf[66] << 8) | buf[65];
	press = press/4096;

	uint16_t temp = (buf[69] << 8) | buf[68];
	temp = (temp * 175.72) / 65536 - 46.85;

	printf("%d %d %d %d %d %d %d %d\n", accelx, accely, accelz, humid, light, mic, press, temp);
	//printf("hellow world\n");
	fflush(stdout);

	return; 
}
