#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/spi-arch.h"
#include "core/dev/spi.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include <stdio.h>
#include "usb-serial.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;

#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W

#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_TEMP_OUT_H         0x41   // R  
#define MPU9150_TEMP_OUT_L         0x42   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R  	

#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W

uint8_t TARGET_ADDRESS = 0x68; // 0'b1101000

// writes data to register reg_addr on the cc2538
void MPU9150_writeSensor(uint8_t reg_addr, uint8_t data) {

	uint8_t transmit_bufL[] = {reg_addr, data};
	i2c_burst_send(TARGET_ADDRESS, transmit_bufL, sizeof(transmit_bufL));

}

// reads from register reg_addrL and reg_addrH on the cc2538
// the sensor data of the MPU9150 are 16bits so you have to read the lower and upper 8bits separately
// to read from a specific register, write the address of the register you want to read to the slave
// then read from slave device
int MPU9150_readSensor(uint8_t reg_addrL, uint8_t reg_addrH) {

	/*uint8_t transmit_bufL[] = {reg_addrL};
	uint8_t receive_bufL[1];

	// tell the cc2538 which register you want to read from
	i2c_single_send(TARGET_ADDRESS, *transmit_bufL);
	// the following read will be the value from the register you wrote to the slave about
	i2c_single_receive(TARGET_ADDRESS, receive_bufL);

	uint8_t transmit_bufH[] = {reg_addrH};
	uint8_t receive_bufH[1];

	// tell the cc2538 which register you want to read from
	i2c_single_send(TARGET_ADDRESS, *transmit_bufH);
	// the following read will be the value from the register you wrote to the slave about
	i2c_single_receive(TARGET_ADDRESS, receive_bufH);

	// concatenate the lower and upper 8bits to a signed 16 bit integer
	uint16_t L = receive_bufL[0];
	uint16_t H = receive_bufH[0];
	*/

	uint8_t transmit_bufBase[] = {reg_addrH}; // upper bits is at lower address
	uint8_t receive_bufBase[2];
	i2c_single_send(TARGET_ADDRESS, *transmit_bufBase);
	i2c_burst_receive(TARGET_ADDRESS, receive_bufBase, 2);

	uint16_t L = receive_bufBase[1];
	uint16_t H = receive_bufBase[0];

	return (int16_t)((H<<8) + L);


}


/*---------------------------------------------------------------------------*/
PROCESS(mpu9150_process, "MPU9150");
AUTOSTART_PROCESSES(&mpu9150_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mpu9150_process, ev, data) {

	PROCESS_BEGIN();

	etimer_set(&periodic_timer_red, CLOCK_SECOND);
	etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

	// Initialize i2c
	i2c_init(GPIO_C_NUM, 5, GPIO_C_NUM, 4, I2C_SCL_NORMAL_BUS_SPEED);
	i2c_master_enable();
	i2c_set_frequency(I2C_SCL_NORMAL_BUS_SPEED);

	// Clear sleep bit to start sensor
	MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0x00);

	// ACCEL CONFIG
	MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, 0x18);
	int16_t currentX = 0;
	int16_t currentY = 0;
	int16_t currentZ = 0;

	while(1) {
		PROCESS_YIELD();
		/*
		if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		} else if (etimer_expired(&periodic_timer_green)) {
			leds_toggle(LEDS_GREEN);
			etimer_restart(&periodic_timer_green);
		} else if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}
		*/
		
		if (etimer_expired(&periodic_timer_blue)) {
			currentX = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
			currentY = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
			currentZ = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);

		
			leds_off(LEDS_ALL);

			if (currentY <= 750 && currentY >= -750) {
				//leds_off(LEDS_RED);
				leds_on(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			}
			else if (currentY > 750) {
				leds_on(LEDS_RED);
				//leds_off(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			} 
			else {
				//leds_off(LEDS_RED);
				//leds_off(LEDS_GREEN);
				leds_on(LEDS_BLUE);
			}

			if (currentX <= 750 && currentX >= -750) {
			}
			else if (currentX > 750) {
				leds_on(LEDS_RED);
			}
			else {
				//leds_on()
			}

			printf("X_ACCEL: %d\n\r", currentX);
			printf("Y_ACCEL: %d\n\r", currentY);
			printf("Z_ACCEL: %d\n\r\r", currentZ);



			etimer_restart(&periodic_timer_blue);
		}
		


	}

	PROCESS_END();
}

