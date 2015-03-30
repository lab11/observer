#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/spi-arch.h"
#include "core/dev/spi.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include <stdio.h>
#include "usb-serial.h"
#include "mpu9250.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;


spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
spi_cs_init(GPIO_B_NUM, 4);


/*---------------------------------------------------------------------------*/
PROCESS(mpu9250_process, "MPU9250");
AUTOSTART_PROCESSES(&mpu9250_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mpu9250_process, ev, data) {

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
	//int16_t currentZ = 0;

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

			etimer_restart(&periodic_timer_blue);
		}
		


	}

	PROCESS_END();
}

