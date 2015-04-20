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
#include "cpu/cc2538/dev/cc2538-rf.h"

//static struct etimer periodic_timer_red;
//static struct etimer periodic_timer_green;
//static struct etimer periodic_timer_blue;


/*---------------------------------------------------------------------------*/
PROCESS(mpu9250_process, "MPU9250");
//PROCESS(slave, "slave");
AUTOSTART_PROCESSES(&mpu9250_process/*,&slave*/);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mpu9250_process, ev, data) {

	PROCESS_BEGIN();

	// Setup SPI clock high while idle, data valid on clock trailing edge
	spix_set_mode(0,SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
	spix_cs_init(MPU9250_CS_PORT, MPU9250_CS_PIN);

	//REG(SSI1_BASE + SSI_CR1) |= SSI_CR1_MS;

	//etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

	// Initialize i2c
	//i2c_init(GPIO_C_NUM, 5, GPIO_C_NUM, 4, I2C_SCL_NORMAL_BUS_SPEED);
	//i2c_master_enable();
	//i2c_set_frequency(I2C_SCL_NORMAL_BUS_SPEED);

	
	//if(etimer_expired(&periodic_timer_red)) {
		// Disable I2C
		//MPU9250_writeSensor(MPU9250_USER_CTRL, 0x10);
		// Clear sleep bit to start sensor
		//MPU9250_writeSensor(MPU9250_PWR_MGMT_1, 0x00);
		//etimer_restart(&periodic_timer_green);
		//MPU9250_writeSensor(MPU9250_ACCEL_CONFIG, 0x18);
		
	//}
	

	// Disable I2C
	MPU9250_writeSensor(MPU9250_USER_CTRL, 0x10);
	// Clear sleep bit to start sensor
	MPU9250_writeSensor(MPU9250_PWR_MGMT_1, 0x00);
	

	// ACCEL CONFIG
	MPU9250_writeSensor(MPU9250_ACCEL_CONFIG, 0x18);
	
	// setup motion interrupt
	//setup_motion_interrupt();

	int16_t currentX = 0;
	int16_t currentY = 0;
	uint8_t whoami = 0;
	uint16_t whoami2 = 0;
	uint8_t accel_config_reg = 0;
	uint8_t channel_num = 0;
 	//int16_t currentZ = 0;

 	channel_num = get_channel();

	while(1) {

		printf("channel_num: %d\n", channel_num);
		//PROCESS_YIELD();

			/*if(etimer_expired(&periodic_timer_blue)) {
				// Disable I2C
				MPU9250_writeSensor(MPU9250_USER_CTRL, 0x10);
				// Clear sleep bit to start sensor
				MPU9250_writeSensor(MPU9250_PWR_MGMT_1, 0x00);
				// ACCEL CONFIG
				MPU9250_writeSensor(MPU9250_ACCEL_CONFIG, 0x18);


				//etimer_restart(&periodic_timer_blue);
			}*/

			//uint16_t recv, recv2, recv3, recv4;

			//MPU9250_writeSensor(MPU9250_PWR_MGMT_1, 0x00);
			//recv = MPU9250_readSensor(MPU9250_PWR_MGMT_1, MPU9250_PWR_MGMT_1);
			//recv2 = MPU9250_readSensor(MPU9250_PWR_MGMT_1, MPU9250_PWR_MGMT_1);
			//recv3 = MPU9250_readSensor(MPU9250_PWR_MGMT_1, MPU9250_PWR_MGMT_1);
			//recv4 = MPU9250_readSensor(MPU9250_PWR_MGMT_1, MPU9250_PWR_MGMT_1);




			/*if (recv == 0x7B) {
				printf("HELLO WORLD\n");
			}*/
		
		/*if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		} else if (etimer_expired(&periodic_timer_green)) {
			leds_toggle(LEDS_GREEN);
			etimer_restart(&periodic_timer_green);
		} else if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}*/
		
		/*if (etimer_expired(&periodic_timer_blue)) {
			MPU9250_writeSensor(MPU9250_PWR_MGMT_1, 0x00);
			leds_toggle(LEDS_RED);
		}*/
		//currentX = MPU9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
		currentY = MPU9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
		//printf("ACCEL_Y: %x\n", currentY);
		//accel_config_reg = MPU9250_readByte(MPU9250_ACCEL_CONFIG);
		//printf("accel_config_reg value: %x\n", accel_config_reg);
		//MPU9250_writeSensor(MPU9250_USER_CTRL, 0x10);
		//whoami = MPU9250_readByte(MPU9250_WHO_AM_I);
		//printf("whoami: %08x\n", whoami);
		// whoami2 = MPU9250_readSensor(MPU9250_WHO_AM_I, MPU9250_WHO_AM_I);
		// printf("whoami2: %08x\n", whoami2 );
	
		leds_off(LEDS_ALL);

		if (currentY <= 750 && currentY >= -750) {
			leds_off(LEDS_RED);
			leds_on(LEDS_GREEN);
			leds_off(LEDS_BLUE);
		}
		else if (currentY > 750) {
			leds_on(LEDS_RED);
			leds_off(LEDS_GREEN);
			leds_off(LEDS_BLUE);
		} 
		else {
			leds_off(LEDS_RED);
			leds_off(LEDS_GREEN);
			leds_on(LEDS_BLUE);
		}
		/*
		if (currentX <= 750 && currentX >= -750) {
		}
		else if (currentX > 750) {
			leds_on(LEDS_RED);
		}
		else {
			//leds_on()
		}
		*/
			


	}

	PROCESS_END();
}


/*---------------------------------------------------------------------------*/
/*PROCESS_THREAD(slave, ev, data) {
	PROCESS_BEGIN();

	// Setup SPI clock high while idle, data valid on clock trailing edge
	spix_set_mode(1, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
	//spix_cs_init(MPU9250_CS_PORT, MPU9250_CS_PIN);

	// chip select
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_A_NUM),GPIO_PIN_MASK(6));
  	ioc_set_over(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(6), IOC_OVERRIDE_DIS);
  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(6));
  	GPIO_SET_PIN(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(6));

  	// clock
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_A_NUM),GPIO_PIN_MASK(5));
  	ioc_set_over(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(5), IOC_OVERRIDE_DIS);
  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(5));
  	GPIO_SET_PIN(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(5));

  	// miso
  	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI1_RX_PORT),GPIO_PIN_MASK(SPI1_RX_PIN));
  	ioc_set_over(GPIO_PORT_TO_BASE(SPI1_RX_PORT), GPIO_PIN_MASK(SPI1_RX_PIN), IOC_OVERRIDE_DIS);
  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(SPI1_RX_PORT), GPIO_PIN_MASK(SPI1_RX_PIN));
  	GPIO_SET_PIN(GPIO_PORT_TO_BASE(SPI1_RX_PORT), GPIO_PIN_MASK(SPI1_RX_PIN));

  	// mosi
  	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI1_TX_PORT),GPIO_PIN_MASK(SPI1_TX_PIN));
  	ioc_set_over(GPIO_PORT_TO_BASE(SPI1_TX_PORT), GPIO_PIN_MASK(SPI1_TX_PIN), IOC_OVERRIDE_DIS);
  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(SPI1_TX_PORT), GPIO_PIN_MASK(SPI1_TX_PIN));
  	GPIO_SET_PIN(GPIO_PORT_TO_BASE(SPI1_TX_PORT), GPIO_PIN_MASK(SPI1_TX_PIN));

  	// ssi1
	REG(SSI1_BASE + SSI_CR1) |= SSI_CR1_MS;

	etimer_set(&periodic_timer_blue, CLOCK_SECOND/2);

	uint8_t data = 0x32;

	while(1) {
		PROCESS_YIELD();

		read_slave(&data);

		printf("%x\n", data);

		if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}


	}

	PROCESS_END();

}
*/
