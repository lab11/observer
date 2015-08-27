#include "cpu.h"
#include "contiki.h"
#include "lps331ap.h"
#include "si1147.h"
#include "mpu9250.h"
#include "si7021.h"
#include "amn41122.h"
#include "adc121c021.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "lpm.h"

#include "spi-arch.h"
#include "spi.c"
#include "i2c.h"
#include "assert.h"
#include <stdio.h>
#include "stdbool.h"
#include "gpio.h"
#include "nvic.h"
#include "net/netstack.h"

#include "cc2538-rf.h"
#include "net/packetbuf.h"
#include "sys/rtimer.h"

static struct rtimer my_timer;

volatile uint8_t rtimer_expired = 0;
volatile uint8_t accel_event = 0;
volatile uint8_t motion_event = 0;

void setup_before_resume(void);
void cleanup_before_sleep(void);
static void periodic_rtimer(struct rtimer *rt, void* ptr);

#define PERIOD_T 30*RTIMER_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();
	i2c_init(GPIO_C_NUM, 5,
		GPIO_C_NUM, 4,
		I2C_SCL_FAST_BUS_SPEED);
	leds_toggle(LEDS_GREEN);
	//amn41122_init();
	//amn41122_irq_enable();

	lps331ap_init();
//  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
//	leds_toggle(LEDS_RED);
	mpu9250_init();
//	leds_toggle(LEDS_BLUE);
	//mpu9250_motion_interrupt_init(0x0F, 0x06);
leds_toggle(LEDS_RED);
//	si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
leds_toggle(LEDS_BLUE);
//	si1147_als_data_t als_data;
	//cc2538_rf_driver.off();
	//cleanup_before_sleep();
	//rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, &periodic_rtimer, NULL);
leds_off(LEDS_ALL);
//	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(6));
//	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(6));
//	ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS);
//	GPIO_SET_PIN(GPIO_PORT_TO_BASE(GPIO_B_NUM),GPIO_PIN_MASK(6));
	while(1) {
		uint32_t press = lps331ap_one_shot();
//		printf("press %u\n", press);
		printf("WAI: %u\n", mpu9250_readByte(MPU9250_WHO_AM_I));
		int16_t ax = mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
  		int16_t ay = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
   		int16_t az = mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H);
//		printf("x: %i, y: %i, z: %i\n", ax, ay, az);		
//		si1147_als_force_read(&als_data);
		uint16_t temp = si7021_readTemp(TEMP_NOHOLD);
//		printf("temp\n");
//		adc121c021_read_amplitude();
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		leds_toggle(LEDS_GREEN);
	}

	while(1) {
		PROCESS_YIELD();

		setup_before_resume();
		cc2538_rf_driver.off();
		rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, &periodic_rtimer, NULL);
		cleanup_before_sleep();
	}

	PROCESS_END();
}


void setup_before_resume(void) {
	/* ssi0 ports/pins */
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
                  			GPIO_PIN_MASK(6));
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
                  			GPIO_PIN_MASK(7));
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM),
                  			GPIO_PIN_MASK(0));

	i2c_master_enable();

	return;
}

void cleanup_before_sleep(void) {
	/* ssi0 ports/pins that need to be set/clr so not floating */
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(6));
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(7));
	GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(0));
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x80);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x80);
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x40);
	GPIO_SET_PIN(GPIO_C_BASE, 0x40);
	GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01);
	GPIO_CLR_PIN(GPIO_B_BASE, 0x01);

	i2c_master_disable();

  	return;
}


static void periodic_rtimer(struct rtimer *rt, void* ptr){
	INTERRUPTS_DISABLE();
	rtimer_expired = 1;

     uint8_t ret;

     //leds_go(counter++);   //u gonna get the led counting from 0-7 
     printf("time now: %d\n", RTIMER_NOW());
     printf("timer plus period: %d\n", RTIMER_NOW() + PERIOD_T);

     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, (void*)periodic_rtimer, NULL);
     process_poll(&blink_process);
     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
     //           (void (*)(struct rtimer *, void *))periodic_rtimer, NULL);
     //if(ret){
     //    printf("Error Timer: %u\n", ret);
     //}
     INTERRUPTS_ENABLE();
   return;
}
