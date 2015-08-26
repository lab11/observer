#include "cpu.h"
#include "contiki.h"
#include "lps331ap.h"
#include "si1147.h"
#include "mpu9250.h"
#include "si7021.h"
#include "amn41122.h"
#include "adc121c021.h"
#include "rv3049.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "lpm.h"

#include "spi-arch.h"
#include "spi.c"
#include "assert.h"
#include <stdio.h>
#include "stdbool.h"
#include "gpio.h"
#include "nvic.h"
#include "net/netstack.h"

#include "cc2538-rf.h"
#include "net/packetbuf.h"
#include "sys/rtimer.h"
#include "vtimer.h"


#define PERIOD_T 30*RTIMER_SECOND
#define PERIOD_T2 45*RTIMER_SECOND

// prototypes
void setup_before_resume(void);
void cleanup_before_sleep(void);
//static void periodic_rtimer(struct rtimer *rt, void* ptr);
static void periodic_vtimer(void);
static void rtc_callback(void);
static void amn41122_callback(void);

typedef enum WakeEvents { 
			DEFAULTEV, 
			PERIODIC_EV,
			ACCEL_EV, 
			MOTION_EV 
} wakeevents_t;

static wakeevents_t wakeevent = DEFAULTEV;


static uint8_t counter = 0;
//static struct rtimer my_timer;
//static struct rtimer my_timer2;
//static struct vtimer my_vtimer;
//static struct timer mytime;
static struct vtimer pir_vtimer;
static uint8_t pir_motion = 0;

volatile uint8_t rtimer_expired = 0;
volatile uint8_t accel_event = 0;
volatile uint8_t motion_event = 0;



/*---------------------------------------------------------------------------*/
PROCESS(observer_lp_process, "Observer_lp");
AUTOSTART_PROCESSES(&observer_lp_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_lp_process, ev, data) {

	PROCESS_BEGIN();

	leds_toggle(LEDS_GREEN);
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
	leds_toggle(LEDS_GREEN);
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

	leds_toggle(LEDS_GREEN);
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
	leds_toggle(LEDS_GREEN);
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

	leds_toggle(LEDS_GREEN);
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
	leds_toggle(LEDS_GREEN);
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


	uint8_t ret;
	uint8_t ret2;

	//cc2538_rf_driver.off();

	//static struct etimer et;
	static uint8_t buf[11];
	//uint8_t buff[1];

	//cc2538_rf_driver.off();
	//NETSTACK_RADIO.off();
	pir_vtimer = get_vtimer(periodic_vtimer);
	amn41122_init();
	//amn41122_irq_enable();
	//pir_vtimer = get_vtimer(periodic_rtimer);

	lps331ap_init();

	mpu9250_init();

	mpu9250_motion_interrupt_init(0x0F, 0x06);

	si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
	si1147_als_data_t als_data;

	amn41122_irq_enable(amn41122_callback);
	//adc121c021_config();


//*******************************************************************

	//rv3049_time_t times;

	rv3049_time_t alarm_time;
	rv3049_read_time(&alarm_time);
	// uint8_t minutes;

	// minutes = roundUp(alarm_time.minutes);
	// (minutes == 60) ? (alarm_time.minutes = 0): (alarm_time.minutes = minutes);



	// 	SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

	//     SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

	//     SPI_FLUSH();

	//     // null byte read
	//     SPI_READ(buff[0]);

	//     // Then actually read the alarm interrupt control reg
	//     SPI_READ(buff[0]);

	//     SPI_FLUSH();

	//     SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  
	//     SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

	//     SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

	//     SPI_WRITE(buff[0] & 0xFE);

 //        SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);





	alarm_time.seconds = 0;
	uint8_t ae_mask = 0x01;
	rv3049_set_alarm(&alarm_time, ae_mask);
	rv3049_clear_int_flag();
	clock_delay_usec(50000);
	rv3049_interrupt_enable(rtc_callback);
	//rv3049_set_alarm(&alarm_time, ae_mask);






//*********************************************************************

	//CC2538_RF_CSP_ISRFOFF();
	NETSTACK_RDC.off(0);
	NETSTACK_MAC.off(0);
	cc2538_rf_driver.off();

	cleanup_before_sleep();
	//cc2538_rf_driver.off();




	//my_vtimer = get_vtimer(periodic_rtimer);
	//schedule_vtimer(&my_vtimer, 30*VTIMER_SECOND);
	//ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, &periodic_rtimer, NULL);

	//uint8_t press = 0;
	while(1) {
		//leds_toggle(LEDS_GREEN);

		//cc2538_rf_driver.off();
		// CC2538_RF_CSP_ISRFOFF();

		// ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
  //               (void*)periodic_rtimer, NULL);
		// if(ret){
  //       	//printf("Error Timer: %u\n", ret);
  //       	rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
  //               (void*)periodic_rtimer, NULL);
  //    	}


		// cleanup_before_sleep();
		//printf("GOING TO SLEEP\n");
		// PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		//printf("YIELDING\n");
		PROCESS_YIELD();
		//INTERRUPTS_DISABLE();
		setup_before_resume();
		

		//rv3049_clear_int_flag();
		//unsigned char leds_result = leds_get();
		//if (leds_result & LEDS_RED) {
		if (pir_motion) {
			schedule_vtimer(&pir_vtimer, 30*VTIMER_SECOND);
			//leds_off(LEDS_RED);
		//	pir_motion = 1;
		} else {
			rv3049_clear_int_flag();
		}

		//printf("UNDER YIELD\n");

		/* check who woke me up */
		/*switch (wakeevent) {
			case PERIODIC_EV:
					//
					break;
			case ACCEL_EV:
					//
					break;
			case ACCEL_INT_REENABLE:
					//
					break;
			case MOTION_EV:
					//
					break;
			case DEFAULTEV:
			default:
					// blink red led? idk something to indicate error
					break;

		}*/
		//rv3049_read_time(&times);
		//printf("TIME: %u/%u/%u, %u:%u:%u\n", times.month, times.days, times.year, times.hours, times.minutes, times.seconds);

		uint32_t press = lps331ap_one_shot();// lps331ap_one_shot();
		// printf("PRESS: %d\n", press);
		uint16_t temp = si7021_readTemp(TEMP_NOHOLD);
		uint16_t rh = si7021_readHumd(RH_NOHOLD);
		si1147_als_force_read(&als_data);
		// printf("LIGHT: %d\n", als_data.vis.val);
		//adc121c021_read_amplitude();
		buf[0] = 0x02;
		buf[1] = temp;
		buf[2] = (temp & 0xFF00) >> 8;
		buf[3] = rh;
		buf[4] = (rh & 0xFF00) >> 8;
		buf[5] = als_data.vis.b.lo;
		buf[6] = als_data.vis.b.hi;
		buf[7] = press & 0x000000FF;
		buf[8] = (press & 0x0000FF00) >> 8;
		buf[9] = (press & 0x00FF0000) >> 16;
		buf[10] = pir_motion;
		pir_motion = 0;
		// buf[0] = 0x01;
		// buf[1] = 0x02;
		// buf[2] = 0x03;
		// buf[3] = 0x04;
		// buf[4] = 0x04;
		// buf[5] = 0x06;
		// buf[6] = 0x07;
		// buf[7] = 0x08;
		// buf[8] = 0x09;
		// buf[9] = 0x01;


		/*leds_toggle(LEDS_GREEN);
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
		leds_toggle(LEDS_GREEN);*/
	
		packetbuf_copyfrom(buf, 11);
		cc2538_on_and_transmit();

		/*leds_toggle(LEDS_RED);
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
		leds_toggle(LEDS_RED);*/

		NETSTACK_RDC.off(0);
		NETSTACK_MAC.off(0);
		cc2538_rf_driver.off();
		clock_delay_usec(50000);

		//volatile uint8_t i = 0;
		/*timer_set(&mytime, 5*CLOCK_SECOND);
		while ( !(timer_expired(&mytime)) ) {
			asm("");
		}*/

		leds_toggle(LEDS_GREEN);
		/*clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);
		clock_delay_usec(50000);*/
		//leds_toggle(LEDS_GREEN);

		//cc2538_rf_driver.send(buf, 6);
		// cc2538_rf_driver.off();
		//if (cc2538_on_and_transmit() != 0) {
		//	leds_toggle(LEDS_RED);
		//}
		//else {
		//	leds_toggle(LEDS_GREEN);
		//}
		//CC2538_RF_CSP_ISRFOFF();


		//ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, &periodic_rtimer, NULL);
		//schedule_vtimer(&my_vtimer, 30*VTIMER_SECOND);
		//if (ret) {
		//	printf("rtimer set error\n");
		//}
	
		cleanup_before_sleep();

		INTERRUPTS_ENABLE();
		// setup_before_resume();

		//printf("PRESS: %d\n", lps331ap_one_shot());
		//lps331ap_power_up();

		//lps331ap_read(LPS331AP_WHO_AM_I, 1, &press);
		//printf("WHOAMI_PRESS: %x\n", press);
		//printf("PRESS: %d\n", lps331ap_get_pressure());

		//printf("WHOAMI_ACCEL: %x\n", mpu9250_readByte(MPU9250_WHO_AM_I));
		//printf("ACCEL_X: %d\n", mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H));
    	//printf("ACCEL_Y: %d\n", mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H));
    	//printf("ACCEL_Z: %d\n", mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H));

		//mpu9250_writeByte(0x2F, 0x6B);

		//printf("about to go back to start of loop\n");

	}

	PROCESS_END();
}

void setup_before_resume(void) {
	spix_enable(0);
	/* ssi0 ports/pins */
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
                  			GPIO_PIN_MASK(6));
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
                  			GPIO_PIN_MASK(7));
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM),
                  			GPIO_PIN_MASK(0));

	i2c_master_enable();
	/* ic2 ports/pins */
	// GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
 //  	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
 //  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
 //  	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));

	/* mpu9250 cs port/pin */
	//GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM),
    //                    	GPIO_PIN_MASK(3));

	/* lps331a cs port/pin */
	//GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(1));

}

void cleanup_before_sleep(void) {
	spix_disable(0);
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
	/* i2c ports/pins that need to be set/clr so not floating */
	// GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
	// GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
	// GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
	// GPIO_SET_PIN(GPIO_C_BASE, GPIO_PIN_MASK(5));
	// GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
	// GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));

	/* mpu9250 cs port/pin that need to be set/clr so not floating and not enabling ic */
	//GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM),
	//						GPIO_PIN_MASK(3));
	//GPIO_SET_OUTPUT(GPIO_B_BASE, 0x08);
	//GPIO_SET_PIN(GPIO_B_BASE, 0x08);

	/* lps331A cs port/pin that need to be set/clr so not floating and not enabling ic */
	//GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(1));
	//GPIO_SET_OUTPUT(GPIO_C_BASE, 0x02);
	//GPIO_SET_PIN(GPIO_C_BASE, 0x02);

  	return;
}


//static void periodic_rtimer(struct rtimer *rt, void* ptr){
static void periodic_vtimer() {
	INTERRUPTS_DISABLE();
	//rtimer_expired = 1;

	GPIO_CLEAR_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
	GPIO_ENABLE_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
	//leds_off(LEDS_RED);
     //uint8_t ret;

     //leds_go(counter++);   //u gonna get the led counting from 0-7 
     //printf("time now: %d\n", RTIMER_NOW());
     //printf("timer plus period: %d\n", RTIMER_NOW() + PERIOD_T);

     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, (void*)periodic_rtimer, NULL);
		//schedule_vtimer(&my_vtimer, 30*VTIMER_SECOND);
     
     //process_poll(&observer_lp_process);
     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
     //           (void (*)(struct rtimer *, void *))periodic_rtimer, NULL);
     //if(ret){
     //    printf("Error Timer: %u\n", ret);
     //}
     INTERRUPTS_ENABLE();
   return;
}


static void rtc_callback() {
	INTERRUPTS_DISABLE();

	leds_toggle(LEDS_BLUE);
	process_poll(&observer_lp_process);

	return;
}

static void amn41122_callback() {
	//leds_on(LEDS_RED);
	pir_motion = 1;
 	GPIO_DISABLE_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
 	GPIO_CLEAR_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  	process_poll(&observer_lp_process);

	return;
}