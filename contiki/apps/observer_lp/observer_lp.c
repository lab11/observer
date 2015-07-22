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
static void periodic_rtimer(void);

typedef enum WakeEvents { 
			DEFAULTEV, 
			PERIODIC_EV,
			ACCEL_EV, 
			MOTION_EV 
} wakeevents_t;

static wakeevents_t wakeevent = DEFAULTEV;


static uint8_t counter = 0;
static struct rtimer my_timer;
static struct rtimer my_timer2;
static struct vtimer my_vtimer;

volatile uint8_t rtimer_expired = 0;
volatile uint8_t accel_event = 0;
volatile uint8_t motion_event = 0;





/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
PROCESS(observer_lp_process, "Observer_lp");
AUTOSTART_PROCESSES(&observer_lp_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_lp_process, ev, data) {

	PROCESS_BEGIN();

	uint8_t ret;
	uint8_t ret2;

	//cc2538_rf_driver.off();
	//periodic_rtimer(&my_timer, NULL);
	// ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
                // (void*)periodic_rtimer, NULL);
	//if(ret){
    //     printf("Error Timer: %u\n", ret);
    //}
	//lpm_enter();

	/*uint32_t ui32Val;


	//etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

	printf("HELLO WORLD\n");
	*/
	//GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM),
    //             GPIO_PIN_MASK(RV3049_INT_N_PIN));
	static struct etimer et;
	static uint8_t buf[7];
	buf[0] = 1;
	buf[1] = 2;
	buf[2] = 3;
	buf[3] = 4;
	buf[4] = 5;
	buf[5] = 6;

	//cc2538_rf_driver.off();
	//NETSTACK_RADIO.off();

	amn41122_init();
	//amn41122_irq_enable();

	lps331ap_init();

	mpu9250_init();

	mpu9250_motion_interrupt_init(0x0F, 0x06);

	si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
	si1147_als_data_t als_data;
	//adc121c021_config();

	//CC2538_RF_CSP_ISRFOFF();
	cc2538_rf_driver.off();

	cleanup_before_sleep();
	//cc2538_rf_driver.off();

	my_vtimer = get_vtimer(periodic_rtimer);
	schedule_vtimer(&my_vtimer, 30*VTIMER_SECOND);
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
		setup_before_resume();

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

		uint32_t press = lps331ap_one_shot();// lps331ap_one_shot();
		// printf("PRESS: %d\n", press);
		uint16_t temp = si7021_readTemp(TEMP_NOHOLD);
		uint16_t rh = si7021_readHumd(RH_NOHOLD);
		si1147_als_force_read(&als_data);
		// printf("LIGHT: %d\n", als_data.vis.val);
		adc121c021_read_amplitude();

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
	
		//packetbuf_copyfrom(buf, 6);
		//cc2538_on_and_transmit();
		//cc2538_rf_driver.on();
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
		schedule_vtimer(&my_vtimer, 30*VTIMER_SECOND);
		//if (ret) {
		//	printf("rtimer set error\n");
		//}

		cleanup_before_sleep();
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
static void periodic_rtimer() {
	INTERRUPTS_DISABLE();
	rtimer_expired = 1;

     uint8_t ret;

     //leds_go(counter++);   //u gonna get the led counting from 0-7 
     printf("time now: %d\n", RTIMER_NOW());
     printf("timer plus period: %d\n", RTIMER_NOW() + PERIOD_T);

     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, (void*)periodic_rtimer, NULL);
     process_poll(&observer_lp_process);
     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
     //           (void (*)(struct rtimer *, void *))periodic_rtimer, NULL);
     //if(ret){
     //    printf("Error Timer: %u\n", ret);
     //}
     INTERRUPTS_ENABLE();
   return;
}




void SleepModeIntHandler(void) {
	printf("HELLOW\n");
	return;
}