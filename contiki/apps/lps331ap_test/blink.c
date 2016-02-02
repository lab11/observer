
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
//#include "mpu9250.h"
#include "lps331ap.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;

void accel_irq(uint8_t port, uint8_t pin){
    leds_toggle(LEDS_GREEN);
}

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();

	etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);
    //mpu9250_init();
    //mpu9250_motion_interrupt_init(20, 7, accel_irq);

    lps331ap_init();

	while(1) {
		PROCESS_YIELD();
        
        //int16_t value;
        //value = mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H);
        //printf("Z val %d\n", value);

        uint32_t value = lps331ap_one_shot();
        printf("Press val %d\n", value);

        leds_toggle(LEDS_RED);
        leds_toggle(LEDS_GREEN);
        etimer_restart(&periodic_timer_red);

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
	}

	PROCESS_END();
}
