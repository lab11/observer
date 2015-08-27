
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "vtimer.h"
#include "cpu.h"
#include "rv3049.h"

rv3049_time_t alarm_time;

static void vtimer_callback() {
	INTERRUPTS_DISABLE();
	leds_toggle(LEDS_RED);
	INTERRUPTS_ENABLE();
}

static void timer_callback() {
	INTERRUPTS_DISABLE();
	
	uint8_t ae_mask = 0x02;
	uint8_t minutes;

	leds_toggle(LEDS_RED);
	rv3049_clear_int_flag();

	rv3049_read_time(&alarm_time);
	minutes = roundUp(alarm_time.minutes, 5);
	(minutes == 60) ? (alarm_time.minutes = 0): (alarm_time.minutes = minutes);

	rv3049_set_alarm(&alarm_time, ae_mask);

	INTERRUPTS_ENABLE();
}

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;
//static struct vtimer my_vtimer;
//static struct rtimer my_rtimer;
	
//rv3049_time_t alarm_time;


/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();

	uint8_t minutes;
	rv3049_read_time(&alarm_time);
	minutes = roundUp(alarm_time.minutes, 5);
	(minutes == 60) ? (alarm_time.minutes = 0): (alarm_time.minutes = minutes);

	uint8_t ae_mask = 0x02;
	rv3049_set_alarm(&alarm_time, ae_mask);
	rv3049_clear_int_flag();
	clock_delay_usec(50000);
	rv3049_interrupt_enable(timer_callback);

	//etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);
	
//	my_vtimer = get_vtimer(vtimer_callback);
//	schedule_vtimer(&my_vtimer, 10*VTIMER_SECOND);
//	rtimer_set(&my_rtimer, RTIMER_NOW() + 15*RTIMER_SECOND, 1, &vtimer_callback, NULL);
//	leds_toggle(LEDS_ALL);
	while(1) {
		PROCESS_YIELD();

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
	}

	PROCESS_END();
}
