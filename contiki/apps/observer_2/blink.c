/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-platforms
 * @{
 *
 * \defgroup cc2538-examples cc2538 Example Projects
 * @{
 *
 * \defgroup cc2538-demo cc2538dk Demo Project
 *
 *   Example project demonstrating the cc2538dk functionality
 *
 *   This assumes that you are using a SmartRF06EB with a cc2538 EM
 *
 * - Boot sequence: LEDs flashing, LED2 followed by LED3 then LED4
 * - etimer/clock : Every LOOP_INTERVAL clock ticks the LED defined as
 *                  LEDS_PERIODIC will turn on
 * - rtimer       : Exactly LEDS_OFF_HYSTERISIS rtimer ticks later,
 *                  LEDS_PERIODIC will turn back off
 * - Buttons      :
 *                - BTN_DOWN turns on LEDS_REBOOT and causes a watchdog reboot
 *                - BTN_UP to soft reset (SYS_CTRL_PWRDBG::FORCE_WARM_RESET)
 *                - BTN_LEFT and BTN_RIGHT flash the LED defined as LEDS_BUTTON
 * - ADC sensors  : On-chip VDD / 3 and temperature, and ambient light sensor
 *                  values are printed over UART periodically.
 * - UART         : Every LOOP_INTERVAL the EM will print something over the
 *                  UART. Receiving an entire line of text over UART (ending
 *                  in \\r) will cause LEDS_SERIAL_IN to toggle
 * - Radio comms  : BTN_SELECT sends a rime broadcast. Reception of a rime
 *                  packet will toggle LEDs defined as LEDS_RF_RX
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc2538dk platform
 */
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
//#include "dev/cc2538-sensors.h"
//#include "dev/button-sensor.h"
//#include "dev/als-sensor.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
#include "net/rime/broadcast.h"
#include "netstack.h"
#include "cpu/cc2538/dev/i2c.h"
#include "dev/cc2538-rf.h"

#include "power_manage2.h"
#include "lps331ap.h"
#include "mpu9250.h"
#include "si1147.h"
#include "si7021.h"
#include "amn41122.h"
#include "rv3049.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define LOOP_INTERVAL       CLOCK_SECOND
#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
#define LEDS_PERIODIC       LEDS_BLUE
#define LEDS_BUTTON         LEDS_RED
#define LEDS_SERIAL_IN      LEDS_GREEN
#define LEDS_REBOOT         LEDS_ALL
#define LEDS_RF_RX          (LEDS_BLUE | LEDS_GREEN)
#define BROADCAST_CHANNEL   129
/*---------------------------------------------------------------------------*/
static struct timer  t;
//static struct etimer et;
static struct rtimer rt;
static struct rtimer rtc_rtimer;
static struct rtimer pir_int_reenable_rtimer;
static uint16_t counter;
static uint8_t buf[11];
static uint8_t rtc_ya = 0;
static uint32_t press;
static uint16_t temp;
static uint16_t rh;
static si1147_als_data_t als_data;
static uint8_t mpuwai;
static uint8_t ak8963wia;

/*---------------------------------------------------------------------------*/
PROCESS(rtc_process, "rtc process");
PROCESS(accel_process, "accel process");
PROCESS(pir_process, "pir process");
AUTOSTART_PROCESSES(&rtc_process, &accel_process, &pir_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_toggle(LEDS_RF_RX);
  printf("Received %u bytes: '0x%04x'\n", packetbuf_datalen(),
         *(uint16_t *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks bc_rx = { broadcast_recv };
static struct broadcast_conn bc;
/*---------------------------------------------------------------------------*/
void
rt_callback(struct rtimer *t, void *ptr)
{
  //leds_off(LEDS_PERIODIC);
	process_poll(&rtc_process);
	//leds_toggle(LEDS_GREEN);
}

void
rtc_callback(struct rtimer *t, void *ptr)
{
	rtc_ya = 1;
	process_poll(&rtc_process);
	//leds_toggle(LEDS_BLUE);
}

// rtimer callback to reenable PIR power up interrupt
void
pir_rt_int_enable_callback(struct rtimer *t, void *ptr) 
{
	// clear in case another was pended after right before disabling it
	GPIO_CLEAR_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
	amn41122_int_reenable();
}


void
accel_irq_handler(uint8_t port, uint8_t pin)
{
	leds_toggle(LEDS_BLUE);
	process_poll(&accel_process);
	//leds_toggle(LEDS_ALL);
}

// PIR handler to disable interrupt (backoff time) and poll process
void
pir_irq_handler(uint8_t port, uint8_t pin)
{
	amn41122_int_disable();
	process_poll(&pir_process);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rtc_process, ev, data)
{
	PROCESS_EXITHANDLER(broadcast_close(&bc))

  	PROCESS_BEGIN();

	counter = 0;
	//broadcast_open(&bc, BROADCAST_CHANNEL, &bc_rx);

	disable_unused_pins();

	i2c_init(GPIO_C_NUM, 5, GPIO_C_NUM, 4, I2C_SCL_FAST_BUS_SPEED);
	

	// mic pfet
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(6));
    ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS);
    GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(6));
    GPIO_SET_PIN(GPIO_PORT_TO_BASE(GPIO_B_NUM), GPIO_PIN_MASK(6));

	lps331ap_init();
	mpu9250_init();
	ak8963_init(0x06);
	ak8963wia = ak8963_readWIA();

	// re-disable I2C slave and disable I2C master
	uint8_t USER_CTRL_reg;
	mpu9250_readByte(MPU9250_USER_CTRL, &USER_CTRL_reg);
	USER_CTRL_reg &= 0xDF; // 0b11011111
    mpu9250_writeByte(MPU9250_USER_CTRL, USER_CTRL_reg);

	mpu9250_motion_interrupt_init(0x7F, 6);
	mpu9250_interrupt_enable(accel_irq_handler);
	//ak8963_init(0x06);
	si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
	leds_on(LEDS_RED);
	amn41122_init();
	amn41122_int_enable(pir_irq_handler);
	leds_off(LEDS_RED);
	leds_on(LEDS_GREEN);

	timer_set(&t, CLOCK_SECOND*3);
    do {
        volatile uint8_t i=0;
        while(!timer_expired(&t)) i++;
    } while(0) ;
	leds_off(LEDS_GREEN);
	

	static rv3049_time_t alarm_time;
	rv3049_read_time(&alarm_time);
	alarm_time.seconds = 0;
	rv3049_clear_int_flag(); //in case rtc wasn't powered off and alarm fired
	rv3049_set_alarm(&alarm_time, 0x01);
    rv3049_interrupt_enable(rtc_callback);

	//mpu9250_readWAI();

  	//etimer_set(&et, CLOCK_SECOND);
	rtimer_set(&rtc_rtimer, RTIMER_NOW() + RTIMER_SECOND*2, 1, 						rt_callback, NULL);	

  	while(1) {
		//spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
		// arbitrary write to get spi periph into above mode	
		//SPI_WRITE(0x01);
		cleanup_before_sleep();
		//i2c_master_disable();
    	PROCESS_YIELD();
		//i2c_master_enable();
		setup_before_wake();
		
		//etimer_reset(&et);
		if (rtc_ya) {
			rv3049_clear_int_flag();
			rtc_ya = 0;
		} else {
			//rtimer_set(&rtc_rtimer, RTIMER_NOW() + RTIMER_SECOND*8, 1, 						rt_callback, NULL);
		}
		counter++;
		//leds_toggle(LEDS_GREEN);

		press = lps331ap_one_shot();
		temp = si7021_readTemp(TEMP_NOHOLD);
		rh = si7021_readHumd(RH_NOHOLD);
		si1147_als_force_read(&als_data);
		//mpuwai = mpu9250_readWAI();
		//ak8963wia = ak8963_readWIA();

		//buf[0] = (press_val & 0x000000FF);
		//buf[1] = (press_val & 0x0000FF00) >> 8;
		//buf[2] = (press_val & 0x00FF0000) >> 16; 
		buf[0] = 0x01;
		buf[1] = temp;
		buf[2] = (temp & 0xFF00) >> 8;
		buf[3] = rh;
		buf[4] = (rh & 0xFF00) >> 8;
		buf[5] = als_data.vis.b.lo;
		buf[6] = als_data.vis.b.hi;
		buf[7] = press & 0x000000FF;
		buf[8] = (press & 0x0000FF00) >> 8;
		buf[9] = (press & 0x00FF0000) >> 16;
		buf[10] = ak8963wia; // not a PIR sample

		CC2538_RF_CSP_ISTXON();
		CC2538_RF_CSP_ISFLUSHTX();
		//NETSTACK_MAC.on();
		broadcast_open(&bc, BROADCAST_CHANNEL, &bc_rx);
		packetbuf_copyfrom(buf, 11);
		broadcast_send(&bc);
		broadcast_close(&bc);
		//NETSTACK_MAC.off(0);
		CC2538_RF_CSP_ISRFOFF();
	
		//leds_toggle(LEDS_RED);

		/*if(ev == PROCESS_EVENT_TIMER) {
		  	leds_on(LEDS_PERIODIC);
		  	printf("-----------------------------------------\n"
				 "Counter = 0x%08x\n", counter);

		  	printf("VDD = %d mV\n",
				 vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED));

		 	printf("Temperature = %d mC\n",
				  cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED));

		  	printf("Ambient light sensor = %d raw\n", als_sensor.value(0));

		  	etimer_set(&et, CLOCK_SECOND);
		  	rtimer_set(&rt, RTIMER_NOW() + LEDS_OFF_HYSTERISIS, 1,
				     rt_callback, NULL);
		  	counter++;
		} else if(ev == sensors_event) {
		  	if(data == &button_select_sensor) {
		    packetbuf_copyfrom(&counter, sizeof(counter));
		    broadcast_send(&bc);
		} else if(data == &button_left_sensor || data == &button_right_sensor) {
		    leds_toggle(LEDS_BUTTON);
		  } else if(data == &button_down_sensor) {
		    cpu_cpsid();
		    leds_on(LEDS_REBOOT);
		    watchdog_reboot();
		  } else if(data == &button_up_sensor) {
		    sys_ctrl_reset();
		  }
		} else if(ev == serial_line_event_message) {
		  leds_toggle(LEDS_SERIAL_IN);
		}

        */
        
	}

	PROCESS_END();
}

PROCESS_THREAD(accel_process, ev, data) {

	PROCESS_BEGIN();

	while(1) {
		
		PROCESS_YIELD();
		setup_before_wake();

		mpu9250_int_clear();

		cleanup_before_sleep();
		//leds_toggle(LEDS_ALL);

	}

	PROCESS_END();
}

PROCESS_THREAD(pir_process, ev, data) {
	PROCESS_BEGIN();

	while(1) {
		
		PROCESS_YIELD();
		setup_before_wake();

		// rtimer will reenable PIR interrupt after 30s
		rtimer_set(&pir_int_reenable_rtimer, RTIMER_NOW()+RTIMER_SECOND*30, 1, pir_rt_int_enable_callback, NULL);

		press = lps331ap_one_shot();
		temp = si7021_readTemp(TEMP_NOHOLD);
		rh = si7021_readHumd(RH_NOHOLD);
		si1147_als_force_read(&als_data);
		mpuwai = mpu9250_readWAI();
		//ak8963wia = ak8963_readWIA();

		//buf[0] = (press_val & 0x000000FF);
		//buf[1] = (press_val & 0x0000FF00) >> 8;
		//buf[2] = (press_val & 0x00FF0000) >> 16; 
		buf[0] = 0x01;
		buf[1] = temp;
		buf[2] = (temp & 0xFF00) >> 8;
		buf[3] = rh;
		buf[4] = (rh & 0xFF00) >> 8;
		buf[5] = als_data.vis.b.lo;
		buf[6] = als_data.vis.b.hi;
		buf[7] = press & 0x000000FF;
		buf[8] = (press & 0x0000FF00) >> 8;
		buf[9] = (press & 0x00FF0000) >> 16;
		buf[10] = 0x01;

		CC2538_RF_CSP_ISTXON();
		CC2538_RF_CSP_ISFLUSHTX();
		//NETSTACK_MAC.on();
		broadcast_open(&bc, BROADCAST_CHANNEL, &bc_rx);
		packetbuf_copyfrom(buf, 11);
		broadcast_send(&bc);
		broadcast_close(&bc);
		//NETSTACK_MAC.off(0);
		CC2538_RF_CSP_ISRFOFF();


		cleanup_before_sleep();
		//leds_toggle(LEDS_ALL);

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
