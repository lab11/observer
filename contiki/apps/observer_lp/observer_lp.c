
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
#include "scb.h"
#include "sys-ctrl.h"
#include "spi-arch.h"
#include "spi.c"
#include "assert.h"
#include <stdio.h>
#include "stdbool.h"
#include "gpio.h"
#include "nvic.h"
#include "smwdthrosc.h"
#include "cc2538-rf.h"
#include "sys/rtimer.h"
//#include "vtimer-arch.h"

//static struct etimer periodic_timer_red;
//static struct etimer periodic_timer_green;
//static struct etimer periodic_timer_blue;
static uint8_t counter = 0;
static struct rtimer my_timer;
static struct rtimer my_timer2;

#define SYS_CTRL_TIMEOUT	0x0000FFFF
#define SYS_CTRL_PERIPH_INDEX(a) (((a) >> 8) & 0xF)
#define SYS_CTRL_PERIPH_MASKBIT(a) (0x00000001 << ((a) & 0xF))

#define assert_wfi() do { asm("wfi"::); } while(0)

static const uint32_t g_pui32DCGDRegs[] = {
	SYS_CTRL_DCGCGPT,
	SYS_CTRL_DCGCSSI,
	SYS_CTRL_DCGCUART,
	SYS_CTRL_DCGCI2C,
	SYS_CTRL_DCGCSEC,
	SYS_CTRL_DCGCRFC
};

void SysCtrlClockSet(bool bExternalOsc32k, bool bInternalOsc, uint32_t ui32SysDiv);
void SysCtrlIOClockSet(uint32_t ui32IODiv);
void __attribute__((naked))
SysCtrlDelay(uint32_t ui32Count) {
	__asm(	"	subs	r0, #1\n"
			"	bne 	SysCtrlDelay\n"
			"	bx 		lr");
}
void SysCtrlPeripheralDeepSleepDisable(uint32_t ui32Peripheral);
void SysCtrlPowerModeSet(uint32_t ui32PowerMode);
void GPIOIntWakeupEnable(uint32_t ui32Config);
uint32_t SleepModeTimerCountGet(void);
void SleepModeTimerCompareSet(uint32_t ui32Compare);
void SysCtrlDeepSleep(void);

#define PERIOD_T 30*RTIMER_SECOND
#define PERIOD_T2 45*RTIMER_SECOND

static void leds_go(uint8_t count){

 //the shift is due to the change on the leds in the Tmote Sky platform
		leds_off(LEDS_ALL);
       if (count % 3 == 0) {
       	leds_on(LEDS_GREEN);
       } else if (count % 3 == 1) {
       	leds_on(LEDS_BLUE);
       } else if (count % 3 == 2) {
       	leds_on(LEDS_RED);
       } else {
       	//
       }
}


static void
select_16_mhz_rcosc(void)
{
  /*
   * Power up both oscillators in order to speed up the transition to the 32-MHz
   * XOSC after wake up.
   */
  REG(SYS_CTRL_CLOCK_CTRL) &= ~SYS_CTRL_CLOCK_CTRL_OSC_PD;

  /*First, make sure there is no ongoing clock source change */
  while((REG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_SOURCE_CHANGE) != 0);

  /* Set the System Clock to use the 16MHz RC OSC */
  REG(SYS_CTRL_CLOCK_CTRL) |= SYS_CTRL_CLOCK_CTRL_OSC;

  /* Wait till it's happened */
  while((REG(SYS_CTRL_CLOCK_STA) & SYS_CTRL_CLOCK_STA_OSC) == 0);
}

/*---------------------------------------------------------------------------------*/
/*
You can change it using if/OR clauses to regulate the period... 
this is just an example
*/
static char periodic_rtimer(struct rtimer *rt, void* ptr){
     uint8_t ret;

     //leds_go(counter++);   //u gonna get the led counting from 0-7 
     printf("time now: %d\n", RTIMER_NOW());
     printf("timer plus period: %d\n", RTIMER_NOW() + PERIOD_T);
     //ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
     //           (void (*)(struct rtimer *, void *))periodic_rtimer, NULL);
     //if(ret){
     //    printf("Error Timer: %u\n", ret);
     //}
   return 1;
}

static char periodic_rtimer2(struct rtimer *rt, void* ptr) {
	 printf("time now2: %d\n", RTIMER_NOW());
     printf("timer plus period2: %d\n", RTIMER_NOW() + PERIOD_T2);
}


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

	// Set the clocking to run from external crystal/oscillator
	SysCtrlClockSet(false, false, SYS_CTRL_CLOCK_CTRL_SYS_DIV_32MHZ);

	// Set the IO to the same as system clock
	SysCtrlIOClockSet(SYS_CTRL_CLOCK_CTRL_IO_DIV_32MHZ);

	// Display the setup on the console
	printf("Sleepmode timer\n");

	// Disable UART0(and any other peripherals in deep sleep)
	SysCtrlPeripheralDeepSleepDisable(0x00000200); // 0x00000200 = SYS_CTRL_PERIPH_UART0
	SysCtrlPeripheralDeepSleepDisable(0x00000201); // 0x00000201 = SYS_CTRL_PERIPH_UART1

	// Let system enter powermode 2 when going to deep sleep
	SysCtrlPowerModeSet(SYS_CTRL_PMCTL_PM2); // 0x00000002 = SYS_CTRL_PMCTL_PM2

	// Enable the sleep timer wakeup
	GPIOIntWakeupEnable(0x00000020); // 0x00000020 = GPIO_IWE_SM_TIMER

	// ENable sleep mode interrupt
	//IntEnable(INT_SMTIM); // 161 = INT_SMTIM (SMTimer)
	nvic_interrupt_enable(NVIC_INT_SM_TIMER); // same as above

	int i = 0;
	while(i++ < 15000 ) {
		asm("");
	}
	// Set timer to 10000  above current value
	ui32Val = SleepModeTimerCountGet();
	SleepModeTimerCompareSet(ui32Val + 1000000);

	// Display the timer value on the console
	printf("Timer val = %d\n", ui32Val);

	// Go to sleep
	SysCtrlDeepSleep();

	// Display the timer value on the console
	ui32Val = SleepModeTimerCountGet();
	printf("Timter val = %d (after wakeup)\n", ui32Val);

	//etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

	printf("HELLO WORLD\n");
	*/
	//GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM),
    //             GPIO_PIN_MASK(RV3049_INT_N_PIN));
	static struct etimer et;

//cc2538_rf_driver.off();
	//select_16_mhz_rcosc();
	//REG(SCB_SYSCTRL) |= SCB_SYSCTRL_SLEEPDEEP;
	//REG(SYS_CTRL_PMCTL) = SYS_CTRL_PMCTL_PM2;
	//assert_wfi();
	lps331ap_init();

	mpu9250_init();

	mpu9250_motion_interrupt_init(0x0F, 0x06);

	
	uint8_t press = 0;
	while(1) {
		//leds_toggle(LEDS_GREEN);
		//static struct etimer et;
		etimer_set(&et, 5*CLOCK_SECOND);
		//cc2538_rf_driver.off();
		CC2538_RF_CSP_ISRFOFF();
		//spix_disable(0);
		//REG(SYS_CTRL_SCGCSSI) &= ~(1);
		//REG(SYS_CTRL_DCGCSSI) &= ~(1);
		//spix_disable(1);
		//REG(SSI0_BASE + SSI_CR1) = 0;
		//REG(SYS_CTRL_SRSSI) |= 1;
		// ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
  //               (void*)periodic_rtimer, NULL);
		// if(ret){
  //       	//printf("Error Timer: %u\n", ret);
  //       	rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, 
  //               (void*)periodic_rtimer, NULL);
  //    	}

		/**** test ***/
		//printf("SET RTIMER1\n");
		ret = rtimer_set(&my_timer, RTIMER_NOW() + PERIOD_T, 1, (void*)periodic_rtimer, NULL);
		//printf("SET_RTIMER2\n");
		//ret2 = rtimer_set(&my_timer2, RTIMER_NOW() + PERIOD_T2, 1 , (void*)periodic_rtimer2, NULL);

		//printf("ret1: %d, ret2: %d\n", ret, ret2);

		//lps331ap_power_down();

		cleanup_before_sleep();
		//printf("GOING TO SLEEP\n");
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		//PROCESS_YIELD();
		setup_before_resume();

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

	/* i2c ports/pins that need to be set/clr so not floating */
	// GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
	// GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
	// GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
	// GPIO_CLR_PIN(GPIO_C_BASE, GPIO_PIN_MASK(5));
	// GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
	// GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));

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



void SysCtrlClockSet(bool bExternalOsc32k, bool bInternalOsc, uint32_t ui32SysDiv) {
	uint32_t ui32STA;
	uint32_t ui32Reg;
	uint32_t ui32TimeoutVal;
	uint32_t ui32Osc;

	//
	// Enable AMP detect to make sure XOSC starts correctly
	// 
	if(!bInternalOsc) {
		ui32Reg = REG(SYS_CTRL_CLOCK_CTRL) | SYS_CTRL_CLOCK_CTRL_AMP_DET;
		REG(SYS_CTRL_CLOCK_CTRL) = ui32Reg;
	}

	//
	// Set 32kHz clock, Osc and SysDiv
	//
	ui32Reg = REG(SYS_CTRL_CLOCK_CTRL);
	ui32Reg &= ~(SYS_CTRL_CLOCK_CTRL_OSC32K | SYS_CTRL_CLOCK_CTRL_OSC | SYS_CTRL_CLOCK_CTRL_SYS_DIV);

	if(!bExternalOsc32k) {
		ui32Reg |= SYS_CTRL_CLOCK_CTRL_OSC32K;
	}

	ui32Osc = (bInternalOsc) ? SYS_CTRL_CLOCK_CTRL_OSC : 0;
	ui32Reg |= ui32Osc;
	ui32Reg |= ui32SysDiv;
	REG(SYS_CTRL_CLOCK_CTRL) = ui32Reg;

	//
	// If we have changed Osc settings, wait until change happens
	//
	ui32STA = REG(SYS_CTRL_CLOCK_STA);
	ui32TimeoutVal = 0;
	while((ui32Osc != (ui32STA & SYS_CTRL_CLOCK_CTRL_OSC)) && (ui32TimeoutVal < SYS_CTRL_TIMEOUT)) {
		SysCtrlDelay(16);
		ui32STA = REG(SYS_CTRL_CLOCK_STA);
		ui32TimeoutVal++;
	}

	if(!(ui32TimeoutVal < SYS_CTRL_TIMEOUT)) {
		printf("ERROR!\n");
		return;	
	}


}

void SysCtrlIOClockSet(uint32_t ui32IODiv) {
	uint32_t ui32RegVal;

	ui32RegVal = REG(SYS_CTRL_CLOCK_CTRL);
	ui32RegVal &= ~SYS_CTRL_CLOCK_CTRL_IO_DIV; // mask
	ui32RegVal |= (ui32IODiv << 8); // shift; 8 = SYS_CTRL_CLOCK_CTRL_IO_DIV_S
	REG(SYS_CTRL_CLOCK_CTRL) = ui32RegVal;
}

void SysCtrlPeripheralDeepSleepDisable(uint32_t ui32Peripheral) {
	REG(g_pui32DCGDRegs[SYS_CTRL_PERIPH_INDEX(ui32Peripheral)]) &= ~(SYS_CTRL_PERIPH_MASKBIT(ui32Peripheral));
}


void SysCtrlPowerModeSet(uint32_t ui32PowerMode) {
	REG(SYS_CTRL_PMCTL) = ui32PowerMode;
}

void GPIOIntWakeupEnable(uint32_t ui32Config) {
	REG(SYS_CTRL_IWE) |= ui32Config;
}

uint32_t SleepModeTimerCountGet(void) {
	uint32_t ui32Val;

	ui32Val = REG(SMWDTHROSC_ST0);
	ui32Val |= REG(SMWDTHROSC_ST1) << 8;
	ui32Val |= REG(SMWDTHROSC_ST2) << 16;
	ui32Val |= REG(SMWDTHROSC_ST3) << 24;

	return ui32Val;
}

void SleepModeTimerCompareSet(uint32_t ui32Compare) {
	// Wait for ST0, ST3 regs to be ready for writing
	while(!(REG(SMWDTHROSC_STLOAD) & SMWDTHROSC_STLOAD_STLOAD))
	{
	}

	REG(SMWDTHROSC_ST3) = (ui32Compare >> 24) & 0x000000FF;
	REG(SMWDTHROSC_ST2) = (ui32Compare >> 16) & 0x000000FF;
	REG(SMWDTHROSC_ST1) = (ui32Compare >>  8) & 0x000000FF;
	REG(SMWDTHROSC_ST0) = (ui32Compare) & 0x000000FF;
}

void SysCtrlDeepSleep(void) {
#ifndef NO_CLOCK_DIVIDER_RESTORE
		bool bRestoreSys;
		bool bRestoreIO;
		uint32_t ui32Reg;

		ui32Reg = REG(SYS_CTRL_CLOCK_STA);
		bRestoreSys = (ui32Reg & SYS_CTRL_CLOCK_STA_SYS_DIV) == 0;
		bRestoreIO = (ui32Reg & SYS_CTRL_CLOCK_STA_IO_DIV) == 0;
		if (bRestoreSys || bRestoreIO) {
			ui32Reg = REG(SYS_CTRL_CLOCK_CTRL);
			ui32Reg |= bRestoreSys ? 0x1 : 0x0;
			ui32Reg |= bRestoreIO ? 0x100 : 0x0;
			REG(SYS_CTRL_CLOCK_CTRL) = ui32Reg;
		}
#endif
		// Enable deep sleep
		// 0xE000ED10 = NVIC_SYS_CTRL
		// 0x00000004 = NVIC_SYS_CTRL_SLEEPDEEP
		REG(0xE000ED10) |= 0x00000004;

#ifndef NO_CLOCK_DIVIDER_RESTORE
		if(bRestoreSys || bRestoreIO) {
			ui32Reg = REG(SYS_CTRL_CLOCK_CTRL);
			ui32Reg &= (bRestoreSys) ? ~SYS_CTRL_CLOCK_CTRL_SYS_DIV : 0xFFFFFFFF;
			ui32Reg &= (bRestoreIO) ? ~SYS_CTRL_CLOCK_CTRL_IO_DIV : 0xFFFFFFFF;
			REG(SYS_CTRL_CLOCK_CTRL) = ui32Reg;
		}
#endif
}

void SleepModeIntHandler(void) {
	printf("HELLOW\n");
	return;
}