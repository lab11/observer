#include "contiki.h"
#include "sys/timer.h"
#include "dev/leds.h"
#include "gpio.h"
#include "ioc.h"
#include "amn41122.h"
#include "mpu9250.h"
#include "cpu.h"

#include <stdio.h>

static struct timer amn41122_startup_timer;
static struct timer amn41122_int_timer;

extern const struct process observer_lp_process;


void amn41122_init() {
  timer_set(&amn41122_startup_timer, AMN41122_STARTUP_TIME);
  WAIT_WHILE(!timer_expired(&amn41122_startup_timer));

  // pull it down
  ioc_set_over(AMN41122_OUT_PORT, AMN41122_OUT_PIN, IOC_OVERRIDE_PDE);
}

uint8_t amn41122_read() {
  uint8_t val;
  uint8_t count;
  val = (GPIO_READ_PIN(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK));
  val = val >> AMN41122_OUT_PIN;
  
  for (count = 0; count < 50; ++count) {
    val = (GPIO_READ_PIN(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK));
    val = val >> AMN41122_OUT_PIN;
    if (val == 1) {
      break;
    }
  }

  if(AMN41122_DBG) printf("amn41122: pir %d\n", val);
  return val;
}


void amn41122_irq_enable(gpio_callback_t callback) {

  GPIO_SOFTWARE_CONTROL(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_SET_INPUT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  //GPIO_DETECT_EDGE(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  //GPIO_TRIGGER_SINGLE_EDGE(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  //GPIO_DETECT_RISING(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  //GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_POWER_UP_ON_RISING(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  GPIO_ENABLE_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  ioc_set_over(AMN41122_OUT_PORT, AMN41122_OUT_PIN, IOC_OVERRIDE_PDE);

  nvic_interrupt_enable(AMN41122_OUT_VECTOR);
  gpio_register_callback(callback, AMN41122_OUT_PORT, AMN41122_OUT_PIN);
 
}

void amn41122_irq_handler(uint8_t port, uint8_t pin) {
  INTERRUPTS_DISABLE();
  //setup_before_resume();
  //printf("PIR");
  leds_on(LEDS_RED);
  GPIO_DISABLE_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  GPIO_CLEAR_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  process_poll(&observer_lp_process);
  // GPIO_DISABLE_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  // GPIO_CLEAR_POWER_UP_INTERRUPT(AMN41122_OUT_PORT, GPIO_PIN_MASK(AMN41122_OUT_PIN));
  //mpu9250_readByte(MPU9250_INT_STATUS);

}

/*void amn41122_irq_handler(uint8_t port, uint8_t pin) {
    if (AMN41122_DBG) printf("amn41122: motion detected\n");

    if (timer_expired(&amn41122_int_timer)) {

        printf("BEFORE DISABLE MOTION INTERRUPT\n");
        //asm("CPSID i");
        INTERRUPTS_DISABLE();

        //GPIO_DISABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
        //GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));

        
        printf("BEFORE RESTART CLOCK TIMER\n");
        timer_restart(&amn41122_int_timer);
        
        printf("START MOTION ISR\n");

        printf("BEFORE CLEAR: PRINT OUT A LONG LONG LONG LONG MESSAGE TO SEE IF INTERRUPTS WILL STILL PREEMPT THIS\n");
        GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
        GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
        //GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, 0xFF);

        nvic_interrupt_unpend(AMN41122_OUT_VECTOR);

        //timer_reset(&amn41122_int_timer);

        printf("AFTER CLEAR: PRINT OUT A LONG LONG LONG LONG MESSAGE TO SEE IF INTERRUPTS WILL STILL PREEMPT THIS\n");

        process_poll(&observer_main_process);
        //GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);


        //timer_reset(&amn41122_int_timer);

    } else {
        //asm("CPSID i");
        INTERRUPTS_DISABLE();
        //GPIO_DISABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
        //GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));

        printf("MOTION INT WAIT\n");
        
        mpu9250_readByte(MPU9250_INT_STATUS);


        GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
        GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
        //GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, 0xFF);
        
        nvic_interrupt_unpend(AMN41122_OUT_VECTOR);

        //GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
        //GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));

        //asm("CPSIE i");
        INTERRUPTS_ENABLE();

    }


}*/