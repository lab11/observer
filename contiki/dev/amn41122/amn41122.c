#include "contiki.h"
#include "sys/timer.h"
#include "dev/leds.h"
#include "gpio.h"
#include "ioc.h"
#include "amn41122.h"

#include <stdio.h>

static struct timer amn41122_startup_timer;

void amn41122_init() {
  timer_set(&amn41122_startup_timer, AMN41122_STARTUP_TIME);
  WAIT_WHILE(!timer_expired(&amn41122_startup_timer));

  // pull it down
  ioc_set_over(AMN41122_OUT_PORT, AMN41122_OUT_PIN, IOC_OVERRIDE_PDE);
}

uint8_t amn41122_read() {
  uint8_t val;
  val = (GPIO_READ_PIN(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK));
  val = val >> AMN41122_OUT_PIN;
  
  if(AMN41122_DBG) printf("amn41122: pir %d\n", val);
  return val;
}

void amn41122_irq_handler() {
  if (AMN41122_DBG) printf("amn41122: motion detected\n");
}

void amn41122_irq_enable() {
  nvic_init();
  GPIO_SOFTWARE_CONTROL(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_SET_INPUT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_SET_INPUT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_DETECT_EDGE(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_DETECT_RISING(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
  nvic_interrupt_enable(AMN41122_OUT_VECTOR);
  gpio_register_callback((gpio_callback_t)amn41122_irq_handler, AMN41122_OUT_PORT, AMN41122_OUT_PIN);
}
