#ifndef AMN41122_H
#define AMN41122_H

#define AMN41122_DBG                1

#define AMN41122_OUT_BASE           GPIO_PORT_TO_BASE(AMN41122_OUT_PORT)
#define AMN41122_OUT_PORT           GPIO_B_NUM
#define AMN41122_OUT_PIN            5
#define AMN41122_OUT_PIN_MASK       GPIO_PIN_MASK(AMN41122_OUT_PIN)
#define AMN41122_OUT_VECTOR         NVIC_INT_GPIO_PORT_B

#define AMN41122_STARTUP_TIME       30*CLOCK_SECOND

#define WAIT_WHILE(cond) \
  do { \
    volatile int8_t i=0; \
    while(cond) i++; \
  } while(0)

void amn41122_init();
uint8_t amn41122_read();
void amn41122_int_enable(gpio_callback_t callback);
void amn41122_int_disable();
void amn41122_irq_handler(uint8_t port, uint8_t pin);

#endif /*AMN41122_H*/
