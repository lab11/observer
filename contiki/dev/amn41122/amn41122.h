#ifndef AMN41122_H
#define AMN41122_H

#define AMN41122_DBG                1

#define AMN41122_OUT_BASE           GPIO_B_BASE
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

void amn41122_init_enable();
uint8_t amn41122_read();
void amn41122_irq_enable();
void amn41122_irq_handler();

#endif /*AMN41122_H*/
