#include "contiki.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "gpio.h"
#include "spi-arch.h"
#include "dev/spi.h"
#include "dev/nvic.h"
#include "dev/ioc.h"
#include "lps331ap.h"


static struct etimer periodic_timer;

/*---------------------------------------------------------------------------*/
PROCESS(lps331ap_process, "lps331ap");
AUTOSTART_PROCESSES(&lps331ap_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lps331ap_process, ev, data) {
  PROCESS_BEGIN();
  etimer_set(&periodic_timer, CLOCK_SECOND);

  // Setup SPI clock high while idle, data valid on clock trailing edge
  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  spi_cs_init(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  // Setup pressure irq
  GPIO_SOFTWARE_CONTROL(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  GPIO_SET_INPUT(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  GPIO_DETECT_EDGE(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  GPIO_DETECT_RISING(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  GPIO_ENABLE_INTERRUPT(LPS331AP_IRQ_BASE, LPS331AP_IRQ_PIN_MASK);
  ioc_set_over(LPS331AP_IRQ_PORT, LPS331AP_IRQ_PIN, IOC_OVERRIDE_DIS);
  nvic_interrupt_enable(NVIC_INT_GPIO_PORT_C);
  gpio_register_callback((gpio_callback_t)pressure_irq_handler, LPS331AP_IRQ_PORT, LPS331AP_IRQ_PIN);

  lps331ap_write_byte(LPS331AP_CTRL_REG1, lps331ap_ctrl_reg1_default.value);
  lps331ap_write_byte(LPS331AP_CTRL_REG3, lps331ap_ctrl_reg3_default.value);
  lps331ap_write_byte(LPS331AP_RES_CONF, LPS331AP_025_NOISE);
  
  while(1) {
    PROCESS_YIELD();

    uint8_t who_am_i = lps331ap_read_byte(LPS331AP_WHO_AM_I);
    printf("%x\n", who_am_i);

    if (etimer_expired(&periodic_timer))
    {
        leds_toggle(LEDS_RED);
        etimer_restart(&periodic_timer);
    }
    
  }
  PROCESS_END();
}

int8_t lps331ap_read_byte(uint8_t addr){
  addr |= LPS331AP_READ_MASK;
  uint8_t read;
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_READ(read);
  SPI_WAITFOREOTx(); // Extra wait loop before asserting CS
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  return read;
}



void lps331ap_write_byte(uint8_t addr, uint8_t write){
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_WRITE(write);
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
}

void pressure_irq_handler(){
  int pout = 0;
  int8_t read = 0;
  read = lps331ap_read_byte(LPS331AP_PRESS_POUT_XL_REH);
  pout = read;
  read = lps331ap_read_byte(LPS331AP_PRESS_OUT_L);
  pout |= (read << 8);
  read = lps331ap_read_byte(LPS331AP_PRESS_OUT_H);
  pout |= (read << 16);
  printf("pressure: %d\n", pout/4096);
}
