#include "contiki.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/gpio.h"
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

  // Setup SPI clock high while idle, data valid on clock trailing edge
  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  spi_cs_init(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  // Setup pressure interrupt
  //interrupt_en();

  lps331ap_write_byte(LPS331AP_CTRL_REG2, lps331ap_ctrl_reg2_reset.value);

  while(1)
  {
    uint8_t reg2 = lps331ap_read_byte(LPS331AP_CTRL_REG2);
    if(!(reg2 & 0x80))
      break;
  }
  leds_toggle(LEDS_RED);
  etimer_set(&periodic_timer, CLOCK_SECOND/100);
  PROCESS_YIELD();


  lps331ap_write_byte(LPS331AP_CTRL_REG2, lps331ap_ctrl_reg2_default.value);
  lps331ap_write_byte(LPS331AP_CTRL_REG1, lps331ap_ctrl_reg1_default.value);
  lps331ap_write_byte(LPS331AP_CTRL_REG3, lps331ap_ctrl_reg3_default.value);
  lps331ap_write_byte(LPS331AP_RES_CONF, LPS331AP_020_NOISE);

  etimer_reset(&periodic_timer);
  
  while(1) {
    //PROCESS_YIELD();
    unsigned pressure = get_pressure();
    //if(etimer_expired(&periodic_timer))
    //  etimer_reset(&periodic_timer);
    
  }
  PROCESS_END();
}

int8_t lps331ap_read_byte(uint8_t addr){
  addr |= LPS331AP_READ_MASK;
  uint8_t read;
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_READ(read);
  SPI_WAITFOREORx();
  SPI_WAITFORTxREADY(); // Extra wait loop before asserting CS
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  return read;
}

void lps331ap_read_bytes(uint8_t addr, uint8_t * bytes, unsigned size)
{
  addr |= LPS331AP_READ_MASK;
  addr |= LPS331AP_INC_MASK;
  unsigned i;
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_WAITFORTxREADY();
  SPI_WAITFOREORx();
  for(i = 0; i < size; ++i)
  {
    SPI_READ(bytes[i]);
    SPI_WAITFOREORx();
  }
  SPI_WAITFORTxREADY(); // Extra wait loop before asserting CS
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
}



void lps331ap_write_byte(uint8_t addr, uint8_t write){
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_WRITE(write);
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
}

void pressure_int_handler(uint8_t port, uint8_t pin){

}

unsigned get_pressure()
{
  unsigned pout = 0;
  unsigned i;
  uint8_t pout_parts[3];
  lps331ap_read_bytes(LPS331AP_PRESS_OUT_XL, pout_parts, 3);
  //For some fucking stupid reason, the highest order bit is byte 1 instead of byte 2
  pout = pout_parts[2] | pout_parts[0] << 8 | pout_parts[1] << 16;
  return pout;
}

void interrupt_en()
{
  nvic_init();
  GPIO_SOFTWARE_CONTROL(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  GPIO_SET_INPUT(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  GPIO_DETECT_EDGE(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  GPIO_DETECT_RISING(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  GPIO_ENABLE_INTERRUPT(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
  ioc_set_over(LPS331AP_INT_PORT, LPS331AP_INT_PIN, IOC_OVERRIDE_DIS);
  nvic_interrupt_enable(LPS331AP_INT_VECTOR);
  gpio_register_callback(pressure_int_handler, LPS331AP_INT_PORT, LPS331AP_INT_PIN);
}