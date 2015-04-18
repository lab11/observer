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
#include "misc.h"

static struct timer lps331ap_startup_timer;

void lps331ap_init() {
  spi_cs_init(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  lps331ap_write_byte(LPS331AP_CTRL_REG2, lps331ap_ctrl_reg2_reset.value);
  
  while(1)
  {
    uint8_t reg2 = lps331ap_read_byte(LPS331AP_CTRL_REG2);
    if(!(reg2 & 0x80))
      break;
  }
  
  timer_set(&lps331ap_startup_timer, LPS331AP_STARTUP_TIME);
  WAIT_WHILE(!timer_expired(&lps331ap_startup_timer));
  
  lps331ap_write_byte(LPS331AP_CTRL_REG2, lps331ap_ctrl_reg2_default.value);
  lps331ap_write_byte(LPS331AP_CTRL_REG1, lps331ap_ctrl_reg1_default.value);
  lps331ap_write_byte(LPS331AP_CTRL_REG3, lps331ap_ctrl_reg3_default.value);
  lps331ap_write_byte(LPS331AP_RES_CONF, LPS331AP_020_NOISE);
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

unsigned lps331ap_get_pressure()
{
  unsigned pout = 0;
  uint8_t pout_parts[3];
  lps331ap_read_bytes(LPS331AP_PRESS_OUT_XL, pout_parts, 3);
  //For some fucking stupid reason, the highest order bit is byte 1 instead of byte 2
  pout = pout_parts[2] | pout_parts[0] << 8 | pout_parts[1] << 16;

  if (LPS331AP_DBG) printf("lps331ap: pressure 0x%x\n", pout);
  return pout;
}
