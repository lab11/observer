#include <stdio.h>

#include "contiki.h"
#include "serial-line.h"
#include "gpio.h"
#include "spi-arch.h"
#include "spi.h"
#include "nvic.h"
#include "ioc.h"
#include "lps331ap.h"

void
lps331ap_init()
{
  spix_cs_init(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  lps331ap_write(LPS331AP_CTRL_REG2, 1, &lps331ap_ctrl_reg2_reset.value);
  //lps331ap_write(LPS331AP_CTRL_REG2, 1, &(0x84));

  while(1)
  {
    uint8_t reg2;
    lps331ap_read(LPS331AP_CTRL_REG2, 1, &reg2);
    if(!(reg2 & 0x80))
      break;
  }

  lps331ap_write(LPS331AP_CTRL_REG2, 1, &lps331ap_ctrl_reg2_default.value);

  lps331ap_write(LPS331AP_CTRL_REG1, 1, &lps331ap_ctrl_reg1_default.value);
  //lps331ap_write(LPS331AP_CTRL_REG1, 1, &(0x96));

  lps331ap_write(LPS331AP_CTRL_REG3, 1, &lps331ap_ctrl_reg3_default.value);
  //lps331ap_write(LPS331AP_CTRL_REG3, 1, &(0x00));

  lps331ap_write(LPS331AP_RES_CONF, 1, &lps332ap_res_cfg_default.value);
  //lps331ap_write(LPS331AP_RES_CONF, 1, &(0x73));

	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(LPS331AP_INT_PORT), GPIO_PIN_MASK(LPS331AP_INT_PIN));
	ioc_set_over(LPS331AP_INT_PORT, LPS331AP_INT_PIN, IOC_OVERRIDE_DIS);
	GPIO_SET_INPUT(GPIO_PORT_TO_BASE(LPS331AP_INT_PORT), GPIO_PIN_MASK(LPS331AP_INT_PIN));
}

int
lps331ap_read(uint8_t address, uint16_t len, uint8_t * buf)
{
  uint16_t i;
  uint8_t read_address = address | LPS331AP_READ_MASK | LPS331AP_INC_MASK;

  // Setup SPI clock high while idle, data valid on clock trailing edge
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  SPI_WRITE(read_address);

  SPI_FLUSH();

  for(i=0; i < len; ++i)
  {
    SPI_READ(buf[i]);
  }

  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  return 0;
}



int
lps331ap_write(uint8_t address, uint16_t len, uint8_t * buf)
{
  uint16_t i;
  uint8_t write_address = address | LPS331AP_INC_MASK;

  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  SPI_WRITE(write_address);

  for(i=0; i<len; ++i){
    SPI_WRITE(buf[i]);
  }

  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  return 0;
}

int lps331ap_write_noinc(uint8_t address, uint8_t * buf) {
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  SPI_WRITE(address);

  SPI_WRITE(*buf);

  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);

  return 0;

}

uint32_t
lps331ap_get_pressure()
{
  uint8_t buf[3];
  uint32_t val;
  lps331ap_read(LPS331AP_PRESS_OUT_XL, 3, buf);
  val = (buf[0] | (buf[1]<<8) | (buf[2]<<16));
  
  if (LPS331AP_DBG) printf("lps331ap: read %d\n", val/4096);
  return val;
}

uint32_t
lps331ap_one_shot(void) 
{
  uint8_t reg2_one_shot = 0x01;
  //uint8_t temp;
  //uint8_t buf[3];
  uint32_t val;
  //lps331ap_read(LPS331AP_CTRL_REG1, 1, &temp);
  //lps331ap_write_noinc(LPS331AP_CTRL_REG2, &reg2_one_shot);
  lps331ap_write_noinc(LPS331AP_CTRL_REG2, &reg2_one_shot);

  while(1)
  {
    uint8_t status_reg;
    lps331ap_read(LPS331AP_STATUS_REG, 1, &status_reg);
    if(status_reg & 0x02) { // data ready
      break;
    }
  }

  val = lps331ap_get_pressure(); // will clear status reg when MSB is read

  return val;
}

void
lps331ap_power_up(void)
{
  uint8_t buf;
  lps331ap_read(LPS331AP_CTRL_REG1, 1, &buf);

  buf |= LPS331AP_POWER_UP_MASK;

  lps331ap_write(LPS331AP_CTRL_REG1, 1, &buf);

  return;
}

void 
lps331ap_power_down(void) 
{
  uint8_t buf;
  lps331ap_read(LPS331AP_CTRL_REG1, 1, &buf);

  buf &= LPS331AP_POWER_DOWN_MASK;

  lps331ap_write(LPS331AP_CTRL_REG1, 1, &buf);

  return;
}

// void interrupt_en()
// {
//   nvic_init();
//   GPIO_SOFTWARE_CONTROL(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   GPIO_SET_INPUT(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   GPIO_DETECT_EDGE(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   GPIO_TRIGGER_SINGLE_EDGE(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   GPIO_DETECT_RISING(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   GPIO_ENABLE_INTERRUPT(LPS331AP_INT_BASE, LPS331AP_INT_PIN_MASK);
//   ioc_set_over(LPS331AP_INT_PORT, LPS331AP_INT_PIN, IOC_OVERRIDE_DIS);
//   nvic_interrupt_enable(LPS331AP_INT_VECTOR);
//   gpio_register_callback(pressure_int_handler, LPS331AP_INT_PORT, LPS331AP_INT_PIN);
// }
