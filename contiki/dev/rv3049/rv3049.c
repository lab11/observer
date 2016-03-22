#include "contiki.h"
#include "rv3049.h"
#include "spi-arch.h"
#include "spi.h"
#include "dev/ssi.h"
#include <stdio.h>
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/ioc.h"
#include "cpu.h"

/**
* \file   Contiki driver for the SPI based Micro Crystal RV-3049 RTC.
* \author Brad Campbell <bradjc@umich.edu>
*/


// Check that the application was compiled with the RTC constants for
// initialization
#ifndef RTC_SECONDS
#error "To use the RTC you must compile with RTC_ initial values."
#endif

extern const struct process observer_lp_process;

void temp_handler(uint8_t port, uint8_t pin);

uint8_t roundUp(uint8_t n, uint8_t amount) {
    return (n + amount) / amount * amount;
}

uint8_t rv3049_binary_to_bcd (uint8_t binary) {
  uint8_t out = 0;

  if (binary >= 40) {
    out |= 0x40;
    binary -= 40;
  }
  if (binary >= 20) {
    out |= 0x20;
    binary -= 20;
  }
  if (binary >= 10) {
    out |= 0x10;
    binary -= 10;
  }
  out |= binary;
  return out;
}

void
rv3049_init()
{
  /* Set the HOLD_N and WP_N pins to outputs and high */
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM),
                 GPIO_PIN_MASK(RV3049_INT_N_PIN));
  ioc_set_over(RV3049_INT_N_PORT_NUM, RV3049_INT_N_PIN, IOC_OVERRIDE_PUE);

  spix_cs_init(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Write the initial values
  {
    rv3049_time_t start_time = {RTC_SECONDS, RTC_MINUTES, RTC_HOURS,
                                RTC_DAYS,    RTC_WEEKDAY, RTC_MONTH,
                                RTC_YEAR};
    rv3049_set_time(&start_time);
  }
}

int
rv3049_read_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Tell the RTC we want to read the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_CLOCK));

  SPI_FLUSH();

  // Read a null byte here. Not exactly sure why.
  SPI_READ(buf[0]);

  // Then actually read the clock
  for (i=0; i<RV3049_READ_LEN_TIME; i++) {
    SPI_READ(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Convert the values
  time->seconds = BCD_TO_BINARY(buf[0]);
  time->minutes = BCD_TO_BINARY(buf[1]);
  time->hours   = BCD_TO_BINARY((buf[2])&0x3F);
  time->days    = BCD_TO_BINARY(buf[3]);
  time->weekday = buf[4];
  time->month   = buf[5];
  time->year    = BCD_TO_BINARY(buf[6])+2000;

  return 0;
}

/***************************************************/
int
rv3049_read_alarm(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Tell the RTC we want to read the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM));

  SPI_FLUSH();

  // Read a null byte here. Not exactly sure why.
  SPI_READ(buf[0]);

  // Then actually read the clock
  for (i=0; i<RV3049_READ_LEN_TIME; i++) {
    SPI_READ(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Convert the values
  time->seconds = BCD_TO_BINARY(buf[0]);
  time->minutes = BCD_TO_BINARY(buf[1]);
  time->hours   = BCD_TO_BINARY((buf[2])&0x3F);
  time->days    = BCD_TO_BINARY(buf[3]);
  time->weekday = buf[4];
  time->month   = buf[5];
  time->year    = BCD_TO_BINARY(buf[6])+2000;

  return 0;
}

uint8_t rv3049_read_int_flag() {
  uint8_t buf[1];
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Tell the RTC we want to read the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

  SPI_FLUSH();

  // Read a null byte here. Not exactly sure why.
  SPI_READ(buf[0]);

  SPI_READ(buf[0]);

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  return *buf;
}
/***************************************************/

int
rv3049_set_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  buf[0] = rv3049_binary_to_bcd(time->seconds);
  buf[1] = rv3049_binary_to_bcd(time->minutes);
  buf[2] = rv3049_binary_to_bcd(time->hours); // 24 hour mode
  buf[3] = rv3049_binary_to_bcd(time->days);
  buf[4] = time->weekday;
  buf[5] = time->month;
  buf[6] = rv3049_binary_to_bcd(time->year - 2000);

  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Signal a write to the clock
  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_CLOCK));

  // Write the clock values
  for (i=0; i<RV3049_WRITE_LEN_TIME; i++) {
    SPI_WRITE(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  return 0;
}

int 
rv3049_set_alarm(rv3049_time_t* time, uint8_t ae_mask)
{
  uint8_t buf[8];
  int i;

  buf[0] = rv3049_binary_to_bcd(time->seconds);
  buf[1] = rv3049_binary_to_bcd(time->minutes);
  buf[2] = rv3049_binary_to_bcd(time->hours); // 24 hour mode
  buf[3] = rv3049_binary_to_bcd(time->days);
  buf[4] = time->weekday;
  buf[5] = time->month;
  buf[6] = rv3049_binary_to_bcd(time->year - 2000);

  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

  /********************** set alarm date-time *********************************/

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Signal a write to the clock
  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM));

  // Write the clock values
  for (i=0; i<RV3049_WRITE_LEN_TIME; i++) {
    if ((1 << i) & (ae_mask)) {
      SPI_WRITE(RV3049_SET_ALARM_ENABLE_BIT(buf[i]));
    } else {
      SPI_WRITE(RV3049_SET_ALARM_DISABLE_BIT(buf[i]));
    }
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  /****************************************************************************/


  /*********************** enable alarm bit ***********************************/

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Signal a write to the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_CONTROL));

  SPI_FLUSH();

  /* read null byte here */
  SPI_READ(buf[0]);

  // Then actually read the alarm interrupt control reg
  SPI_READ(buf[0]);

  SPI_FLUSH();

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);


  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM_INT_CONTROL));

  SPI_WRITE(buf[0] | 0x01);

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  /****************************************************************************/

  return 0;
}

int
rv3049_disable_alarm() 
{
  uint8_t buf[1];
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

 /*********************** disable alarm bit ***********************************/

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_CONTROL));

  SPI_FLUSH();

  SPI_READ(buf[0]);

  // Then actually read the alarm interrupt control reg
  SPI_READ(buf[0]);

  SPI_FLUSH();

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);


  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM_INT_CONTROL));

  SPI_WRITE(buf[0] & 0xFE);

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  return 0;
}
void
rv3049_interrupt_enable(gpio_callback_t callback) {
// Enable GPIO Pin as Interrupt
    //gpio_register_callback(temp_handler, RV3049_INT_N_PORT_NUM, RV3049_INT_N_PIN);
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    //GPIO_DETECT_EDGE(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    //GPIO_TRIGGER_SINGLE_EDGE(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    //GPIO_DETECT_FALLING(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    //GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));
    GPIO_POWER_UP_ON_FALLING(RV3049_INT_N_PORT_NUM, GPIO_PIN_MASK(RV3049_INT_N_PIN));
    GPIO_ENABLE_POWER_UP_INTERRUPT(RV3049_INT_N_PORT_NUM, GPIO_PIN_MASK(RV3049_INT_N_PIN));
    //ioc_set_over(RV3049_INT_N_PORT_NUM, RV3049_INT_N_PIN, IOC_OVERRIDE_PUE);
    nvic_interrupt_enable(NVIC_INT_GPIO_PORT_B);
    gpio_register_callback(callback, RV3049_INT_N_PORT_NUM, RV3049_INT_N_PIN);
}

void temp_handler(uint8_t port, uint8_t pin) {
INTERRUPTS_DISABLE();
  //printf ("hello\n");
  leds_toggle(LEDS_BLUE);
  process_poll(&observer_lp_process);
/*  uint8_t buf[1];

  GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM), GPIO_PIN_MASK(RV3049_INT_N_PIN));

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

  SPI_FLUSH();

  // null byte read
  SPI_READ(buf[0]);

  // Then actually read the alarm interrupt control reg
  SPI_READ(buf[0]);

  SPI_FLUSH();

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);


  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

  SPI_WRITE(buf[0] & 0xFE); // to clear AF flag

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
*/

/*
  rv3049_time_t alarm_time;
  rv3049_read_time(&alarm_time);
  uint8_t minutes;

  minutes = roundUp(alarm_time.minutes);
  (minutes == 60) ? (alarm_time.minutes = 0): (alarm_time.minutes = minutes);


  uint8_t ae_mask = 0x02;
  //rv3049_interrupt_enable();
  rv3049_set_alarm(&alarm_time, ae_mask);
*/
  //INTERRUPTS_ENABLE();

  return;
}

void rv3049_clear_int_flag() {
  //INTERRUPTS_DISABLE();
  uint8_t buf[1];

  GPIO_CLEAR_POWER_UP_INTERRUPT(RV3049_INT_N_PORT_NUM, GPIO_PIN_MASK(RV3049_INT_N_PIN));
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);
  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

  SPI_FLUSH();

  // null byte read
  SPI_READ(buf[0]);

  // Then actually read the alarm interrupt control reg
  SPI_READ(buf[0]);

  SPI_FLUSH();

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);


  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_ALARM_INT_FLAG));

  SPI_WRITE(buf[0] & 0xFE); // to clear AF flag

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  //INTERRUPTS_ENABLE();
}
