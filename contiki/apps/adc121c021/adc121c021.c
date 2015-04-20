#include <stdio.h>

#include "contiki.h"
#include "sys/timer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/i2c.h"
#include "dev/soc-adc.h"
#include "adc121c021.h"

static struct timer periodic_timer_blue;
static struct timer periodic_timer_samples;
static struct timer adc121c021_sampling_timer;

#define NUM_SAMPLES 2000

/*---------------------------------------------------------------------------*/
PROCESS(adc121c021_process, "ADC121C021");
AUTOSTART_PROCESSES(&adc121c021_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(adc121c021_process, ev, data) {

	PROCESS_BEGIN(); 

	timer_set(&periodic_timer_blue, 0.5*CLOCK_SECOND);
	timer_set(&periodic_timer_samples, 0.05*CLOCK_SECOND);
  
  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_NORMAL_BUS_SPEED);

  adc121c021_write_reg8(ADC121C021_CONFIG, 0x00);
  adc121c021_read_reg16(ADC121C021_LOW_CONV);
  adc121c021_read_reg16(ADC121C021_HIGH_CONV);
  adc121c021_write_reg16(ADC121C021_LOW_CONV, 0x000F);
  adc121c021_read_reg16(ADC121C021_LOW_CONV);
  adc121c021_write_reg16(ADC121C021_HIGH_CONV, 0xC12);
  adc121c021_read_reg16(ADC121C021_HIGH_CONV);
  adc121c021_write_reg8(ADC121C021_CONFIG, 0xe0);
  adc121c021_read_reg8(ADC121C021_CONFIG);
  adc121c021_read_reg16(ADC121C021_HIGH_CONV);
  while(1) {
    printf("Hello world!\n");
    adc121c021_read_reg16(ADC121C021_CONV_RESULT);
    adc121c021_read_reg8(ADC121C021_LOW_CONV);
    //adc121c021_write_reg8(ADC121C021_CONFIG, 0xe0);
    //adc121c021_read_reg8(ADC121C021_CONFIG);
    ////adc121c021_write_reg8(ADC121C021_CONFIG, 0x40);
    ////adc121c021_read_reg8(ADC121C021_CONFIG);
    ////adc121c021_write_reg16(ADC121C021_LOW_LIMIT, 0x0FFF);
    ////adc121c021_read_reg16(ADC121C021_LOW_LIMIT);
    //adc121c021_read_reg16(ADC121C021_CONV_RESULT);

    uint16_t amp =   adc121c021_read_amplitude();
    printf("Microphone amplitude: %x\n", amp);
	  while (!timer_expired(&periodic_timer_blue));
  	leds_toggle(LEDS_BLUE);
		timer_restart(&periodic_timer_blue);
	}

	PROCESS_END();
}

uint16_t adc121c021_read_amplitude() {
  uint16_t low, high;
/*
  adc121c021_write_reg16(ADC121C021_LOW_CONV, ADC121C021_LOW_CONV_CLEAR);
  low = adc121c021_read_reg16(ADC121C021_LOW_CONV);
  adc121c021_write_reg16(ADC121C021_HIGH_CONV, ADC121C021_HIGH_CONV_CLEAR);
  high = adc121c021_read_reg16(ADC121C021_HIGH_CONV);*/

  // start automatic conversion
  adc121c021_write_reg8(ADC121C021_CONFIG, 0x20);
  adc121c021_read_reg8(ADC121C021_CONFIG);
  
	
  timer_set(&adc121c021_sampling_timer, ADC121C021_SAMPLING_TIME);
  while(!timer_expired(&adc121c021_sampling_timer));

  // stop automatic conversion
  adc121c021_write_reg8(ADC121C021_CONFIG, 0x00);

  low = adc121c021_read_reg16(ADC121C021_LOW_CONV);
  high = adc121c021_read_reg16(ADC121C021_HIGH_CONV);;

  return high-low;
}

void adc121c021_write_reg8(uint8_t reg_addr, uint8_t data) {
  uint8_t tx[] = {reg_addr, data};

  i2c_buffer_flush();
  if (ADC121C021_DBG)
    printf("adc121c021_write_reg: [0x%x] <- 0x%x\n", reg_addr, data);

  i2c_burst_send(ADC121C021_DEFAULT_SLAVE_ADDR, tx, sizeof(tx));
  return;
}

void adc121c021_write_reg16(uint8_t reg_addr, uint16_t data) {
  uint8_t tx[] = {reg_addr, (uint8_t) (data >> 8), (uint8_t) data};

  i2c_buffer_flush();
  if (ADC121C021_DBG)
    printf("adc121c021_write_reg: [0x%x] <- 0x%x\n", reg_addr, data);

  i2c_burst_send(ADC121C021_DEFAULT_SLAVE_ADDR, tx, sizeof(tx));
  return;
}

uint8_t adc121c021_read_reg8(uint8_t reg_addr) {
  uint8_t tx[] = {reg_addr};
  uint8_t rx[1];
  i2c_buffer_flush();

  i2c_single_send(ADC121C021_DEFAULT_SLAVE_ADDR, *tx);
  //uint8_t tx2[] = {reg_addr, ADC121C021_ALERT_STATUS};
  //i2c_burst_send(ADC121C021_DEFAULT_SLAVE_ADDR, tx2, 2);
  i2c_single_receive(ADC121C021_DEFAULT_SLAVE_ADDR, rx);

  if (ADC121C021_DBG)
    printf("adc121c021_read_reg: [0x%x] -> 0x%x\n", reg_addr, *rx);
  return *rx;
}



uint16_t adc121c021_read_reg16(uint16_t reg_addr) {
  uint8_t tx[] = {reg_addr};
  uint8_t rx[2];
  uint16_t data;
  i2c_buffer_flush();

  i2c_single_send(ADC121C021_DEFAULT_SLAVE_ADDR, *tx);
  i2c_single_receive(ADC121C021_DEFAULT_SLAVE_ADDR, rx);
  i2c_burst_receive(ADC121C021_DEFAULT_SLAVE_ADDR, rx, 2);

  data = rx[0];
  data = data << 8;
  data |= rx[1];

  if (ADC121C021_DBG)
    printf("adc121c021_read_reg: [0x%x] -> 0x%x\n", reg_addr, data & 0x0FFF);
  return data;
}

void i2c_buffer_flush() {
  int i;
  for (i=0; i < 9; i++) {
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(i2c_master_busy());
  }
}

