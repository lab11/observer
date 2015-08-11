#include "contiki.h"
#include <stdio.h>
//#include <cmath.h>
#include "sys/timer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "adc121c021.h"

void adc121c021_config(){
  adc121c021_write_reg8(ADC121C021_CONFIG, 0xe0);
}

uint16_t adc121c021_read_amplitude() {

  int16_t max = 0;
  uint16_t count = 0;
  int16_t temp = 0;
  for(count = 0; count < 10; ++count){
    temp = adc121c021_read_reg16(ADC121C021_CONV_RESULT);
    //temp &= 0x0FFF;
    //temp -= 0xA00;
    //temp = abs(temp);
    if (max < temp) max = temp;
  }

  if (ADC121C021_DBG)
    printf("adc121c021: %d\n", max);
  return max;
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
  i2c_burst_receive(ADC121C021_DEFAULT_SLAVE_ADDR, rx, 2);

  data = rx[0];
  data = data << 8;
  data |= rx[1];

 
  return data;
}

void i2c_buffer_flush() {
  int i;
  for (i=0; i < 9; i++) {
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(i2c_master_busy());
  }
}

