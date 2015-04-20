#ifndef ADC121C021_H
#define ADC121C021_H

#include "sys/etimer.h"

// when set, enables print statements
#define ADC121C021_DBG 1
#define ADC121C021_DBG_ALS 1

#define ADC121C021_STARTUP_TIME 0.025*CLOCK_SECOND
#define ADC121C021_SAMPLING_TIME 0.05*CLOCK_SECOND
#define ADC121C021_DEFAULT_SLAVE_ADDR 0x54

// I2C Registers
#define ADC121C021_CONV_RESULT    0x00 // the only read-only reg
#define ADC121C021_ALERT_STATUS   0x01
#define ADC121C021_CONFIG         0x02
#define ADC121C021_LOW_LIMIT      0x03
#define ADC121C021_HIGH_LIMIT     0x04
#define ADC121C021_HYSTERESIS     0x05
#define ADC121C021_LOW_CONV       0x06
#define ADC121C021_HIGH_CONV      0x07

#define ADC121C021_LOW_CONV_CLEAR   0x0FFF
#define ADC121C021_HIGH_CONV_CLEAR  0x0000

void i2c_buffer_flush();

void adc121c021_config();
void adc121c021_write_reg8(uint8_t reg_addr, uint8_t data);
void adc121c021_write_reg16(uint8_t reg_addr, uint16_t data);

uint8_t adc121c021_read_reg8(uint8_t reg_addr);
uint16_t adc121c021_read_reg16(uint16_t reg_addr);

uint16_t adc121c021_read_amplitude();

#endif /*ADC121C021_H*/

