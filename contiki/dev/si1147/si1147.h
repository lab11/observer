#ifndef SI1147_H
#define SI1147_H

#include "sys/etimer.h"

// when set, enables print statements
#define SI1147_DBG 0
#define SI1147_DBG_ALS 1

#define SI1147_STARTUP_TIME 0.025*CLOCK_SECOND
#define SI1147_DEFAULT_SLAVE_ADDR 0x60

#define SI1147_IRQ_BASE           GPIO_C_BASE
#define SI1147_IRQ_PORT           GPIO_C_NUM
#define SI1147_IRQ_PIN            2
#define SI1147_IRQ_PIN_MASK       GPIO_PIN_MASK(SI1147_IRQ_PIN)

// bit 6 disables auto incr when set
#define SI1147_AUTO_INCR_DISABLE 0x40
#define SI1147_PS_ENABLE 0x07
#define SI1147_ALS_ENABLE 0xF0
#define SI1147_PSALS_ENABLE 0xF7

// measurement rates
#define SI1147_FORCED_CONVERSION 0
#define SI1147_US_PER_MEAS_RATE 31.25

// I2C Registers
#define SI1147_PART_ID        0x00
#define SI1147_REV_ID         0x01
#define SI1147_SEQ_ID         0x02
#define SI1147_INT_CFG        0x03
#define SI1147_IRQ_ENABLE     0x04
#define SI1147_HW_KEY         0x07
#define SI1147_MEAS_RATE0     0x08
#define SI1147_MEAS_RATE1     0x09
#define SI1147_PS_LED21       0x0F
#define SI1147_PS_LED3        0x10
#define SI1147_UCOEF0         0x13
#define SI1147_UCOEF1         0x14
#define SI1147_UCOEF2         0x15
#define SI1147_UCOEF3         0x16
#define SI1147_PARAM_WR       0x17
#define SI1147_COMMAND        0x18
#define SI1147_RESPONSE       0x20
#define SI1147_IRQ_STATUS     0x21
#define SI1147_ALS_VIS_DATA0  0x22
#define SI1147_ALS_VIS_DATA1  0x23
#define SI1147_ALS_IR_DATA0   0x24
#define SI1147_ALS_IR_DATA1   0x25
#define SI1147_PS1_DATA0      0x26
#define SI1147_PS1_DATA1      0x27
#define SI1147_PS2_DATA0      0x28
#define SI1147_PS2_DATA1      0x29
#define SI1147_PS3_DATA0      0x2A
#define SI1147_PS3_DATA1      0x2B
#define SI1147_AUX_DATA0      0x2C // aka UVINDEX0
#define SI1147_UVINDEX0       0x2C // aka AUX_DATA0
#define SI1147_AUX_DATA1      0x2D // aka UVINDEX1
#define SI1147_UVINDEX1       0x2D // aka AUX_DATA1
#define SI1147_PARAM_RD       0x2E
#define SI1147_CHIP_STAT      0x30
#define SI1147_ANA_IN_KEY0    0x3B
#define SI1147_ANA_IN_KEY1    0x3C
#define SI1147_ANA_IN_KEY2    0x3D
#define SI1147_ANA_IN_KEY3    0x3E

// COMMAND register commands
#define SI1147_COMMAND_PARAM_QUERY  0x80 // [4:0] are the parameter bits
#define SI1147_COMMAND_PARAM_SET    0xA0 // [4:0] are the parameter bits
#define SI1147_COMMAND_NOP          0x00
#define SI1147_COMMAND_RESET        0x01
#define SI1147_COMMAND_BUSADDR      0x02
#define SI1147_COMMAND_PS_FORCE     0x05
#define SI1147_COMMAND_GET_CAL      0x12
#define SI1147_COMMAND_ALS_FORCE    0x06
#define SI1147_COMMAND_PSALS_FORCE  0x07
#define SI1147_COMMAND_PS_PAUSE     0x09
#define SI1147_COMMAND_ALS_PAUSE    0x0A
#define SI1147_COMMAND_PSALS_PAUSE  0x0B
#define SI1147_COMMAND_PS_AUTO      0x0D
#define SI1147_COMMAND_ALS_AUTO     0x0E
#define SI1147_COMMAND_PSALS_AUTO   0x0F

// RESPONSE register error codes
#define SI1147_RESPONSE_NO_ERROR              0x00 // [3:0] is counter
#define SI1147_RESPONSE_INVALID_SETTING       0x80
#define SI1147_RESPONSE_PS1_ADC_OVERFLOW      0x88
#define SI1147_RESPONSE_PS2_ADC_OVERFLOW      0x89
#define SI1147_RESPONSE_PS3_ADC_OVERFLOW      0x8A
#define SI1147_RESPONSE_ALS_VIS_ADC_OVERFLOW  0x8C
#define SI1147_RESPONSE_ALS_IR_ADC_OVERFLOW   0x8D
#define SI1147_RESPONSE_AUX_ADC_OVERFLOW      0x8E

// Parameter RAM addresses
#define SI1147_PARAM_I2C_ADDR             0x00
#define SI1147_PARAM_CHLIST               0x01
#define SI1147_PARAM_PSLED12_SELECT       0x02
#define SI1147_PARAM_PSLED3_SELECT        0x03
#define SI1147_PARAM_PS_ENCODING          0x05
#define SI1147_PARAM_ALS_ENCODING         0x06
#define SI1147_PARAM_PS1_ADCMUX           0x07
#define SI1147_PARAM_PS2_ADCMUX           0x08
#define SI1147_PARAM_PS3_ADCMUX           0x09
#define SI1147_PARAM_PS_ADC_COUNTER       0x0A
#define SI1147_PARAM_PS_ADC_GAIN          0x0B
#define SI1147_PARAM_PS_ADC_MISC          0x0C
#define SI1147_PARAM_ALS_IR_ADCMUX        0x0E
#define SI1147_PARAM_AUX_ADCMUX           0x0F
#define SI1147_PARAM_ALS_VIS_ADC_COUNTER  0x10
#define SI1147_PARAM_ALS_VIS_ADC_GAIN     0x11
#define SI1147_PARAM_ALS_VIS_ADC_MISC     0x12
#define SI1147_PARAM_LED_REC              0x1C
#define SI1147_PARAM_ALS_IR_ADC_COUNTER   0x1D
#define SI1147_PARAM_ALS_IR_ADC_GAIN      0x1E
#define SI1147_PARAM_ALS_IR_ADC_MISC      0x1F

#define WAIT_WHILE(cond) \
  do { \
    volatile int8_t i=0; \
    while(cond) i++; \
  } while(0)

typedef union si1147_uint16
{
  uint16_t val;
  struct {
    uint8_t lo;
    uint8_t hi;
  } b;
} si1147_uint16_t;

typedef struct si1147_als_data
{
  si1147_uint16_t vis;
  si1147_uint16_t ir;
  si1147_uint16_t aux;
} si1147_als_data_t;

void si1147_init(uint16_t meas_rate, uint8_t meas_enable);
void si1147_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t si1147_read_reg(uint8_t reg_addr);

uint8_t si1147_write_command(uint8_t data);
void si1147_write_param(uint8_t param, uint8_t data);
uint8_t si1147_read_param(uint8_t param);

void si1147_als_read(si1147_als_data_t *data);
void si1147_als_force_read(si1147_als_data_t *data);

void si1147_irq_enable();
void si1147_als_irq_enable();
void si1147_irq_handler();

#endif /*SI1147_H*/
