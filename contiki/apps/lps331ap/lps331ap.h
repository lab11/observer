#ifndef LPS331AP_H
#define LPS331AP_H

// SPI
#define LPS331AP_CS_PORT            GPIO_C_NUM
#define LPS331AP_CS_PIN             1
#define LPS331AP_READ_MASK          0x80
#define LPS331AP_INC_MASK           0x40

#define LPS331AP_IRQ_BASE           GPIO_C_BASE
#define LPS331AP_IRQ_PORT           GPIO_C_NUM
#define LPS331AP_IRQ_PIN            0
#define LPS331AP_IRQ_PIN_MASK       GPIO_PIN_MASK(LPS331AP_IRQ_PIN)

// Register Mapping
#define LPS331AP_REF_P_XL           0x08
#define LPS331AP_REF_P_L            0x09
#define LPS331AP_REF_P_H            0x0A
#define LPS331AP_WHO_AM_I           0x0F
#define LPS331AP_RES_CONF           0x10
#define LPS331AP_CTRL_REG1          0x20
#define LPS331AP_CTRL_REG2          0x21
#define LPS331AP_CTRL_REG3          0x22
#define LPS331AP_INT_CFG_REG        0x23
#define LPS331AP_INT_SOURCE_REG     0x24
#define LPS331AP_THS_P_LOW_REG      0x25
#define LPS331AP_THS_P_HIGH_REG     0x26
#define LPS331AP_STATUS_REG         0x27
#define LPS331AP_PRESS_POUT_XL_REH  0x28
#define LPS331AP_PRESS_OUT_L        0x29
#define LPS331AP_PRESS_OUT_H        0x2A
#define LPS331AP_TEMP_OUT_L         0x2B
#define LPS331AP_TEMP_OUT_H         0x2C
#define LPS331AP_AMP_CTRL           0x30

// Output data rate (odr)
#define LPS331AP_ONE_SHOT   0x0
#define LPS331AP_7_HZ       0x5
#define LPS331AP_12_5_HZ    0x6
#define LPS331AP_25_HZ      0x7

// Pressure resolution
#define LPS331AP_020_NOISE    0x7A // Not compatible with 25Hz sample rate
#define LPS331AP_025_NOISE    0x79
#define LPS331AP_25_HZ_NOISE  0x6A // Recommended precision for 25Hz


// ctrl_reg1
typedef union lps331ap_ctrl_reg1
{
  uint8_t value;
  struct {
    unsigned spi_interface: 1;
    unsigned bdu:           1;
    unsigned delta_en:      1;
    unsigned irq_en:        1;
    unsigned odr:           3;
    unsigned active:        1;
  } f;
} lps331ap_ctrl_reg1_t;

static lps331ap_ctrl_reg1_t lps331ap_ctrl_reg1_default = {.value = 0xF8};

typedef union lps331ap_ctrl_reg2
{
  uint8_t value;
  struct {
    unsigned oneshot_en:    1;
    unsigned autozero_en:   1;
    unsigned sw_reset:      1;
    unsigned RESERVED:      4;
    unsigned mem_reset:     1;
  } f;
} lps331ap_ctrl_reg2_t;

static lps331ap_ctrl_reg2_t lps331ap_ctrl_reg2_default = {.value = 0x00};


typedef union lps331ap_ctrl_reg3
{
  uint8_t value;
  struct {
    unsigned data1:         3; // data mode selections
    unsigned data2:         3; 
    unsigned pp_od:         1; // push-pull: 0, open-drain: 1
    unsigned irq_polarity:  1; // active high:0, active low: 1
  } f;
} lps331ap_ctrl_reg3_t;

static lps331ap_ctrl_reg3_t lps331ap_ctrl_reg3_default = {.value = 0x04};

int8_t lps331ap_read_byte(uint8_t addr);
void lps331ap_write_byte(uint8_t addr, uint8_t write);
void pressure_irq_handler();


#endif /*LPS331AP_H*/