#include "cpu.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/spi-arch.h"
#include "board.h"

#define ACCEL_CONNECTED 0
#define PRESS_CONNECTED	0
#define LIGHT_CONNECTED	0
#define TEMP_CONNECTED	0
#define PIR_CONNECTED	0
#define MIC_CONNECTED	0

void disable_unused_pins() {
    uint8_t pin_mask_A = 0xFF; // ADC7|ADC6|ADC5|ADC4|ADC3|BTLDRCTRL|BTLDRTX|BTLDRRX
//#if UART_CONF_ENABLE
    pin_mask_A = 0xFC;
//#endif
    GPIO_SOFTWARE_CONTROL(GPIO_A_BASE, pin_mask_A);
    GPIO_SET_OUTPUT(GPIO_A_BASE, pin_mask_A);
    GPIO_CLR_PIN(GPIO_A_BASE, pin_mask_A);



    uint8_t pin_mask_B = 0xF8; // JTAG|ADCPFET|PIR|ACCELINT|ACCELCS|RTCINT|RTCCE|SPIMISO
#if ACCEL_CONNECTED
	pin_mask_B &= 0xE7; // 0b1110 0111
#endif
#if PIR_CONNECTED
	pin_mask_B &= 0xDF; // 0b1101 1111
#endif
#if MIC_CONNECTED
	pin_mask_B &= 0xBF; // 0b1011 1111
#endif
	GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, pin_mask_B);
    GPIO_SET_OUTPUT(GPIO_B_BASE, pin_mask_B);
    GPIO_CLR_PIN(GPIO_B_BASE, pin_mask_B);



    uint8_t pin_mask_C = 0x0F; // SPIMOSI|SPISCLK|I2CSDA|I2CSCL|X|LITEINT|PRESCS|PRESINT
#if LIGHT_CONNECTED
	pin_mask_C &= 0xFB; // 0b1111 1011
#endif
#if PRESS_CONNECTED
	pin_mask_C &= 0xFC; // 0b1111 1100
#endif
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE, pin_mask_C);
    GPIO_SET_OUTPUT(GPIO_C_BASE, pin_mask_C);
    GPIO_CLR_PIN(GPIO_C_BASE, pin_mask_C);



    uint8_t pin_mask_D = 0x00; // XTAL|XTAL|LEDG|LEDB|LEDR|FRAMWP|FRAMCS|FRAMHOLD
    GPIO_SOFTWARE_CONTROL(GPIO_D_BASE, pin_mask_D);
    GPIO_SET_OUTPUT(GPIO_D_BASE, pin_mask_D);
    GPIO_CLR_PIN(GPIO_D_BASE, pin_mask_D);


// ***************** GPIO_A_NUM **********************
//#if UART_CONF_ENABLE
    //ioc_set_over(GPIO_A_NUM, 0, IOC_OVERRIDE_DIS);
    //ioc_set_over(GPIO_A_NUM, 1, IOC_OVERRIDE_DIS);
//#endif
    ioc_set_over(GPIO_A_NUM, 2, IOC_OVERRIDE_DIS);
    ioc_set_over(GPIO_A_NUM, 3, IOC_OVERRIDE_DIS);
    ioc_set_over(GPIO_A_NUM, 4, IOC_OVERRIDE_DIS);
    ioc_set_over(GPIO_A_NUM, 5, IOC_OVERRIDE_DIS);
    ioc_set_over(GPIO_A_NUM, 6, IOC_OVERRIDE_DIS);
    ioc_set_over(GPIO_A_NUM, 7, IOC_OVERRIDE_DIS);


// ***************** GPIO_B_NUM **********************
#if !ACCEL_CONNECTED
	ioc_set_over(GPIO_B_NUM, 3, IOC_OVERRIDE_DIS);
	ioc_set_over(GPIO_B_NUM, 4, IOC_OVERRIDE_DIS);
#endif    
#if !PIR_CONNECTED
	ioc_set_over(GPIO_B_NUM, 5, IOC_OVERRIDE_DIS);
#endif
#if !MIC_CONNECTED
	ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS);
#endif
	ioc_set_over(GPIO_B_NUM, 7, IOC_OVERRIDE_DIS);


// ***************** GPIO_C_NUM *********************
#if !LIGHT_CONNECTED
	ioc_set_over(GPIO_C_NUM, 2, IOC_OVERRIDE_DIS);
#endif
#if !PRESS_CONNECTED
	ioc_set_over(GPIO_C_NUM, 0, IOC_OVERRIDE_DIS);
	ioc_set_over(GPIO_C_NUM, 1, IOC_OVERRIDE_DIS);
#endif
    ioc_set_over(GPIO_C_NUM, 3, IOC_OVERRIDE_DIS);
/*
*/

}

void cleanup_before_sleep() {
    spix_disable(0);

    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(6));
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(7));
    GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SPI0_RX_PORT), GPIO_PIN_MASK(SPI0_RX_PIN));
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0x80);
    GPIO_CLR_PIN(GPIO_C_BASE, 0x80);
    GPIO_SET_OUTPUT(GPIO_C_BASE, 0x40);
    GPIO_SET_PIN(GPIO_C_BASE, 0x40);
    GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SPI0_RX_PORT), GPIO_PIN_MASK(SPI0_RX_PIN));
    GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SPI0_RX_PORT), GPIO_PIN_MASK(SPI0_RX_PIN));

    i2c_master_disable();
}

void setup_before_wake() {
    spix_enable(0);
    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(6));
    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(7));

    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI0_RX_PORT), GPIO_PIN_MASK(SPI0_RX_PIN));

    i2c_master_enable();
}
