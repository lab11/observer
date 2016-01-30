#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/spi-arch.h"
#include "core/dev/spi.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include <stdio.h>
#include "usb-serial.h"
#include "mpu9250.h"
#include "amn41122.h"
#include "cpu.h"

static struct timer mpu9250_startup_timer;
static struct timer accel_int_timer;
//extern const struct process observer_main_process;
extern void setup_before_resume();

void mpu9250_init() {
	spix_cs_init(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

    // Clear sleep bit to start sensor
    mpu9250_writeByte(MPU9250_PWR_MGMT_1, 0x80); //80
  
    // disable mpu9250 i2c
    //mpu9250_writeSensor(MPU9250_USER_CTRL, 0x10);

    timer_set(&mpu9250_startup_timer, MPU9250_STARTUP_TIME);
    WAIT_WHILE(!timer_expired(&mpu9250_startup_timer));

    // disable mpu9250 i2c
    mpu9250_writeByte(MPU9250_USER_CTRL, 0x10);

    // ACCEL CONFIG
	mpu9250_writeByte(MPU9250_ACCEL_CONFIG, 0x18);

}

void mpu9250_readByte(uint8_t reg_addr, uint8_t *data) {

	reg_addr |= MPU9250_READ_MASK;

	spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addr);
  	SPI_FLUSH();
	SPI_READ(*data);
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

	return;
}

void mpu9250_writeByte(uint8_t reg_addr, uint8_t data) {
	reg_addr &= MPU9250_WRITE_MASK;

	spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addr);
  	SPI_FLUSH();
	SPI_WRITE(data);
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);	
}

void mpu9250_writeSensor(uint8_t reg_addr, uint8_t data) {

	spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
  SPI_WRITE(reg_addr & MPU9250_WRITE_MASK);
  SPI_WRITE(data);
  SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);
}

int16_t mpu9250_readSensor(uint8_t reg_addrL, uint8_t reg_addrH) {
	uint8_t readL;
	uint8_t readH;

	mpu9250_readByte(reg_addrL, &readL);
	mpu9250_readByte(reg_addrH, &readH);
	
	uint16_t L = readL;
	uint16_t H = readH;
  int16_t val = ((H<<8)+L);

  if (MPU9250_DBG)
    printf("mpu9250:  read [L-0x%x H-0x%x] <- %d\n", reg_addrL, reg_addrH, val);
	
  return val;
}


void mpu9250_motion_interrupt_init(uint8_t WOM_Threshold, uint8_t Wakeup_Frequency, gpio_callback_t accel_irq_handler) {
	// Ensure Accel is running
	uint8_t PWR_MGMT_2_reg;
    mpu9250_readByte(MPU9250_PWR_MGMT_2, &PWR_MGMT_2_reg); // 0xEC
	PWR_MGMT_2_reg &= MPU9250_ACCEL_ENABLE_MASK; // 0xC7
	PWR_MGMT_2_reg |= MPU9250_GYRO_DISABLE_MASK; // 0x07
	mpu9250_writeByte(MPU9250_PWR_MGMT_2, PWR_MGMT_2_reg); // 0x6C

	// Set Accel LPF setting to 184Hz Bandwidth
	uint8_t ACCEL_CONFIG2_reg;
    mpu9250_readByte(MPU9250_ACCEL_CONFIG2, &ACCEL_CONFIG2_reg); //0x9D
	ACCEL_CONFIG2_reg &= 0xF9; // 0b11111001
	ACCEL_CONFIG2_reg |= 0x01; // 0b00000001
	ACCEL_CONFIG2_reg |= MPU9250_ACCEL2_DLPF_ENABLE_MASK; // 0x08
	mpu9250_writeByte(MPU9250_ACCEL_CONFIG2, ACCEL_CONFIG2_reg); // 0x1D

	// Enable Motion Interrupt
	uint8_t INT_ENABLE_reg;
    mpu9250_readByte(MPU9250_INT_ENABLE, &INT_ENABLE_reg); // 0xB8
	INT_ENABLE_reg &= 0xA6; // 0b10100110
	// INT_ENABLE_reg &= 0x80;
	INT_ENABLE_reg |= 0x40;
	mpu9250_writeByte(MPU9250_INT_ENABLE, INT_ENABLE_reg); // 0x38

	// Enable Accel Hardware Intelligence
	uint8_t MOT_DETECT_CTRL_reg;
    mpu9250_readByte(MPU9250_MOT_DETECT_CTRL, &MOT_DETECT_CTRL_reg); // 0xE9
	MOT_DETECT_CTRL_reg &= 0x3F; // 0b 0011 1111
	MOT_DETECT_CTRL_reg |= 0xC0; // 0b 1100 0000
	mpu9250_writeByte(MPU9250_MOT_DETECT_CTRL, MOT_DETECT_CTRL_reg); // 0x69

	// Set Motion Threshold
	mpu9250_writeByte(MPU9250_WOM_THR, WOM_Threshold); // 0x1F

	// Set Frequency of Wake-Up
	if (Wakeup_Frequency >= (uint8_t)12) {
		return;
	}
	uint8_t LP_ACCEL_ODR_reg;
    mpu9250_readByte(MPU9250_LP_ACCEL_ODR, &LP_ACCEL_ODR_reg); // 0x9E
	LP_ACCEL_ODR_reg |= Wakeup_Frequency; // should only set the bottom 4 bits
	mpu9250_writeByte(MPU9250_LP_ACCEL_ODR, LP_ACCEL_ODR_reg); // 0x1E

	// Enable Cycle Mode (Accel Low Power Mode)
	uint8_t PWR_MGMT_1_reg;
    mpu9250_readByte(MPU9250_PWR_MGMT_1, &PWR_MGMT_1_reg); // 0xEB
	PWR_MGMT_1_reg |= 0x20;
	mpu9250_writeByte(MPU9250_PWR_MGMT_1, PWR_MGMT_1_reg); // 0x6B

	//uint8_t INT_PIN_CFG_reg = mpu9250_readByte(MPU9250_INT_PIN_CFG);
	//INT_PIN_CFG_reg |= 0x20; // interrupt latch enable
	//mpu9250_writeSensor(MPU9250_INT_PIN_CFG, INT_PIN_CFG_reg);

	//timer_set(&accel_int_timer, 0.1*CLOCK_SECOND);

	// Enable GPIO Pin as Interrupt
	// GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
    // GPIO_SET_INPUT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_DETECT_EDGE(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_TRIGGER_SINGLE_EDGE(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
    // GPIO_DETECT_RISING(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	GPIO_POWER_UP_ON_RISING(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
	GPIO_ENABLE_POWER_UP_INTERRUPT(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
  	ioc_set_over(MPU9250_INT_PORT, MPU9250_INT_PIN, IOC_OVERRIDE_DIS);
  	nvic_interrupt_enable(MPU9250_INT_VECTOR);
  	//gpio_register_callback(temp_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);
  	gpio_register_callback(accel_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);



	return;

}

void temp_irq_handler(uint8_t port, uint8_t pin) {
	INTERRUPTS_DISABLE();
	//setup_before_resume();
	printf("HALLO");
	//mpu9250_readByte(MPU9250_INT_STATUS);
	INTERRUPTS_ENABLE();
}

/*
void accel_irq_handler(uint8_t port, uint8_t pin) {

 	if (timer_expired(&accel_int_timer)) {
 		//asm("CPSID i");
 		INTERRUPTS_DISABLE();
 		//GPIO_DISABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
		//GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
 		timer_restart(&accel_int_timer);
 		//GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
		printf("START ACCEL ISR\n");
		

		// clear interrupt status by reading it
		// have to clear because interrupt is latched
		//mpu9250_readByte(MPU9250_INT_STATUS);

		//process_poll(&observer_main_process);
		mpu9250_readByte(MPU9250_INT_STATUS);
		//GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));

		GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
		GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
        //GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), 0xFF);

		nvic_interrupt_unpend(NVIC_INT_GPIO_PORT_B);
		//mpu9250_readByte(MPU9250_INT_STATUS);
		process_poll(&observer_main_process);
		//GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));

		//timer_reset(&accel_int_timer);
	} else {
		//asm("CPSID i");
		INTERRUPTS_DISABLE();
		//GPIO_DISABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
		//GPIO_DISABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
		printf("ACCEL INT WAIT\n");
		mpu9250_readByte(MPU9250_INT_STATUS);

		GPIO_CLEAR_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
		GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
		//GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), 0xFF);

		//GPIO_CLEAR_INTERRUPT(MPU9250_INT_PORT, MPU9250_INT_PIN);
		nvic_interrupt_unpend(NVIC_INT_GPIO_PORT_B);
		//mpu9250_readByte(MPU9250_INT_STATUS);

		//GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
		//GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
		//asm("CPSIE i");
		INTERRUPTS_ENABLE();
	}

}*/
