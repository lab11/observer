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
    mpu9250_writeByte(MPU9250_PWR_MGMT_1, 0x00); //80
  
    // disable mpu9250 i2c
    //mpu9250_writeSensor(MPU9250_USER_CTRL, 0x10);

    timer_set(&mpu9250_startup_timer, MPU9250_STARTUP_TIME);
    WAIT_WHILE(!timer_expired(&mpu9250_startup_timer));

    // disable FSYNC; also sets gyro and temp filter?
    mpu9250_writeByte(MPU9250_CONFIG, 0x03);

    // disable mpu9250 i2c
    //mpu9250_writeByte(MPU9250_USER_CTRL, 0x10);

    // ACCEL CONFIG
	mpu9250_writeByte(MPU9250_ACCEL_CONFIG, 0x18);

    // ext i2c bypass
    //mpu9250_writeByte(MPU9250_INT_PIN_CFG, 0x02);

}

void ak8963_init(uint8_t mode) {
    uint8_t calibration_data[3];

    
    // Enable MPU9250 internal I2C bus
    mpu9250_writeByte(MPU9250_USER_CTRL, 0x20);

    // Configure MPU9250 I2C frequency 400KHz
    mpu9250_writeByte(MPU9250_I2C_MST_CTRL, 0x0D);

    /* To Communicate with the built-in ak8963, you must use specific 
     * "mailbox" registers to pass data to-and-from the mpu9250(master)
     * to the ak8963(slave)
     */

    // set i2c address for write
    //mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    // write starting at cntl2 register to reset ak8963
    //mpu9250_writeByte(MPU9250_I2C_SLV0_REG, AK8963_CNTL2);
    // load the data we want to write
    //mpu9250_writeByte(MPU9250_I2C_SLV0_DO, 0x01);
    // do a write from the mpu9250
    //mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x81);

    // reset the ak8963 in the mpu9250
    ak8963_writeByte(AK8963_CNTL2, 0x01);
    clock_delay_usec(50000);
    
    // Enter Fuse ROM Mode
    ak8963_writeByte(AK8963_CNTL1, 0x0F);
    clock_delay_usec(10000);

    // Read the x,y,z calibration values
    ak8963_readMultiple(AK8963_ASAX, 3, calibration_data);
   
    printf("calibration data: %d, %d, %d, %d, %d, %d", calibration_data[0], calibration_data[1], calibration_data[2]);
 
    // Set the adjustment values
    AK8963_ADJUST_X = (float)(calibration_data[0] - 128)/(256.0) + 1;
    AK8963_ADJUST_Y = (float)(calibration_data[1] - 128)/(256.0) + 1;
    AK8963_ADJUST_Z = (float)(calibration_data[2] - 128)/(256.0) + 1;
    //AK8963_ADJUST_X = calibration_data[0];
    //AK8963_ADJUST_Y = calibration_data[1];
    //AK8963_ADJUST_Z = calibration_data[2];    

    // Power down ak8963 (b/c we'll do single measurement mode for sampling)
    ak8963_writeByte(AK8963_CNTL1, 0x10); // 0x10 is for 16 bit output
    clock_delay_usec(10000);

   /*
    // set i2c address for write
    mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    // write starting at cntl1 register to change mode
    mpu9250_writeByte(MPU9250_I2C_SLV0_REG, AK8963_CNTL1);
    // load the data we want to write
    mpu9250_writeByte(MPU9250_I2C_SLV0_DO, 0x00);
    // do a write from the mpu9250
    mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x81);
    clock_delay_usec(10000); // delay 10ms

    mpu9250_writeByte(MPU9250_MAG_CNTRL1, 0x0F); // enter Fuse ROM mode
    clock_delay_usec(10000);
    mpu9250_readMultiple(MPU9250_MAG_ASAX, 3, calibration_data);
    
    MAG_ASAX = (float)(calibration_data[0] - 128)/(256.0) + 1;
    MAG_ASAY = (float)(calibration_data[1] - 128)/(256.0) + 1;
    MAG_ASAZ = (float)(calibration_data[2] - 128)/(256.0) + 1;
    
    mpu9250_writeByte(MPU9250_MAG_CNTRL1, 0x00); // power down mag

    clock_delay_usec(10000);

    mpu9250_writeByte(MPU9250_MAG_CNTRL1, 0x10 | mode);
    clock_delay_usec(10000);
*/
}

uint8_t ak8963_readWIA() {
    uint8_t response = 23;    

     /*** read device id ***/
    // set slave0 address for a read
    //mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    //uint8_t addr;
    //mpu9250_readByte(MPU9250_I2C_SLV0_ADDR, &addr);
    //printf("addr: %d\n", addr);
 
    // actual register address to start reading from
    //mpu9250_writeByte(MPU9250_I2C_SLV0_REG, AK8963_WIA);
    //uint8_t reg;
    //mpu9250_readByte(MPU9250_I2C_SLV0_REG, &reg);
    //printf("reg: %d\n", reg);

    // then signal an I2C read to the mpu9250 to read 1 byte
    //mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x81);
    //clock_delay_usec(50000);
    //clock_delay_usec(50000);
    //uint8_t signal;
    //mpu9250_readByte(MPU9250_I2C_SLV0_CTRL, &signal);    
    //printf("signal: %d\n", signal);

    // then read the result from the mpu9250 "mailbox" reg
    //mpu9250_readByte(MPU9250_EXT_SENS_DATA_00, &response);

    ak8963_readByte(AK8963_WIA, &response);

    printf("id: %d\n", response);
    return response;

}

void ak8963_readByte(uint8_t reg_addr, uint8_t *data) {
    // set i2c address for read
    mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR|0x80);
    // read starting at reg_addr register address
    mpu9250_writeByte(MPU9250_I2C_SLV0_REG, reg_addr);
    // send the read command from mpu9250
    mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x81);
    clock_delay_usec(50000); // delay 10ms

    // then read result from the "mailbox" reg
    mpu9250_readByte(MPU9250_EXT_SENS_DATA_00, data);

    return;
}


// len must be <= 24 as there are only  mpu9250 "mailbox" registers
// to return respones from the ak8963
void ak8963_readMultiple(uint8_t reg_addr, uint8_t len, uint8_t *data) {
    // set i2c address for read
    mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR|0x80);
    // read starting at reg_addr register address
    mpu9250_writeByte(MPU9250_I2C_SLV0_REG, reg_addr);
    // send the read command from mpu9250
    mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x80 | len);
    clock_delay_usec(50000); // delay 10ms

    // then read result from the "mailbox" reg
    mpu9250_readMultiple(MPU9250_EXT_SENS_DATA_00, len, data);

}

uint8_t mpu9250_readWAI() {
	uint8_t data;
	mpu9250_readByte(MPU9250_WHO_AM_I, &data);

	return data;
}

void mpu9250_readByte(uint8_t reg_addr, uint8_t *data) {

	reg_addr |= MPU9250_READ_MASK;

	spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
    //spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, 0, SSI_CR0_SPH, 8);

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addr);
  	SPI_FLUSH();
	SPI_READ(*data);
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

	return;
}

void mpu9250_readMultiple(uint8_t reg_addr, uint16_t len, uint8_t *data) {
    uint16_t i;
    
    reg_addr |= MPU9250_READ_MASK;

    spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);

    SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
    SPI_WRITE(reg_addr);

    SPI_FLUSH();
    for(i = 0; i < len; ++i) {
        SPI_READ(data[i]);
    }

    SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);
}

void ak8963_writeByte(uint8_t reg_addr, uint8_t data) {
    
     // set i2c address for write
    mpu9250_writeByte(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    // write starting at reg_addr register address
    mpu9250_writeByte(MPU9250_I2C_SLV0_REG, reg_addr);
    // load the data we want to write
    mpu9250_writeByte(MPU9250_I2C_SLV0_DO, data);
    // do a write of 1byte from the mpu9250
    mpu9250_writeByte(MPU9250_I2C_SLV0_CTRL, 0x81);
    clock_delay_usec(10000); // delay 10ms


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

int8_t ak8963_read_Mag(uint8_t *data) {
    uint8_t response[7];
    uint8_t dataReady;
    uint8_t dataReady_tries = 0;
    uint8_t status2;
    uint8_t i;

    // Currently in power-down mode "0000", must change to Single Measurement
    // Mode to take a sample
    ak8963_writeByte(AK8963_CNTL1, 0x11); // 0b0001 0001 for 16 bit single meas
    clock_delay_usec(10000); // 9ms max required for single measure
    
    // Now check to see if data is ready
    do {
        ak8963_readByte(AK8963_ST1, &dataReady);
        dataReady_tries++;
    } while( !(dataReady & 0x01) && dataReady_tries < 10);

    if (dataReady_tries >= 10) {
        return -1; // failed to read in timely manner
    }

    // Now read all 3 axis (6 regs) plus the status2 reg
    ak8963_readMultiple(AK8963_HXL, 7, response);
    printf("responses: %x, %x, %x, %x, %x, %x\n", response[0], response[1], response[2], response[3], response[4], response[5]);
    
    // check to see that data 
    if (!(status2 & 0x08)) {
        for(i=0; i < 6; i++) {
            *(data+i) = *(response+i);
        }
        return 0; // data is good
    } else {
        return -2; // data is bad, overflow   
    }
}

/*int16_t mpu9250_readMagX() {
    uint8_t dataReady_tries = 0;
    uint8_t dataReady;
    
    uint8_t status2;
    uint8_t magX[2];

    do { 
        mpu9250_readByte(MPU9250_MAG_STATUS1, &dataReady);
        dataReady_tries++;
    } while( ((dataReady & 0x01) != 0x01) && dataReady_tries < 256);

    mpu9250_readMultiple(MPU9250_MAG_XOUT_L, 2, magX); // read meas data
    mpu9250_readByte(MPU9250_MAG_STATUS2, &status2); // read to end
    
    if (!(status2 & 0x08)) {
        return (int16_t)(((uint16_t)magX[1] << 8) | magX[0]);
    } else {
        return 0x0000;
    }

}
*/

// when INT pin is latched and ANYRD_2CLEAR is 0, must read
// INT_STATUS reg to clear pin
void mpu9250_int_clear() {
	uint8_t data;
	mpu9250_readByte(MPU9250_INT_STATUS, &data);
}

void mpu9250_motion_interrupt_init(uint8_t WOM_Threshold, uint8_t Wakeup_Frequency) {
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

	uint8_t INT_PIN_CFG_reg;
	mpu9250_readByte(MPU9250_INT_PIN_CFG, &INT_PIN_CFG_reg);
	INT_PIN_CFG_reg |= 0x20; // interrupt latch enable
	mpu9250_writeSensor(MPU9250_INT_PIN_CFG, INT_PIN_CFG_reg);

	//timer_set(&accel_int_timer, 0.1*CLOCK_SECOND);

	// Enable GPIO Pin as Interrupt
//	 GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
//     GPIO_SET_INPUT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_DETECT_EDGE(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_TRIGGER_SINGLE_EDGE(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
    // GPIO_DETECT_RISING(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
  	// GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
//  	GPIO_POWER_UP_ON_RISING(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
//	GPIO_ENABLE_POWER_UP_INTERRUPT(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
//  	ioc_set_over(MPU9250_INT_PORT, MPU9250_INT_PIN, IOC_OVERRIDE_DIS);
//  	nvic_interrupt_enable(MPU9250_INT_VECTOR);
  	//gpio_register_callback(temp_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);
//  	gpio_register_callback(accel_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);

	return;

}

void mpu9250_interrupt_enable(gpio_callback_t accel_irq_handler){
	GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
     GPIO_SET_INPUT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
	GPIO_POWER_UP_ON_RISING(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
	GPIO_ENABLE_POWER_UP_INTERRUPT(MPU9250_INT_PORT, GPIO_PIN_MASK(MPU9250_INT_PIN));
  	ioc_set_over(MPU9250_INT_PORT, MPU9250_INT_PIN, IOC_OVERRIDE_DIS);
  	nvic_interrupt_enable(MPU9250_INT_VECTOR);
  	//gpio_register_callback(temp_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);
  	gpio_register_callback(accel_irq_handler, MPU9250_INT_PORT, MPU9250_INT_PIN);
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
