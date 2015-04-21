#ifndef MPU9250_H
#define MPU9250_H


// I2C address 0x69 could be 0x68 depends on your wiring.
#define MPU9250_I2C_ADDRESS  0x69;

// SPI chipselect port and pin
#define MPU9250_CS_PORT           GPIO_B_NUM
#define MPU9250_CS_PIN			  3

#define MPU9250_READ_MASK		   0x80
#define MPU9250_WRITE_MASK		   0x7F

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9250 Register Map and Descriptions Revision 1.4",

#define MPU9250_SELF_TEST_X_GYRO   0x00   // R/W
#define MPU9250_SELF_TEST_Y_GYRO   0x01   // R/W
#define MPU9250_SELF_TEST_Z_GYRO   0x02   // R/W
#define MPU9250_SELF_TEST_X_ACCEL  0x0D   // R/W
#define MPU9250_SELF_TEST_Y_ACCEL  0x0E   // R/W
#define MPU9250_SELF_TEST_Z_ACCEL  0x0F   // R/W

#define MPU9250_SMPLRT_DIV         0x19   // R/W
#define MPU9250_CONFIG             0x1A   // R/W
#define MPU9250_GYRO_CONFIG        0x1B   // R/W
#define MPU9250_ACCEL_CONFIG       0x1C   // R/W
#define MPU9250_ACCEL_CONFIG2      0x1D	  // R/W
#define MPU9250_LP_ACCEL_ODR       0x1E   // R/W
#define MPU9250_WOM_THR            0x1F   // R/W

#define MPU9250_FIFO_EN            0x23   // R/W
#define MPU9250_I2C_MST_CTRL       0x24   // R/W
#define MPU9250_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9250_I2C_SLV0_REG       0x26   // R/W
#define MPU9250_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9250_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9250_I2C_SLV1_REG       0x29   // R/W
#define MPU9250_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9250_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9250_I2C_SLV2_REG       0x2C   // R/W
#define MPU9250_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9250_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9250_I2C_SLV3_REG       0x2F   // R/W
#define MPU9250_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9250_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9250_I2C_SLV4_REG       0x32   // R/W
#define MPU9250_I2C_SLV4_DO        0x33   // R/W
#define MPU9250_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9250_I2C_SLV4_DI        0x35   // R  
#define MPU9250_I2C_MST_STATUS     0x36   // R
#define MPU9250_INT_PIN_CFG        0x37   // R/W
#define MPU9250_INT_ENABLE         0x38   // R/W
#define MPU9250_INT_STATUS         0x3A   // R  
#define MPU9250_ACCEL_XOUT_H       0x3B   // R  
#define MPU9250_ACCEL_XOUT_L       0x3C   // R  
#define MPU9250_ACCEL_YOUT_H       0x3D   // R  
#define MPU9250_ACCEL_YOUT_L       0x3E   // R  
#define MPU9250_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9250_ACCEL_ZOUT_L       0x40   // R  
#define MPU9250_TEMP_OUT_H         0x41   // R  
#define MPU9250_TEMP_OUT_L         0x42   // R  
#define MPU9250_GYRO_XOUT_H        0x43   // R  
#define MPU9250_GYRO_XOUT_L        0x44   // R  
#define MPU9250_GYRO_YOUT_H        0x45   // R  
#define MPU9250_GYRO_YOUT_L        0x46   // R  
#define MPU9250_GYRO_ZOUT_H        0x47   // R  
#define MPU9250_GYRO_ZOUT_L        0x48   // R  
#define MPU9250_EXT_SENS_DATA_00   0x49   // R  
#define MPU9250_EXT_SENS_DATA_01   0x4A   // R  
#define MPU9250_EXT_SENS_DATA_02   0x4B   // R  
#define MPU9250_EXT_SENS_DATA_03   0x4C   // R  
#define MPU9250_EXT_SENS_DATA_04   0x4D   // R  
#define MPU9250_EXT_SENS_DATA_05   0x4E   // R  
#define MPU9250_EXT_SENS_DATA_06   0x4F   // R  
#define MPU9250_EXT_SENS_DATA_07   0x50   // R  
#define MPU9250_EXT_SENS_DATA_08   0x51   // R  
#define MPU9250_EXT_SENS_DATA_09   0x52   // R  
#define MPU9250_EXT_SENS_DATA_10   0x53   // R  
#define MPU9250_EXT_SENS_DATA_11   0x54   // R  
#define MPU9250_EXT_SENS_DATA_12   0x55   // R  
#define MPU9250_EXT_SENS_DATA_13   0x56   // R  
#define MPU9250_EXT_SENS_DATA_14   0x57   // R  
#define MPU9250_EXT_SENS_DATA_15   0x58   // R  
#define MPU9250_EXT_SENS_DATA_16   0x59   // R  
#define MPU9250_EXT_SENS_DATA_17   0x5A   // R  
#define MPU9250_EXT_SENS_DATA_18   0x5B   // R  
#define MPU9250_EXT_SENS_DATA_19   0x5C   // R  
#define MPU9250_EXT_SENS_DATA_20   0x5D   // R  
#define MPU9250_EXT_SENS_DATA_21   0x5E   // R  
#define MPU9250_EXT_SENS_DATA_22   0x5F   // R  
#define MPU9250_EXT_SENS_DATA_23   0x60   // R  

#define MPU9250_I2C_SLV0_DO        0x63   // R/W
#define MPU9250_I2C_SLV1_DO        0x64   // R/W
#define MPU9250_I2C_SLV2_DO        0x65   // R/W
#define MPU9250_I2C_SLV3_DO        0x66   // R/W
#define MPU9250_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9250_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9250_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9250_USER_CTRL          0x6A   // R/W
#define MPU9250_PWR_MGMT_1         0x6B   // R/W
#define MPU9250_PWR_MGMT_2         0x6C   // R/W
#define MPU9250_FIFO_COUNTH        0x72   // R/W
#define MPU9250_FIFO_COUNTL        0x73   // R/W
#define MPU9250_FIFO_R_W           0x74   // R/W
#define MPU9250_WHO_AM_I           0x75   // R
#define MPU9250_XA_OFFSET_H        0x77   // R/W
#define MPU9250_XA_OFFSET_L        0x78   // R/W
#define MPU9250_YA_OFFSET_H        0x7A   // R/W
#define MPU9250_YA_OFFSET_L        0x7B   // R/W
#define MPU9250_ZA_OFFSET_H        0x7D   // R/W
#define MPU9250_ZA_OFFSET_L        0x7E   // R/W

//MPU9250 Compass ? copied from MPU9150 so can't guarantee this to work
#define MPU9250_CMPS_XOUT_L        0x4A   // R
#define MPU9250_CMPS_XOUT_H        0x4B   // R
#define MPU9250_CMPS_YOUT_L        0x4C   // R
#define MPU9250_CMPS_YOUT_H        0x4D   // R
#define MPU9250_CMPS_ZOUT_L        0x4E   // R
#define MPU9250_CMPS_ZOUT_H        0x4F   // R

/*void my_init() {
	// Enable clock for SSI0 peripheral
	// D0es same thing as SPI_ENABLE
	REG(SYS_CTRL_RCGCSSI) |= 1;

	// Disable SSI0 function before configuring module
    REG(SSI1_BASE + SSI_CR1) = 0;

    // Set IO clock as SSI clock source
    REG(SSI1_BASE + SSI_CC) = 1;


    // INPUT: Mux port CC2538_SPI_CLK_PORT_NUM / pin CC2538_SPI_CLK_PIN_NUM to SSI0 CLK
    REG(IOC_CLK_SSIIN_SSI1) = (SPI1_CLK_PORT * 8) + SPI1_CLK_PIN;

    // INPUT: Mux port CC2538_SPI_SEL_PORT_NUM / pin CC2538_SPI_SEL_PIN_NUM to SSI0 CS
    REG(IOC_SSIFSSIN_SSI1) = (GPIO_A_NUM * 8) + 6;

    // OUTPUT: Mux MISO output to the port CC2538_SPI_MISO_PORT_NUM / pin CC2538_SPI_MISO_PIN_NUM
    ioc_set_sel(SPI1_RX_PORT, SPI1_RX_PIN, IOC_PXX_SEL_SSI1_TXD);

    // INPUT: Mux port CC2538_SPI_MOSI_PORT_NUM / pin CC2538_SPI_MOSI_PIN_NUM to SSI0 MOSI
    REG(IOC_SSIRXD_SSI0) = (SPI1_TX_PORT * 8) + SPI1_TX_PIN;


    // Put all the SSI gpios into peripheral mode

    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI1_CLK_PORT), GPIO_PIN_MASK(SPI1_CLK_PIN));

    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI1_TX_PORT), GPIO_PIN_MASK(SPI1_TX_PIN));

    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(SPI1_RX_PORT), GPIO_PIN_MASK(SPI1_RX_PIN));

    GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_A_NUM), GPIO_PIN_MASK(6));


	// Disable any control
    ioc_set_over(SPI1_CLK_PORT, SPI1_CLK_PIN, IOC_OVERRIDE_DIS);

    ioc_set_over(SPI1_TX_PORT, SPI1_TX_PIN, IOC_OVERRIDE_DIS);

    ioc_set_over(SPI1_RX_PORT, SPI1_RX_PIN, IOC_OVERRIDE_DIS);

    ioc_set_over(GPIO_A_NUM, 6, IOC_OVERRIDE_DIS);



    // Specify division factor [CPSDVSR] which is used to derive the SSIClk from the system clock
    // CPSDVSR: 8 bits, 0 ... 254 (even number)
    // SSIClk = SysClk / (CPSDVSR x (1 + SCR))
    // Where:
    //          SysClk - system clock
    //          SCR - value, programmed in SSICR0 register
    REG(SSI1_BASE + SSI_CPSR) = 2;

    // Put the SSI1 in Motorola_2 SPI mode (Polarity=1, Phase=0) with 8 bit data
    // SCR = 0, so Transmit/Receive Baud rates = SysClk/(2*(1+0)) = 32MHz/2 = 16MHz
    REG(SSI1_BASE + SSI_CR0) = SSI_CR0_SPO_M | 0x07;

    // Device is "SLAVE"
    REG(SSI1_BASE + SSI_CR1) |= SSI_CR1_MS;

    // Enable Receive Time-Out interrupt
    REG(SSI1_BASE + SSI_IM) = SSI_IM_RTIM;

	// Clear SSI1 interrupt flags
    REG(SSI1_BASE + SSI_ICR) = SSI_ICR_RORIC | SSI_ICR_RTIC;

    // Enable SSI1 Interrupts
    //nvic_interrupt_enable(NVIC_INT_SSI1);

    // Enable the SSI1
    REG(SSI1_BASE + SSI_CR1) |= SSI_CR1_SSE;
    // SPI initialization finish

}
*/

uint8_t MPU9250_readByte(uint8_t reg_addr) {
	uint8_t data;

	reg_addr |= MPU9250_READ_MASK;

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addr);
	SPI_READ(data);
	SPI_WAITFOREOTx();
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

	return data;
}

// writes data to register reg_addr
void MPU9250_writeSensor(uint8_t reg_addr, uint8_t data) {

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
    SPI_WRITE(reg_addr & MPU9250_WRITE_MASK);
    //SPI_WRITE(data);
    //SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

    //SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
    SPI_WRITE(data);

	//SPI_WAITFOREOTx();
    SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

    //SPIX_FLUSH(0);
    
    /*SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);

    SPIX_WAITFORTxREADY(0);
    SPIX_BUF(0) = reg_addr;
	SPIX_WAITFOREOTx(0);

	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);
	*/

}

void read_slave(uint8_t *data) {
	SPI_RXBUF = 0; \
    SPI_WAITFOREORx(); \
    *data = SPI_RXBUF; \
}



/* reads from register reg_addrL and reg_addrH
*  the sensor data of the MPU9150 are 16bits so you have to read the lower and upper 8bits separately
*  to read from a specific register, write the address of the register you want to read to the slave
*  then read from slave device
*/
int16_t MPU9250_readSensor(uint8_t reg_addrL, uint8_t reg_addrH) {

	uint8_t readL;
	uint8_t readH;

	readL = MPU9250_readByte(reg_addrL);
	readH = MPU9250_readByte(reg_addrH);

	/*SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addrL | MPU9250_READ_MASK);
	SPI_READ(readL);
	SPI_WAITFOREOTx(); // Extra wait loop before asserting CS
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addrH | MPU9250_READ_MASK);
	SPI_READ(readH);
	SPI_WAITFOREOTx(); // Extra wait loop before asserting CS
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);
	*/
	
	uint16_t L = readL;
	uint16_t H = readH;

	//printf("lowerAddr: %02x\n", reg_addrL);
	//printf("hihgerAddr: %02x\n", reg_addrH);

	printf("lowerByte: %02x\n", readL);
	printf("higherByte: %02x\n", readH);
	printf("combinedByte: %04x\n", ((H<<8)+L) );

	return (int16_t)((H<<8) + L);

}


void setup_motion_interrupt() {
	leds_on(LEDS_RED);
	uint8_t PWR_MGMT_2_reg = MPU9250_readByte(MPU9250_PWR_MGMT_2);
	
	MPU9250_writeSensor(MPU9250_PWR_MGMT_2, PWR_MGMT_2_reg);
	leds_off(LEDS_RED);
}



#endif /*MPU9250_H*/