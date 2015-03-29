#ifndef MPU9250_H
#define MPU9250_H


// I2C address 0x69 could be 0x68 depends on your wiring.
#define MPU9250_I2C_ADDRESS  0x69;

// SPI chipselect port and pin
#define MPU9250_CS_PORT GPIO_C_NUM
#define MPU9250_CS_PIN 3


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




// writes data to register reg_addr
void MPU9250_writeSensor(uint8_t reg_addr, uint8_t data) {

	//uint8_t transmit_buf[] = {reg_addr, data};
	//i2c_burst_send(TARGET_ADDRESS, transmit_buf, sizeof(transmit_buf));

	for(int i = 0; i < sizeof(transmit_buf); ++i) {
		SPI_CS_CLR(GPIO_B_NUM, );
	    SPI_WRITE(transmit_buf[i]);
	    SPI_CS_SET(GPIO_B_NUM, 4);
	}

}



/* reads from register reg_addrL and reg_addrH
*  the sensor data of the MPU9150 are 16bits so you have to read the lower and upper 8bits separately
*  to read from a specific register, write the address of the register you want to read to the slave
*  then read from slave device
*/
int16_t MPU9250_readSensor(uint8_t reg_addrL, uint8_t reg_addrH) {

	/*uint8_t transmit_bufL[] = {reg_addrL};
	uint8_t receive_bufL[1];

	// tell the cc2538 which register you want to read from
	i2c_single_send(TARGET_ADDRESS, *transmit_bufL);
	// the following read will be the value from the register you wrote to the slave about
	i2c_single_receive(TARGET_ADDRESS, receive_bufL);

	uint8_t transmit_bufH[] = {reg_addrH};
	uint8_t receive_bufH[1];

	// tell the cc2538 which register you want to read from
	i2c_single_send(TARGET_ADDRESS, *transmit_bufH);
	// the following read will be the value from the register you wrote to the slave about
	i2c_single_receive(TARGET_ADDRESS, receive_bufH);

	// concatenate the lower and upper 8bits to a signed 16 bit integer
	uint16_t L = receive_bufL[0];
	uint16_t H = receive_bufH[0];
	*/

	uint8_t transmit_bufBase[] = {reg_addrH}; // upper bits is at lower address
	uint8_t receive_bufBase[2];
	i2c_single_send(TARGET_ADDRESS, *transmit_bufBase);
	i2c_burst_receive(TARGET_ADDRESS, receive_bufBase, 2);

	uint16_t L = receive_bufBase[1];
	uint16_t H = receive_bufBase[0];

	return (int16_t)((H<<8) + L);


}









#endif /*MPU9250_H*/