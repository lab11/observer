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

static struct timer mpu9250_startup_timer;

void mpu9250_init() {
	spi_cs_init(MPU9250_CS_PORT, MPU9250_CS_PIN);
  
  // Clear sleep bit to start sensor
  mpu9250_writeSensor(MPU9250_PWR_MGMT_1, 0x80);

  timer_set(&mpu9250_startup_timer, SI1147_STARTUP_TIME);
  while (!timer_expired(&mpu9250_startup_timer));
	
  // ACCEL CONFIG
	mpu9250_writeSensor(MPU9250_ACCEL_CONFIG, 0x18);
}

uint8_t mpu9250_readByte(uint8_t reg_addr) {
	uint8_t data;

	reg_addr |= MPU9250_READ_MASK;

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
	SPI_WRITE(reg_addr);
	SPI_READ(data);
	SPI_WAITFOREOTx();
	SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);

	return data;
}

void mpu9250_writeSensor(uint8_t reg_addr, uint8_t data) {

	SPI_CS_CLR(MPU9250_CS_PORT, MPU9250_CS_PIN);
  SPI_WRITE(reg_addr & MPU9250_WRITE_MASK);
  SPI_WRITE(data);

  SPI_CS_SET(MPU9250_CS_PORT, MPU9250_CS_PIN);
}

int16_t mpu9250_readSensor(uint8_t reg_addrL, uint8_t reg_addrH) {
	uint8_t readL;
	uint8_t readH;

	readL = mpu9250_readByte(reg_addrL);
	readH = mpu9250_readByte(reg_addrH);
	
	uint16_t L = readL;
	uint16_t H = readH;
  uint16_t val = ((H<<8)+L);

  if (MPU9250_DBG)
    printf("mpu9250: read [L-0x%x H-0x%x] <- %d", reg_addrL, reg_addrH, val);
	
  return val;
}
