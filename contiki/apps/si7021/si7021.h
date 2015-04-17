#ifndef SI7021_H
#define SI7021_H

#include "contiki.h"

#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"

//static struct etimer periodic_timer;

#define Si7021_debug						1

#define Si7021_SLAVE_ADDRESS 				0x40

#define Si7021_STARTUP_TIME					0.08*CLOCK_SECOND

// These are the measurement commands
#define Si7021_TEMP_MEASURE_HOLD 			0xE3
#define Si7021_HUMD_MEASURE_HOLD 			0xE5
#define Si7021_TEMP_MEASURE_NOHOLD 			0xF3
#define Si7021_HUMD_MEASURE_NOHOLD 			0xF5
#define Si7021_READ_TEMP_FROM_PREV_HUMD 	0xE0
#define Si7021_RESET						0xFE
#define Si7021_WRITE_USER_REG				0xE6
#define Si7021_READ_USER_REG				0xE7
#define Si7021_READ_ELEC_ID_FIRST_BYTE1		0xFA
#define Si7021_READ_ELEC_ID_FIRST_BYTE2 	0x0F
#define Si7021_READ_ELEC_ID_LAST_BYTE1 		0xFC
#define Si7021_READ_ELEC_ID_LAST_BYTE2 		0xC9
#define Si7021_READ_FIRMWARE_REVISION1 		0x84
#define Si7021_READ_FIRMWARE_REVISION2 		0xB8

typedef enum {TEMP_HOLD, TEMP_NOHOLD, TEMP_READ_FROM_RH} TEMP_READ_t;
typedef enum {RH_HOLD, RH_NOHOLD} HUMD_READ_t;


// These are for the user registers
#define Si7021_RESOLUTION_R12_T14			0x00	// R/W
#define Si7021_RESOLUTION_R08_T12			0x01	// R/W
#define Si7021_RESOLUTION_R10_T13			0x80	// R/W
#define Si7021_RESOLUTION_R11_T11			0x81	// R/W
#define Si7021_VDD_LOW						0x40	// R
#define Si7021_VDD_OK						0x00	// R
#define Si7021_HEATER_ENABLE				0x04 	// R/W
#define Si7021_HEATER_DISABLE 				0x00 	// R/W

uint16_t Si7021_readTemp(TEMP_READ_t read_type);
uint16_t Si7021_readHumd(HUMD_READ_t read_type);
void Si7021_write_userreg(uint8_t data);
int Si7021_read_userreg();
double Si7021_read_electronicID() ;

#endif /*SI7021_H*/