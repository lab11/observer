#ifndef CHECKSUM_H
#define CHECKSUM_H
#include <stdio.h>
#include <stdint.h>

uint16_t checksum(uint8_t accelx1, uint8_t accelx2, uint8_t accely1, uint8_t accely2, 
		uint8_t accelz1, uint8_t accelz2, uint8_t humd1, uint8_t humd2, 
		uint8_t light1, uint8_t light2, uint8_t mic1, uint8_t mic2,
		uint8_t press1, uint8_t press2, uint8_t press3, uint8_t temp1, uint8_t temp2, uint8_t pir){
	
	uint32_t sum;
	uint16_t accelx = accelx1 << 8 | accelx2;
	uint16_t accely = accely1 << 8 | accely2;
	uint16_t accelz = accelz1 << 8 | accelz2;
	uint16_t humd = humd1 << 8 | humd2;
	uint16_t light = light1 << 8 | light2;
	uint16_t mic = mic1 << 8 | mic2;
	uint16_t press = press1 << 8 | press2;
	uint16_t press3_temp1 = press3 << 8 | temp1;
	uint16_t temp2_pir = temp2 << 8 | pir;

	sum = accelx + accely + accelz + humd + light + mic + press + press3_temp1 + temp2_pir;

	uint16_t lower = sum & 0xFFFF;
	uint16_t  carry = (sum & 0xFFFF0000) >> 16;

	uint32_t result = lower + carry;

	lower = result & 0xFFFF;
	carry = (result & 0xFFFF0000) >> 16;

	return ~(lower + carry);
}

int validation(uint8_t accelx1, uint8_t accelx2, uint8_t accely1, uint8_t accely2, 
		uint8_t accelz1, uint8_t accelz2, uint8_t humd1, uint8_t humd2, 
		uint8_t light1, uint8_t light2, uint8_t mic1, uint8_t mic2,
		uint8_t press1, uint8_t press2, uint8_t press3, uint8_t temp1, uint8_t temp2, uint8_t pir,
		uint8_t checksum1, uint8_t checksum2){
		
	uint32_t sum;
	uint16_t accelx = accelx1 << 8 | accelx2;
	uint16_t accely = accely1 << 8 | accely2;
	uint16_t accelz = accelz1 << 8 | accelz2;
	uint16_t humd = humd1 << 8 | humd2;
	uint16_t light = light1 << 8 | light2;
	uint16_t mic = mic1 << 8 | mic2;
	uint16_t press = press1 << 8 | press2;
	uint16_t press3_temp1 = press3 << 8 | temp1;
	uint16_t temp2_pir = temp2 << 8 | pir;
	uint16_t checksum = checksum1 << 8 | checksum2;

	sum = accelx + accely + accelz + humd + light + mic + press + press3_temp1 + temp2_pir + checksum;

	uint16_t lower = sum & 0xFFFF;
	uint16_t carry = (sum & 0xFFFF0000) >> 16;

	uint16_t error = ~(lower + carry);
	
	if (error == 0) {
		return 1;
	}
	else {
		return 0;
	}
	

}

#endif
