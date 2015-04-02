#include "contiki.h"

#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si7021.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;


// writes data to register reg_addr on the cc2538
void Si7021_writeSensor(uint8_t reg_addr, uint8_t data) {

	uint8_t transmit_bufL[] = {reg_addr, data};
	i2c_burst_send(Si7021_SLAVE_ADDRESS , transmit_bufL, sizeof(transmit_bufL));

}

// reads from register reg_addrL and reg_addrH on the cc2538
// the sensor data of the Si7021 are 16bits so you have to read the lower and upper 8bits separately
// to read from a specific register, write the address of the register you want to read to the slave
// then read from slave device
int Si7021_readSensor(uint8_t reg_addr) {


	uint8_t transmit_bufBase[] = {reg_addr};
	uint8_t receive_bufBase[2];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 2);

	uint16_t L = receive_bufBase[0];
	uint16_t H = receive_bufBase[1];

	return (int16_t)((H<<8) + L);
}

int Si7021_read_userreg(uint8_t reg_addr) {

	uint8_t transmit_bufBase[] = {reg_addr};
	uint8_t receive_bufBase[1];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	i2c_single_receive(Si7021_SLAVE_ADDRESS, receive_bufBase);

	return *receive_bufBase;
}

void Si7021_write_userreg(uint8_t reg_addr, uint8_t data){
	uint8_t transmit_bufBase[] = {reg_addr, data};
	i2c_burst_send(Si7021_SLAVE_ADDRESS, transmit_bufBase, sizeof(transmit_bufBase));
}


/*---------------------------------------------------------------------------*/
PROCESS(si7021_process, "Si7021");
AUTOSTART_PROCESSES(&si7021_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(si7021_process, ev, data) {

	PROCESS_BEGIN();

	etimer_set(&periodic_timer_red, CLOCK_SECOND);
	etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

	// Initialize i2c
	i2c_init(GPIO_C_NUM, 5, GPIO_C_NUM, 4, I2C_SCL_NORMAL_BUS_SPEED);
	i2c_master_enable();
	i2c_set_frequency(I2C_SCL_NORMAL_BUS_SPEED);

	// Clear sleep bit to start sensor
	MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0x00);

	// ACCEL CONFIG
	MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, 0x18);
	int16_t currentX = 0;
	int16_t currentY = 0;
	//int16_t currentZ = 0;

	while(1) {
		PROCESS_YIELD();
		/*
		if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		} else if (etimer_expired(&periodic_timer_green)) {
			leds_toggle(LEDS_GREEN);
			etimer_restart(&periodic_timer_green);
		} else if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}
		*/
		
		if (etimer_expired(&periodic_timer_blue)) {
			currentX = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
			currentY = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
		
			leds_off(LEDS_ALL);

			if (currentY <= 750 && currentY >= -750) {
				//leds_off(LEDS_RED);
				leds_on(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			}
			else if (currentY > 750) {
				leds_on(LEDS_RED);
				//leds_off(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			} 
			else {
				//leds_off(LEDS_RED);
				//leds_off(LEDS_GREEN);
				leds_on(LEDS_BLUE);
			}

			if (currentX <= 750 && currentX >= -750) {
			}
			else if (currentX > 750) {
				leds_on(LEDS_RED);
			}
			else {
				//leds_on()
			}

			etimer_restart(&periodic_timer_blue);
		}
		


	}

	PROCESS_END();
}

