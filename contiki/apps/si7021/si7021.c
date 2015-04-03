#include "contiki.h"

#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si7021.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;


// reads from register reg_addrL and reg_addrH on the cc2538
// the sensor data of the Si7021 are 16bits so you have to read the lower and upper 8bits separately
// to read from a specific register, write the address of the register you want to read to the slave
// then read from slave device
double Si7021_readTemp(TEMP_READ_t read_type) {

	uint8_t command;

	//Assign the command to be sent to the slave address
	if(read_type == TEMP_HOLD)
		command = Si7021_TEMP_MEASURE_HOLD;
	else if(read_type == TEMP_NOHOLD)
		command = Si7021_TEMP_MEASURE_NOHOLD;
	else 
		command = Si7021_READ_TEMP_FROM_PREV_HUMD;

	uint8_t transmit_bufBase[] = {command};
	uint8_t receive_bufBase[2];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 2);

	uint16_t L = receive_bufBase[0];
	uint16_t H = receive_bufBase[1];
	double temperature = (((H << 8) + L) * 175.72) / 65536 - 46.85;

	return temperature;
}

double Si7021_readHumd(HUMD_READ_t read_type) {

	uint8_t command;

	if(read_type == RH_HOLD)
		command = Si7021_HUMD_MEASURE_HOLD;
	else
		command = Si7021_HUMD_MEASURE_NOHOLD;

	uint8_t transmit_bufBase[] = {command};
	uint8_t receive_bufBase[2];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 2);

	uint16_t L = receive_bufBase[0];
	uint16_t H = receive_bufBase[1];
	double humidity = (((H << 8) + L) * 125) / 65536 - 6;

	return humidity;


}

void Si7021_write_userreg(uint8_t data){
	uint8_t transmit_bufBase[] = {Si7021_SLAVE_ADDRESS, Si7021_WRITE_USER_REG, data};
	i2c_burst_send(Si7021_SLAVE_ADDRESS, transmit_bufBase, sizeof(transmit_bufBase));

}

int Si7021_read_userreg() {

	uint8_t transmit_bufBase[] = {Si7021_READ_USER_REG};
	uint8_t receive_bufBase[1];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	i2c_single_receive(Si7021_SLAVE_ADDRESS, receive_bufBase);

	return *receive_bufBase;
}

double Si7021_read_electronicID() {
	double ID = 0;

	uint8_t transmit_bufBase[] = {Si7021_READ_ELEC_ID_FIRST_BYTE1, Si7021_READ_ELEC_ID_FIRST_BYTE2};
	uint8_t receive_bufBase[8];
	i2c_burst_send(Si7021_SLAVE_ADDRESS, transmit_bufBase, sizeof(transmit_bufBase));
	i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 8);

	//ID = (receive_bufBase[6] << 32) + (receive_bufBase[4] << 40) + (receive_bufBase[2] << 48) + (receive_bufBase[0] << 56);

	uint8_t transmit_bufBase2[] = {Si7021_READ_ELEC_ID_LAST_BYTE1, Si7021_READ_ELEC_ID_LAST_BYTE2};
	uint8_t receive_bufBase2[8];
	i2c_burst_send(Si7021_SLAVE_ADDRESS, transmit_bufBase2, sizeof(transmit_bufBase2));
	i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase2, 8);

	ID += ((receive_bufBase2[6]) + (receive_bufBase2[4] << 8) + (receive_bufBase2[2] << 16) + (receive_bufBase2[0] << 24));

	return ID; 
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
	Si7021_write_userreg(Si7021_RESOLUTION_R12_T14 | Si7021_HEATER_DISABLE);

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
			double temp_nohold = Si7021_readTemp(TEMP_NOHOLD);
			double temp_hold = Si7021_readTemp(TEMP_HOLD);

			double humidity_hold = Si7021_readHumd(RH_HOLD);
			double humidity_nohold = Si7021_readHumd(RH_NOHOLD);

			double temp_fromRH = Si7021_readTemp(TEMP_READ_FROM_RH);

			//printf("temp_hold: %f\n", temp_hold);

		
			leds_on(LEDS_ALL);

			if (temp_hold == temp_nohold) {
				//leds_off(LEDS_RED);
				leds_on(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			}
			else if (temp_hold == temp_fromRH) {
				leds_on(LEDS_RED);
				//leds_off(LEDS_GREEN);
				//leds_off(LEDS_BLUE);
			} 
			else {
				//leds_off(LEDS_RED);
				//leds_off(LEDS_GREEN);
				leds_on(LEDS_BLUE);
			}

			if (temp_hold >= 100) {
				leds_on(LEDS_ALL);
			}
			/*else {
				leds_off(LEDS_ALL);
			}*/
			
			if(humidity_hold == humidity_nohold){
				leds_toggle(LEDS_RED);
			}

			etimer_restart(&periodic_timer_blue);
		}
		


	}

	PROCESS_END();
}

