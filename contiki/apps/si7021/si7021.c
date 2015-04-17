#include "contiki.h"

#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si7021.h"

//static struct etimer periodic_timer;


// reads from register reg_addrL and reg_addrH on the cc2538
// the sensor data of the Si7021 are 16bits so you have to read the lower and upper 8bits separately
// to read from a specific register, write the address of the register you want to read to the slave
// then read from slave device
uint16_t Si7021_readTemp(TEMP_READ_t read_type) {

	uint8_t command;
	i2c_master_enable();
	//Assign the command to be sent to the slave address
	if(read_type == TEMP_HOLD)
		command = Si7021_TEMP_MEASURE_HOLD;
	else if(read_type == TEMP_NOHOLD){
		command = Si7021_TEMP_MEASURE_NOHOLD;
		leds_toggle(LEDS_RED);
	}
	else {
		command = Si7021_READ_TEMP_FROM_PREV_HUMD;
		//leds_toggle(LEDS_ALL);
	}
	
	uint8_t transmit_bufBase[] = {command};
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	uint8_t receive_bufBase[2];
	while(i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 2) != I2C_MASTER_ERR_NONE);
	uint8_t H = receive_bufBase[0];
	uint8_t L = receive_bufBase[1];
	uint16_t temperature = (((H << 8) + L) * 175.72) / 65536 - 46.85;

	leds_toggle(LEDS_GREEN);
	printf("temp H: %x\n L: %x\n", H, L);
	printf("temperature: %d\n", temperature);
	return temperature;
}

uint16_t Si7021_readHumd(HUMD_READ_t read_type) {

	uint8_t command;

	if(read_type == RH_HOLD)
		command = Si7021_HUMD_MEASURE_HOLD;
	else
		command = Si7021_HUMD_MEASURE_NOHOLD;

	uint8_t transmit_bufBase[] = {command};
	uint8_t receive_bufBase[2];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	while(i2c_burst_receive(Si7021_SLAVE_ADDRESS, receive_bufBase, 2) != I2C_MASTER_ERR_NONE);

	uint8_t H = receive_bufBase[0];
	uint8_t L = receive_bufBase[1];
	printf("humd H: %x \n L: %x \n", H, L);
	uint16_t humidity = (((H << 8) + L) * 125) / 65536 - 6;
	printf("humidity: %d\n", humidity);
	//leds_toggle(LEDS_BLUE);
	return humidity;


}

void Si7021_write_userreg(uint8_t data){
	uint8_t transmit_bufBase[2] = {Si7021_WRITE_USER_REG, data};
	i2c_burst_send(Si7021_SLAVE_ADDRESS, transmit_bufBase, 2);
	leds_toggle(LEDS_GREEN);
}

int Si7021_read_userreg() {

	uint8_t transmit_bufBase[] = {Si7021_READ_USER_REG};
	uint8_t receive_bufBase[1];
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	while(i2c_single_receive(Si7021_SLAVE_ADDRESS, receive_bufBase)!= I2C_MASTER_ERR_NONE);
	leds_toggle(LEDS_BLUE);
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

void Si7021_reset(){
	uint8_t transmit_bufBase[] = {Si7021_RESET};
	i2c_single_send(Si7021_SLAVE_ADDRESS, *transmit_bufBase);
	leds_toggle(LEDS_RED);

}

void i2c_bufferflush(){
	int i;
	for(i = 0; i < 9; ++i){
		i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
	}

}
/*---------------------------------------------------------------------------*/
PROCESS(si7021_process, "Si7021");
AUTOSTART_PROCESSES(&si7021_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(si7021_process, ev, data) {

	//PROCESS_BEGIN();
	//the startup time for SI7021 is 80 ms
	//etimer_set(&periodic_timer, Si7021_STARTUP_TIME);

	printf("temp_hold: \n");


	// Initialize i2c
	i2c_init(GPIO_C_NUM, 5, GPIO_C_NUM, 4, I2C_SCL_NORMAL_BUS_SPEED);
	//i2c_master_enable();
	i2c_set_frequency(I2C_SCL_NORMAL_BUS_SPEED);
	uint16_t temp = 0;
	uint16_t humd = 0;
		//Si7021_write_userreg(Si7021_RESOLUTION_R10_T13 | Si7021_HEATER_DISABLE);
		//i2c_bufferflush();
		uint16_t userreg;
	while(1){
		//PROCESS_YIELD();		

		//Si7021_reset();
		//userreg = Si7021_read_userreg();
		//printf("userreg reset val: %x\n", userreg);
		//Si7021_write_userreg(Si7021_RESOLUTION_R10_T13 | Si7021_HEATER_DISABLE);
		userreg = Si7021_read_userreg();
		//i2c_bufferflush();


		temp = Si7021_readTemp(TEMP_NOHOLD);
		i2c_bufferflush();
		humd = Si7021_readHumd(RH_NOHOLD);
		i2c_bufferflush();
		//userreg = Si7021_read_userreg();
		//i2c_bufferflush();
	}

	return 0;
		
}

