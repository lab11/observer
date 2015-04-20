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

static void i2c_buffer_flush() {
  int i;
  for (i=0; i < 9; i++) {
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    WAIT_WHILE(i2c_master_busy());
  }
}

uint16_t si7021_readTemp(TEMP_READ_t read_type) {

  uint8_t command;
  i2c_master_enable();
  //Assign the command to be sent to the slave address
  if(read_type == TEMP_HOLD)
    command = SI7021_TEMP_MEASURE_HOLD;
  else if(read_type == TEMP_NOHOLD){
    command = SI7021_TEMP_MEASURE_NOHOLD;
    leds_toggle(LEDS_RED);
  }
  else {
    command = SI7021_READ_TEMP_FROM_PREV_HUMD;
    //leds_toggle(LEDS_ALL);
  }
  
  uint8_t transmit_bufBase[] = {command};
  i2c_single_send(SI7021_SLAVE_ADDRESS, *transmit_bufBase);
  uint8_t receive_bufBase[2];
  while(i2c_burst_receive(SI7021_SLAVE_ADDRESS, receive_bufBase, 2) != I2C_MASTER_ERR_NONE);
  uint8_t H = receive_bufBase[0];
  uint8_t L = receive_bufBase[1];
  uint16_t temperature = (((H << 8) + L) * 175.72) / 65536 - 46.85;

  leds_toggle(LEDS_GREEN);
  if (SI7021_DBG) printf("si7021:   temp %d\n", temperature);
  return temperature;
}

uint16_t si7021_readHumd(HUMD_READ_t read_type) {

  uint8_t command;

  if(read_type == RH_HOLD)
    command = SI7021_HUMD_MEASURE_HOLD;
  else
    command = SI7021_HUMD_MEASURE_NOHOLD;

  uint8_t transmit_bufBase[] = {command};
  uint8_t receive_bufBase[2];
  i2c_single_send(SI7021_SLAVE_ADDRESS, *transmit_bufBase);
  while(i2c_burst_receive(SI7021_SLAVE_ADDRESS, receive_bufBase, 2) != I2C_MASTER_ERR_NONE);

  uint8_t H = receive_bufBase[0];
  uint8_t L = receive_bufBase[1];
  uint16_t humidity = (((H << 8) + L) * 125) / 65536 - 6;
  if (SI7021_DBG) printf("si7021:   humd %d\n", humidity);
  return humidity;


}

void si7021_write_userreg(uint8_t data){
  uint8_t transmit_bufBase[2] = {SI7021_WRITE_USER_REG, data};
  i2c_burst_send(SI7021_SLAVE_ADDRESS, transmit_bufBase, 2);
  leds_toggle(LEDS_GREEN);
}

int si7021_read_userreg() {

  uint8_t transmit_bufBase[] = {SI7021_READ_USER_REG};
  uint8_t receive_bufBase[1];
  i2c_single_send(SI7021_SLAVE_ADDRESS, *transmit_bufBase);
  while(i2c_single_receive(SI7021_SLAVE_ADDRESS, receive_bufBase)!= I2C_MASTER_ERR_NONE);
  leds_toggle(LEDS_BLUE);
  return *receive_bufBase;
}

double si7021_read_electronicID() {
  double ID = 0;

  uint8_t transmit_bufBase[] = {SI7021_READ_ELEC_ID_FIRST_BYTE1, SI7021_READ_ELEC_ID_FIRST_BYTE2};
  uint8_t receive_bufBase[8];
  i2c_burst_send(SI7021_SLAVE_ADDRESS, transmit_bufBase, sizeof(transmit_bufBase));
  i2c_burst_receive(SI7021_SLAVE_ADDRESS, receive_bufBase, 8);

  //ID = (receive_bufBase[6] << 32) + (receive_bufBase[4] << 40) + (receive_bufBase[2] << 48) + (receive_bufBase[0] << 56);

  uint8_t transmit_bufBase2[] = {SI7021_READ_ELEC_ID_LAST_BYTE1, SI7021_READ_ELEC_ID_LAST_BYTE2};
  uint8_t receive_bufBase2[8];
  i2c_burst_send(SI7021_SLAVE_ADDRESS, transmit_bufBase2, sizeof(transmit_bufBase2));
  i2c_burst_receive(SI7021_SLAVE_ADDRESS, receive_bufBase2, 8);

  ID += ((receive_bufBase2[6]) + (receive_bufBase2[4] << 8) + (receive_bufBase2[2] << 16) + (receive_bufBase2[0] << 24));

  return ID; 
}

void si7021_reset(){
  uint8_t transmit_bufBase[] = {SI7021_RESET};
  i2c_single_send(SI7021_SLAVE_ADDRESS, *transmit_bufBase);
  leds_toggle(LEDS_RED);

}
