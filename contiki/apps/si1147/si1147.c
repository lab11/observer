#include "contiki.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si1147.h"

static struct etimer periodic_timer;

uint8_t g_slave_addr;

void si1147_init(int16_t meas_rate);
void si1147_write_reg(uint8_t reg_addr, uint8_t data);
int8_t si1147_read_reg(uint8_t reg_addr);

int8_t si1147_write_command(uint8_t data);
void si1147_set_slave_address(uint8_t addr);

//TODO: error handling

/*---------------------------------------------------------------------------*/
PROCESS(si1147_process, "si1147");
AUTOSTART_PROCESSES(&si1147_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(si1147_process, ev, data) {
  PROCESS_BEGIN();
  
  // startup time for SI1147 is 25 ms
  etimer_set(&periodic_timer, SI1147_STARTUP_TIME);
  while (!etimer_expired(&periodic_timer))
    PROCESS_YIELD();

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_NORMAL_BUS_SPEED);   
  
  si1147_init(SI1147_FORCED_CONVERSION);
 
  // forced conversion mode is entered if either the ALS_FORCE
  // or PLS_FORCE command is sent

  etimer_set(&periodic_timer, CLOCK_SECOND);
 
  while(1) {
    PROCESS_YIELD();
    printf("hello world\n");
    
    // stuffs
  }
  PROCESS_END();
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
void si1147_write_reg(uint8_t reg_addr, uint8_t data) {
  // second byte is 0/AI/reg_address
  uint8_t tx[] = {reg_addr, data};
  
  i2c_burst_send(g_slave_addr, tx, sizeof(tx));
  return;
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
int8_t si1147_read_reg(uint8_t reg_addr) {
  uint8_t tx[] = {reg_addr};
  uint8_t rx[1];

  i2c_single_send(g_slave_addr, *tx);
  i2c_single_receive(g_slave_addr, rx);
  return *rx;
}

int8_t si1147_write_command(uint8_t data) {
  int8_t resp;
  struct etimer timer;

  do {
    // 1 - write 0x00 to command
    si1147_write_reg(SI1147_COMMAND, SI1147_COMMAND_NOP);

    // 2 - read response and ensure 0x00
    resp = si1147_read_reg(SI1147_RESPONSE);
    if (resp != SI1147_RESPONSE_NO_ERROR) {
      printf("si1147_write_command(0x%x): error= 0x%x", data, resp);
      return -1;
    }

    // 3 - write command value
    si1147_write_reg(SI1147_COMMAND, data);

    // 4 - read response and verify nonzero (unneccessary if RESET)
    if (data == SI1147_COMMAND_RESET) return 0;
    
    etimer_set(&timer, SI1147_STARTUP_TIME);
    do {
      resp = si1147_read_reg(SI1147_RESPONSE);
      // 5 - goto 4 if response 0x00
    } while (resp == 0 && !etimer_expired(&periodic_timer));

    // 6 - if 25ms pass, goto 1
  } while (resp == 0);

  return 0;
}

// addr must be 6 bits
void si1147_set_slave_address(uint8_t addr) {
  si1147_write_command(SI1147_COMMAND_PARAM_SET | addr);
  si1147_write_command(SI1147_COMMAND_BUSADDR);
  
  g_slave_addr = addr;
  return;
}

// meas_rate:
//  - represents the rate at which the sensor wakes up in
//  - when non-zero, sensor is in autonomous mode
void si1147_init(int16_t meas_rate) {
  // initialize bookkeeping
  g_slave_addr = SI1147_DEFAULT_SLAVE_ADDR;
 
  // after initialization, moves to standby mode
  // host must write 0x17 to HW_KEY for proper operation
  si1147_write_reg(SI1147_HW_KEY, 0x17); 
  
  si1147_write_reg(SI1147_MEAS_RATE0, meas_rate);
  si1147_write_reg(SI1147_MEAS_RATE1, meas_rate >> 8);

  return;
}
