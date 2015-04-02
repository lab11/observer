#include "contiki.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si1147.h"

static struct etimer periodic_timer;
static struct etimer command_timer;

void si1147_init(uint16_t meas_rate);
void si1147_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t si1147_read_reg(uint8_t reg_addr);

uint8_t si1147_write_command(uint8_t data);
void si1147_write_param(uint8_t param, uint8_t data);
uint8_t si1147_read_param(uint8_t param);

void si1147_ALS_enable();
void si1147_ALS_force(si1147_als_data *data);

//TODO: error handling

/*---------------------------------------------------------------------------*/
PROCESS(si1147_process, "si1147");
AUTOSTART_PROCESSES(&si1147_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(si1147_process, ev, data) {
  uint8_t rx;

  PROCESS_BEGIN();
  
  // startup time for SI1147 is 25 ms
  etimer_set(&periodic_timer, SI1147_STARTUP_TIME);
  while (!etimer_expired(&periodic_timer))
    PROCESS_YIELD();

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_NORMAL_BUS_SPEED);   
  
  si1147_init(SI1147_FORCED_CONVERSION);

  etimer_set(&periodic_timer, CLOCK_SECOND);
 
  while(1) {
    PROCESS_YIELD();
    
    if (etimer_expired(&periodic_timer)) {
      leds_toggle(LEDS_RED);
      etimer_restart(&periodic_timer);
    }
  }
  PROCESS_END();
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
void si1147_write_reg(uint8_t reg_addr, uint8_t data) {
  // second byte is 0/AI/reg_address
  uint8_t tx[] = {reg_addr, data};
  
  if (SI1147_DBG)
    printf("si1147_write_reg: [0x%x] <- 0x%x\n", reg_addr, data);
  
  i2c_burst_send(SI1147_DEFAULT_SLAVE_ADDR, tx, sizeof(tx));
  return;
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
uint8_t si1147_read_reg(uint8_t reg_addr) {
  uint8_t tx[] = {reg_addr};
  uint8_t rx[1];

  i2c_single_send(SI1147_DEFAULT_SLAVE_ADDR, *tx);
  i2c_single_receive(SI1147_DEFAULT_SLAVE_ADDR, rx);

  if (SI1147_DBG)
    printf("si1147_read_reg: [0x%x] -> 0x%x\n", reg_addr, *rx);
  return *rx;
}

uint8_t si1147_write_command(uint8_t data) {
  uint8_t resp;
  
  if (SI1147_DBG) printf("si1147_write_command: 0x%x\n", data);

  do {
    // 1 - write 0x00 to command
    si1147_write_reg(SI1147_COMMAND, SI1147_COMMAND_NOP);

    // 2 - read response and ensure 0x00
    resp = si1147_read_reg(SI1147_RESPONSE);
    if (SI1147_DBG && resp != SI1147_RESPONSE_NO_ERROR) {
      printf("si1147_write_command: NOP error= 0x%x", resp);
      return -1;
    }

    // 3 - write command value
    si1147_write_reg(SI1147_COMMAND, data);

    // 4 - read response and verify nonzero (unneccessary if RESET)
    if (data == SI1147_COMMAND_RESET) return 0;
    
    etimer_set(&command_timer, SI1147_STARTUP_TIME);
    do {
      resp = si1147_read_reg(SI1147_RESPONSE);
      // 5 - goto 4 if response 0x00
    } while (resp == 0 && !etimer_expired(&command_timer));

    // 6 - if 25ms pass, goto 1
  } while (resp == 0);

  return 0;
}

void si1147_write_param(uint8_t param, uint8_t data) {
  si1147_write_reg(SI1147_PARAM_WR, data);
  // tell to write PARAM_WR to SI1147_PARAM_I2C_ADDR
  si1147_write_command(SI1147_COMMAND_PARAM_SET | param);

  return;
}

uint8_t si1147_read_param(uint8_t param) {
  // tell to write parameter to PARAM_RD
  si1147_write_command(SI1147_COMMAND_PARAM_QUERY | param);
  return si1147_read_reg(SI1147_PARAM_RD);
}

// meas_rate:
//  - represents the rate at which the sensor wakes up to take measurements
//  - when non-zero, sensor is in autonomous mode
void si1147_init(uint16_t meas_rate) {
  if(SI1147_DBG) printf("si1147_init\n");
 
  // after initialization, moves to standby mode
  // host must write 0x17 to HW_KEY for proper operation
  si1147_write_reg(SI1147_HW_KEY, 0x17); 
  
  si1147_write_reg(SI1147_MEAS_RATE0, meas_rate);
  si1147_write_reg(SI1147_MEAS_RATE1, meas_rate >> 8);

  return;
}

void si1147_ALS_enable() {
  si1147_write_param(SI1147_PARAM_CHLIST, SI1147_ALS_ENABLE);
}

// caller mus allocate rx_data as 6 bytes
void si1147_ALS_force(si1147_als_data *data) {
  si1147_write_command(SI1147_COMMAND_ALS_FORCE);
 
  data->vis.b.lo = si1147_read_reg(SI1147_ALS_VIS_DATA0);
  data->vis.b.hi = si1147_read_reg(SI1147_ALS_VIS_DATA1);
  data->ir.b.lo = si1147_read_reg(SI1147_ALS_IR_DATA0);
  data->ir.b.hi = si1147_read_reg(SI1147_ALS_IR_DATA1);
  data->aux.b.lo = si1147_read_reg(SI1147_AUX_DATA0);
  data->aux.b.hi = si1147_read_reg(SI1147_AUX_DATA1);
  
  return;
}
