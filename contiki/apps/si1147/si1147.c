#include "contiki.h"
#include <stdio.h>
#include "sys/timer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si1147.h"
static struct timer wait_timer;

//TODO: error handling

/*---------------------------------------------------------------------------*/
PROCESS(si1147_process, "si1147");
AUTOSTART_PROCESSES(&si1147_process);
/*---------------------------------------------------------------------------*/
void i2c_buffer_flush();

PROCESS_THREAD(si1147_process, ev, data) {
  uint8_t rx;
  PROCESS_BEGIN();

  timer_set(&wait_timer, CLOCK_SECOND);


  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);   
  
  si1147_init(0, SI1147_ALS_ENABLE); 

  while(1) {
    i2c_buffer_flush(); 
    si1147_write_command(SI1147_COMMAND_ALS_FORCE);

    timer_restart(&si1147_startup_timer, SI1147_STARTUP_TIME);
    while (!timer_expired(&si1147_startup_timer));
    si1147_irq_handler();
  }
  PROCESS_END();
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
void si1147_write_reg(uint8_t reg_addr, uint8_t data) {
  // second byte is 0/AI/reg_address
  uint8_t tx[] = {reg_addr, data};

  i2c_buffer_flush();
  if (SI1147_DBG)
    printf("si1147_write_reg: [0x%x] <- 0x%x\n", reg_addr, data);

  i2c_burst_send(SI1147_DEFAULT_SLAVE_ADDR, tx, sizeof(tx));
  return;
}

// autoincrement disable if reg_addr is anded with SI1147_AUTO_INCR_DISABLE 
uint8_t si1147_read_reg(uint8_t reg_addr) {
  uint8_t tx[] = {reg_addr};
  uint8_t rx[1];
  i2c_buffer_flush();

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
    while (SI1147_DBG && resp != SI1147_RESPONSE_NO_ERROR) {
      printf("si1147_write_command: 0x%x NOP error= 0x%x\n", data,resp);
      i2c_buffer_flush();
      si1147_write_reg(SI1147_COMMAND, SI1147_COMMAND_NOP);
      resp = si1147_read_reg(SI1147_RESPONSE);
    }

    // 3 - write command value
    si1147_write_reg(SI1147_COMMAND, data);

    // 4 - read response and verify nonzero (unneccessary if RESET)
    if (data == SI1147_COMMAND_RESET) return 0;
    
    timer_set(&si1147_command_timer, SI1147_STARTUP_TIME);
    do {
      resp = si1147_read_reg(SI1147_RESPONSE);
      // 5 - goto 4 if response 0x00
    } while (resp == 0 && !timer_expired(&si1147_command_timer));

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
void si1147_init(uint16_t meas_rate, uint8_t meas_enable) {
  if(SI1147_DBG) printf("si1147_init\n");
  
  // startup time for SI1147 is 25 ms
  timer_set(&si1147_startup_timer, SI1147_STARTUP_TIME);
  while (!timer_expired(&si1147_startup_timer));
 
  // after initialization, moves to standby mode
  // host must write 0x17 to HW_KEY for proper operation
  si1147_write_reg(SI1147_HW_KEY, 0x17);

  si1147_write_reg(SI1147_MEAS_RATE0, meas_rate);
  si1147_write_reg(SI1147_MEAS_RATE1, meas_rate >> 8);

  // must be written before any measurement is forced or auto
  si1147_write_param(SI1147_PARAM_CHLIST, meas_enable);

  // set auto mode
  if (meas_rate > 0) {
    if (meas_enable == SI1147_ALS_ENABLE)
      si1147_write_command(SI1147_COMMAND_ALS_AUTO);
    else if (meas_enable == SI1147_PS_ENABLE)
      si1147_write_command(SI1147_COMMAND_PS_AUTO);
    else if (meas_enable == SI1147_PSALS_ENABLE)
      si1147_write_command(SI1147_COMMAND_PSALS_AUTO);
  }

  return;
}

// caller must allocate data as 6 bytes
void si1147_als_read(si1147_als_data *data) { 
  data->vis.b.lo = si1147_read_reg(SI1147_ALS_VIS_DATA0);
  data->vis.b.hi = si1147_read_reg(SI1147_ALS_VIS_DATA1);
  data->ir.b.lo = si1147_read_reg(SI1147_ALS_IR_DATA0);
  data->ir.b.hi = si1147_read_reg(SI1147_ALS_IR_DATA1); data->aux.b.lo = si1147_read_reg(SI1147_AUX_DATA0);
  data->aux.b.hi = si1147_read_reg(SI1147_AUX_DATA1);
  
  if (SI1147_DBG_ALS) {
    printf("ALS vis: %hd\n ALS ir: %hd\n ALS aux: %hd\n",
      data->vis.val,
      data->ir.val,
      data->aux.val);
  }
  
  return;
}

void si1147_als_force_read(si1147_als_data *data) {
  si1147_write_command(SI1147_COMMAND_ALS_FORCE);
  si1147_als_read(data);

  return;
}

void si1147_irq_enable() {
  GPIO_SOFTWARE_CONTROL(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  GPIO_SET_INPUT(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  GPIO_DETECT_EDGE(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  GPIO_DETECT_FALLING(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  GPIO_ENABLE_INTERRUPT(SI1147_IRQ_BASE, SI1147_IRQ_PIN_MASK);
  ioc_set_over(SI1147_IRQ_PORT, SI1147_IRQ_PIN, IOC_OVERRIDE_DIS);
  nvic_interrupt_enable(NVIC_INT_GPIO_PORT_C);
  gpio_register_callback((gpio_callback_t)si1147_irq_handler, SI1147_IRQ_PORT, SI1147_IRQ_PIN);

  return;
}

void si1147_als_irq_enable() {
  si1147_write_reg(SI1147_IRQ_ENABLE, 0x01);
  si1147_write_reg(SI1147_INT_CFG, 0x01);

  return;
}

void si1147_irq_handler() {
  uint8_t rx;

  si1147_als_data als_data;

  si1147_als_read(&als_data);

  rx = si1147_read_reg(SI1147_IRQ_STATUS);
  si1147_write_reg(SI1147_IRQ_STATUS, rx);
  return;
}

void i2c_buffer_flush() {
  int i;
  for (i=0; i < 9; i++) {
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(i2c_master_busy());
  }
}
