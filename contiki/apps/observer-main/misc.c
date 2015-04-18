#include "contiki.h"
#include <stdio.h>
#include "cpu/cc2538/dev/i2c.h"
#include "misc.h"

void i2c_buffer_flush() {
  int i;
  for (i=0; i < 9; i++) {
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
    WAIT_WHILE(i2c_master_busy());
  }
}

