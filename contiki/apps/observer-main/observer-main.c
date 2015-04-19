#include "contiki.h"
#include "lps331ap.h"
#include "si1147.h"
#include "mpu9250.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "spi-arch.h"
#include "spi.h"
#include "gpio.h"
#include "i2c.h"

#include <stdio.h>

static struct etimer wait_timer;
static struct timer test_timer;
#define POLL_PERIOD CLOCK_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(observer_main_process, "observer-main");
AUTOSTART_PROCESSES(&observer_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_main_process, ev, data) {
  PROCESS_BEGIN();

  int16_t accel_y;
  unsigned press;
  si1147_als_data_t als_data; 
//  etimer_set(&wait_timer, POLL_PERIOD);
//  leds_toggle(LEDS_BLUE);
	spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);

  si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
  mpu9250_init();
  lps331ap_init();

  while(1) {
//    PROCESS_YIELD();
//    etimer_stop(&wait_timer);

    printf("---------------\n");
    i2c_buffer_flush();

    si1147_als_force_read(&als_data);
    press = lps331ap_get_pressure();
    accel_y = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
//    etimer_reset(&wait_timer);
  }
  PROCESS_END();
}

// void i2c_buffer_flush() {
//   int i;
//   for (i=0; i < 9; i++) {
//     i2c_master_command(I2C_MASTER_CMD_BURST_SEND_FINISH);
//     WAIT_WHILE(i2c_master_busy());
//   }
// }