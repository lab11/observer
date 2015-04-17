#include "contiki.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/spi-arch.h"
#include "core/dev/spi.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si1147.h"
#include "mpu9250.h"

static struct etimer wait_timer;

#define POLL_PERIOD CLOCK_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(observer_main_process, "observer-main");
AUTOSTART_PROCESSES(&observer_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_main_process, ev, data) {
  PROCESS_BEGIN();
	 
  int16_t accel_cury; 
  si1147_als_data als_data;

  etimer_set(&wait_timer, POLL_PERIOD);

	spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);
  
  si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
  mpu9250_init(); 

  while(1) {
    PROCESS_YIELD();
    etimer_stop(&wait_timer);

    printf("---------------\n");
    i2c_buffer_flush();

    si1147_als_force_read(&als_data);
    accel_cury = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
 
    etimer_reset(&wait_timer);
  }
  PROCESS_END();
}
