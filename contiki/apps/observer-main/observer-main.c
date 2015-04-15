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
PROCESS(observer_main_process, "observer-main");
AUTOSTART_PROCESSES(&observer_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_main_process, ev, data) {
  PROCESS_BEGIN();
  
  si1147_als_data als_data;

  timer_set(&wait_timer, CLOCK_SECOND);

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);   
  
  si1147_init(0, SI1147_ALS_ENABLE); 

  while(1) {
    printf("---------------\n");
    i2c_buffer_flush();
    si1147_als_force_read(&als_data);

    timer_restart(&wait_timer);
    while (!timer_expired(&wait_timer));
  }
  PROCESS_END();
}
