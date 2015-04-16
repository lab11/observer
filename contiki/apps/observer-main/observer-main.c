#include "contiki.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/dev/i2c.h"
#include "si1147.h"

static struct etimer wait_timer;

#define POLL_PERIOD CLOCK_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(observer_main_process, "observer-main");
AUTOSTART_PROCESSES(&observer_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_main_process, ev, data) {
  PROCESS_BEGIN();
  
  si1147_als_data als_data;

  etimer_set(&wait_timer, POLL_PERIOD);

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);   
  
  si1147_init(0, SI1147_ALS_ENABLE); 

  while(1) {
    PROCESS_YIELD();
    etimer_stop(&wait_timer);

    printf("---------------\n");
    i2c_buffer_flush();
    si1147_als_force_read(&als_data);
 
    etimer_reset(&wait_timer);
  }
  PROCESS_END();
}
