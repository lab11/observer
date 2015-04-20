#include "contiki.h"
#include "lps331ap.h"
#include "si1147.h"
#include "mpu9250.h"
#include "si7021.h"
#include "amn41122.h"
#include "adc121c021.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "spi-arch.h"
#include "spi.h"
#include "gpio.h"
#include "i2c.h"
// networking
#include "checksum.h"
#include "coilcube_ip.h"
#include "sys/rtimer.h"
#include "dev/button-sensor.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
#include "net/rime/broadcast.h"
#include "net/ip/uip.h"
#include "net/ip/uip-udp-packet.h"
#include "net/ip/uiplib.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"

#include <stdio.h>
#include <stdint.h>

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
  uint16_t temp, humd;
  unsigned press;
  si1147_als_data_t als_data; 
  uint16_t mic_amp;
  etimer_set(&wait_timer, POLL_PERIOD);
  
  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);

  si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
  mpu9250_init();
  lps331ap_init();
  amn41122_init();
  adc121c021_config();

  // signal that init is done
  leds_toggle(LEDS_GREEN);

  while(1) {
    PROCESS_YIELD();
    etimer_stop(&wait_timer);

    printf("---------------\n");

    si1147_als_force_read(&als_data);
    press = lps331ap_get_pressure();
    accel_y = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
    amn41122_read();
    temp = si7021_readTemp(TEMP_NOHOLD);
    humd = si7021_readHumd(RH_NOHOLD);
    mic_amp = adc121c021_read_reg16(ADC121C021_CONV_RESULT);



    etimer_reset(&wait_timer);
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
