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

/*------------------------ NETWORKING SETUP ---------------------------------*/
/*---------------------------------------------------------------------------*/
#define LOOP_INTERVAL       CLOCK_SECOND
#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
#define LEDS_PERIODIC       LEDS_YELLOW
#define LEDS_BUTTON         LEDS_RED
#define LEDS_SERIAL_IN      LEDS_ORANGE
#define LEDS_REBOOT         LEDS_ALL
#define LEDS_RF_RX          (LEDS_YELLOW | LEDS_ORANGE)
#define BROADCAST_CHANNEL   129
/*---------------------------------------------------------------------------*/
#define MACDEBUG 0

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF(" %02x:%02x:%02x:%02x:%02x:%02x ",lladdr->addr[0], lladdr->addr[1], lladdr->addr[2], lladdr->addr[3],lladdr->addr[4], lladdr->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#endif

#define PING6_NB 5
#define PING6_DATALEN 16

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

static uip_ipaddr_t my_addr;
static uip_ipaddr_t dest_addr;
static uip_ipaddr_t bcast_ipaddr;
static uip_lladdr_t bcast_lladdr = {{0, 0, 0, 0, 0, 0, 0, 0}};
static struct uip_udp_conn *client_conn;
/*---------------------------------------------------------------------------*/
/*----------------------- END NETWORKING SETUP ------------------------------*/


static void send_handler(process_event_t ev, process_data_t data);
static void pack_data(uint16_t accel_x, uint16_t accel_y, uint16_t accel_z, 
                          uint16_t humid, si1147_als_data_t light, uint16_t mic, 
                          uint32_t press, uint16_t temp, uint8_t pir);


pkt_data_t pkt_data = {PROFILE_ID, SEHNSOR_VERSION, /*accel*/0x01, 0x02, 0x03, 0x04, 0x05, 0x06, /*humid*/0x07, 0x08, 
                        /*light*/0x09, 0x0A, /*mic*/ 0x0B, 0x0C, /*press*/0x0D, 0x0E, 0x0F, /*temp*/0x10, 0x11, /*pir*/0x00, 
                        /*checksum*/0xFF, 0xFF, 0, 0, 0xBB};
uint32_t packet_count = 0;


//static struct etimer wait_timer;
static struct timer test_timer;
#define POLL_PERIOD 0.5*CLOCK_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(observer_main_process, "observer-main");
AUTOSTART_PROCESSES(&observer_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_main_process, ev, data) {
  PROCESS_BEGIN();

  int16_t accel_x, accel_y, accel_z;
  uint16_t temp, humd;
  unsigned press;
  si1147_als_data_t als_data; 
  uint16_t mic_amp;
  uint8_t pir;
  //etimer_set(&wait_timer, POLL_PERIOD);
  
  spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);

  si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
  mpu9250_init();
  mpu9250_motion_interrupt_init(0xF0, 0x08);

  lps331ap_init();
  amn41122_init();
  amn41122_irq_enable();

  adc121c021_config();


  /*----------------------- Initialize Networking ---------------------------*/
  // Set the local address
  uip_ip6addr(&my_addr, 0, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&my_addr, &uip_lladdr);
  uip_ds6_addr_add(&my_addr, 0, ADDR_MANUAL);

  // Setup the destination address
  uiplib_ipaddrconv(RECEIVER_ADDR, &dest_addr);

  // Add a "neighbor" for our custom route
  // Setup the default broadcast route
  uiplib_ipaddrconv(ADDR_ALL_ROUTERS, &bcast_ipaddr);
  uip_ds6_nbr_add(&bcast_ipaddr, &bcast_lladdr, 0, NBR_REACHABLE);
  uip_ds6_route_add(&dest_addr, 128, &bcast_ipaddr);

  // Setup a udp "connection"
  client_conn = udp_new(&dest_addr, UIP_HTONS(RECEIVER_PORT), NULL);
  if (client_conn == NULL) {
    // Too many udp connections
    // not sure how to exit...stupid contiki
  }
  udp_bind(client_conn, UIP_HTONS(3001));

/*--------------------- End Initialize Networking ---------------------------*/


  // signal that init is done
  leds_toggle(LEDS_GREEN);

  while(1) {
    //PROCESS_WAIT_EVENT();

    asm("CPSIE i");
    printf("after interrupt enable in process?\n");

    PROCESS_YIELD();
    //etimer_stop(&wait_timer);

    printf("---------------\n");

    si1147_als_force_read(&als_data);
    press = lps331ap_get_pressure();
    accel_x = mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
    accel_y = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
    accel_z = mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H);
    pir = amn41122_read();
    temp = si7021_readTemp(TEMP_NOHOLD);
    humd = si7021_readHumd(RH_NOHOLD);
    mic_amp = adc121c021_read_amplitude();

    pack_data(accel_x, accel_y, accel_z, humd, als_data, mic_amp, press, temp, pir);
    send_handler(ev, data);
    printf("PACKETS SENT: %d\n", ++packet_count);

    //mpu9250_readByte(MPU9250_INT_STATUS);
    //GPIO_CLEAR_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
    //nvic_interrupt_unpend(NVIC_INT_GPIO_PORT_B);
    //GPIO_ENABLE_INTERRUPT(GPIO_PORT_TO_BASE(MPU9250_INT_PORT), GPIO_PIN_MASK(MPU9250_INT_PIN));
    //GPIO_ENABLE_INTERRUPT(AMN41122_OUT_BASE, AMN41122_OUT_PIN_MASK);
    
    //etimer_reset(&wait_timer);
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



/*---------------------------------------------------------------------------*/

static void
send_handler(process_event_t ev, process_data_t data)
{
  pkt_data.counter++;
  pkt_data.seq_no++;

  PRINTF("Sending UDP!!! packet\n");
  uip_udp_packet_send(client_conn, (uint8_t*) &pkt_data, sizeof(pkt_data_t));
  leds_toggle(LEDS_BLUE);

  PRINTF("After sent udp packet.\n");
  PRINTF("size of data: %d\n", sizeof(pkt_data_t));
}


static void pack_data(uint16_t accel_x, uint16_t accel_y, uint16_t accel_z, 
                          uint16_t humid, si1147_als_data_t light, uint16_t mic, 
                          uint32_t press, uint16_t temp, uint8_t pir) {

  uint8_t accelx1 = accel_x & 0xFF;
  uint8_t accelx2 = (accel_x & 0xFF00) >> 8;
  uint8_t accely1 = accel_y & 0xFF;
  uint8_t accely2 = (accel_y & 0xFF00) >> 8;
  uint8_t accelz1 = accel_z & 0xFF;
  uint8_t accelz2 = (accel_z & 0xFF00) >> 8;
  
  uint8_t humid1 = humid & 0xFF;
  uint8_t humid2 = (humid & 0xFF00) >> 8;

  uint8_t light1 = light.vis.b.lo;
  uint8_t light2 = light.vis.b.hi;

  uint8_t mic1 = mic & 0xFF;
  uint8_t mic2 = (mic & 0xFF00) >> 8;

  uint8_t press1 = press & 0x000000FF;
  uint8_t press2 = (press & 0x0000FF00) >> 8;
  uint8_t press3 = (press & 0x00FF0000) >> 16;

  uint8_t temp1 = temp & 0xFF;
  uint8_t temp2 = (temp & 0xFF00) >> 8;

  uint16_t checksums = checksum(accelx1, accelx2, accely1, accely2, accelz1, accelz2,
                                humid1, humid2, light1, light2, mic1, mic2, press1,
                                press2, press3, temp1, temp2, pir);

  uint8_t check1 = checksums & 0x0FF;
  uint8_t check2 = (checksums & 0xFF00) >> 8;

  pkt_data_t new_pkt_data = {PROFILE_ID, SEHNSOR_VERSION, accelx1, accelx2, accely1, accely2, accelz1, 
                accelz2, humid1, humid2, light1, light2, mic1, mic2, press1,
                press2, press3, temp1, temp2, pir, check1, check2, pkt_data.counter, pkt_data.seq_no, 0xBB};
  pkt_data = new_pkt_data;

  return;
}
