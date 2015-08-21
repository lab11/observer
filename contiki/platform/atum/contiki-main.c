/**
 * \addtogroup platform
 * @{
 *
 * \defgroup atum The atum sensor mote module.
 * @{
 *
 * \file
 *   Main module for the atum platform
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/leds.h"
#include "dev/sys-ctrl.h"
#include "dev/scb.h"
#include "dev/nvic.h"
#include "dev/uart.h"
#include "dev/watchdog.h"
#include "dev/ioc.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/cc2538-rf.h"
#include "dev/udma.h"
#include "usb/usb-serial.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/queuebuf.h"
#include "net/ip/tcpip.h"
#include "net/ip/uip.h"
#include "net/mac/frame802154.h"
#include "cpu.h"
#include "reg.h"
#include "ieee-addr.h"
#include "lpm.h"
#include "spi.h"
#include "i2c.h"
#include "fm25l04b.h"
#include "rv3049.h"


#include "vtimer.h"


#include <stdint.h>
#include <string.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#if STARTUP_CONF_VERBOSE
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if UART_CONF_ENABLE
#define PUTS(s) puts(s)
#else
#define PUTS(s)
#endif
/*---------------------------------------------------------------------------*/
static void
set_rf_params(void)
{
    uint16_t short_addr;
    uint8_t ext_addr[8];

    ieee_addr_cpy_to(ext_addr, 8);

    short_addr = ext_addr[7];
    short_addr |= ext_addr[6] << 8;

    /* Populate linkaddr_node_addr. Maintain endianness */
    memcpy(&linkaddr_node_addr, &ext_addr[8 - LINKADDR_SIZE], LINKADDR_SIZE);

#if STARTUP_CONF_VERBOSE
    {
        int i;
        printf("Rime configured with address ");
        for(i = 0; i < LINKADDR_SIZE - 1; i++) {
            printf("%02x:", linkaddr_node_addr.u8[i]);
        }
        printf("%02x\n", linkaddr_node_addr.u8[i]);
    }
#endif

    NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
    NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, short_addr);
    NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, CC2538_RF_CHANNEL);
    NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, ext_addr, 8);
}

/*---------------------------------------------------------------------------*/
/**
 *
 */
static void disable_all_ioc_override() {
  uint8_t portnum = 0;
  uint8_t pinnum = 0;
  for(portnum = 0; portnum < 4; portnum++) {
      for(pinnum = 0; pinnum < 8; pinnum++) {
        if ((portnum == 3) && (pinnum & LEDS_ALL)) {
            // do nuthin
        }

        else {
          ioc_set_over(portnum, pinnum, IOC_OVERRIDE_DIS);
        }
      }
  }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Main routine for the cc2538dk platform
 */
int
main(void)
{

  nvic_init();
  ioc_init();
  sys_ctrl_init();
  clock_init();
  lpm_init();
  rtimer_init();
  vtimer_init();
  gpio_init();
  #if UART_CONF_ENABLE
  GPIO_SET_OUTPUT(GPIO_A_BASE, 0xFC);
  GPIO_CLR_PIN(GPIO_A_BASE, 0xFC);
  #else
  GPIO_SET_OUTPUT(GPIO_A_BASE, 0xFF);
  GPIO_CLR_PIN(GPIO_A_BASE, 0xFF);
  #endif
  
  GPIO_SET_OUTPUT(GPIO_B_BASE, 0x80); // 0b1110 0000 //noperiphs, just spi = 0b11111000 // 0xA0 includes opamp gate // 80 includes pir
  GPIO_CLR_PIN(GPIO_B_BASE, 0x80);
  GPIO_SET_OUTPUT(GPIO_C_BASE, 0x09); // 0b0011 1101 //noperiphs, just spi = 0b00111111 // 0x0D spi and i2c
  GPIO_CLR_PIN(GPIO_C_BASE, 0x09);
  GPIO_SET_OUTPUT(GPIO_D_BASE, 0x00);
  GPIO_CLR_PIN(GPIO_D_BASE, 0x00);
  //GPIO_SET_OUTPUT(GPIO_D_BASE, 0xFF);
  //GPIO_CLR_PIN(GPIO_D_BASE, 0xFF);

  //disable_all_ioc_override();
  //ioc_set_over(GPIO_A_NUM, 6, IOC_OVERRIDE_PUE);
  // set high to test for the P-MOSFET for power gating the opamp (PORT A, PIN 6)
  //GPIO_SET_PIN(GPIO_A_BASE, 0x40);
  //GPIO_SET_PIN(GPIO_D_BASE, 0x07);

  /*GPIO_SET_OUTPUT(GPIO_A_BASE, 0xFF);
  GPIO_CLR_PIN(GPIO_A_BASE, 0xFF);*/

  #if UART_CONF_ENABLE
  //ioc_set_over(GPIO_A_NUM, 0, IOC_OVERRIDE_DIS);
  //ioc_set_over(GPIO_A_NUM, 1, IOC_OVERRIDE_DIS);
  #else
  ioc_set_over(GPIO_A_NUM, 0, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 1, IOC_OVERRIDE_DIS);
  #endif
  ioc_set_over(GPIO_A_NUM, 2, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 3, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 4, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 5, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 6, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_A_NUM, 7, IOC_OVERRIDE_DIS);

  // ioc_set_over(GPIO_B_NUM, 3, IOC_OVERRIDE_DIS);
  // ioc_set_over(GPIO_B_NUM, 4, IOC_OVERRIDE_DIS);
  //ioc_set_over(GPIO_B_NUM, 5, IOC_OVERRIDE_DIS); // pir int
  //ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS); // opamp gate
  ioc_set_over(GPIO_B_NUM, 7, IOC_OVERRIDE_DIS);

  ioc_set_over(GPIO_C_NUM, 0, IOC_OVERRIDE_DIS);
  // ioc_set_over(GPIO_C_NUM, 1, IOC_OVERRIDE_DIS); // pressure cs
  //ioc_set_over(GPIO_C_NUM, 2, IOC_OVERRIDE_DIS);
    GPIO_SET_INPUT(GPIO_C_NUM, 0x04);
    ioc_set_over(GPIO_C_NUM, 2, IOC_OVERRIDE_PUE);
  ioc_set_over(GPIO_C_NUM, 3, IOC_OVERRIDE_DIS);
  //ioc_set_over(GPIO_C_NUM, 4, IOC_OVERRIDE_DIS); // i2c scl
  //ioc_set_over(GPIO_C_NUM, 5, IOC_OVERRIDE_DIS); // i2c sda

  // lps331ap
  GPIO_SET_OUTPUT(GPIO_C_BASE, 0x02);
  GPIO_SET_PIN(GPIO_C_BASE, 0x02);
  ioc_set_over(GPIO_C_NUM, 1, IOC_OVERRIDE_DIS);

  // mpu9250
  GPIO_SET_OUTPUT(GPIO_B_BASE, 0x08);
  GPIO_SET_PIN(GPIO_B_BASE, 0x08);
  ioc_set_over(GPIO_B_NUM, 3, IOC_OVERRIDE_DIS);

  // opamp gate
  GPIO_SET_OUTPUT(GPIO_B_BASE, 0x40);
  GPIO_SET_PIN(GPIO_B_BASE, 0x40);
  ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS);


  /*GPIO_SET_OUTPUT(GPIO_B_BASE, 0xF9);
  GPIO_CLR_PIN(GPIO_B_BASE, 0xF9);
  ioc_set_over(GPIO_B_NUM, 0, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_B_NUM, 3, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_B_NUM, 4, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_B_NUM, 5, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_B_NUM, 6, IOC_OVERRIDE_DIS);
  ioc_set_over(GPIO_B_NUM, 7, IOC_OVERRIDE_DIS);*/



  leds_init();

  process_init();

  watchdog_init();
  //button_sensor_init();
  spi_init();
  // GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
  //                         GPIO_PIN_MASK(6));
  // GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM),
  //                         GPIO_PIN_MASK(7));
  // GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_B_NUM),
  //                         GPIO_PIN_MASK(0));

  fm25l04b_init();
  rv3049_init();

  //REG(SYS_CTRL_SRSSI) |= 1;
  // disable ssi gpios after theyre used before going to sleep
  // GPIO_SET_OUTPUT(GPIO_C_BASE, 0xC0);
  // GPIO_CLR_PIN(GPIO_C_BASE, 0xC0);
  // GPIO_SET_OUTPUT(GPIO_B_BASE, 0x01);
 //  GPIO_CLR_PIN(GPIO_B_BASE, 0x01);

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);

  // i2c_master_disable();
  // REG(SYS_CTRL_RCGCI2C) &= ~(1); /* Run mode */
  // REG(SYS_CTRL_SRI2C) |= 1;
  // GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
  // GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));

  // GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(5));
  // GPIO_CLR_PIN(GPIO_C_BASE, GPIO_PIN_MASK(5));
  // GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(4));
  // GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));


  /*
   * Character I/O Initialization.
   * When the UART receives a character it will call serial_line_input_byte to
   * notify the core. The same applies for the USB driver.
   *
   * If slip-arch is also linked in afterwards (e.g. if we are a border router)
   * it will overwrite one of the two peripheral input callbacks. Characters
   * received over the relevant peripheral will be handled by
   * slip_input_byte instead
   */
#if UART_CONF_ENABLE
  uart_init(0);
  uart_init(1);
  uart_set_input(SERIAL_LINE_CONF_UART, serial_line_input_byte);
#endif

#if USB_SERIAL_CONF_ENABLE
  usb_serial_init();
  usb_serial_set_input(serial_line_input_byte);
#endif

  serial_line_init();

  INTERRUPTS_ENABLE();

  PUTS(CONTIKI_VERSION_STRING);
  PUTS(BOARD_STRING);

  PRINTF(" Net: ");
  PRINTF("%s\n", NETSTACK_NETWORK.name);
  PRINTF(" MAC: ");
  PRINTF("%s\n", NETSTACK_MAC.name);
  PRINTF(" RDC: ");
  PRINTF("%s\n", NETSTACK_RDC.name);

  /* Initialise the H/W RNG engine. */
  random_init(0);

  udma_init();

  process_start(&etimer_process, NULL);
  ctimer_init();

  set_rf_params();
  netstack_init();

#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
  queuebuf_init();
  process_start(&tcpip_process, NULL);
#endif /* NETSTACK_CONF_WITH_IPV6 */

  //process_start(&sensors_process, NULL);

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  autostart_start(autostart_processes);

#if WATCHDOG_CONF_ENABLE
  watchdog_start();
#endif

  while(1) {
    uint8_t r;
    do {
      /* Reset watchdog and handle polls and events */
      watchdog_periodic();

      r = process_run();
    } while(r > 0);

    /* We have serviced all pending events. Enter a Low-Power mode. */
    lpm_enter();
  }
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 * @}
 */
