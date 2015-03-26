#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "gpio.h"
#include "spi-arch.h"
#include "dev/spi.h"

static struct etimer periodic_timer_red;

/*---------------------------------------------------------------------------*/
PROCESS(spi_process, "Spi");
AUTOSTART_PROCESSES(&spi_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(spi_process, ev, data) {
  PROCESS_BEGIN();
  etimer_set(&periodic_timer_red, CLOCK_SECOND);
  spi_enable();
  // clock high while idle, data valid on clock trailing edge
  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  spi_cs_init(GPIO_B_NUM, 4);
  while(1) {
    PROCESS_YIELD();
    // Send SPI message
    char * s = "Hello World!";
    int i = 0;
    for(i = 0; i < strlen(s); ++i)
    {
      SPI_CS_CLR(GPIO_B_NUM, 4);

      SPI_WRITE(s[i]);

      SPI_CS_SET(GPIO_B_NUM, 4);
    }
    // Blink LED
    if (etimer_expired(&periodic_timer_red))
    {
        leds_toggle(LEDS_RED);
        etimer_restart(&periodic_timer_red);
    }
  }
  PROCESS_END();
}