#include "contiki.h"
#include "dev/serial-line.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "dev/leds.h"
#include "gpio.h"
#include "spi-arch.h"
#include "dev/spi.h"
#include "lps331ap.h"


static struct etimer periodic_timer;
int8_t lps331ap_read(uint8_t addr);
void lps331ap_write(uint8_t addr, int8_t write);

/*---------------------------------------------------------------------------*/
PROCESS(lps331ap_process, "lps331ap");
AUTOSTART_PROCESSES(&lps331ap_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lps331ap_process, ev, data) {
  PROCESS_BEGIN();
  etimer_set(&periodic_timer, CLOCK_SECOND);
  // clock high while idle, data valid on clock trailing edge
  spi_set_mode(SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
  spi_cs_init(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  while(1) {
    PROCESS_YIELD();
    lps331ap_write(LPS331AP_CTRL_REG1, 0x80);
    uint8_t who_am_i = lps331ap_read(LPS331AP_WHO_AM_I);
    printf("hello\n");
    printf("%x\n", who_am_i);

    // int8_t reg_addrs[3] = {0x28,0x29,0x2A};
    // int pout = 0;
    // int read = 0;
    // read = lps331ap_read(reg_addrs[0]);
    // pout = read;
    // read = lps331ap_read(reg_addrs[1]);
    // pout |= (read << 8);
    // read = lps331ap_read(reg_addrs[2]);
    // pout |= (read << 16);
    // pout /= 4096;
    // printf("pressure: %d\n", pout);

    if (etimer_expired(&periodic_timer))
    {
        leds_toggle(LEDS_RED);
        etimer_restart(&periodic_timer);
    }
    
  }
  PROCESS_END();
}

int8_t lps331ap_read(uint8_t addr){
  addr |= LPS331AP_READ_MASK;
  int8_t read;
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_READ(read);
  SPI_WAITFOREOTx(); // Extra wait loop before asserting CS
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  return read;
}

void lps331ap_write(uint8_t addr, int8_t write){
  SPI_CS_CLR(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
  SPI_WRITE(addr);
  SPI_WRITE(write);
  SPI_CS_SET(LPS331AP_CS_PORT, LPS331AP_CS_PIN);
}