
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
//#include "spiSlave.h"
#include "cpu/cc2538/spi-arch.h"
#include "core/dev/spi.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;

static void spiFIFOcallBack() {
    leds_toggle(LEDS_GREEN);
}

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();

	etimer_set(&periodic_timer_red, CLOCK_SECOND*4);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);

    // spi slave setup

    spi_init();
    spix_cs_init(GPIO_C_NUM, 0);
    SPI_CS_SET(GPIO_C_NUM, 0);
	while(1) {
		PROCESS_YIELD();

		if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		   
            //spix_set_mode(0, SSI_CR0_FRF_MOTOROLA, SSI_CR0_SPO, SSI_CR0_SPH, 8);
 
            SPI_CS_CLR(GPIO_C_NUM, 0);
            SPI_WRITE(0xAF);
            SPI_FLUSH(); 
            SPI_CS_SET(GPIO_C_NUM, 0);

            SPI_CS_CLR(GPIO_C_NUM, 0);
            SPI_WRITE(0xEE);
            SPI_FLUSH();
            SPI_CS_SET(GPIO_C_NUM, 0);
            
            SPI_CS_CLR(GPIO_C_NUM, 0);
            SPI_WRITE(0x39);
            SPI_FLUSH(); 
            SPI_CS_SET(GPIO_C_NUM, 0);

            SPI_CS_CLR(GPIO_C_NUM, 0);
            SPI_WRITE(0x6B);
            SPI_FLUSH();
            SPI_CS_SET(GPIO_C_NUM, 0);
        }
    }
	PROCESS_END();
}
