
#include "cpu.h"
#include "nvic.h"
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "spiSlave.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;

static grabdata = 0;


/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
static void spiFIFOcallBack() {
    spix_interrupt_disable(1, SSI_IM_RXIM_M);
    leds_toggle(LEDS_GREEN);
    leds_toggle(LEDS_RED);
    grabdata = 1;
    process_poll(&blink_process);
}


PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();

	//etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND*3);
    
    // spi slave setup
    spix_slave_init(1);

    /*REG(SSI0_BASE + SSI_CR1) = 0;
    REG(SSI0_BASE + SSI_CR0) = 0 | SSI_CR0_FRF_MOTOROLA | SSI_CR0_SPO | SSI_CR0_SPH | 7;
    REG(SSI0_BASE + SSI_CR1) |= SSI_CR1_SSE;
    */

    spix_txdma_enable(1);
    spi_register_callback(1, spiFIFOcallBack);
    spix_interrupt_enable(1, SSI_IM_RXIM_M);
    nvic_interrupt_enable(NVIC_INT_SSI1);
static uint8_t data[32];
static uint8_t spi_data_ptr = 0;
static char buffer[64];
buffer[63] = '\0';
	while(1) {
		PROCESS_YIELD();
        if (etimer_expired(&periodic_timer_blue)) {
            leds_toggle(LEDS_BLUE);
            //etimer_restart(&periodic_timer_blue);
            if (grabdata) {
                spi_data_ptr += spix_get_data(1, data);
                //printf("%x, %x, %x, %x", data+spi_data_ptr-3, data+spi_data_ptr-2, data+spi_data_ptr-1, data+spi_data_ptr-0);
                printf("%x, %x, %x, %x\n", data[0], data[1], data[2], data[3]);
                grabdata = 0;
                spix_interrupt_enable(1, SSI_IM_RXIM_M);
            }
            printf("data_ptr: %d\n", spi_data_ptr);
            /*if(spix_check_rx_fifo_empty(0)) {
                printf("empty\n");
            }
            else {
                static uint8_t dat;
                spix_get_data(0, &dat);
                printf("data: %x\n", dat);
            }*/
        }

		/*if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		} else if (etimer_expired(&periodic_timer_green)) {
			leds_toggle(LEDS_GREEN);
			etimer_restart(&periodic_timer_green);
		} else if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}*/
	}

	PROCESS_END();
}
