
#include "contiki.h"
#include "cpu/cc2538/dev/gpio.h"
#include "cpu/cc2538/spi-arch.h"
#include "dev/spi.h"

/*---------------------------------------------------------------------------*/
PROCESS(spi_process, "SPI");
AUTOSTART_PROCESSES(&spi_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(spi_process, ev, data) {

	PROCESS_BEGIN();

	spi_enable();
	spi_cs_init(GPIO_C_NUM, 0);

	while(1) {
		PROCESS_YIELD();
		char * s = "Hello World!";
		int i = 0;
		for(i = 0; i < strlen(s); ++i){
      GPIO_WRITE_PIN(GPIO_C_BASE, 0x01, 0);
			SPI_WRITE(s[i]);
      GPIO_WRITE_PIN(GPIO_C_BASE, 0x01, 1);
		}
	}

	PROCESS_END();
}
