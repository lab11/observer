#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// No need for UART
#define STARTUP_CONF_VERBOSE 0
#define UART_CONF_ENABLE 1
#define SPI_CONF_DEFAULT_INSTANCE 0

#define SPI0_CONF_CPRS_CPSDVSR 8 // divide clock by 8 so 16MHz/8MHz = 2MHz spi clock
#define SPI1_CONF_CPRS_CPSDVSR 8

#endif /* PROJECT_CONF_H_ */

/** @} */
