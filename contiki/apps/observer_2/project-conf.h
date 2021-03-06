#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// No need for UART
#define STARTUP_CONF_VERBOSE 0
#define UART_CONF_ENABLE 0
#define SPI_CONF_DEFAULT_INSTANCE 0

#define CC2538_RF_CONF_CHANNEL 22
#define CC2538_RF_CONF_TX_POWER 0xF5

#define PROCESS_CONF_NO_PROCESS_NAMES 1

#endif /* PROJECT_CONF_H_ */

/** @} */
