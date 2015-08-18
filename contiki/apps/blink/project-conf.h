#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * @author: Brad Campbell <bradjc@umich.edu>
 */

// #define UIP_CONF_ICMP6 0

// #define UIP_CONF_TCP 0

// #define UIP_CONF_UDP 0

#define PROCESS_CONF_NO_PROCESS_NAMES 1

// No need for UART
#define STARTUP_CONF_VERBOSE 0
#define UART_CONF_ENABLE 1
#define SPI_CONF_DEFAULT_INSTANCE 0

 //#define CC2538_CONF_QUIET 0
#define CC2538_RF_CONF_CHANNEL	22

#define ENERGEST_CONF_ON 1

#define NO_CLOCK_DIVIDER_RESTORE 1

#define VTIMER_ENABLE 1


#endif /* PROJECT_CONF_H_ */

/** @} */
