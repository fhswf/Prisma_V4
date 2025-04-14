/*
 * bluetooth.h
 *
 *  Created on: 02.03.2020
 *      Author: Tobias Ellermeyer
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include <stdint.h>

#define BT_BAUDRATE  	 115200     // Baud rate in transmit mode (to be configured)
#define BT_BAUDRATE_CMD  38400		// Baud rate in CMD mode (see spec)
#define BT_SUFFIX    "HC05"


uint8_t bt_init();
uint8_t bt_send(char* buf);
uint8_t bt_receive(char *recv, uint8_t maxlen);
uint8_t bt_program(char *name);
char *bt_getname();



#endif /* INC_ZUMO_BLUETOOTH_H_ */
