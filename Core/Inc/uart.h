/*
  ___          _____     __  __
   _/    _  _ (_  | |\/|  _)  _)
  /__|_||||(_)__) | |  | __) /__

 Fachhochschule S�dwestfalen
 Mechatronik/Mikrocomputer
 Prof. Dr.-Ing. Tobias Ellermeyer

 Datei: uart_printf.c
 Pfad:  zumolib/src/zumo

 Erstellt:    02.03.2020
 Autor:       Tobias Ellermeyer

 see the following web page for details of IRQ handling:
 https://stackoverflow.com/questions/37336527/why-does-uart-transmit-interrupt-fail-to-work-in-this-case
*/

#ifndef INC_UART_H_
#define INC_UART_H_


// Define Buffer Sizes
#define RXBUF_LEN            128 // must be power of 2
#define TXBUF_LEN            128 // must be power of 2
#define RXBUF_MSK            (RXBUF_LEN-1)
#define TXBUF_MSK            (TXBUF_LEN-1)

#define UART_EMPTY 		0

void uart_init();
void uart_usb_putc(uint8_t);
uint8_t uart_usb_getc();

void uart_printf_to_bt(uint8_t);

void uart_bt_putc(uint8_t);
uint8_t uart_bt_getc();
void uart_bt_baudrate(uint32_t);
uint8_t uart_bt_tx_done();

#endif
