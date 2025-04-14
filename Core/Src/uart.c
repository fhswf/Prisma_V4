/*
  ___          _____     __  __
   _/    _  _ (_  | |\/|  _)  _)
  /__|_||||(_)__) | |  | __) /__

 Fachhochschule S�dwestfalen
 Mechatronik/Mikrocomputer
 Prof. Dr.-Ing. Tobias Ellermeyer

 Datei: uart_printf.c
 Pfad:  zumolib/src/zumo

 Erstellt:    23.09.2019
 Autor:       Tobias Ellermeyer

 see the following web page for details of IRQ handling:
 https://stackoverflow.com/questions/37336527/why-does-uart-transmit-interrupt-fail-to-work-in-this-case
*/


#include "main.h"
#include <stdio.h>
#include "uart_setup.h"
#include "uart.h"


#ifdef USART_USB_USED
static uint8_t rx1_buf[RXBUF_LEN];		// Receive Buffer UART1
static uint8_t tx1_buf[TXBUF_LEN];		// Transmit Buffer UART1
static volatile uint16_t rx1_i, tx1_o;
static uint16_t rx1_e, tx1_i;
static volatile uint8_t tx1_busy;
static volatile uint8_t tx1_cnt;
#endif

#ifdef USART_BT_USED
static uint8_t rx2_buf[RXBUF_LEN];		// Receive Buffer UART2
static uint8_t tx2_buf[TXBUF_LEN];		// Transmit Buffer UART2
static volatile uint16_t rx2_i, tx2_o;
static uint16_t rx2_e, tx2_i;
static volatile uint8_t tx2_busy;
static volatile uint8_t tx2_cnt;
#endif


static uint8_t echo_to_bt=0;			// Echo printf to bluetooth




void uart_init()
{
	// Enable the Interrupts
	#ifdef USART_USB_USED
	tx1_i  = 0;
	rx1_i  = 0;
	rx1_e  = 0;
	tx1_busy = 0;
	tx1_cnt = 0;
	__HAL_UART_ENABLE_IT(puart_usb, UART_IT_RXNE);
	#endif
	#ifdef USART_BT_USED
	tx2_i = 0;
	rx2_i = 0;
	rx2_e = 0;
	tx2_busy = 0;
	tx2_cnt = 0;
	__HAL_UART_ENABLE_IT(puart_bt, UART_IT_RXNE);
	#endif
}

void uart_printf_to_bt(uint8_t echo)
{
#ifdef USART_BT_USED
	echo_to_bt = echo;
#endif
}
/*********************************************************
 * printf to usb if usb is defined
 ********************************************************/
#ifdef USART_USB_USED
int __io_putchar(int ch)
{

	uart_usb_putc(ch);
	if (echo_to_bt)
	{
		uart_bt_putc(ch);
	}
	return ch;
}

int _write(int file,char *ptr, int len)
{
	int i;

	for(i = 0; i < len; i++)
	{
		if (echo_to_bt)
		{
			uart_bt_putc(*ptr);
		}
		uart_usb_putc( *(ptr++) );
	}

	return len;
}
#endif

/*********************************************************
 *
 *	USB Routines tx_buf etc
 *
 *********************************************************/
#ifdef USART_USB_USED

void uart_usb_putc(uint8_t byte)
{
	// Wait until buffer has free space
	while (tx1_cnt > (TXBUF_LEN-1));

    tx1_buf[TXBUF_MSK & tx1_i] = byte;
    tx1_i++;
    tx1_cnt++;
    tx1_busy = 1;
    __HAL_UART_ENABLE_IT(puart_usb, UART_IT_TXE);
}

uint8_t uart_usb_getc()
{
	uint8_t c;
	if (rx1_i == rx1_e)  // No chars received
	{
		c = UART_EMPTY;
	}
	else		// There are chars in ringbuffer
	{
		c = rx1_buf[RXBUF_MSK & rx1_e];
		rx1_e++;
	}
	return c;
}

#endif

/*********************************************************
 *
 * Bluetooth Routines tx_buf etc
 *
 *********************************************************/
#ifdef USART_BT_USED
void uart_bt_putc(uint8_t byte)
{
	// Wait until buffer has free space
	while (tx2_cnt > (TXBUF_LEN-1));

    tx2_buf[TXBUF_MSK & tx2_i] = byte;
    tx2_i++;
    tx2_cnt++;
    tx2_busy = 1;
    //HAL_UART_Transmit(puart_bt, &byte, 1, 0xFFFF);
    __HAL_UART_ENABLE_IT(puart_bt, UART_IT_TXE);
}

uint8_t uart_bt_getc()
{
	uint8_t c;
	if (rx2_i == rx2_e)  // No chars received
	{
		c = UART_EMPTY;
	}
	else		// There are chars in ringbuffer
	{
		c = rx2_buf[RXBUF_MSK & rx2_e];
		rx2_e++;
	}
	return c;
}

uint8_t uart_bt_tx_done()
{
	return (tx2_busy == 0);
}
#endif

#ifdef USART_USB_USED
void USART_USB_ISR(void)			// Defined in uart_setup
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    if((__HAL_UART_GET_FLAG(puart_usb, UART_FLAG_RXNE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_usb, UART_IT_RXNE) != RESET))
    {
#ifdef STM32F1XX
        rx1_buf[rx1_i & RXBUF_MSK] = (uint8_t)(puart_usb->Instance->DR & 0x00FF);
#else
        rx1_buf[rx1_i & RXBUF_MSK] = (uint8_t)(puart_usb->Instance->RDR & 0x00FF);
#endif
    	rx1_i++;
        /* Clear RXNE interrupt flag */
        //__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
        __HAL_UART_CLEAR_NEFLAG(puart_usb);
    }
    if((__HAL_UART_GET_FLAG(puart_usb, UART_FLAG_TXE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_usb, UART_IT_TXE) != RESET))
    {
        if (tx1_i == tx1_o) {
            __HAL_UART_DISABLE_IT(puart_usb, UART_IT_TXE);
            __HAL_UART_ENABLE_IT(puart_usb, UART_IT_TC);
            tx1_cnt = 0;
        } else {
#ifdef STM32F1XX
        	puart_usb->Instance->DR = (uint8_t)(tx1_buf[TXBUF_MSK & tx1_o] & (uint8_t)0xFF);
#else
        	puart_usb->Instance->TDR = (uint8_t)(tx1_buf[TXBUF_MSK & tx1_o] & (uint8_t)0xFF);
#endif
            tx1_o++;
            tx1_cnt--;
        }
    }
    if((__HAL_UART_GET_FLAG(puart_usb, UART_FLAG_TC) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_usb, UART_IT_TC) != RESET))
    {
        tx1_busy = 0;
        __HAL_UART_DISABLE_IT(puart_usb, UART_IT_TC);
    }
    /* And never call default handler */
    return;
}
#endif


#ifdef USART_BT_USED
void USART_BT_ISR(void)     // Defined in uart_setup
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    if((__HAL_UART_GET_FLAG(puart_bt, UART_FLAG_RXNE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_bt, UART_IT_RXNE) != RESET))
    {
#ifdef STM32F1XX
        rx2_buf[rx2_i & RXBUF_MSK] = (uint8_t)(puart_bt->Instance->RD & 0x00FF);
#else
        rx2_buf[rx2_i & RXBUF_MSK] = (uint8_t)(puart_bt->Instance->RDR & 0x00FF);
#endif
        rx2_i++;
        /* Clear RXNE interrupt flag */
        //__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
        __HAL_UART_CLEAR_NEFLAG(puart_bt);
    }
    if((__HAL_UART_GET_FLAG(puart_bt, UART_FLAG_TXE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_bt, UART_IT_TXE) != RESET))
    {
        if (tx2_i == tx2_o) {
            __HAL_UART_DISABLE_IT(puart_bt, UART_IT_TXE);
            __HAL_UART_ENABLE_IT(puart_bt, UART_IT_TC);
            tx2_cnt = 0;
        } else {
#ifdef STM32F1XX
        	puart_bt->Instance->DR = (uint8_t)(tx2_buf[TXBUF_MSK & tx2_o] & (uint8_t)0xFF);
#else
        	puart_bt->Instance->TDR = (uint8_t)(tx2_buf[TXBUF_MSK & tx2_o] & (uint8_t)0xFF);
#endif
            tx2_o++;
            tx2_cnt--;
        }
    }
    if((__HAL_UART_GET_FLAG(puart_bt, UART_FLAG_TC) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(puart_bt, UART_IT_TC) != RESET))
    {
        tx2_busy = 0;
        __HAL_UART_DISABLE_IT(puart_bt, UART_IT_TC);
    }
    /* And never call default handler */
    return;
}

void uart_bt_baudrate(uint32_t br)
{
	puart_bt->Init.BaudRate=br;
	if (HAL_UART_Init(puart_bt) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
#endif
