/*
 * uart_setup.h
 *
 *  Created on: Oct 30, 2023
 *      Author: tobi
 */

#ifndef INC_UART_SETUP_H_
#define INC_UART_SETUP_H_


/**** EDIT THE THREE DEFINES BELOW *****/

//#define USART_NUMBER_USB 2          // Define which USART is used for BT and USB
#define USART_NUMBER_BT  1          // Only number of usart, not full name!
									// If eg. Bluetooth is not used, do not define USART_NUMBER_BT

// #define STM32F1XX				// Enable if STM32F1xx is used; leave undefined for STM§2L47x

/**********************************************************/
/******** DO NOT TOUCH ANYTHING BELOW *********************/
/******** DO NOT TOUCH ANYTHING BELOW *********************/
/******** DO NOT TOUCH ANYTHING BELOW *********************/
/**********************************************************/

#define PPCAT_NX(A, B) A ## B			// String cat
#define PPCAT(A, B) PPCAT_NX(A, B)

#define STRING2(x) #x
#define STRING(x) STRING2(x)

// Needed, as USART is already defined and crashes PPCAT below
#define UART1_ISR USART1_IRQHandler
#define UART2_ISR USART2_IRQHandler
#define UART3_ISR USART3_IRQHandler
#define UART4_ISR USART4_IRQHandler
#define UART5_ISR USART5_IRQHandler


#ifdef USART_NUMBER_USB
#define USART_USB_USED
#define USART_USB_ISR PPCAT(PPCAT(UART,USART_NUMBER_USB),_ISR)	// -> UARTx_ISR
#define USART_USB_HD  PPCAT(huart,USART_NUMBER_USB)			    // huartx
extern UART_HandleTypeDef USART_USB_HD;
UART_HandleTypeDef *puart_usb = &USART_USB_HD;
#endif

#ifdef USART_NUMBER_BT
#define USART_BT_USED
#define USART_BT_ISR PPCAT(PPCAT(UART,USART_NUMBER_BT),_ISR)	   // -> UARTx_ISR
#define USART_BT_HD  PPCAT(huart,USART_NUMBER_BT)					   // huartx
extern UART_HandleTypeDef USART_BT_HD;
UART_HandleTypeDef *puart_bt= &USART_BT_HD;
#endif



#endif /* INC_UART_SETUP_H_ */
