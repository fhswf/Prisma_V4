/*
 * cdc_printf.c
 *
 *  Created on: Mar 26, 2025
 *      Author: tobi
 */

#include "main.h"
#include "usbd_cdc_if.h"

#define CDC_BLOCKING

int _write(int file, char *ptr, int len) {
    static uint8_t rc = USBD_OK;
    uint16_t timeout = 300;	/* milliseconds */
    static uint8_t was_timeout = 0;

    if (HAL_GPIO_ReadPin(USB_PWRD_GPIO_Port, USB_PWRD_Pin) == 0) // No USB connected
    {
    	return 0;
    }
#ifdef CDC_BLOCKING
    do {
        rc = CDC_Transmit_FS((unsigned char*)ptr, len);
        if (was_timeout) return 0;
        HAL_Delay(1);
        timeout--;
    } while ( (USBD_BUSY == rc) && (timeout>0) );

    if (timeout == 0)
    {
    	was_timeout = 1;
    	return 0;
    }
    was_timeout = 0;
    if (USBD_FAIL == rc) {
        return 0;
    }
#else
    CDC_Transmit_FS((unsigned char*)ptr, len);
#endif
    return len;
}
