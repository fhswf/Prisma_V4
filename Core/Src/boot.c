/*****************************************************
 *
 *   **
 *  ****   ***
 *  ****   ***
 *  ****
 *   **           Fachhochschule Suedwestfalen
 *   **           Mechatronik / Mikrocomputer / EmbSys
 *                (c) Prof. Tobias Ellermeyer
 *   **
 *   **
 *
 *------------------------------------------------------
 *
 *  boot.c
 *
 *  Jump to bootloader with "double-click" reset
 *
 *  Created on: Jun 12, 2024
 *      Author: tobi
 *
 *  2025-03-25: Updated to STM32G431, added USB re-enumeration
 *****************************************************/

#include "main.h"
#include "dwt_delay.h"

#define BOOTLOADER_FLAG_VALUE  0xDEADBEEF   // Value to identify bootloader request
#define BOOTLOADER_FLAG_OFFSET 500			// Offset from top of RAM (within stack)
#define BOOTLOADER_ADDRESS     0x1FFF0000   // Start address of bootloader -> see ST documentation
#define BOOTLOADER_WAIT        1000         // "Doubleclick"-time in ms
                                            //    -> repeated Reset within this time will start bootloader

extern int _estack;					  // Linker constant - End of stack (highest address) -> stack starts from here downwards
static void (*execBootloader)();      // Function pointer -> jump to bootloader
static uint32_t *bootloader_flag;     // Flag to memorize if doubleclick happened
static void MX_GPIO_Init_boot(void);  // GPIO init for re-enumeration of USB

// USB Pins for Forcing enumeration by assigning DP=0
#define USB_DP_0_Pin GPIO_PIN_12
#define USB_DP_0_GPIO_Port GPIOA
#define USB_DM_Z_Pin GPIO_PIN_11
#define USB_DM_Z_GPIO_Port GPIOA

/*******************************************************
 * boot_stage1
 *
 * - needs to be called first in main()
 * - checks if boot flag is set end jumps to bootloader
 *   if so...
 *
 ******************************************************/
void boot_stage1()
{
	uint32_t jumpAddress=0;

	bootloader_flag = (uint32_t*)(&_estack - BOOTLOADER_FLAG_OFFSET);

	if ( (*bootloader_flag) == BOOTLOADER_FLAG_VALUE)
	{
		*bootloader_flag = 0;	// Reset flag
		jumpAddress = *(volatile uint32_t*)(BOOTLOADER_ADDRESS+4);
		(execBootloader) = (void*) jumpAddress;
		execBootloader();	// Jump to Bootloader
	}
}

/******************************************************
 * boot_stage2
 *
 * - called from main after everything is initialized
 *   (i.e. before going to main_user)
 * - sets bootflag, then waits for BOOTLOADER_WAIT millis
 * - if reset occurs within this time, uC will execute
 *   bootloader
 * - otherwise bootflag is reset -> next reset will be
 *   "normal" reset
 ******************************************************/
void boot_stage2()
{
	// Next two lines to avoid jumping into bootloader due to
	// Power On Reset ... (i.e. plugging in USB cable)
	*bootloader_flag = 0;
	HAL_Delay(10);
	dwt_delay_init();
	// Init GPIO for bootloader only
	MX_GPIO_Init_boot();

	// Continue with normal start if no USB cable plugged
	if (HAL_GPIO_ReadPin(USB_PWRD_GPIO_Port, USB_PWRD_Pin)==0) return;


	// Now set BOOT Flag
	*bootloader_flag = BOOTLOADER_FLAG_VALUE;

	// Blink fast while waiting ...
	// If Reset is pressed within this for loop, bootloader will be startet on next run...
	for (uint16_t i=0;i<(BOOTLOADER_WAIT/100);i++)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(50);
	}

	// No reset within Wait time -> ensure next reset is a "normal" one
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	*bootloader_flag = 0;
}


/***************************************************
 * Initialize USB and LED GPIO
 *
 * USB_DP -> Low to force enumeration
 * USB_DM -> TriState / Input
 *
 **************************************************/
static void MX_GPIO_Init_boot(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_DP_0_GPIO_Port, USB_DP_0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USB_DM_Z_Pin */
	GPIO_InitStruct.Pin = USB_DM_Z_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_DM_Z_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_DP_0_Pin */
	GPIO_InitStruct.Pin = USB_DP_0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_DP_0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PWRD_Pin */
	GPIO_InitStruct.Pin = USB_PWRD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(USB_PWRD_GPIO_Port, &GPIO_InitStruct);

}

/******************************************************
 *
 * Send S.O.S.
 *
 * -> Dont use HAL_Delay, since IRQs are stopped...
 *
 *****************************************************/
void sos_error()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	//  0 ...  5: 3x short   6 ... 11: 3x long    12 ... 17: 3x short
	for (uint8_t i=0;i<18;i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		dwt_delay_ms(100);
		if ( (i>5) && (i<12) )
		{
			dwt_delay_ms(200);
		}
	}
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	dwt_delay_ms(2000);
}
