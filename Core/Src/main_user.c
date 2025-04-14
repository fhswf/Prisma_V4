/************************************************************************************
 *
 *      oooooooooooooo          **
 *      o   .---.    o         ****   ***
 *      o   | * |   oo         ****   ***
 *       oo '---' oo           ****
 *         oo   oo              **           Fachhochschule Suedwestfalen
 *           ooo                **           Mechatronik / Mikrocomputer / EmbSys
 *            o                              (c) Prof. Tobias Ellermeyer
 *    ==================        **
 *     BALANCING PRISMA         **
 *
 *
 * main_user.c
 *
 *  Created on: Mar 25, 2025
 *      Author: tobi
 *      Corrections and further improvements: EMP & EDEG
 */


/***** TIMER Usage *****
 *
 *  TIM1  - Motor PWM
 *  TIM3  - Motor PWM
 *  TIM6  - Motor RPM calculation
 *  TIM7  - NeoPixel periodic update
 *  TIM8  - NeoPixel PWM/DMA LED
 *  TIM17 - 100Hz control loop (and optionally LED1 dimming)
 */

#include "main.h"
#include "uart.h"
#include "bluetooth.h"
#include <stdio.h>
#include "bldc.h"
#include "display.h"
#include "leds.h"

#define MAIN_C
#include "globals.h"
#include "dwt_delay.h"

extern I2C_HandleTypeDef hi2c1;

void main_bmi(); //read sensor

void main_user(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISP_ON_GPIO_Port, DISP_ON_Pin, GPIO_PIN_SET);
	display_init();		// initialize display driver
	printf("UART Initializiation\r\n");
	uart_init();
	HAL_Delay(100);
	printf("UART Initialized done\r\n");
	// Init Bluetooth
	//bt_init();

	//bldc_init();
	ledring_init(DMA_NON_BLOCKING);
    ledring_welcome();

    main_bmi();

    while(1)
	{
		//ledring_set_rgb(4,125,52,148);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_Delay(500);
	}
	bldc_init();

	while(1)
	{

	}
}
