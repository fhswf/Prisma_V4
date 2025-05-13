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
 *  TIM2  - Timebase for Microsecond measurement, running at 100 kHz -> 10us
 *          32 Bit Timer -> 2^32 / 100 kHz -> Overflow after 11 hours
 *  TIM3  - Motor PWM
 *  TIM6  - Motor RPM calculation
 *  TIM7  - NeoPixel periodic update
 *  TIM8  - NeoPixel PWM/DMA LED
 *  TIM17 - 100Hz control loop (and optionally LED1 dimming)
 */

//Lorem

#include "main.h"
#include "uart.h"
#include "bluetooth.h"
#include <stdio.h>
#include "bldc.h"
#include "foc.h"
#include "display.h"
#include "leds.h"

#define MAIN_C
#include "globals.h"
#include "dwt_delay.h"
#include "adc.h"
extern uint16_t adc_samples[];

extern I2C_HandleTypeDef hi2c1;

void main_bmi(); //read sensor
void foc_test();
uint16_t TouchSense3();	//ok


void main_user(void)
{
	char buf[80];
	int16_t cnt=0, locked=0;
	uint16_t res;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISP_ON_GPIO_Port, DISP_ON_Pin, GPIO_PIN_SET);
	display_init();		// initialize display driver
	printf("UART Initializiation\r\n");
	uart_init();
	HAL_Delay(100);
	printf("UART Initialized done\r\n");
	// Init Bluetooth
	//bt_init();
    //adc_init();

	ledring_init(DMA_NON_BLOCKING);
    ledring_welcome();
    ledring_set_rgb(22,30,30,0);
    ledring_update();


/*    while(1)
    {
//    	display_clear();
		res=TouchSense3();
		if ((res>1800) && (locked ==0))
		{
			locked =1;
			cnt++;
		}
		if (res<1700)
		{
			locked = 0;
		}

    	sprintf(buf,"%5i  %4i   %4i", res, cnt , locked );
    	display_println(buf);

    	HAL_Delay(200);
    }
*/
    foc_init();
    foc_test();

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
