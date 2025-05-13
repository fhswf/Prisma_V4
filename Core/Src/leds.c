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
 *
 * Led_user.c
 *
 *  Created on: Apr 4, 2022
 *      see also: https://controllerstech.com/interface-ws2812-with-stm32/
 *
 */

#include "main.h"
#include "globals.h"
#include "leds.h"
#include <stdio.h>

#define PI 3.14159265					t

#define SGN(x) ((x>0)-(x<0))

extern TIM_HandleTypeDef LED_TMR_UPD, LED_TMR_DMA;

static volatile int datasentflag=1;				// is DMA transfer completed (1=yes)
static uint16_t pwm_data[(24*LEDRING_CNT)+50];	// Array with PWM values for DMA transfer
static color_t led_data[LEDRING_CNT];		    // Array with LED data
static uint8_t dma_mode = DMA_NON_BLOCKING;

void ledring_init(uint8_t mode)
{
	for (int i=0;i<24*LEDRING_CNT+50;i++)
	{
		pwm_data[i]=0;
	}
	ledring_black();

	dma_mode = mode;

	// Start timer for periodic update of LED Status
	//HAL_TIM_Base_Start_IT(&htim4);
}

/************************************************
 * Switch all LEDs of the ring off
 ************************************************/
void ledring_black()
{
	for (int i=0;i<LEDRING_CNT;i++)
	{
		led_data[i].r=0;
		led_data[i].g=0;
		led_data[i].b=0;
	}
}

/**************************************************
 * Welcome animation
 *************************************************/
void ledring_welcome()
{
	HAL_TIM_Base_Stop_IT(&LED_TMR_DMA);
	for (int i=0;i<24;i++)
	{
		ledring_set_rgb(i,i,24-i,0);
		ledring_update();
		HAL_Delay(20);
	}
	for (int i=0;i<24;i++)
	{
		ledring_set_rgb(i,24-i,0,i);
		ledring_update();
		HAL_Delay(2);
	}
	for (int i=0;i<24;i++)
	{
		ledring_set_rgb(i,0,0,0);
		ledring_update();
		HAL_Delay(2);
	}
	//HAL_TIM_Base_Start_IT(&htim4);
}


/****************************************************
 * Set one led to the specified color (rgb values)
 ***************************************************/
void ledring_set_rgb(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue)
{
	if (led_num<LEDRING_CNT)
	{
		led_data[led_num].r = red;
		led_data[led_num].g = green;
		led_data[led_num].b = blue;
	}
}

/****************************************************
 * Set one led to the specified color (color_t)
 ***************************************************/
void ledring_set_color(uint8_t led_num, color_t color)
{
	if (led_num<LEDRING_CNT)
	{
		led_data[led_num].r = color.r;
		led_data[led_num].g = color.g;
		led_data[led_num].b = color.b;
	}
}

/****************************************************
 * Get led color (color_t)
 ***************************************************/
color_t ledring_get_color(uint8_t led_num)
{
	color_t color={0,0,0};
	if (led_num<LEDRING_CNT)
	{
		color.r = led_data[led_num].r;
		color.g = led_data[led_num].g;
		color.b = led_data[led_num].b;
	}
	return color;
}

/****************************************************
 * Get led color (rgb)
 ***************************************************/
void ledring_get_rgb(uint8_t led_num, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	if (led_num<LEDRING_CNT)
	{
		(*red)=led_data[led_num].r;
		(*green)=led_data[led_num].g;
		(*blue)=led_data[led_num].b;
	}
}

/****************************************************
 * Set color for multiple leds (start...stop) (color_t)
 ***************************************************/
void ledring_set_rng_color(uint8_t start, uint8_t stop, color_t color)
{
	if (start>=LEDRING_CNT) return;
	if (stop>=LEDRING_CNT) return;
	if (start>stop)	// swap if necessary
	{
		int16_t h;
		h=start; start=stop; stop=h;
	}
	for (int i=start;i<=stop;i++)
	{
		led_data[i] = color;
	}
}

color_t color_brake = RGB(32,0,0);
color_t color_accel = RGB(16,0,16);
color_t color_angle = RGB(32,32,0);
color_t color_bat   = RGB(24,8,0);
color_t color_green = RGB(0,32,0);
color_t color_bt_ok   = RGB(0,16,16);
color_t color_bt_init = RGB(0,0,8);





void ledring_status()
{
	int16_t pixels;
	int16_t dx;

	ledring_black();		// Delete old pattern

	// Show brake/accel force
	if (force_brake_g)
	{
		pixels = force_brake_g / 20;
		dx = SGN(force_brake_g);
		if (pixels)
		{
			ledring_set_rng_color(6,6+(pixels-dx),color_brake);
		}
	}
	else if (force_accel_g)
	{
		pixels = force_accel_g / 20;
		dx = SGN(force_accel_g);
		if (pixels)
		{
			ledring_set_rng_color(6,6+(pixels-dx),color_accel);
		}
	}

	// show angle
	ledring_set_color(18+angle_g/15,color_angle);

	// set battery
	ledring_set_rng_color(14,14-vbat_mv_g/2500,color_bat);

	// BT status
	if (bt_ok_g == BT_INITIALIZING)
	{ ledring_set_color(0,color_bt_init); }
	if (bt_ok_g == BT_READY)
	{ ledring_set_color(0,color_bt_ok); }

	ledring_update();
}

/*****************************************************************************
 * UPDATE pwm_data and trigger sending them to the Neopixel ring
 *****************************************************************************/
void ledring_update(void)
{
	uint32_t *color;
	uint32_t idx = 0;

	// if non-blocking and dma not completed -> dismiss
	if ( (datasentflag==0) && (dma_mode == DMA_NON_BLOCKING) ) {
		return;}

	// if previous transfer is still ongoing -> wait for completion
	while (!datasentflag){};

	// uint32 pointer to struct (we arranged the struct that way)
	color = (uint32_t*)&led_data;

	for (int i=0;i<LEDRING_CNT;i++)
	{
		for (uint32_t i=0x00800000; i!=0; i>>=1) 		// step through all 24 bits of color value
		{
			pwm_data[idx] = ( (*color)&i?127:64);		// if bit is set, use pwm=60 otherwise 30
			idx++;	// next pwm array element
		}
		color++;	// next led
	}

	// start DMA transfer -> LED_TMR_HAL_START_DMA is defined in leds.h depending on usage of CH or CHN
	LED_TMR_HAL_START_DMA(&LED_TMR_DMA, LED_TMR_CH, (uint32_t *)pwm_data, 24*LEDRING_CNT+50);
	datasentflag = 0;  // mark transfer ongoing
}


//Funktion die nach senden der Bytes aufgerufen wird
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	// stop DMA transfer -> LED_TMR_HAL_STOP_DMA is defined in leds.h depending on usage of CH or CHN
	if (htim == &LED_TMR_DMA)
	{
		LED_TMR_HAL_STOP_DMA(htim, LED_TMR_CH);
		datasentflag=1;									// set to 1 to indicate that data were send

	}
}

// TIM4 ISR for periodic update of led status
void TIM7_IRQHandler(void)
{
	if (datasentflag==0) return; 	// dont wait for transfer to complete
	ledring_status();
}
