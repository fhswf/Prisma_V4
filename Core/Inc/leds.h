/*
 * Led_user.h
 *
 *  Created on: Apr 4, 2022
 *      Author: Jonathan Lennhoff
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

typedef struct
{
	uint8_t b;				// We need byte order GRB in
	uint8_t r;				// 32 bit integer
	uint8_t g;
	uint8_t stuff_byte;		/// To fill up 32bit integer
} color_t;

#define DMA_NON_BLOCKING   0	// do not wait for old DMA to complete, dismiss update
#define DMA_WAIT    	   1    // wait for DMA to complete, then send new data

#define LED_TMR_DMA  htim2
#define LED_TMR_CH   TIM_CHANNEL_2
#define LED_TMR_UPD  htim7

#define RGB(a,b,c)  {c,a,b}

void ledring_init(uint8_t mode);
void ledring_set_rgb(uint8_t, uint8_t,uint8_t,uint8_t);
void ledring_set_color(uint8_t led_num, color_t color);
color_t ledring_get_color(uint8_t led_num);
void ledring_get_rgb(uint8_t led_num, uint8_t *red, uint8_t *green, uint8_t *blue);
void ledring_set_rng_color(uint8_t start, uint8_t stop, color_t color);

void ledring_black();



void ledring_update(void);
void ledring_welcome();
void ledring_status();




#endif /* INC_LEDS_H_ */
