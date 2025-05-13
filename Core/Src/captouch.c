/*
 * captouch.c
 *
 *  Created on: May 13, 2025
 *      Author: tobi
 */

#include "main.h"
#include "display.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;

static int16_t cap_buf[6];
void ADC1_ReInit();

static void captouch_precharge(uint8_t btn);
static void captouch_sample(uint8_t btn);
static void captouch_read(uint8_t btn);

void captouch_sequencer()
{
	static uint8_t state = 0;

	switch(state)
	{
		case 0: captouch_precharge(0);
				break;
		case 1: captouch_sample(0);
				break;
		case 2: captouch_read(0);
				break;
		case 3: captouch_precharge(1);
				break;
		case 4: captouch_sample(1);
				break;
		case 5: captouch_read(1);
				break;
		case 6: captouch_precharge(2);
				break;
		case 7: captouch_sample(2);
				break;
		case 8: captouch_read(2);
				break;
		default: break;
	}

	state = (state+1)%9;
}

static void captouch_precharge(uint8_t btn)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/*Configure GPIO pins : CapTouch_Second pin and set it to VDD */
    HAL_GPIO_WritePin(TOUCHREF_GPIO_Port, TOUCHREF_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = TOUCHREF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TOUCHREF_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TOUCHREF_Pin;

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		return;
	}
	HAL_ADC_Start(&hadc1);

	/*Configure GPIO pins : CapTouch and ground the pin*/
    HAL_GPIO_WritePin(BTN0_GPIO_Port, (1<<btn), GPIO_PIN_SET);
	GPIO_InitStruct.Pin = (1<<btn);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN0_GPIO_Port, &GPIO_InitStruct);
}

void captouch_sample(uint8_t btn)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	cap_buf[btn+3] = HAL_ADC_GetValue(&hadc1);

	GPIO_InitStruct.Pin = (1<<btn);
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN0_GPIO_Port, &GPIO_InitStruct);

	/* Point ADC to sensor channel*/
	switch(btn)
	{
		case 0: sConfig.Channel = ADC_CHANNEL_1;
				break;
		case 1: sConfig.Channel = ADC_CHANNEL_2;
				break;
		case 2: sConfig.Channel = ADC_CHANNEL_3;
				break;
		default: return; break;
	}
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		return;
	}

	HAL_ADC_Start(&hadc1);

}

static void captouch_read(uint8_t btn)
{
	cap_buf[btn] = HAL_ADC_GetValue(&hadc1);
}

void captouch_test()
{
	char buf[60];
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	while(1)
	{
		for(int i=0;i<9;i++)
		{
			captouch_sequencer();
			HAL_Delay(10);
		}
		display_clear();
		display_println("CAPTOUCH");
		sprintf(buf,"%4i %4i %4i",cap_buf[0],cap_buf[1],cap_buf[2]);
		display_println(buf);
		sprintf(buf,"%4i %4i %4i",cap_buf[3],cap_buf[4],cap_buf[5]);
		display_println(buf);
		HAL_Delay(1000);
	}
}
