/*
 * captouch.c
 *
 *  Created on: May 13, 2025
 *      Author: tobi
 */

#include "main.h"
#include "display.h"
#include <stdio.h>

#define CAL_RUNS 32

#define CAP_TRG_LVL 30		// Changes more than this indicate touch/release
#define CAP_FILT    0.1  	// Low pass reference adjust filter

#define BTN_RISE  0b001
#define BTN_TOUCH 0b010
#define BTN_FALL  0b100
#define BTN_MASK  0b111

#define ABS(x) ( (x<0)?(-x):x)
static uint16_t btn_state=0;

extern ADC_HandleTypeDef hadc1;

static int16_t cap_buf[6];
static int16_t cap_ref[3];
static int16_t adc_buf[2];

void ADC1_ReInit();

static void captouch_precharge(uint8_t btn);
static void captouch_sample(uint8_t btn);
static void captouch_read(uint8_t btn);

const uint32_t btn_map[3] = {BTN0_Pin, BTN1_Pin, BTN2_Pin};
const uint32_t adc_ch_map[3] = {ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3};
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

	/*Configure GPIO pins : CapTouch_Second pin and set it to VDD */
    HAL_GPIO_WritePin(TOUCHREF_GPIO_Port, TOUCHREF_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = TOUCHREF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TOUCHREF_GPIO_Port, &GPIO_InitStruct);

	//GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
/*
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	//HAL_GPIO_Init(TOUCHREF_GPIO_Port, &GPIO_InitStruct);
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		return;
	}*/
//	HAL_ADC_Start(&hadc1);

	/*Configure GPIO pins : CapTouch and ground the pin*/
    HAL_GPIO_WritePin(BTN0_GPIO_Port, btn_map[btn], GPIO_PIN_SET);
	GPIO_InitStruct.Pin = btn_map[btn];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN0_GPIO_Port, &GPIO_InitStruct);
}

void captouch_sample(uint8_t btn)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	//cap_buf[btn+3] = HAL_ADC_GetValue(&hadc1);

	GPIO_InitStruct.Pin = btn_map[btn];
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	//HAL_GPIO_Init(BTN0_GPIO_Port, &GPIO_InitStruct);

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	*/
	sConfig.Channel = adc_ch_map[btn];
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_Init(BTN0_GPIO_Port, &GPIO_InitStruct);

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		return;
	}

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,2);

}

static void captouch_read(uint8_t btn)
{
	int16_t diff;
	int16_t dlast;
	uint8_t sh;

	sh = btn*3;

	diff = adc_buf[1]-cap_ref[btn];
	dlast = adc_buf[1]-cap_buf[btn];
	if ( ABS(diff) < CAP_TRG_LVL/2 )  // No press detected
	{
		// Automatic Low Pass Ref Leveladjust
		cap_ref[btn] = (float)cap_ref[btn]*(1.-CAP_FILT) + (float)cap_buf[btn]*CAP_FILT;
	}
	else
	{
		if ( dlast > CAP_TRG_LVL )	// Rising
		{
			btn_state |= (BTN_RISE<<sh);
		}
		else if ( dlast < -CAP_TRG_LVL ) // Falling
		{
			btn_state |=  (BTN_FALL<<sh);
		}
		else if ( diff > CAP_TRG_LVL )
		{
			btn_state |=  (BTN_TOUCH<<sh);
		}
		else
		{
			btn_state &= (BTN_MASK<<sh);	// Delete old state
		}
	}
	cap_buf[btn] = adc_buf[1];	// Store state for edge detection
}

void captouch_cal()
{
	int32_t val0=0,val1=0,val2=0;
	int16_t i;
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_Delay(10);
	for (i=0;i<CAL_RUNS;i++)
	{
		for(int8_t j=0;j<9;j++)
		{
			captouch_sequencer();
			HAL_Delay(10);
		}
		val0 += cap_buf[0];
		val1 += cap_buf[1];
		val2 += cap_buf[2];
	}
	cap_ref[0] = (int16_t)(val0/CAL_RUNS);
	cap_ref[1] = (int16_t)(val1/CAL_RUNS);
	cap_ref[2] = (int16_t)(val2/CAL_RUNS);

}

void captouch_test()
{
	char buf[60];
	captouch_cal();		//  000 001 010  011  100  101  110  111
	const char btn_sn[8] = {'.','^','o', '3', 'v', '5', '6', '7'};
	while(1)
	{
		for(int i=0;i<9;i++)
		{
			captouch_sequencer();
			HAL_Delay(1);
		}
		display_clear();
		display_println("CAPTOUCH");
		sprintf(buf,"%4i %4i %4i",cap_buf[0],cap_buf[1],cap_buf[2]);
		display_println(buf);
		sprintf(buf,"%c %c %c", btn_sn[btn_state & 0x03], btn_sn[btn_state>>2 & 0x03], btn_sn[btn_state>>4 & 0x03] );
		display_println(buf);
		sprintf(buf,"%4i %4i %4i",cap_ref[0],cap_ref[1],cap_ref[2]);
		display_println(buf);
		sprintf(buf,"%4i %4i %4i",cap_buf[0]-cap_ref[0],cap_buf[1]-cap_ref[1],cap_buf[2]-cap_ref[2]);
		display_println(buf);
		HAL_Delay(100);
	}
}
