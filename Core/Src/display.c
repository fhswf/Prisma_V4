/*
  ___          _____     __  __
   _/    _  _ (_  | |\/|  _)  _)
  /__|_||||(_)__) | |  | __) /__

 Fachhochschule Suedwestfalen
 Mechatronik/Mikrocomputer
 Prof. Dr.-Ing. Tobias Ellermeyer

 File: display.c
 Path:  zumolib/src/zumo

 Created:    30.06.2019
 Author:     Tobias Ellermeyer
  Update:	  08.11.2022 - STM32CubeIDE / stdint

*/

/**
 @file display.c
 @code #include "zumo/display.h" @endcode

 @brief High-Level control of 128x64 LVD graphics panel using the u8g2 Library

 Note: The display uses a so called framebuffer. This means all content shown on
       the display is also kept in the microprocessors RAM.
	   Changes are done in the uC RAM and are not immediately visible on the display
	   To submit data to the display, use the command "display_update()"

	   The framebuffer is always transmitted as a whole, i.e. 128x64/8 = 1024 Bytes
	   It is not necessary to issue a "display_update()" after every change.
	   Instead, severeal changes can be "bundled" and then the result is transmitted
	   to the display; this also reduces display flicker...

 @author Tobias Ellermeyer
 */

#include <u8g2.h>
#include "main.h"
#include "logo128x64.h"

#include "display.h"

static u8g2_t display_u8g2;			// Display Handle
extern SPI_HandleTypeDef hspi3;		// handle for the SPI bus


static uint8_t u8x8_stm32_gpio_and_delay (U8X8_UNUSED u8x8_t*,U8X8_UNUSED uint8_t, U8X8_UNUSED uint8_t,U8X8_UNUSED void*);
static uint8_t u8x8_byte_4wire_hw_spi (u8x8_t*, uint8_t, uint8_t , void*);

static uint8_t line_no;		// to which line will the next text be printed

volatile uint8_t DMAcompleted=1;

/**
 * @brief Initialize the display and show the uC-Mechatronik Logo
 *
 * ** IMPORTANT ** This routine is mandatory and must be called
 *                 prior to any other display routine.
 *
 * @params none
 * @return none
 */
void display_init()
{
	u8g2_Setup_sh1106_128x64_noname_f(&display_u8g2, U8G2_R0,	u8x8_byte_4wire_hw_spi,u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&display_u8g2);
	u8g2_SetPowerSave(&display_u8g2, 0);
	u8g2_ClearDisplay(&display_u8g2);

	u8g2_DrawXBM(&display_u8g2, 0, 0, logo128x64_width, logo128x64_height, logo128x64_bits);

	u8g2_UpdateDisplay(&display_u8g2);

	u8g2_SetFont(&display_u8g2, u8g2_font_5x7_mf   );  // Select Font

	line_no=0;
}


/**
 * @brief Clear display (erase contents) and send framebuffer to display
 *
 * Display is cleared immediately
 *
 * @params none
 * @return none
 *
 */
void display_clear()
{
	u8g2_ClearDisplay(&display_u8g2);
	u8g2_UpdateDisplay(&display_u8g2);
	line_no=0;
}

/**
 * @brief Erase framebuffer only, but do not update display
 *
 * I.e. display shows old content until display_update() is executed
 *
 * New image can be prepared "in background" framebuffer and then
 * transferred to display in a whole to reduce flicker and to speed up drawings
 *
 */
void display_clear_f()
{
	u8g2_ClearDisplay(&display_u8g2);
	line_no=0;
}

/**
 * @brief Write string to current line
 *
 * Currently printing always starts at x=0. Line is incremented when print is finished.
 * If required, display is scrolled one line up.
 *
 * Multiple lines in one string are supported, i.e. "\n" starts a new line; display is
 * scrolled if necessary.
 *
 * @param msg String (char*) containing text.
 *
 */
#define __MAX_STING_LEN 50  /// maximum number
void display_println(char* msg)
{
	char *ptr_s;
	char tmp[__MAX_STING_LEN];		// Need to copy string, as it may be in program memory ...
	uint8_t idx;

	ptr_s = msg;
	idx = 0;
	do
	{
		if ((*ptr_s) == '\r') {ptr_s++;}  // Skip \r

		if ( ((*ptr_s) == '\n') || ((*ptr_s)=='\0') ) 	// NewLine or End of String found
		{
			tmp[idx]=0;
			// Draw this part of string to screen, first check if scroll is necessary
			if (line_no>7)  {display_scroll(); }
			  else {line_no++;}

			u8g2_DrawStr(&display_u8g2,0,8*line_no,tmp);
			idx = 0;
		}
		else
		{
			tmp[idx] = (*ptr_s);
			if (idx < (__MAX_STING_LEN-1)) idx++;
		}
	}
	while( (*(ptr_s++)) != '\0' );

	u8g2_UpdateDisplay(&display_u8g2);
}


/**
 * @brief jump to line and column
 *
 * **Important:** at the moment, only line is supported
 *
 * @param x  currently not used/supported
 * @param y  line 0...7 of the display
 *
 */
void display_gotoxy(int16_t x, int16_t y)
{
	if ((y>=0) && (y<8))
	{
		line_no = y;
	}
}

/**
 * @brief Scroll display one line up
 *
 * **Important:** display_update() is not called.
 */
void display_scroll()
{
	// The Display memory is organized as follows:
	// The 8 bits of a Byte form one column.
	// The first 128 Byte are the upper 128x8 pixel
	uint8_t *src, *dst;
	uint16_t i;

	dst=u8g2_GetBufferPtr(&display_u8g2);
	src = dst + 128;	// Next line


	// Copy 7 lines to one line up
	i=128*7;
	do
	{
		*(dst++) = *(src++);
	}
	while ((--i) > 0);

	// Fill last line with zeros
	i=128;
	do
	{
		*(dst++) = 0x00;
	}
	while ((--i) > 0);

}


/**
 * @brief Draw a line
 *
 * **Important:** Draws only in framebuffer, i.e. display_update() is not executed
 *
 * @param x1,y1  starting point (x=0...127; y=0...63)
 * @param x2,y2  end point
 *
 */
void display_line_f(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	u8g2_DrawLine(&display_u8g2, x1, y1, x2, y2);
}

/**
 * @brief Send framebuffer to display
 *
 * Always sends complete framebuffer, i.e. 128x64 pixel = 1024 Bytes
 * which takes several microseconds
 *
 */
void display_update()
{
	u8g2_UpdateDisplay(&display_u8g2);
}

//******************************************************************************
// Internal (static) routines
//******************************************************************************

static uint8_t u8x8_stm32_gpio_and_delay (U8X8_UNUSED u8x8_t *u8x8,U8X8_UNUSED	uint8_t msg, U8X8_UNUSED uint8_t arg_int,U8X8_UNUSED void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
				HAL_Delay(1);
				break;
		case U8X8_MSG_DELAY_MILLI:
				HAL_Delay(arg_int);
				break;
		case U8X8_MSG_GPIO_DC:
				HAL_GPIO_WritePin(DISP_DC_GPIO_Port, DISP_DC_Pin, arg_int);
				break;
		case U8X8_MSG_GPIO_RESET:
			HAL_GPIO_WritePin(DISP_RST_GPIO_Port, DISP_RST_Pin, arg_int);
			HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, 1);
			break;
	}
	return 1;
}

static uint8_t u8x8_byte_4wire_hw_spi (u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,	void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_BYTE_SEND:
				//while (!DMAcompleted);	// Wait for DMA to complete transmission
				DMAcompleted = 0;		// lock for new transmission
				HAL_SPI_Transmit(&hspi3, (uint8_t *) arg_ptr, arg_int,100);
				break;
		case U8X8_MSG_BYTE_INIT:
				break;
		case U8X8_MSG_BYTE_SET_DC:
				//while (!DMAcompleted);	// Wait for DMA to complete transmission
				HAL_GPIO_WritePin(DISP_DC_GPIO_Port, DISP_DC_Pin, arg_int);
				break;
		case U8X8_MSG_BYTE_START_TRANSFER:
				//while (!DMAcompleted);	// Wait for DMA to complete transmission
				HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, 0);
				break;
		case U8X8_MSG_BYTE_END_TRANSFER:
				//while (!DMAcompleted);	// Wait for DMA to complete transmission
				HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, 1);
				break;
		default:
			return 0;
	}
	return 1;
}

void display_free_dma()
{
	DMAcompleted = 1;
//	HAL_GPIO_TogglePin(LED_USR0_GPIO_Port, LED_USR0_Pin);
}


