/*
 * dwt_delay.h
 *
 *  Created on: Mar 25, 2025
 *      Author: tobi
 */

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#include "main.h"

uint32_t dwt_delay_init(void);

// This Function Provides Delay In Microseconds Using DWT

__STATIC_INLINE void dwt_delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

// This Function Provides Delay In Milliseconds Using DWT

__STATIC_INLINE void dwt_delay_ms(volatile uint32_t au32_milliseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}


#endif /* INC_DWT_DELAY_H_ */
