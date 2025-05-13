/*
 * foc_filters.h
 *
 *  Created on: Apr 30, 2025
 *      Author: tobi
 */

#ifndef INC_FOC_FILTERS_H_
#define INC_FOC_FILTERS_H_

#include <stdint.h>

typedef struct
{
	float Tf;
	float y_prev;
	uint32_t timestamp_prev;
} lp_filter_t;

typedef struct
{
	float KP, KI, KD;
	float lim_slew;
	float lim_out;
	float err_prev;
	float int_prev;
	float out_prev;
	uint32_t timestamp_prev;
} pid_controller_t;


float lp_filter(float x, lp_filter_t *filt);
float pid_control(float error, pid_controller_t *params);


#endif /* INC_FOC_FILTERS_H_ */
