/*
 * foc_filters.c
 *
 *  Created on: Apr 30, 2025
 *      Author: tobi
 */

#include "main.h"
#include "foc_filters.h"
#include "globals.h"

extern TIM_HandleTypeDef htim2;					//Timer 2 -> 100kHz -> 10us for RPM calculations etc
#define MICROS    (htim2.Instance->CNT * 10)	// Time stamp in microseconds



float lp_filter(float x, lp_filter_t *filt)
 {
     uint32_t timestamp = MICROS;

     // Start-up
     if (filt->timestamp_prev == 0)
     {
    	 filt->timestamp_prev = timestamp;
    	 return 0;
     }
     float dt = (timestamp - filt->timestamp_prev)*1e-6f;

     if (dt < 0.0f || dt > 0.5f)
         dt = 1e-3f;

     float alpha = filt->Tf/(filt->Tf + dt);
     float y = alpha*filt->y_prev + (1.0f - alpha)*x;

     filt->y_prev = y;
     filt->timestamp_prev = timestamp;
     return y;
 }

float pid_control(float error, pid_controller_t *params)
{
	float p,i,d;
	float out, out_rate;
    uint32_t timestamp = MICROS;
    // Start-up
    if (params->timestamp_prev == 0)
    {
		params->timestamp_prev = timestamp;
		return 0;
    }

    float dt = (timestamp - params->timestamp_prev)*1e-6f;

	p = params->KP * error;

	i = params->int_prev + params->KI * dt * 0.5 * (error + params->err_prev);
    i = CONSTRAIN(i, -params->lim_out, params->lim_out);   // Limit output

    d = params->KD * (error - params->err_prev)/dt;

    out = p + i + d;
    out = CONSTRAIN(out, -params->lim_out, params->lim_out);   // Limit output

    // Limit slew rate
    out_rate = (out - params->out_prev)/dt;
    if (out_rate > params->lim_slew)
		out = params->out_prev + params->lim_slew*dt;
    else if (out_rate < -params->lim_slew)
		out = params->out_prev - params->lim_slew*dt;

	// saving for the next pass
	params->int_prev = i;
	params->out_prev = out;
	params->err_prev = error;
	params->timestamp_prev = timestamp;

	return out;
}
