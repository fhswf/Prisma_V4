/*
 * foc.h
 *
 *  Created on: Apr 28, 2025
 *      Author: tobi
 */

#ifndef INC_FOC_H_
#define INC_FOC_H

#include <stdint.h>
#include <math.h>

#define DIR_CCW  -1
#define DIR_CW    1
#define DIR_UNDEF 0

#define MOTOR_CPR    48   // Hall Counts per mechanical rotataion


void foc_init();
void foc_disable();
void foc_setpwm(int16_t,int16_t,int16_t);

float foc_getAngle();
float foc_electric_angle();
float foc_getVelocity();
void foc_loop();


#endif


