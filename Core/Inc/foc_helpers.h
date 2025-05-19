/*
 * foc_helpers.c
 *
 *  Created on: Apr 30, 2025
 *      Author: tobi
 */

#ifndef INC_FOC_HELPERS_C_
#define INC_FOC_HELPERS_C_

#include <math.h>

#define SQRT3     1.732050807568877
#define M_2PI     (2.0*M_PI)
//#define M_PI_2    M_PI / 2.0
#define M_PI_3    (M_PI / 3.0)
//#define M_PI_4    (M_PI / 4.0)
#define M_3PI_2   (M_PI * 3.0 / 2.0)
//#define M_3PI_4   (M_PI * 3.0 / 4.0)
#define M_PI_6    (M_PI / 6.0)
#define DEG2RAD   (M_PI/180.0)


float fast_sin(float a);
float fast_cos(float a);
float fast_atan2(float y, float x);


float fast_sqrt(float number);

float normalizeAngle(float angle);

#define fast_round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))



#endif /* INC_FOC_HELPERS_C_ */
