/*
 * foc_helpers.c
 *
 *  Created on: Apr 30, 2025
 *      Author: tobi
 */

#include <stdint.h>
#include <math.h>
#include "foc_helpers.h"

const int16_t sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};


float fast_sin(float a)
{
	if(a < M_PI_2)
	{
		return 0.0001*sine_array[fast_round(126.6873* a)];      // int array optimized
	}
	else if(a < M_PI)
	{
		return 0.0001*sine_array[398 - fast_round(126.6873*a)];     // int array optimized
	}
	else if(a < M_3PI_2)
	{
		return -0.0001*sine_array[-398 + fast_round(126.6873*a)];      // int array optimized
	}
	else
	{
		return -0.0001*sine_array[796 - fast_round(126.6873*a)];      // int array optimized
	}
}

 // function approximating cosine calculation by using fixed size array
 // ~55us (float array)
 // ~56us (int array)
 // precision +-0.005
 // it has to receive an angle in between 0 and 2PI
float fast_cos(float a)
{
	float a_sin = a + M_PI_2;
	a_sin = a_sin > M_2PI ? a_sin - M_2PI : a_sin;
	return fast_sin(a_sin);
}

//http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
//Volkan SALMA

float fast_atan2(float y, float x)
{

 	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = M_3PI_4;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = M_PI_4;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );
}

 // normalizing radian angle to [0,2PI]
float normalizeAngle(float angle)
{
   float a = fmod(angle, M_2PI);
   return a >= 0 ? a : (a + M_2PI);
}

 // Electrical angle calculation
 float electricalAngle(float shaft_angle, int pole_pairs) {
   return (shaft_angle * pole_pairs);
 }


// square root approximation function using
 // https://reprap.org/forum/read.php?147,219210
 // https://en.wikipedia.org/wiki/Fast_inverse_square_root
float fast_sqrt(float number)
{//low in fat
	int32_t i;
	float y;
	// float x;
	// const float f = 1.5F; // better precision

	// x = number * 0.5F;
	y = number;
	i = * ( ( long * ) &y);
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( ( float * ) &i);
	// y = y * ( f - ( x * y * y ) ); // better precision
	return number * y;
}
