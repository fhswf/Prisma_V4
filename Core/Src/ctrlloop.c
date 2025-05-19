/*
 * ctrlloop.c
 *
 *  Created on: May 13, 2025
 *      Author: tobi
 */

#include "main.h"
#include "foc.h"
#include "leds.h"
#include "foc_filters.h"
#include "foc_helpers.h"
#include "globals.h"

#include "bmi088a.h"
#include "bmi088g.h"
#include <stdio.h>
#include <stdlib.h>			// atoi()
#include "display.h"
#include "uart.h"



#define RAD2DEG 180./M_PI
#define LEDRING_OFFSET 22

#define LQR_K1   -10.0
#define LQR_K2    -1.0
#define LQR_K4       -1.8

#define SCALE_K1	1.
#define SCALE_K2	2.
#define SCALE_K4	4.

pid_controller_t pid_angle;
lp_filter_t lpf_acceleration;

static char buf[80];
extern volatile uint16_t irq_cnt;

extern acc_data_grav_t  acc;
extern gyro_data_rads_t gyro;

float k1=0.,k2=0.,k4=0.;

extern float velocity_sp ;

void bmi_init();

float angle_filter(float accel_angle, float gyro_rate);

/*
void TIM6_DAC_IRQHandler(void)
{
	t6++;
	// Heartbeat LED
	if ((t6 % 10) == 0)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}

}*/

/***********************************************************
 * Update params by user input
 ***********************************************************/
void param_adjust()
{
	static char buf[11];
	static uint8_t  cnt=0;
	char p;
	char c;
	int32_t v;

	c=uart_bt_getc();
	//if (c==UART_EMPTY) c=uart_bt_getc();

	if (c!= UART_EMPTY)
	{
		if (c==';')	// Done
		{
			buf[cnt]=0;
			p = buf[0];
			v = atoi(buf+1);
			printf("REC: >%s< -> %li\r\n",buf,v);
			cnt=0;
			// Set parameter
			switch(p)
			{
			case 'a': k1 = (float)v*SCALE_K1;
					  printf("k1: %5.3f\n",k1);
					  break;
			case 'b': k2 = (float)v*SCALE_K2;
					  printf("k2: %5.3f\n",k2);
					  break;
			case 'c': k4 = (float)v*SCALE_K4;
			  	  	  printf("k4: %5.3f\n",k4);
					  break;
			default: break;
			}
		}
		else
		{	// Only copy -0123456789abcd
			if ( (c=='-') || ((c>='0') && (c<='9')) || ((c>='a') && (c<='d')) )
			{
				buf[cnt] = c;
				cnt = (cnt+1)%10;
			}
		}
	}
}

void ctrlloop()
{

	uint16_t irq_cnt_i=0;
	float ang, ang1, ang2;
	float wheel_acc = 0.0;

//	uint8_t led;
	uint8_t missed;
    int inten=0;
    uint8_t stopped = 1;

	// start tim6 -> 100 Hz
	bmi_init();

	pid_angle.KP = 3;
	pid_angle.KI = 1;
	pid_angle.KD = 0.15;
	pid_angle.lim_out = 100;
	pid_angle.lim_slew = 5000;

	lpf_acceleration.Tf = 0.001;

	while(1)
	{
		// Check timing; if not equal, irq cnt has advanced before control loop was finished
		if (irq_cnt != irq_cnt_i)
		{
		  missed = 10;		// show f0r 10 cycles
		}

		// Wait for new cycle to start (triggered by IMU)
		while (irq_cnt_i == irq_cnt);
		irq_cnt_i = irq_cnt;

		// Heartbeat LED
		if ((irq_cnt_i % 10) == 0)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}

		ang = fast_atan2(acc.ay, acc.ax)*RAD2DEG;

		ang = angle_filter(ang, gyro.gz);
		ang1 = ang / 360.;

		// LED UPDATE w load balancing
		switch(irq_cnt % 5)
		{
		case 0:
			for (int i=0;i<24;i++)
			{
			  ang2 = (i-LEDRING_OFFSET)/24.0;
			  if (ang2<-.5) ang2+=1.0;
			  if (ang2>0.5) ang2-=1.0;
			  inten = -log(fabs(ang1-ang2)/0.25)*5;
			  if (inten<0) inten = 0;
			  ledring_set_rgb(i, inten,0,(stopped==0)*inten);
			}
			if (missed)
			{
			  missed--;
			  ledring_set_rgb(10,60,0,0);
			}
			break;
		case 1:
			ledring_update();
			break;
		case 2:
			param_adjust();
			break;
		case 3:
//
			break;
		default:
			break;
		}

		// Start / Stop logic
		if ( (ang <-20) || (ang>+20))
		{
			  stopped = 1;
		}
		if (( fabsf(ang)<5 ) && (stopped==1) )
		{
			stopped = 0;
			pid_angle.err_prev = 0.;
			pid_angle.int_prev = ang;
			pid_angle.timestamp_prev = 0;
		}
		if (stopped)
		{
			velocity_sp = velocity_sp*0.95; // slow down, no hard stop!
		}
		else
		{
		  //velocity_sp = -pid_control(ang,&pid_angle);
			//float theta_p = ang;
			//float theta_p_dot = gyro.gx;
			//...
			//float theta_r_dot = velocity_sp;	//velocity of wheel from last loop run

            //motor movement calculation
            //wheel_acc = lp_filter(-LQR_K1*theta_p-LQR_K2*theta_p_dot- LQR_K4*theta_r_dot , &lpf_acceleration);		//lqr formula
            wheel_acc = lp_filter(-LQR_K1*ang - LQR_K2*gyro.gx - LQR_K4*velocity_sp , &lpf_acceleration);		//lqr formula
            velocity_sp += wheel_acc;
            velocity_sp = CONSTRAIN(velocity_sp,-20,20);
		}
		foc_loop();
		if ((irq_cnt_i % 20) == 0)
		{
			sprintf(buf,"ang: %6.1f spd = %5.1f\r\nw_acc %5.1f", ang, velocity_sp,wheel_acc);
			printf(buf);
			display_println(buf);
		}

	}
}
