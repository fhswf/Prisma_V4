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
#include "bmi088a.h"
#include "bmi088g.h"
#include <stdio.h>
#include "display.h"



#define RAD2DEG 180./M_PI
#define LEDRING_OFFSET 22

pid_controller_t pid_angle;

static char buf[80];
extern volatile uint16_t irq_cnt;

extern acc_data_grav_t  acc;
extern gyro_data_rads_t gyro;

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
void ctrlloop()
{

	uint16_t irq_cnt_i=0;
	float ang, ang1, ang2;
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

	while(1)
	{
		  // wait on new data
		  if (irq_cnt != irq_cnt_i)
		  {
			  missed = 10;
		  }
		  while (irq_cnt_i == irq_cnt);
	      irq_cnt_i = irq_cnt;

		// Heartbeat LED
		if ((irq_cnt_i % 10) == 0)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}

	  	  ang = atan2f(acc.ay, acc.ax)*RAD2DEG;

	  	  ang = angle_filter(ang, gyro.gz);
	  	  ang1 = ang / 360.;
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
	      ledring_update();
	      if ( (ang <-20) || (ang>+20))
		{
	    	  stopped = 1;
		}
	      if ( fabsf(ang)<5)
	      {stopped = 0;}
	      if (stopped)
	      {
			velocity_sp = velocity_sp*0.95;
		}
		else
		{
	      velocity_sp = -pid_control(ang,&pid_angle);
		}
          foc_loop();
          if ((irq_cnt_i % 100) == 0)
           {
         	  sprintf(buf,"ang: %6.1f spd = %5.1f\r\n", ang, velocity_sp);
         	  display_println(buf);
           }

	}
}
