/*
 * bmi088_tst.c
 *
 *  Created on: Apr 5, 2025
 *      Author: tobi
 */

#include "main.h"
#include <stdio.h>
#include "bmi088a.h"
#include "bmi088g.h"

void main_bmi()
{
  int status;
  gyro_data_rads_t gyro;
  acc_data_grav_t  acc;

  // INIT Accelerometer
  printf("BMI088 ACC init\r\n");
  status = bmi088a_init();
  if (status != 0) {
    printf("Accel Initialization Error %i\r\n",status);
    while (1) {}
  }

  printf("BMI088 GYRO init\r\n");
  // INIT Gyrometer

  status = bmi088g_init();
  if (status != 0) {
    printf("Gyro Initialization Error %i\r\n",status);
    while (1) {}
  }

  printf("BMI088 init done\r\n");


  while(1)
  {
	  uint8_t acc_ok, gyro_ok;

	  acc_ok  = bmi088a_getDrdyStatus();

	  if ( acc_ok )
	  {
		  bmi088a_readSensor();
		  bmi088a_get_grav(&acc);

		  printf("ACC:  x=%7.3f  y=%7.3f  z=%7.3f\r\n",
				acc.ax, acc.ay, acc.az);
	  }

	  gyro_ok = bmi088g_getDrdyStatus();
	  if ( gyro_ok)
	  {
		  bmi088g_readSensor();
		  bmi088g_get_rads(&gyro);
		  printf("GYRO:  x=%7.3f  y=%7.3f  z=%7.3f\r\n",
				gyro.gx, gyro.gy, gyro.gz);
	  }
	  HAL_Delay(500);
  }
}
