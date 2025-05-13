/*
 * bmi088_tst.c
 *
 *  Created on: Apr 5, 2025
 *      Author: tobi
 */

#include "main.h"
#include <stdio.h>
#include <math.h>
#include "bmi088a.h"
#include "bmi088g.h"
#include "leds.h"
#include "bldc.h"
#include "globals.h"

#define DECIMATE 2
#define RAD2DEG 180./M_PI
#define LEDRING_OFFSET 22

float angle_filter(float accel_angle, float gyro_rate);

volatile uint16_t irq_cnt=0;
acc_data_grav_t  acc;
gyro_data_rads_t gyro;


uint8_t bmi088_is_initialized=0;

/**********************************************************************
 *
 * EXTI Callback for data ready
 *
 **********************************************************************
 * void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *
 * Get ACC / GYRO data via Pin Change IRQ
 *
 * DECIMATE acts on ACC, GYRO is then read on next IRQ and cnt increased
 *
 */


void EXTI9_5_IRQHandler(void)
{
	static uint16_t dec_cnt=0, take_gyro_sample=0;

	// BMI088_INT1 / INT 3 on pins PA5/PA6
    if (__HAL_GPIO_EXTI_GET_IT(BMI088_INT1_Pin) != RESET)
    {
    	__HAL_GPIO_EXTI_CLEAR_IT(BMI088_INT1_Pin);
    	if ( (bmi088_is_initialized) &&  ( (++dec_cnt) >= DECIMATE) )
    	{
			// Clear Interrupt flag
			//bmi088a_getDrdyStatus();
			// Get Data
			bmi088a_readSensor();
			bmi088a_get_grav(&acc);
			take_gyro_sample=1;
			dec_cnt=0;
		}
    }

    if (__HAL_GPIO_EXTI_GET_IT(BMI088_INT3_Pin) != RESET)
    {
    	__HAL_GPIO_EXTI_CLEAR_IT(BMI088_INT3_Pin);
    	if (bmi088_is_initialized && take_gyro_sample )
    	{
			// Clear Interrupt flag
			//bmi088a_getDrdyStatus();
			// Get Data
			bmi088g_readSensor();
			bmi088g_get_rads(&gyro);
			irq_cnt++;
			take_gyro_sample = 0;
		}
    }
}

float get_angle()
{
	float angle;
	angle = atan2f(acc.ay, acc.ax)*RAD2DEG;

	angle = angle_filter(angle, gyro.gz);
}

void bmi_init()
{
  int status;
  uint16_t irq_cnt_i;
  int dist;
  float ang, ang2;
  uint8_t led, ledold=0,prt;
  uint8_t missed;

  // INIT Accelerometer
  printf("BMI088 ACC init\r\n");
  status = bmi088a_init();
  if (status != 0) {
    printf("Accel Initialization Error %i\r\n",status);
    while (1) {}
  }

  if (bmi088a_setOdr(ACC_ODR_200HZ_BW_80HZ)) {
    printf("Failed changing ACC ODR\r\n");
  }


  // Configure Interrupt Pin
  bmi088a_pinModeInt1( ACC_PINIO_OUTPUT, ACC_PINLVL_ACTIVE_HIGH, ACC_PINMODE_PUSH_PULL);
  bmi088a_mapDrdyInt1( BMI088_ENABLE);
  printf("BMI088 GYRO init\r\n");
  // INIT Gyrometer

  status = bmi088g_init();
  if (status != 0) {
    printf("Gyro Initialization Error %i\r\n",status);
    while (1) {}
  }

  if (bmi088g_setOdr(GYRO_ODR_200HZ_BW_64HZ))  {
    printf("Failed changing GYRO ODR\r\n");
  }

  if (bmi088g_setRange(GYRO_RANGE_1000DPS)) {
    printf("Failed changing GYRO RANGE\r\n");
  }

  // Configure Interrupt Pin
  bmi088g_pinModeInt3( GYRO_PINLVL_ACTIVE_HIGH, GYRO_PINMODE_PUSH_PULL);
  bmi088g_mapDrdyInt3( BMI088_ENABLE);

  printf("BMI088 init done\r\n");
  HAL_Delay(3);
  bmi088_is_initialized = 1;
}

/*

  int inten=0;

  while(1)
  {
	  // wait on new data
	  if (irq_cnt != irq_cnt_i)
	  {
		  missed = 10;
	  }
	  while (irq_cnt_i == irq_cnt);
      irq_cnt_i = irq_cnt;

      ang = get_angle() / 360.;

      led = (int)(ang * 24 + LEDRING_OFFSET) % 24;
      if ((irq_cnt % 100) == 0)
    	  prt=1;

      for (int i=0;i<24;i++)
      {
    	  ang2 = (i-LEDRING_OFFSET)/24.0;
    	  if (ang2<-.5) ang2+=1.0;
    	  if (ang2>0.5) ang2-=1.0;
    	  inten = -log(fabs(ang-ang2)/0.25)*5;
    	  if (inten<0) inten = 0;
          ledring_set_rgb(i, inten,0,inten);
      }
      //ledring_set_rgb(led,40,0,40);
      // Speed
      //bldc_calc_rpm();
      ledring_set_rgb(10+rpm_g/60, 0,0,50);
      if (prt)
      {
    	  printf("RPM: %6.1f\r\n",rpm_g);
          prt=0;
      }
      if (missed)
      {
    	  missed--;
    	  ledring_set_rgb(10,60,0,0);
      }
      ledring_update();
      ledold=led;

      // Toggle LED2 on every 10th run
      if ((irq_cnt % 10) == 0)
      {
    	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }
  }
}*/
