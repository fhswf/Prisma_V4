/*
 * foc.c
 *
 *
 * Most of the code is adapted from the Arduino SimpleFOC
 * https://www.simplefoc.com/
 *
 * Kudos to this project!
 *
 *  Created on: Apr 27, 2025
 *      Author: tobi
 *
 *
 *
 */


#include "main.h"
#include <stdio.h>
#include "display.h"
#include <math.h>
#include "globals.h"
#include "foc.h"
#include "foc_filters.h"
#include "foc_helpers.h"

//*** MOTOR PARAMETERS

#define PID_KP   0.3
#define PID_KI   1.0
#define PID_KD   0.0
#define U_LIMIT  12.0
#define SLEW_LIMIT 70.0

//*** Downsample / ADC
#define ADC_DOWNSAMPLE	10		// ADC is triggered by TIM1; Downsample 10 -> if TIM1 PWM speed is 20kHz -> ADC Sample = 2kHz


// Phase currents measured with adc2
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;

// ******* TIMERS *******
extern TIM_HandleTypeDef htim1;					//Timer 1 -> LS FETs  for three coils
extern TIM_HandleTypeDef htim2;					//Timer 2 -> 100kHz -> 10us for RPM calculations etc
extern TIM_HandleTypeDef htim3;					//Timer 3 -> HS FETs  for three coils

static uint16_t adc2_buf[3];
static float adc2_off[3];

static int16_t pwm_ul=0, pwm_uh=0;	  // int, since we only need values <1000 and to avoid underrun when subtracting
static int16_t pwm_vl=0, pwm_vh=0;
static int16_t pwm_wl=0, pwm_wh=0;

volatile static uint16_t pgood_err=0;			  // PGOOD Error Count
static float zero_electric_angle=0.0;
static int8_t sensor_direction=0;



static uint8_t foc_enabled=0;

volatile float Iu, Iv, Iw;		// Measured Phase Currents
float ud_g=0., uq_g=0.;
float id_q=0., iq_g=0.;

volatile uint16_t adc2_cnt = 0;

//*** Hall State detection
uint8_t hallstate = 0;
int8_t  electric_sector = 0;
int8_t  direction = DIR_UNDEF;
int32_t electric_rotations = 0;
int32_t pulse_diff = 0;
uint32_t pulse_timestamp = 0;
uint32_t total_interrupts = 0;


//*** Filters and PID
lp_filter_t lpf_curr_d = {0.005,0,0};
lp_filter_t lpf_curr_q = {0.005,0,0};
pid_controller_t pid_velocity;
pid_controller_t pid_curr_d;
pid_controller_t pid_curr_q;


//*** Setpoint
float velocity_sp = 0.0;
float current_sp  = 0.0;



//**** ADC voltage to current
//
// see Datasheet of MP6541
// Vs = Vref + Rsens*Iout/11000
// Iout = (Vs-Vref)*11000/Rsens
// Since we have 3k3 to +3V3 and 3k3 to GND, the effective Rsens is 3k3/2
#define RSENSE    (3300.0/2.0)
#define ADC2VOLT  (3.3/4095.0)
#define ISCALE    (ADC2VOLT*11200/RSENSE)

#define PWM_DEADTIME    1								// PWM Dead Time between nMOS / pMOS (time is doubled...)
#define PWM_MAX 	    (int16_t)htim3.Instance->ARR 	// Get ARR-Value from Timer
#define PWM_MIN         0
#define PWM_LMT			40								// Maximum PWM allowed (to limit while testing)

#define MICROS    (htim2.Instance->CNT * 10)	// Time stamp in microseconds


const int8_t ELECTRIC_SECTORS [8] = { -1, 0, 4, 5, 2, 1, 3 , -1 };	// in simpleFOC they count from 0 ...

/**** Private Function Prototypes ****/
static void foc_hallstate();
static void foc_calcPhaseVoltage(float Uq, float Ud, float angle_el);
static void foc_sync_tmr_start();
static uint8_t foc_align_sensor();
static void foc_fastloop();
static void foc_write_pwm();

volatile uint32_t tcnt=0;
uint16_t adv = 0;

float Vd, Vq;
/****************************************************
 *
 * DMA transfer from ADC finished
 *
 * Calculate currents and do all the fast loop stuff,
 * i.e. update pwm values
 *
 ***************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc != &hadc2)
	{
		return;
	}

	Iu = ( (float)adc2_buf[0] - adc2_off[0] ) * ISCALE;
	Iv = ( (float)adc2_buf[1] - adc2_off[1] ) * ISCALE;
	Iw = ( (float)adc2_buf[2] - adc2_off[2] ) * ISCALE;
	adv = adc2_buf[0];
	adc2_cnt++;
	foc_fastloop();
	foc_write_pwm();
}

static void foc_fastloop()
{
	float Ialpha, Ibeta, Iq, Id;
	float theta_e;

    if (foc_enabled)
    {
	   // *** Clarke-Transformation ***

	   //float Ialpha = ia;
	   //float Ibeta = (ia + 2 * ib) / sqrt(3.0);
	   // -> Full Clarke, since we only measure to gnd?

       //Ialpha = ( 2.*Iu - Iv - Iw)/3.;
       Ialpha = 2./3.*( Iu - (Iv-Iw));

       Ibeta  = ( Iv - Iw) / SQRT3;
       theta_e = foc_electric_angle();
       float cosv = fast_cos(theta_e);
       float sinv = fast_sin(theta_e);
	   // *** Park-Transformation ***
	   Id =  Ialpha * cosv + Ibeta * sinv;
	   Iq = -Ialpha * sinv + Ibeta * cosv;

	   // *** Control loop ***
	   Iq = lp_filter(Iq, &lpf_curr_q);
	   Id = lp_filter(Id, &lpf_curr_d);
	   iq_g = Iq;
	   id_q = Id;

	   Vq = pid_control(current_sp - Iq, &pid_curr_q);
	   Vd = pid_control(-Id, &pid_curr_d);

	   foc_calcPhaseVoltage(Vq, Vd, theta_e);
    }
}

/**************************************************************
 *
 * Update PWM values when update-event happens
 *
 * should ensure that all values are updated in the same period
 * but introduces a half pwm cycle delay
 *
 **************************************************************/
//void TIM1_UP_TIM16_IRQHandler(void)
static void foc_write_pwm()
{

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)pwm_uh);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)pwm_vh);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)pwm_wh);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pwm_ul);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)pwm_vl);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)pwm_wl);
//    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    tcnt++;
}

/**********************************************************************
 *
 * EXTI Callback on changes of hall sensors to initiate commutation
 *
 **********************************************************************/
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// -> Don't use HAL, since it generates three calls, one for each pin!

void EXTI15_10_IRQHandler(void)
{
	// Check for PGOOD-Errors (EXTI10)
	if ( HAL_GPIO_ReadPin(MOT_PGOOD_GPIO_Port, MOT_PGOOD_Pin)==0)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(MOT_PGOOD_Pin);
		pgood_err++;
	}
	else
	{
		// Clear all EXTI-Sources (as we don't use HAL)
		__HAL_GPIO_EXTI_CLEAR_IT(HALL_A_Pin|HALL_B_Pin|HALL_C_Pin);
		foc_hallstate();  // Evaluate new hall state
	}
}

/***************************************************************
 *
 * voc_init()
 *
 * Initialize the FOC algo
 *
 ***************************************************************/
void foc_init()
{
	uint16_t cnt=0;
	char buf[40];
	uint32_t lst_tick=0;
	// Calibrate ADC
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);


	// Sync start TIM1->TIM3 and ADC Downsample
	foc_sync_tmr_start();


	for (uint8_t i=0;i<100;i++)
	{
		display_println(buf);

		while(adc2_cnt == cnt)
			{
			if (HAL_GetTick()-lst_tick>500)
			{
				sprintf(buf,"ADC Cal %i t %lu\r\n",i,tcnt);
				display_println(buf);
				lst_tick=HAL_GetTick();
			}
			};
		cnt = adc2_cnt;
		adc2_off[0] += (float)adc2_buf[0];
		adc2_off[1] += (float)adc2_buf[1];
		adc2_off[2] += (float)adc2_buf[2];
	}
	adc2_off[0] /= 100.;
	adc2_off[1] /= 100.;
	adc2_off[2] /= 100.;
	display_clear();
	sprintf(buf,"ADC OFF %5.1f %5.1f %5.1f",adc2_off[0],adc2_off[1],adc2_off[2]);
	display_println(buf);
	HAL_Delay(1000);

	// Start Timer for micros measurement
	HAL_TIM_Base_Start(&htim2);

	// Init PID
	pid_velocity.KP = PID_KP;
	pid_velocity.KI = PID_KI;
	pid_velocity.KD = PID_KD;
	pid_velocity.lim_out = U_LIMIT;
	pid_velocity.lim_slew = SLEW_LIMIT;

	pid_curr_d.KP = 3.0;
	pid_curr_d.KI = 300.0;
	pid_curr_d.KD = 0.0;
	pid_curr_d.lim_out = U_LIMIT;
	pid_curr_d.lim_slew = 1e11;

	pid_curr_q.KP = 3.0;
	pid_curr_q.KI = 300.0;
	pid_curr_q.KD = 0.0;
	pid_curr_q.lim_out = U_LIMIT;
	pid_curr_q.lim_slew = 1e11;

	foc_enabled = 1;
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_SET);
	foc_align_sensor();
}

static void foc_sync_tmr_start()
{
	// Set all PWM to 50% duty ...
	uint16_t pwm_mid = PWM_MAX/2;

	pwm_ul = pwm_vl = pwm_wl = pwm_mid;
	pwm_uh = pwm_vh = pwm_wh = pwm_mid;

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_mid);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_mid);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_mid);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_mid);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_mid);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_mid);

	// Set TIM1 Repetiton Counter for ADC downsample...
	// Important: Update Event is triggered on top and bottom, i.e. two times per PWM cycle
	// -> RCC=1 -> every second event, at i.e. every PWM top
	// -> RCC=3 -> every fourth event ...

	htim1.Instance->RCR =(ADC_DOWNSAMPLE-1)*2+1;

	// Init timer 3 -> High side
	// Needs to be initialized first, since it is triggered by timer
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	// Init timer 1 -> Enable -> Triggers TIM3
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim1);

}

void foc_disable()
{
	HAL_GPIO_WritePin(NSLEEP_GPIO_Port, NSLEEP_Pin, GPIO_PIN_RESET);
	foc_enabled = 0;
	foc_setpwm(0,0,0);
}

/*****************************************************
 *
 * foc_setpwm(pwm_u, pwm_v, pwm_w)
 *
 * Calculate values for timer, will be written
 * to timer registers on next TIM_UPD IRQ
 *
 *****************************************************/
void foc_setpwm(int16_t pwm_u, int16_t pwm_v, int16_t pwm_w)
{
	pwm_ul = CONSTRAIN(pwm_u-PWM_DEADTIME, PWM_MIN, PWM_MAX);
	pwm_vl = CONSTRAIN(pwm_v-PWM_DEADTIME, PWM_MIN, PWM_MAX);
	pwm_wl = CONSTRAIN(pwm_w-PWM_DEADTIME, PWM_MIN, PWM_MAX);

	pwm_uh = CONSTRAIN(pwm_u+PWM_DEADTIME, PWM_MIN, PWM_MAX);
	pwm_vh = CONSTRAIN(pwm_v+PWM_DEADTIME, PWM_MIN, PWM_MAX);
	pwm_wh = CONSTRAIN(pwm_w+PWM_DEADTIME, PWM_MIN, PWM_MAX);
}

/************************************************************
 *
 * Calculate Phase voltages based on FOC
 *
 ************************************************************/
static void foc_calcPhaseVoltage(float Uq, float Ud, float angle_el)
{
	int sector;
	// the algorithm goes
	// 1) Ualpha, Ubeta
	// 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
	// 3) angle_el = atan2(Ubeta, Ualpha)
	//
	// equivalent to 2)  because the magnitude does not change is:
	// Uout = sqrt(Ud^2 + Uq^2)
	// equivalent to 3) is
	// angle_el = angle_el + atan2(Uq,Ud)
	float Uout;
	ud_g = Ud;
	uq_g = Uq;

/*	if (Uq<0.0)
	{
		angle_el += M_PI;
		Uq = -Uq;
	}*/
	// a bit of optimisation
	if(Ud)   // only if Ud and Uq set
	{
	    // _sqrt is an approx of sqrt (3-4% error)
        Uout = fast_sqrt(Uq*Uq+Ud*Ud) / U_LIMIT;
		angle_el = normalizeAngle(angle_el + ( (Uq==0.0)?.0:atan2f(Uq, Ud) ));
	}
	else
	{
		// only Uq available - no need for atan2 and sqrt
        Uout = Uq / U_LIMIT;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = normalizeAngle(angle_el + M_PI_2);
    }
	// find the sector we are in currently
	sector = floorf(angle_el / M_PI_3) + 1;
	// calculate the duty cycles
	float T1 = M_SQRT3 * fast_sin( sector * M_PI_3 - angle_el) * Uout;
	float T2 = M_SQRT3 * fast_sin( angle_el - (sector - 1.0) * M_PI_3) * Uout;
       // two versions possible
       //float T0 = 0; // pulled to 0 - better for low power supply voltage
    float T0 = 1 - T1 - T2; //modulation_centered around driver->voltage_limit/2

	// calculate the duty cycles(times)
	float Ta,Tb,Tc;
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:
			// possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}

	// Set the PWM values
	// Symmetrical:
	foc_setpwm(100-PWM_LMT/2+Tb * PWM_LMT, 100-PWM_LMT/2+Ta * PWM_LMT, 100-PWM_LMT/2+Tc * PWM_LMT);
}

/***************************************************************************
* Get hall state and compare with last state to detect direction and speed
*
* IMPORTANT: The wheel might rotate in opposite direction as currently
*            driven when braking!
***************************************************************************/
static void foc_hallstate()
{
	uint8_t new_state;
	int8_t new_sector;
	static int8_t old_direction=DIR_UNDEF;


	// Get current timestamp
	uint32_t new_timestamp = MICROS;

	// Get current state
	new_state = (HALL_A_GPIO_Port->IDR & 0xE000) >> 13;	// Faster / one read  (PB13,PB14,PB15)

	/* Glitch avoidance */
	if (new_state == hallstate) return;

	hallstate = new_state;

	/* Electrical Sector */
	new_sector = ELECTRIC_SECTORS[hallstate];

	if (new_sector - electric_sector > 3)
	{
		//underflow
		direction = DIR_CCW;
		electric_rotations += direction;
    }
	else if (new_sector - electric_sector < (-3))
	{
		//overflow
		direction = DIR_CW;
		electric_rotations += direction;
	}
	else
	{
		direction = (new_sector > electric_sector)? DIR_CW : DIR_CCW;
	}
	electric_sector = new_sector;

	// glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
	if (direction == old_direction)
	{
		// not oscilating or just changed direction
		pulse_diff = new_timestamp - pulse_timestamp;
	}
	else
	{
		pulse_diff = 0;
	}

	pulse_timestamp = new_timestamp;
	total_interrupts++;
	old_direction = direction;

	// TODO
	//   if (onSectorChange != nullptr) onSectorChange(electric_sector);
}


/***********************************************************
* Shaft angle calculation
************************************************************/
float foc_getAngle()
{
   return ((float)(electric_rotations * 6 + electric_sector) / MOTOR_CPR) * M_2PI ;
}

/***********************************************************
* Electrical angle calculation
************************************************************/
float foc_electric_angle()
{
	return( (float)electric_sector/6.0*M_2PI);//+zero_electric_angle);
}

/*****************************************************************
*   Shaft velocity calculation
*   function using mixed time and frequency measurement technique
******************************************************************/
float foc_getVelocity()
{
   if (pulse_diff == 0 || ((MICROS - pulse_timestamp) > pulse_diff) )  // last velocity isn't accurate if too old
   {
	   return 0;
   }
   else
   {
	   return direction * (M_2PI / MOTOR_CPR) / (pulse_diff / 1000000.0);
   }
 }

/************************************************
 *
 * Slow control loop
 *
 * Needs to be called periodically
 *
 ************************************************/
void foc_loop()
{
	float velocity,err;

	velocity = foc_getVelocity();
	err = velocity_sp - velocity;
    current_sp = pid_control(err, &pid_velocity);
}

/**********************************************
 *
 * FOC Test routine
 *
 **********************************************/
void foc_test()
{
	float ang, vel;
	char buf[80];
	uint32_t ticks=0,t2=0;
	velocity_sp = -2;
	current_sp = 0.1;
	// Test Rotation
	/*
	for (float w=0;w<10000;w++)
	{
		float el = normalizeAngle(w *30 * DEG2RAD);
		foc_calcPhaseVoltage(4.0,0.0,el);
		if ( ((int)w%500) == 0)
		{
			display_clear();
			sprintf(buf,"I: %5.1f %5.1f %5.1f\n%5.1f",Iu*1000.,Iv*1000.,Iw*1000.,(Iu+Iv+Iw)*1000.);
			display_println(buf);
			sprintf(buf,"ADC %u",adv);
			display_println(buf);
			sprintf(buf,"Ud %5.3f Uq %5.3f el %5.3f",ud_g, uq_g, foc_electric_angle());
			display_println(buf);
			sprintf(buf,"el %5.3f",el/DEG2RAD);
			display_println(buf);
			sprintf(buf,"pwm %3i %3i %3i",pwm_uh, pwm_vh, pwm_wh);
			display_println(buf);
		}
		HAL_Delay(10);
	}*/
	while(1)
	{
		foc_fastloop();
		if (HAL_GetTick()-ticks>200)
		{
			/*
			ang = foc_getAngle();
			vel = foc_getVelocity();
			display_clear();
			sprintf(buf,"ADC %i t %lu",adc2_cnt,tcnt);
			display_println(buf);
			sprintf(buf,"I: %5.1f %5.1f %5.1f",Iu*1000.,Iv*1000.,Iw*1000.);
			display_println(buf);
			sprintf(buf,"vel %5.1f ang %5.1f",vel,ang/DEG2RAD);
			display_println(buf);
			sprintf(buf,"pgood %u",pgood_err);
			display_println(buf);

			sprintf(buf,"I_sp %4.2f %4.2f %4.2f",current_sp, id_q, iq_g);
			display_println(buf);
			sprintf(buf,"Ud %4.1f Uq %4.1f el %5.3f",ud_g, uq_g, foc_electric_angle());
			display_println(buf);
			sprintf(buf,"pwm %3i %3i %3i",pwm_uh, pwm_vh, pwm_wh);
			display_println(buf);

			*/
			ticks = HAL_GetTick();
			velocity_sp += 0.2;
		}
		if (HAL_GetTick()-t2>10)
		{
			foc_loop();
			t2 = HAL_GetTick();
		}
	}
}

uint8_t foc_align_sensor() {
	char buf[50];
   // if unknown natural direction
   display_clear();
   display_println("Angle calibration");

   // find natural direction
   // move one electrical revolution forward
   for (int i = 0; i <=500; i+=5 )
   {
		float angle = M_3PI_2 + M_2PI * i / 500.0;
		foc_calcPhaseVoltage(6.0, 0,  angle);
		HAL_Delay(2);
   }

   // take and angle in the middle
   float mid_angle = foc_getAngle();
   sprintf(buf,"Mid: %5.2f",mid_angle);
   display_println(buf);
   // move one electrical revolution backwards
	for (int i = 500; i >=0; i-=5 )
	{
		float angle = M_3PI_2 + M_2PI * i / 500.0 ;
		foc_calcPhaseVoltage(6.0, 0,  angle);
		HAL_Delay(2);
	}
	float end_angle = foc_getAngle();
	sprintf(buf,"End: %5.2f",end_angle);
    display_println(buf);
	foc_calcPhaseVoltage(0, 0, 0);
	// determine the direction the sensor moved
	if (mid_angle == end_angle)
	{
		display_println("NO MOVEMENT");
		HAL_Delay(2000);
		return 1; // failed calibration
	} else if (mid_angle < end_angle)
	{
	//if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CCW"));
	//sensor_direction = Direction::CCW;
	display_println("DIR CCW");
	sensor_direction = DIR_CCW;
	} else{
//	if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CW"));
//	sensor_direction = Direction::CW;
		display_println("DIR CW");
		sensor_direction = DIR_CW;
	}
	float moved =  fabs(mid_angle - end_angle);
    if( fabs(moved*(MOTOR_CPR/6) - M_2PI) > 0.5 ) { // 0.5 is arbitrary number it can be lower or higher!
    	sprintf(buf,"PPmvd %5.2f",moved);
    	display_println(buf);
    } else {
	   display_println("CPR check ok");
   }

   // zero electric angle not known
     // align the electrical phases of the motor and sensor
     // set angle -90(270 = 3PI/2) degrees
     foc_calcPhaseVoltage(6.0, 0,  M_3PI_2);
     HAL_Delay(100);
     zero_electric_angle = normalizeAngle(sensor_direction*foc_getAngle()* MOTOR_CPR/6);
     sprintf(buf,"ZeroAng: %5.2f",zero_electric_angle);
     display_println(buf);
     // stop everything
     foc_calcPhaseVoltage(0, 0, 0);
   HAL_Delay(1000);
   return 0;
 }
