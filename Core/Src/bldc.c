/************************************************************************************
 *
 *      oooooooooooooo          **
 *      o   .---.    o         ****   ***
 *      o   | * |   oo         ****   ***
 *       oo '---' oo           ****
 *         oo   oo              **           Fachhochschule Suedwestfalen
 *           ooo                **           Mechatronik / Mikrocomputer / EmbSys
 *            o                              (c) Prof. Tobias Ellermeyer
 *    ==================        **
 *     BALANCING PRISMA         **
 *
 *
 *  bldc.c
 *
 *  Created on: Oct 26, 2023
 *      Author: tobi
 */


#include "main.h"
#include "globals.h"
#include "bldc.h"

/* Diagram from Maxon Datasheet
 *
 * Hall 1    |XXX|XXX|XXX|   |   |   |
 * Hall 2    |   |   |XXX|XXX|XXX|   |
 * Hall 3    |XXX|   |   |   |XXX|XXX|
 *  Val      | 5 | 1 | 3 | 2 | 6 | 4 |
 *           |   |   |   |   |   |   |
 * Coil U12  |+++|   |   |---|   |   |
 * Coil U23  |   |   |+++|   |   |---|
 * Coil U31  |   |---|   |   |+++|   |
 */

/* MP6541A FET logic
 *
 * HS_x | LS_x | Sx
 * -----+------+---------
 *   L  |  L   |  TriState
 *   L  |  H   |   Low
 *   H  |  L   |   High
 *   H  |  H   |  TriState
 *
 */
static uint8_t bldc_drive[2][8] = 	{		// array with next coil config for CW or CCW rotation
//    001 010 011 100 101 110
{  0, 13, 21, 23, 32, 12, 31, 0},  //CW Clockwise
{  0, 31, 12, 32, 23, 21, 13, 0},  //CCW Counter Clockwise
};

/*   Arrays to determine direction of rotation */
static uint8_t hall_cw[8] =
//  0  1  2  3  4  5  6  7     // Entry number
  { 0, 3, 6, 2, 5, 1, 4, 0};   // expected next hall state when rotating CW
static uint8_t hall_ccw[8] =
//  0  1  2  3  4  5  6  7     // Entry number
  { 0, 5, 3, 1, 6, 4, 2, 0};   // expected next hall state when rotating CCW

static int pwm_ratio=0;
static uint8_t locked = 1;				// Lock EXTi until initialized
static volatile uint8_t freerun = 0;

// Used for RPM Calculations
static uint32_t cnt_upd=0;
static int8_t dir_rot=0;			// actually measured rotation direction
static volatile int dir_field = 0;	// diection of electromagnetic field, i.e. drive direction of motor (may not be current rotation direction)

extern TIM_HandleTypeDef htim1;					//Timer 1 -> ENx  for three coils
extern TIM_HandleTypeDef htim3;					//Timer 3 -> INx  for three coils
extern TIM_HandleTypeDef htim6;					//Timer 6 -> Counter for RPM

//static volatile int32_t hall_rpm = 0;			// counter to determine RPM

static uint8_t pwm_en, pwm_in;					// Variables for freerun/drive pwm ratios

// Local static functions prototypes
static uint8_t bldc_hallstate();
static void bldc_commute ();



void bldc_init()
{

	pwm_ratio = 0;
	pwm_en = pwm_ratio;
	pwm_in = 100;

	// Init timer 3 -> In
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	// Set all to 0% duty
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	// Init timer 1 -> Enable
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	// Set all to 0% duty
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	force_brake_g = 0;
	force_accel_g = 0;
	locked = 0;
	HAL_TIM_Base_Start(&htim6);	// Start timer for RPM measurement
}

/**************************************************
 * Set torque (range -100 to 100)
 *************************************************/
void bldc_torque(int8_t torque)
{
	dir_field  = (torque>=0);
	pwm_ratio  = (torque<0?-torque:torque);		// abs of torque
	if (pwm_ratio>100) {pwm_ratio = 100;}			// constrain

	// Refresh pwm_en/pwm_in and commute
	bldc_mode(freerun);

	if (freerun)
	{
		force_accel_g = torque;
		force_brake_g = 0;
	}
	else
	{
		force_accel_g = 0;
		force_brake_g = torque;
	}

}

/***********************************************************
 * Switch between freerunning and forced mode
 ***********************************************************/
void bldc_mode(uint8_t mode)
{
	freerun = (mode==1);

	if (freerun==BLDC_FREERUN)	// do not drive IN, but EN -> Free running if torque is lower than current motor speed
	{
		pwm_en = pwm_ratio;
		pwm_in = 100;
	}
	else
	{
		pwm_en = 100;
		pwm_in = pwm_ratio;
	}

	bldc_commute();
}


/***************************************************************************
 * Calculate RPM value from counter sum and number of commutations
 *
 * 	  CNT: 1000000 per Second -> CNT/1MHz = t (elapsed time)
 * 	  this is 1/48 of a rotation -> 	T=t*48 is time for one rotation (in sec)
 * 	  rps = 1/T   (rotations per second)
 * 	  rpm= 60/T   = 60/(48*cnt/1MHz) = (60*1MHz/48) /cnt
 ***************************************************************************/
void bldc_calc_rpm()
{

	if (hallcnt_g ==0)
	{
		rpm_g = 0;
	}
	else
	{
		rpm_g = dir_rot*(float)(60.*1000000./48.)*cnt_upd/hallcnt_g;
		hallcnt_g = 0;
	}

	cnt_upd = 0;

	/*}*/
}
/***************************************************************************
 * Get hall state and compare with last state to detect direction and speed
 *
 * IMPORTANT: The wheel might rotate in opposite direction as currently
 *            driven when braking!
 ***************************************************************************/
static uint8_t bldc_hallstate()
{
	static uint8_t state=0;			// keep old state
	uint8_t cw_state,ccw_state;		// expected next/prev state
	static int8_t dir_old=0;		// keep direction from last run

	cw_state  = hall_cw[state];	 // expected state when rotating CW (at this point state is the old state)
	ccw_state = hall_ccw[state]; // expected state when rotating CCW

	// get current state
	/*
	state  = HAL_GPIO_ReadPin( HALL1_GPIO_Port, HALL1_Pin )
	       | (HAL_GPIO_ReadPin( HALL2_GPIO_Port, HALL2_Pin )<<1)
	       | (HAL_GPIO_ReadPin( HALL3_GPIO_Port, HALL3_Pin )<<2);
	*/
	state = (HALL_A_GPIO_Port->IDR & 0xE000) >> 13;	// Faster / one read  (PB13,PB14,PB15)
	//           |---   0 or 1  ---|
	dir_rot = (cw_state == state) - (ccw_state == state);
	if (dir_rot != dir_old)
	{
		dir_old=dir_rot;
		htim6.Instance->CNT = 0;  // Reset sum counter
		cnt_upd=0;
	}
	// Timer/Counter for rpm measurement
	hallcnt_g  += htim6.Instance->CNT;	// Sum up counter value
	htim6.Instance->CNT = 0;  			// Reset sum counter
	cnt_upd++;							// Count number of commute events

	return state;
}

/**************************************************************************
 *
 * Update coil voltages / pwm / currents according to selected pwm and
 * direction
 *
 * This function is mostly called from the ISR, but also
 * when changing parameters to ensure motors spins up.
 *
 **************************************************************************/
static void bldc_commute(){
	uint8_t state;
	uint8_t coilconf;

	state = bldc_hallstate();	// get current hall state and detect rotation direction/speed

	if (locked) return;		// do nothing if motor is locked / not initialized

	coilconf = bldc_drive[dir_field][state];	// get coils to drive


	// configure coils depending on current state
	switch (coilconf)
	{
	case 12:  // current from coil 1 to coil 2
		// EN1..3
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		// IN1..3
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_in);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		break;

	case 13:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_en);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_in);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		break;

	case 21:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_in);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		break;

	case 23:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_en);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_in);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		break;

	case 31:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_en);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_in);
		break;

	case 32:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_en);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_en);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_in);
		break;
	default: break;
	}
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
	// Clear all EXTI-Sources (as we don't use HAL)
	__HAL_GPIO_EXTI_CLEAR_IT(HALL_A_Pin|HALL_B_Pin|HALL_C_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(HALL2_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(HALL3_Pin);
	bldc_commute();
}






