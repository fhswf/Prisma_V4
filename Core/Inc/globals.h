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
 * globals.h
 *
 *  Created on: 07.11.2023
 *      Author: tobi
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define sgn(x)((x)<0?-1:((x)>0)?1:0)
// #define MAIN_C  // ==> Add this in main_user.c before including


// Define Macro, which either extends to
// GLOBAL(int16_t dummy,0)   =>  int16_t dummy=0
// GLOBAL(int16_t dummy,0)   =>  extern int16_t dummy
// depending whether MAIN_C is set or not

#ifdef MAIN_C
#define GLOBAL(a,b) a = b
#else
#define GLOBAL(a,b) extern a
#endif

#define CONTROL_RATE 100		// Update Rate of the controller in Hz

#define BT_INITIALIZING 1
#define BT_READY 2
#define BT_NOT_INITIALIZED 3

#define BLDC_POLE_PAIRS   8			// Number of pole pairs
#define BLDC_EL_STEPS	  6			// number of hall changes for one electrical rotation


GLOBAL(volatile int16_t force_accel_g,0);
GLOBAL(volatile int16_t force_brake_g,0);
GLOBAL(volatile int16_t vbat_mv_g,0);
GLOBAL(volatile uint8_t bt_ok_g,BT_NOT_INITIALIZED);
GLOBAL(volatile uint8_t system_ok_g,0);
GLOBAL(volatile uint8_t balancing_g,0);
GLOBAL(volatile int16_t angle_g,0);
GLOBAL(volatile int32_t hallcnt_g,0);			// counting hall steps for rpm calculation
GLOBAL(volatile float   rpm_g,0.0);
GLOBAL(volatile float   rpma_g,0.0);
GLOBAL(volatile uint8_t trigger_g,0);
GLOBAL(volatile uint32_t ticks_ms_g,0);
GLOBAL(float vbat1_g, 0.0);
GLOBAL(float vbat2_g, 0.0);
GLOBAL(float vbat3_g, 0.0);
GLOBAL(float imot_g, 0.0);

#endif /* INC_GLOBALS_H_ */
