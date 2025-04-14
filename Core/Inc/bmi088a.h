/*
 * bmi088a.h
 *
 *  Created on: Apr 7, 2025
 *      Author: tobi
 */

#ifndef INC_BMI088A_H_
#define INC_BMI088A_H_

#include "bmi088a_defs.h"

#ifndef BMI088_ENABLE
 	#define BMI088_ENABLE 1
	#define BMI088_DISABLE 0

	#define BMI088_ACCEL_ADDR 0x19
#endif

typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
} acc_data_raw_t;

typedef struct {
	float ax;
	float ay;
	float az;
} acc_data_grav_t;

uint8_t bmi088a_init();
//uint8_t bmi088_accel_setOdr(uint8_t odr);
uint8_t bmi088a_setRange(uint8_t range);
uint8_t bmi088a_pinModeInt1(uint8_t io, uint8_t mode, uint8_t level);
uint8_t bmi088a_pinModeInt2(uint8_t io, uint8_t mode, uint8_t level);
uint8_t bmi088a_mapDrdyInt1(uint8_t enable);
uint8_t bmi088a_mapDrdyInt2(uint8_t enable);
uint8_t bmi088a_getDrdyStatus();
void bmi088a_readSensor();
void bmi088a_get_raw(acc_data_raw_t *raw);
void bmi088a_get_grav(acc_data_grav_t *grav);
uint64_t bmi088a_getTime_ps();
uint8_t bmi088a_selfTest();
uint8_t bmi088a_setMode(uint8_t active);
uint8_t bmi088a_setPower(uint8_t enable);
uint8_t bmi088a_setOdr(uint8_t odr);

void bmi088a_softReset();
uint8_t bmi088a_isConfigErr();
uint8_t bmi088a_isFatalErr();
uint8_t bmi088a_isCorrectId();

#endif /* INC_BMI088A_H_ */
