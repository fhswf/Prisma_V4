/*
 * bmi088g.h
 *
 *  Created on: Apr 9, 2025
 *      Author: tobi
 */

#ifndef INC_BMI088G_H_
#define INC_BMI088G_H_

#include <stdint.h>

#include "bmi088g_defs.h"

#ifndef BMI088_ENABLE
 	#define BMI088_ENABLE 1
	#define BMI088_DISABLE 0

	#define BMI088_GYRO_ADDR 0x69
#endif

typedef struct {
	int16_t gx;
	int16_t gy;
	int16_t gz;
} gyro_data_raw_t;

typedef struct {
	float gx;
	float gy;
	float gz;
} gyro_data_rads_t;

uint8_t bmi088g_init();
uint8_t bmi088g_setOdr(uint8_t odr);
uint8_t bmi088g_setRange(uint8_t range);
uint8_t bmi088g_pinModeInt3(uint8_t mode, uint8_t level);
uint8_t bmi088g_pinModeInt4(uint8_t mode, uint8_t level);
uint8_t bmi088g_mapDrdyInt3(uint8_t enable);
uint8_t bmi088g_mapDrdyInt4(uint8_t enable);
uint8_t bmi088g_getDrdyStatus();

void bmi088g_readSensor();
void bmi088g_get_raw(gyro_data_raw_t *raw);
void bmi088g_get_rads(gyro_data_rads_t *rads);

uint8_t bmi088g_setDrdyInt(uint8_t enable);
void bmi088g_softReset();
uint8_t bmi088g_isCorrectId();



















#endif /* INC_BMI088G_H_ */
