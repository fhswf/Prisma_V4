/*
 * bldc.h
 *
 *  Created on: Oct 26, 2023
 *      Author: tobi
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_

#define BLDC_FREERUN 1
#define BLDC_DRIVE   0

void bldc_init();
void bldc_torque(int8_t torque);
void bldc_freerun(uint8_t mode);
void bldc_mode(uint8_t mode);
void bldc_calc_rpm();



#endif /* INC_BLDC_H_ */
