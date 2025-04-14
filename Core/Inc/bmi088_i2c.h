/*
 * bmi088_i2c.h
 *
 *  Created on: Apr 7, 2025
 *      Author: tobi
 */

#ifndef INC_BMI088_I2C_H_
#define INC_BMI088_I2C_H_

#define BMI088_ACCEL_ADDR 0x19
#define BMI088_GYRO_ADDR  0x69
#define BMI088_I2C_HANDLE hi2c1

uint8_t bmi088_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
uint8_t bmi088_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);
uint8_t bmi088_i2c_write_nack(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);
uint8_t bmi088_i2c_reset();




#endif /* INC_BMI088_I2C_H_ */
