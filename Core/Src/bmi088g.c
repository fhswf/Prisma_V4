/*
 * bmi088g.c
 *
 *  Created on: Apr 9, 2025
 *      Author: tobi
 *
 *  https://github.com/bolderflight/bmi088-arduino/tree/main
 *
 */

#include "main.h"
#include <math.h>
#include "bmi088g.h"
#include "bmi088g_defs.h"
#include "bmi088_i2c.h"



/* Macros to get and set register fields */
#define GET_FIELD(regname,value) (((uint8_t)value & (uint8_t)regname##_MASK) >> (uint8_t)regname##_POS)
#define	SET_FIELD(regval,regname,value) (((uint8_t)regval & (uint8_t)~regname##_MASK) | (((uint8_t)value << (uint8_t)regname##_POS) & (uint8_t)regname##_MASK))

/* writes a byte to BMI088 register given a register address and data */
static void bmi088g_writeRegister(uint8_t subAddress, uint8_t data);

/* reads registers from BMI088 given a starting register address, number of bytes, and a pointer to store data */
static void bmi088g_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);

// convert deg/s to rad/s
#define GYRO_DEG2RAD   M_PI / 180.0f
// gyro full scale range
static float gyro_range_rads;
static gyro_data_raw_t   gyro_raw;


/* begins communication with the BMI088 gyro */
uint8_t bmi088g_init()
{
  /* check device id */
  if (bmi088g_isCorrectId()) {
    return 1;
  }
  /* soft reset */
  bmi088g_softReset();

  /* set default range */
  if (bmi088g_setRange(GYRO_RANGE_2000DPS)) {
    return 2;
  }

  /* enable data ready int */
  if (bmi088g_setDrdyInt(BMI088_ENABLE)) {
    return 3;
  }
  /* set default ODR */
  if (bmi088g_setOdr(GYRO_ODR_2000HZ_BW_532HZ)) {
    return 4;
  }
  return 0;
}

/* sets the BMI088 output data rate */
uint8_t bmi088g_setOdr(uint8_t odr)
{
  uint8_t writeReg = 0, readReg = 0;
  writeReg = SET_FIELD(writeReg,GYRO_ODR,odr);
  bmi088g_writeRegister(GYRO_ODR_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_ODR_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* sets the BMI088 range */
uint8_t bmi088g_setRange(uint8_t range)
{
  uint8_t writeReg = 0, readReg = 0;
  writeReg = SET_FIELD(writeReg,GYRO_RANGE,range);
  bmi088g_writeRegister(GYRO_RANGE_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_RANGE_ADDR,1,&readReg);
  if (readReg == writeReg) {
    switch (range) {
      case GYRO_RANGE_125DPS: {
        gyro_range_rads = 125.0f * GYRO_DEG2RAD;
        break;
      }
      case GYRO_RANGE_250DPS: {
        gyro_range_rads = 250.0f * GYRO_DEG2RAD;
        break;
      }
      case GYRO_RANGE_500DPS: {
        gyro_range_rads = 500.0f * GYRO_DEG2RAD;
        break;
      }
      case GYRO_RANGE_1000DPS: {
        gyro_range_rads = 1000.0f * GYRO_DEG2RAD;
        break;
      }
      case GYRO_RANGE_2000DPS: {
        gyro_range_rads = 2000.0f * GYRO_DEG2RAD;
        break;
      }
    }
    return 0;
  } else {
    return 1;
  }
}

/* sets the Int3 pin configuration */
uint8_t bmi088g_pinModeInt3(uint8_t mode, uint8_t level)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t pin_mode, active_lvl;
  bmi088g_readRegisters(GYRO_INT3_IO_CTRL_ADDR,1,&readReg);
  switch (mode) {
    case GYRO_PINMODE_PUSH_PULL: {
      pin_mode = GYRO_INT_PUSHPULL;
      break;
    }
    case GYRO_PINMODE_OPEN_DRAIN: {
      pin_mode = GYRO_INT_OPENDRAIN;
      break;
    }
    default: {
      pin_mode = GYRO_INT_PUSHPULL;
      break;
    }
  }
  switch (level) {
    case GYRO_PINLVL_ACTIVE_HIGH: {
      active_lvl = GYRO_INT_LVL_HIGH;
      break;
    }
    case GYRO_PINLVL_ACTIVE_LOW: {
      active_lvl = GYRO_INT_LVL_LOW;
      break;
    }
    default: {
      active_lvl = GYRO_INT_LVL_HIGH;
      break;
    }
  }
  writeReg = SET_FIELD(readReg,GYRO_INT3_IO_CTRL,(pin_mode | active_lvl));
  bmi088g_writeRegister(GYRO_INT3_IO_CTRL_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_INT3_IO_CTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* sets the Int4 pin configuration */
uint8_t bmi088g_pinModeInt4(uint8_t mode, uint8_t level)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t pin_mode, active_lvl;
  bmi088g_readRegisters(GYRO_INT4_IO_CTRL_ADDR,1,&readReg);
  switch (mode) {
    case GYRO_PINMODE_PUSH_PULL: {
      pin_mode = GYRO_INT_PUSHPULL;
      break;
    }
    case GYRO_PINMODE_OPEN_DRAIN: {
      pin_mode = GYRO_INT_OPENDRAIN;
      break;
    }
    default: {
      pin_mode = GYRO_INT_PUSHPULL;
      break;
    }
  }
  switch (level) {
    case GYRO_PINLVL_ACTIVE_HIGH: {
      active_lvl = GYRO_INT_LVL_HIGH;
      break;
    }
    case GYRO_PINLVL_ACTIVE_LOW: {
      active_lvl = GYRO_INT_LVL_LOW;
      break;
    }
    default: {
      active_lvl = GYRO_INT_LVL_HIGH;
      break;
    }
  }
  writeReg = SET_FIELD(readReg,GYRO_INT4_IO_CTRL,(pin_mode | active_lvl));
  bmi088g_writeRegister(GYRO_INT4_IO_CTRL_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_INT4_IO_CTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* maps the data ready signal to the Int3 pin */
uint8_t bmi088g_mapDrdyInt3(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  bmi088g_readRegisters(GYRO_INT3_DRDY_ADDR,1,&readReg);
  writeReg = SET_FIELD(readReg,GYRO_INT3_DRDY,enable);
  bmi088g_writeRegister(GYRO_INT3_DRDY_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_INT3_DRDY_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* maps the data ready signal to the Int4 pin */
uint8_t bmi088g_mapDrdyInt4(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  bmi088g_readRegisters(GYRO_INT4_DRDY_ADDR,1,&readReg);
  writeReg = SET_FIELD(readReg,GYRO_INT4_DRDY,enable);
  bmi088g_writeRegister(GYRO_INT4_DRDY_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_INT4_DRDY_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* returns whether data is ready or not */
uint8_t bmi088g_getDrdyStatus()
{
  uint8_t readReg = 0;
  bmi088g_readRegisters(GYRO_DRDY_ADDR,1,&readReg);
  return (GET_FIELD(GYRO_DRDY,readReg)!=0);
}

/* reads the BMI088 gyro */
void bmi088g_readSensor()
{
  uint8_t buffer[6];
  /* accel data */
  bmi088g_readRegisters(GYRO_DATA_ADDR,6, buffer);
  gyro_raw.gx = (buffer[1] << 8) | buffer[0];
  gyro_raw.gy = (buffer[3] << 8) | buffer[2];
  gyro_raw.gz = (buffer[5] << 8) | buffer[4];
}


void bmi088g_get_raw(gyro_data_raw_t *raw)
{
	raw = &gyro_raw;
}

void bmi088g_get_rads(gyro_data_rads_t *rads)
{
	  rads->gx = (float) ( gyro_raw.gx) / 32767.0f * gyro_range_rads;
	  rads->gy = (float) (-gyro_raw.gy) / 32767.0f * gyro_range_rads;
	  rads->gz = (float) (-gyro_raw.gz) / 32767.0f * gyro_range_rads;
}

// bool Bmi088Gyro::selfTest()
// {

// }

/* enables the new data interrupt */
uint8_t bmi088g_setDrdyInt(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t value = (enable) ? GYRO_ENABLE_DRDY_INT : GYRO_DIS_DRDY_INT;
  writeReg = SET_FIELD(writeReg,GYRO_INT_CNTRL,value);
  bmi088g_writeRegister(GYRO_INT_CNTRL_ADDR,writeReg);
  HAL_Delay(1);
  bmi088g_readRegisters(GYRO_INT_CNTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* performs a soft reset */
void bmi088g_softReset()
{
  uint8_t reg = 0;
  reg = SET_FIELD(reg, GYRO_LPM1, GYRO_PWR_SUSPEND);
  bmi088g_writeRegister(GYRO_LPM1_ADDR, reg);
  reg = SET_FIELD(reg,GYRO_SOFT_RESET,GYRO_RESET_CMD);
  bmi088_i2c_write_nack(BMI088_GYRO_ADDR, GYRO_SOFT_RESET_ADDR, & reg, 1);
  HAL_Delay(50);
}

/* checks the BMI088 gyro ID */
uint8_t bmi088g_isCorrectId()
{
  uint8_t readReg = 0;
  bmi088g_readRegisters(GYRO_CHIP_ID_ADDR,1,&readReg);
  return !(GET_FIELD(GYRO_CHIP_ID,readReg) == GYRO_CHIP_ID);
}

/* writes a byte to BMI088 register given a register address and data */
static void bmi088g_writeRegister(uint8_t subAddress, uint8_t data)
{
  bmi088_i2c_write(BMI088_GYRO_ADDR, subAddress, &data, 1);
}

/* reads registers from BMI088 given a starting register address, number of bytes, and a pointer to store data */
static void bmi088g_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  bmi088_i2c_read(BMI088_GYRO_ADDR, subAddress, dest, count);
}
