/*
 * bmi088a.c
 *
 *  Created on: Apr 7, 2025
 *      Author: tobi
 *
 *  https://github.com/bolderflight/bmi088-arduino/tree/main
 */




/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2021 Bolder Flight Systems
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute,
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include "main.h"
#include <math.h>
#include "bmi088a.h"
#include "bmi088a_defs.h"
#include "bmi088_i2c.h"

/* Macros to get and set register fields */
#define GET_FIELD(regname,value) (((uint8_t)value & (uint8_t)regname##_MASK) >> (uint8_t)regname##_POS)
#define	SET_FIELD(regval,regname,value) (((uint8_t)regval & (uint8_t)~regname##_MASK) | (((uint8_t)value << (uint8_t)regname##_POS) & (uint8_t)regname##_MASK))

// Feature Config array
#include <bmi088_feature_cfg.h>


/* writes a byte to BMI088 register given a register address and data */
static void bmi088a_writeRegister(uint8_t subAddress, uint8_t data);

/* writes multiple bytes to BMI088 register given a register address and data */
//static void bmi088a_writeRegisters(uint8_t subAddress, uint8_t count, const uint8_t* data);

/* reads registers from BMI088 given a starting register address, number of bytes, and a pointer to store data */
static void bmi088a_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);


// convert G to m/s/s
#define GRAVITY 9.807f
// accel full scale range
static float accel_range_mss;
static acc_data_raw_t acc_raw;

// temperature data
float temp_c;
// sensor time

static uint8_t _buffer[33];

uint32_t current_time_counter, prev_time_counter = 0;
uint64_t time_counter;

/* begins communication with the BMI088 accel */
uint8_t bmi088a_init()
{
  /* starting the I2C bus */

  /* check device id */
  if (bmi088a_isCorrectId()) {
    return  1;
  }
  HAL_Delay(1);
  /* soft reset */
  bmi088a_softReset();
  /* enable the accel */
  if (bmi088a_setPower(BMI088_ENABLE)) {
    return  2;
  }
  /* enter active mode */
  if (bmi088a_setMode(BMI088_ENABLE)) {
    return  3;
  }
  /* self test */
  if (bmi088a_selfTest()) {
    return  4;
  }
  /* soft reset */
  HAL_Delay(1);
  bmi088a_softReset();
//  bmi088_i2c_reset();

  /* enable the accel */
  if (bmi088a_setPower(BMI088_ENABLE)) {
    return  5;
  }
  /* enter active mode */
  if (bmi088a_setMode(BMI088_ENABLE)) {
    return  6;
  }
  /* set default range */
  if (bmi088a_setRange(ACC_RANGE_24G)) {
    return  7;
  }
  HAL_Delay(5);
  /* set default ODR */
  if (bmi088a_setOdr(ACC_ODR_400HZ_BW_145HZ)) {
    return  8;
  }
  /* check config errors */
  if (bmi088a_isConfigErr()) {
    return  9;
  }
  /* check fatal errors */
  if (bmi088a_isFatalErr()) {
    return  10;
  }
  HAL_Delay(2);


  return 0;
}

/* sets the BMI088 output data rate */
uint8_t bmi088a_setOdr(uint8_t odr)
{
  uint8_t writeReg = 0, readReg = 0;
  writeReg = SET_FIELD(writeReg,ACC_ODR,odr);
  bmi088a_writeRegister(ACC_ODR_ADDR,writeReg);
  HAL_Delay(5);
  bmi088a_readRegisters(ACC_ODR_ADDR,1,&readReg);
  //printf("SetODR: w: 0x%02x  r: 0x%02x\r\n",writeReg, readReg);
  return !(readReg == writeReg);
}

/* sets the BMI088 range */
uint8_t bmi088a_setRange(uint8_t range)
{
  uint8_t writeReg = 0, readReg = 0;
  bmi088a_readRegisters(ACC_RANGE_ADDR,1,&readReg);
  writeReg = SET_FIELD(readReg,ACC_RANGE,range);
  bmi088a_writeRegister(ACC_RANGE_ADDR,writeReg);
  HAL_Delay(1);
  bmi088a_readRegisters(ACC_RANGE_ADDR,1,&readReg);
  if (readReg == writeReg) {
    switch (range) {
      case ACC_RANGE_3G: {
        accel_range_mss = 3.0f * GRAVITY;
        break;
      }
      case ACC_RANGE_6G: {
        accel_range_mss = 6.0f * GRAVITY;
        break;
      }
      case ACC_RANGE_12G: {
        accel_range_mss = 12.0f * GRAVITY;
        break;
      }
      case ACC_RANGE_24G: {
        accel_range_mss = 24.0f * GRAVITY;
        break;
      }
    }
    return 0;
  } else {
    return 1;
  }
}

/* sets the Int1 pin configuration */
/*uint8_t bmi088a_pinModeInt1(PinMode mode, PinLevel level)
{
  // XXX Todo
  return pinModeInt1(PIN_OUTPUT,mode,level);
}
*/
/* sets the Int2 pin configuration */
/*uint8_t bmi088a_pinModeInt2(PinMode mode, PinLevel level)
{
  return pinModeInt2(PIN_OUTPUT,mode,level);
}
*/

/* maps the data
 * y signal to the Int1 pin */
uint8_t bmi088a_mapDrdyInt1(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  bmi088a_readRegisters(ACC_INT1_DRDY_ADDR,1,&readReg);
  writeReg = SET_FIELD(readReg,ACC_INT1_DRDY,enable);
  bmi088a_writeRegister(ACC_INT1_DRDY_ADDR,writeReg);
  HAL_Delay(1);
  bmi088a_readRegisters(ACC_INT1_DRDY_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* maps the data ready signal to the Int2 pin */
uint8_t bmi088a_mapDrdyInt2(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  bmi088a_readRegisters(ACC_INT2_DRDY_ADDR,1,&readReg);
  writeReg = SET_FIELD(readReg,ACC_INT2_DRDY,enable);
  bmi088a_writeRegister(ACC_INT2_DRDY_ADDR,writeReg);
  HAL_Delay(1);
  bmi088a_readRegisters(ACC_INT2_DRDY_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* returns whether data is ready or not */
uint8_t bmi088a_getDrdyStatus()
{
  uint8_t readReg = 0;
  bmi088a_readRegisters(ACC_DRDY_ADDR,1,&readReg);
  return (GET_FIELD(ACC_DRDY,readReg)!=0);
}

/* reads the BMI088 accel */
void bmi088a_readSensor()
{
  /* accel data */
  uint16_t temp_uint11;
  int16_t temp_int11;
  bmi088a_readRegisters(ACC_ACCEL_DATA_ADDR,9,_buffer);
  acc_raw.ax = (_buffer[1] << 8) | _buffer[0];
  acc_raw.ay = (_buffer[3] << 8) | _buffer[2];
  acc_raw.az = (_buffer[5] << 8) | _buffer[4];
  /* time data */
  current_time_counter = (_buffer[8] << 16) | (_buffer[7] << 8) | _buffer[6];
  time_counter = current_time_counter - prev_time_counter;
  prev_time_counter = current_time_counter;
  /* temperature data */
  bmi088a_readRegisters(ACC_TEMP_DATA_ADDR,2,_buffer);
  temp_uint11 = (_buffer[0] * 8) + (_buffer[1] / 32);
  if (temp_uint11 > 1023) {
    temp_int11 = temp_uint11 - 2048;
  } else {
    temp_int11 = temp_uint11;
  }
  temp_c = (float) temp_int11 * 0.125f + 23.0f;
}


void bmi088a_get_raw(acc_data_raw_t *raw)
{
	raw = &acc_raw;
}
void bmi088a_get_grav(acc_data_grav_t *grav)
{
	grav->ax = (float) ( acc_raw.ax) / 32768.0f * accel_range_mss;
	grav->ay = (float) (-acc_raw.ay) / 32768.0f * accel_range_mss;
	grav->az = (float) (-acc_raw.az) / 32768.0f * accel_range_mss;
}

/* returns the temperature, C */
float bmi088a_getTemperature_C()
{
  return temp_c;
}

/* returns the sensor time, ps */
uint64_t bmi088a_getTime_ps()
{
  return time_counter * 39062500;
}

/* sets the Int1 pin configuration */
uint8_t bmi088a_pinModeInt1(uint8_t io, uint8_t mode, uint8_t level)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t pin_io, pin_mode, active_lvl;
  bmi088a_readRegisters(ACC_INT1_IO_CTRL_ADDR,1,&readReg);
  switch (io) {
    case ACC_PINIO_INPUT: {
      pin_io = ACC_INT_INPUT;
      break;
    }
    case ACC_PINIO_OUTPUT: {
      pin_io = ACC_INT_OUTPUT;
      break;
    }
    default: {
      pin_io = ACC_INT_OUTPUT;
      break;
    }
  }
  switch (mode) {
    case ACC_PINMODE_PUSH_PULL: {
      pin_mode = ACC_INT_PUSHPULL;
      break;
    }
    case ACC_PINMODE_OPEN_DRAIN: {
      pin_mode = ACC_INT_OPENDRAIN;
      break;
    }
    default: {
      pin_mode = ACC_INT_PUSHPULL;
      break;
    }
  }
  switch (level) {
    case ACC_PINLVL_ACTIVE_HIGH: {
      active_lvl = ACC_INT_LVL_HIGH;
      break;
    }
    case ACC_PINLVL_ACTIVE_LOW: {
      active_lvl = ACC_INT_LVL_LOW;
      break;
    }
    default: {
      active_lvl = ACC_INT_LVL_HIGH;
      break;
    }
  }
  writeReg = SET_FIELD(readReg,ACC_INT1_IO_CTRL,(pin_io | pin_mode | active_lvl));
  bmi088a_writeRegister(ACC_INT1_IO_CTRL_ADDR,writeReg);
  HAL_Delay(1);
  bmi088a_readRegisters(ACC_INT1_IO_CTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* sets the Int2 pin configuration */
uint8_t bmi088a_pinModeInt2(uint8_t io, uint8_t mode, uint8_t level)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t pin_io, pin_mode, active_lvl;
  bmi088a_readRegisters(ACC_INT2_IO_CTRL_ADDR,1,&readReg);
  switch (io) {
    case ACC_PINIO_INPUT: {
      pin_io = ACC_INT_INPUT;
      break;
    }
    case ACC_PINIO_OUTPUT: {
      pin_io = ACC_INT_OUTPUT;
      break;
    }
    default: {
      pin_io = ACC_INT_OUTPUT;
      break;
    }
  }
  switch (mode) {
    case ACC_PINMODE_PUSH_PULL: {
      pin_mode = ACC_INT_PUSHPULL;
      break;
    }
    case ACC_PINMODE_OPEN_DRAIN: {
      pin_mode = ACC_INT_OPENDRAIN;
      break;
    }
    default: {
      pin_mode = ACC_INT_PUSHPULL;
      break;
    }
  }
  switch (level) {
    case ACC_PINLVL_ACTIVE_HIGH: {
      active_lvl = ACC_INT_LVL_HIGH;
      break;
    }
    case ACC_PINLVL_ACTIVE_LOW: {
      active_lvl = ACC_INT_LVL_LOW;
      break;
    }
    default: {
      active_lvl = ACC_INT_LVL_HIGH;
      break;
    }
  }
  writeReg = SET_FIELD(readReg,ACC_INT2_IO_CTRL,(pin_io | pin_mode | active_lvl));
  bmi088a_writeRegister(ACC_INT2_IO_CTRL_ADDR,writeReg);
  HAL_Delay(1);
  bmi088a_readRegisters(ACC_INT2_IO_CTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* performs BMI088 accel self test */
uint8_t bmi088a_selfTest()
{
  uint8_t writeReg = 0;
  acc_data_grav_t acc, acc_pos_mg, acc_neg_mg;

  /* set 24G range */
  bmi088a_setRange(ACC_RANGE_24G);
  /* set 1.6 kHz ODR, 4x oversampling */
  bmi088a_setOdr(ACC_ODR_1600HZ_BW_145HZ);
  /* wait >2 ms */
  HAL_Delay(3);
  /* enable self test, positive polarity */
  writeReg = SET_FIELD(writeReg,ACC_SELF_TEST,ACC_POS_SELF_TEST);
  bmi088a_writeRegister(ACC_SELF_TEST_ADDR,writeReg);
  /* wait >50 ms */
  HAL_Delay(51);
  /* read self test values */
  bmi088a_readSensor();
  bmi088a_get_grav(&acc);
  acc_pos_mg.ax = acc.ax / GRAVITY * 1000.0f;
  acc_pos_mg.ay = acc.ay / GRAVITY * 1000.0f;
  acc_pos_mg.az = acc.az / GRAVITY * 1000.0f;

  /* enable self test, negative polarity */
  writeReg = SET_FIELD(writeReg,ACC_SELF_TEST,ACC_NEG_SELF_TEST);
  bmi088a_writeRegister(ACC_SELF_TEST_ADDR,writeReg);
  /* wait >50 ms */
  HAL_Delay(51);
  /* read self test values */
  bmi088a_readSensor();
  bmi088a_get_grav(&acc);
  acc_neg_mg.ax = acc.ax / GRAVITY * 1000.0f;
  acc_neg_mg.ay = acc.ay / GRAVITY * 1000.0f;
  acc_neg_mg.az = acc.az / GRAVITY * 1000.0f;

  /* disable self test */
  writeReg = SET_FIELD(writeReg,ACC_SELF_TEST,ACC_DIS_SELF_TEST);
  bmi088a_writeRegister(ACC_SELF_TEST_ADDR,writeReg);
  /* wait >50 ms */
  HAL_Delay(51);
  /* check self test results */
  if ((fabs(acc_pos_mg.ax - acc_neg_mg.ax) >= 1000) &&
	  (fabs(acc_pos_mg.ay - acc_neg_mg.ay) >= 1000) &&
	  (fabs(acc_pos_mg.az - acc_neg_mg.az) >= 500))
  {
	return 0;
  }
  else {
    return 1;
  }
}

/* sets whether the sensor is in active or suspend mode */
uint8_t bmi088a_setMode(uint8_t active)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t value = (active) ? ACC_ACTIVE_MODE_CMD : ACC_SUSPEND_MODE_CMD;
  writeReg = SET_FIELD(writeReg,ACC_PWR_CONF,value);
  bmi088a_writeRegister(ACC_PWR_CONF_ADDR,writeReg);
  HAL_Delay(6); // 5 ms wait after power mode changes
  bmi088a_readRegisters(ACC_PWR_CONF_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* sets whether the sensor is enabled or disabled */
uint8_t bmi088a_setPower(uint8_t enable)
{
  uint8_t writeReg = 0, readReg = 0;
  uint8_t value = (enable) ? ACC_ENABLE_CMD : ACC_DISABLE_CMD;
  writeReg = SET_FIELD(writeReg,ACC_PWR_CNTRL,value);
  bmi088a_writeRegister(ACC_PWR_CNTRL_ADDR,writeReg);
  HAL_Delay(6); // 5 ms wait after power mode changes
  bmi088a_readRegisters(ACC_PWR_CNTRL_ADDR,1,&readReg);
  return !(readReg == writeReg);
}

/* performs a soft reset */
void bmi088a_softReset()
{
  uint8_t reg = 0;

  // Tobi:
  // Disable before Soft reset ...
  // Don't know why ... otherwise not working ...
  bmi088a_setMode(BMI088_DISABLE);
  bmi088a_setPower(BMI088_DISABLE);

  reg = SET_FIELD(reg,ACC_SOFT_RESET,ACC_RESET_CMD);
  bmi088_i2c_write_nack(BMI088_ACCEL_ADDR,  ACC_SOFT_RESET_ADDR, & reg, 1);
  HAL_Delay(50);
}

/* checks the BMI088 for configuration errors */
uint8_t bmi088a_isConfigErr()
{
  uint8_t readReg = 0;
  bmi088a_readRegisters(ACC_ERR_CODE_ADDR,1,&readReg);
  return (GET_FIELD(ACC_ERR_CODE,readReg));
}

/* checks the BMI088 for fatal errors */
uint8_t bmi088a_isFatalErr()
{
  uint8_t readReg = 0;
  bmi088a_readRegisters(ACC_FATAL_ERR_ADDR,1,&readReg);
  return (GET_FIELD(ACC_FATAL_ERR,readReg));
}

/* checks the BMI088 accelerometer ID */
uint8_t bmi088a_isCorrectId()
{
  uint8_t readReg = 0;
  bmi088a_readRegisters(ACC_CHIP_ID_ADDR,1,&readReg);
  return !(GET_FIELD(ACC_CHIP_ID,readReg) == ACC_CHIP_ID);
}

/* writes a byte to BMI088 register given a register address and data */
void bmi088a_writeRegister(uint8_t subAddress, uint8_t data)
{
  bmi088_i2c_write(BMI088_ACCEL_ADDR, subAddress, &data, 1);
}

/* writes multiple bytes to BMI088 register given a register address and data */
/*void bmi088a_writeRegisters(uint8_t subAddress, uint8_t count, const uint8_t* data)
{
  bmi088_i2c_write(BMI088_ACCEL_ADDR, subAddress, data, count);

}*/

/* reads registers from BMI088 given a starting register address, number of bytes, and a pointer to store data */
void bmi088a_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  bmi088_i2c_read(BMI088_ACCEL_ADDR, subAddress, dest, count);
}
