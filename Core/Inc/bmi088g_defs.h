/*
 * bmi088g_defs.h
 *
 *  Created on: Apr 9, 2025
 *      Author: tobi
 */

#ifndef INC_BMI088G_DEFS_H_
#define INC_BMI088G_DEFS_H_

#define GYRO_RANGE_2000DPS   0x00
#define GYRO_RANGE_1000DPS   0x01
#define GYRO_RANGE_500DPS    0x02
#define GYRO_RANGE_250DPS    0x03
#define GYRO_RANGE_125DPS    0x04

#define GYRO_ODR_2000HZ_BW_532HZ   0x80
#define GYRO_ODR_2000HZ_BW_230HZ   0x81
#define GYRO_ODR_1000HZ_BW_116HZ   0x82
#define GYRO_ODR_400HZ_BW_47HZ     0x83
#define GYRO_ODR_200HZ_BW_23HZ     0x84
#define GYRO_ODR_100HZ_BW_12HZ     0x85
#define GYRO_ODR_200HZ_BW_64HZ     0x86
#define GYRO_ODR_100HZ_BW_32HZ     0x87

#define GYRO_PINMODE_PUSH_PULL    0
#define GYRO_PINMODE_OPEN_DRAIN   1
#define GYRO_PINLVL_ACTIVE_HIGH   0
#define GYRO_PINLVL_ACTIVE_LOW    1

#define GYRO_PWR_NORMAL         0x00
#define GYRO_PWR_SUSPEND        0x80
#define GYRO_PWR_DEEP_SUSPEND   0x20

#define GYRO_CHIP_ID          0x0F
#define GYRO_RESET_CMD        0xB6
#define GYRO_ENABLE_DRDY_INT  0x80
#define GYRO_DIS_DRDY_INT     0x00
#define GYRO_INT_OPENDRAIN    0x02
#define GYRO_INT_PUSHPULL     0x00
#define GYRO_INT_LVL_HIGH     0x01
#define GYRO_INT_LVL_LOW      0x00
// registers
#define GYRO_CHIP_ID_ADDR     0x00
#define GYRO_CHIP_ID_MASK     0xFF
#define GYRO_CHIP_ID_POS         0
#define GYRO_DRDY_ADDR        0x0A
#define GYRO_DRDY_MASK        0x80
#define GYRO_DRDY_POS            7
#define GYRO_RANGE_ADDR       0x0F
#define GYRO_RANGE_MASK       0xFF
#define GYRO_RANGE_POS           0
#define GYRO_ODR_ADDR         0x10
#define GYRO_ODR_MASK         0xFF
#define GYRO_ODR_POS             0
#define GYRO_LPM1_ADDR		  0x11
#define GYRO_LPM1_MASK        0xFF
#define GYRO_LPM1_POS            0
#define GYRO_SOFT_RESET_ADDR  0x14
#define GYRO_SOFT_RESET_MASK  0xFF
#define GYRO_SOFT_RESET_POS      0
#define GYRO_INT_CNTRL_ADDR   0x15
#define GYRO_INT_CNTRL_MASK   0xFF
#define GYRO_INT_CNTRL_POS       0
#define GYRO_INT3_IO_CTRL_ADDR   0x16
#define GYRO_INT3_IO_CTRL_MASK   0x03
#define GYRO_INT3_IO_CTRL_POS       0
#define GYRO_INT4_IO_CTRL_ADDR   0x16
#define GYRO_INT4_IO_CTRL_MASK   0x0C
#define GYRO_INT4_IO_CTRL_POS       2
#define GYRO_INT3_DRDY_ADDR      0x18
#define GYRO_INT3_DRDY_MASK      0x01
#define GYRO_INT3_DRDY_POS          0
#define GYRO_INT4_DRDY_ADDR      0x18
#define GYRO_INT4_DRDY_MASK      0x80
#define GYRO_INT4_DRDY_POS          7
#define GYRO_DATA_ADDR           0x02

#endif /* INC_BMI088G_DEFS_H_ */
