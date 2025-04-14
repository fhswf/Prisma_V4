/*
 * bmi088a_defs.h
 *
 *  Created on: Apr 7, 2025
 *      Author: tobi
 */

#ifndef INC_BMI088A_DEFS_H_
#define INC_BMI088A_DEFS_H_

#define ACC_CHIP_ID           0x1E
#define ACC_RESET_CMD         0xB6
#define ACC_ENABLE_CMD        0x04
#define ACC_DISABLE_CMD       0x00
#define ACC_SUSPEND_MODE_CMD  0x03
#define ACC_ACTIVE_MODE_CMD   0x00
#define ACC_INT_INPUT         0x11
#define ACC_INT_OUTPUT        0x08
#define ACC_INT_OPENDRAIN     0x04
#define ACC_INT_PUSHPULL      0x00
#define ACC_INT_LVL_HIGH      0x02
#define ACC_INT_LVL_LOW       0x00
#define ACC_POS_SELF_TEST     0x0D
#define ACC_NEG_SELF_TEST     0x09
#define ACC_DIS_SELF_TEST     0x00
// registers
#define ACC_CHIP_ID_ADDR      0x00
#define ACC_CHIP_ID_MASK      0xFF
#define ACC_CHIP_ID_POS          0
#define ACC_FATAL_ERR_ADDR    0x02
#define ACC_FATAL_ERR_MASK    0x01
#define ACC_FATAL_ERR_POS        0
#define ACC_ERR_CODE_ADDR     0x02
#define ACC_ERR_CODE_MASK     0x1C
#define ACC_ERR_CODE_POS         2
#define ACC_DRDY_ADDR         0x03
#define ACC_DRDY_MASK         0x80
#define ACC_DRDY_POS             7
#define ACC_ODR_ADDR          0x40
#define ACC_ODR_MASK          0xFF
#define ACC_ODR_POS              0
#define ACC_RANGE_ADDR        0x41
#define ACC_RANGE_MASK        0x03
#define ACC_RANGE_POS            0
#define ACC_INT1_IO_CTRL_ADDR 0x53
#define ACC_INT1_IO_CTRL_MASK 0x1F
#define ACC_INT1_IO_CTRL_POS     0
#define ACC_INT2_IO_CTRL_ADDR 0x54
#define ACC_INT2_IO_CTRL_MASK 0x1F
#define ACC_INT2_IO_CTRL_POS     0
#define ACC_INT1_DRDY_ADDR    0x58
#define ACC_INT1_DRDY_MASK    0x04
#define ACC_INT1_DRDY_POS        2
#define ACC_INT2_DRDY_ADDR    0x58
#define ACC_INT2_DRDY_MASK    0x40
#define ACC_INT2_DRDY_POS        6
#define ACC_SELF_TEST_ADDR    0x6D
#define ACC_SELF_TEST_MASK    0xFF
#define ACC_SELF_TEST_POS       0
#define ACC_PWR_CONF_ADDR     0x7C
#define ACC_PWR_CONF_MASK     0xFF
#define ACC_PWR_CONF_POS         0
#define ACC_PWR_CNTRL_ADDR    0x7D
#define ACC_PWR_CNTRL_MASK    0xFF
#define ACC_PWR_CNTRL_POS        0
#define ACC_SOFT_RESET_ADDR   0x7E
#define ACC_SOFT_RESET_MASK   0xFF
#define ACC_SOFT_RESET_POS       0
#define ACC_ACCEL_DATA_ADDR   0x12
#define ACC_TEMP_DATA_ADDR    0x22


#define ACC_RANGE_3G   0x00
#define ACC_RANGE_6G   0x01
#define ACC_RANGE_12G  0x02
#define ACC_RANGE_24G  0x03

#define ACC_ODR_1600HZ_BW_280HZ  0xAC
#define ACC_ODR_1600HZ_BW_234HZ  0x9C
#define ACC_ODR_1600HZ_BW_145HZ  0x8C
#define ACC_ODR_800HZ_BW_230HZ   0xAB
#define ACC_ODR_800HZ_BW_140HZ   0x9B
#define ACC_ODR_800HZ_BW_80HZ    0x8B
#define ACC_ODR_400HZ_BW_145HZ   0xAA
#define ACC_ODR_400HZ_BW_75HZ    0x9A
#define ACC_ODR_400HZ_BW_40HZ    0x8A
#define ACC_ODR_200HZ_BW_80HZ    0xA9
#define ACC_ODR_200HZ_BW_38HZ    0x99
#define ACC_ODR_200HZ_BW_20HZ    0x89
#define ACC_ODR_100HZ_BW_40HZ    0xA8
#define ACC_ODR_100HZ_BW_19HZ    0x98
#define ACC_ODR_100HZ_BW_10HZ    0x88
#define ACC_ODR_50HZ_BW_20HZ     0xA7
#define ACC_ODR_50HZ_BW_9HZ      0x97
#define ACC_ODR_50HZ_BW_5HZ      0x87
#define ACC_ODR_25HZ_BW_10HZ     0xA6
#define ACC_ODR_25HZ_BW_5HZ      0x96
#define ACC_ODR_25HZ_BW_3HZ      0x86
#define ACC_ODR_12_5HZ_BW_5HZ    0xA5
#define ACC_ODR_12_5HZ_BW_2HZ    0x95
#define ACC_ODR_12_5HZ_BW_1HZ    0x85
#define ACC_ODR_DEFAULT          0xAC

#define ACC_PINMODE_PUSH_PULL    0
#define ACC_PINMODE_OPEN_DRAIN   1
#define ACC_PINLVL_ACTIVE_HIGH   0
#define ACC_PINLVL_ACTIVE_LOW    1
#define ACC_PINIO_INPUT          0
#define ACC_PINIO_OUTPUT         1

#endif /* INC_BMI088A_DEFS_H_ */
