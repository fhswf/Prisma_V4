/*
 * bmi088_i2c.c
 *
 *  Created on: Apr 7, 2025
 *      Author: tobi
 */

#include "dwt_delay.h"
#include "bmi088_i2c.h"

#include <stdio.h>
extern I2C_HandleTypeDef BMI088_I2C_HANDLE;

#define BMI08_BUS_TIMEOUT     100


/******************************************************************************/
/*!                User interface functions                                   */
uint8_t bmi088_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    uint8_t status;

    // CAUTION: MEM_READ not working with BMI088!!!
    status = HAL_I2C_Master_Transmit(& BMI088_I2C_HANDLE, dev_addr<<1, &reg_addr, 1,BMI08_BUS_TIMEOUT);
    if (status != HAL_OK)
    {
   		printf("ERROR: BMI Read (1) failed (dev 0x%2x reg 0x%02x) status %i\r\n", dev_addr, reg_addr,status);
    }
    else
    {
    	dwt_delay_us(10);	// At least 2us delay? See docu pp45
    	status = HAL_I2C_Master_Receive(& BMI088_I2C_HANDLE, dev_addr<<1, (uint8_t *)reg_data, len, BMI08_BUS_TIMEOUT);
    	if (status != HAL_OK)
    	{
       		printf("ERROR: BMI Read (2) failed (dev 0x%2x reg 0x%02x len %u) status %i\r\n", dev_addr, reg_addr, len, status);
    	}
    }
	dwt_delay_us(5);	// At least 2us delay? See docu pp45
    return status;
}

/*!
 * I2C write function
 */
uint8_t bmi088_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    uint8_t status;
    uint8_t buf[33];
    if (len>32)
    {
    	printf("ERROR: bmi08_i2c_write len>32");
    }
    buf[0]=reg_addr;
    for (int i=0;i<len; i++)
    {
    	buf[i+1] = reg_data[i];
    }
    status = HAL_I2C_Master_Transmit(&BMI088_I2C_HANDLE, dev_addr<<1, buf, len+1, BMI08_BUS_TIMEOUT);
	dwt_delay_us(5);	// At least 2us delay? See docu pp45
    //status = HAL_I2C_Mem_Write(&BMI08_I2C_HANDLE, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, BMI08_BUS_TIMEOUT);
    // Ignore NACK on RESET?
	if (status != HAL_OK)
    {
   		printf("ERROR: BMI Write (1) failed (dev 0x%2x reg 0x%02x d0: 0x%02x) status %i\r\n",
   				dev_addr, reg_addr,reg_data[0], status);
    }
    return status;
}

/*!
 * I2C write function
 *    // Ignore NACK on RESET?
 *
 */
uint8_t bmi088_i2c_write_nack(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    uint8_t status;
    uint8_t buf[33];
    if (len>32)
    {
    	printf("ERROR: bmi08_i2c_write len>32");
    }
    buf[0]=reg_addr;
    for (int i=0;i<len; i++)
    {
    	buf[i+1] = reg_data[i];
    }
    status = HAL_I2C_Master_Transmit(&BMI088_I2C_HANDLE, dev_addr<<1, buf, len+1, BMI08_BUS_TIMEOUT);
	dwt_delay_us(5);	// At least 2us delay? See docu pp45
    return 0;
}

uint8_t bmi088_i2c_reset()
{
	BMI088_I2C_HANDLE.Instance->CR1 |=(1<<15);
	HAL_Delay(2);
	BMI088_I2C_HANDLE.Instance->CR1 &= ~(1<<15);
	HAL_Delay(2);
}


