/*
 * mpu6600_iic.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef MPU6600_IIC_H_
#define MPU6600_IIC_H_

#include "iic_software.h"

#if(MPU_USE_IIC)

typedef enum{
	I2C_RESULT_OK = 0x00,
	I2C_RESULT_ERROR = 0x01
}I2C_ResultType;


void iicInit(void);

I2C_ResultType iicWriteReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dataBuf, uint16_t Size);
I2C_ResultType iicReadReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dataBuf, uint16_t Size);

I2C_ResultType iicWrite(uint8_t deviceAddr, uint8_t* dataBuf, uint16_t Size);
I2C_ResultType iicRead(uint8_t deviceAddr, uint8_t* dataBuf, uint16_t Size);

I2C_ResultType isDeviceConnected(uint8_t deviceAddr);

#endif
#endif /* MPU6600_IIC_H_ */
