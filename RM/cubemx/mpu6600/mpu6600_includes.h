/*
 * mpu6600_includes.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef MPU6600_INCLUDES_H_
#define MPU6600_INCLUDES_H_

#include "stm32f4xx_hal.h"

#define MPU_USE_SPI				1
#define MPU_USE_IIC				0

#if(!(MPU_USE_SPI ^ MPU_USE_IIC))
#error "Please selete SPI or IIC"
#endif

#if(MPU_USE_IIC)
#define IIC_SPEED				50
#endif

#endif /* MPU6600_INCLUDES_H_ */
