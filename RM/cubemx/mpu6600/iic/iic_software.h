/*
 * iic_software.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef IIC_IIC_SOFTWARE_H_
#define IIC_IIC_SOFTWARE_H_

#include "../mpu6600_includes.h"

#if(MPU_USE_IIC)

#include "stdbool.h"


void IIC_GPIOInit(void);
void IIC_Delay(uint32_t ntime);
bool IIC_Start(void);
void IIC_Stop(void);
void IIC_SendByte(unsigned char chr);
unsigned char IIC_ReadByte(void);
void IIC_Ack(void);
void IIC_NAck(void);
unsigned char IIC_WaitAck(void);
unsigned char IIC_CheckDevice(unsigned char deviceAddr_NWR);

#endif

#endif /* IIC_IIC_SOFTWARE_H_ */
