/*
 * mpu6600_iic.c
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#include "mpu6600_iic.h"

#if(MPU_USE_IIC)

void iicInit(void){
	GPIO_InitTypeDef iicGPIO;

	iicGPIO.Pin = GPIO_PIN_9 | GPIO_PIN_7;
	iicGPIO.Mode = GPIO_MODE_OUTPUT_OD;
	iicGPIO.Pull = GPIO_PULLUP;
	iicGPIO.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOF, &iicGPIO);
}

I2C_ResultType iicWriteReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dataBuf, uint16_t Size){
	IIC_Start();
	IIC_SendByte(deviceAddr&0xfe);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();
	for(uint8_t i = 0; i < Size; i++){
		IIC_SendByte(dataBuf[i]);
		IIC_WaitAck();
	}
	IIC_Stop();
	return I2C_RESULT_OK;
}

I2C_ResultType iicReadReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dataBuf, uint16_t Size){
	uint8_t i;
	IIC_Start();
	IIC_SendByte(deviceAddr&0xfe);
	IIC_WaitAck();
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(deviceAddr|0x01);
	IIC_WaitAck();
	for(i = 0; i < Size; i++){
		if(i != (Size - 1)){
			dataBuf[i] = IIC_ReadByte();
			IIC_Ack();
		}else{
			dataBuf[i] = IIC_ReadByte();
			IIC_NAck();
		}
	}
	IIC_Stop();
	return I2C_RESULT_OK;
}

I2C_ResultType iicWrite(uint8_t deviceAddr, uint8_t* dataBuf, uint16_t Size){
	IIC_Start();
	IIC_SendByte(deviceAddr&0xfe);
	IIC_WaitAck();
	for(uint8_t i = 0; i < Size; i++){
		IIC_SendByte(dataBuf[i]);
		IIC_WaitAck();
	}
	IIC_Stop();
	return I2C_RESULT_OK;
}

I2C_ResultType iicRead(uint8_t deviceAddr, uint8_t* dataBuf, uint16_t Size){
	IIC_Start();
	IIC_SendByte(deviceAddr&0xfe);
//	while(IIC_WaitAck());
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(deviceAddr|0x01);
//	while(IIC_WaitAck());
	IIC_WaitAck();
	for(uint8_t i = 0; i < Size; i++){
		if(i != (Size - 1)){
			dataBuf[i] = IIC_ReadByte();
			IIC_Ack();
		}else{
			dataBuf[i] = IIC_ReadByte();
			IIC_NAck();
		}
	}
	IIC_Stop();
	return I2C_RESULT_OK;
}

I2C_ResultType isDeviceConnected(uint8_t deviceAddr){
	if(IIC_CheckDevice(deviceAddr) == 1){
		return I2C_RESULT_ERROR;
	}else return I2C_RESULT_OK;
}

#endif

