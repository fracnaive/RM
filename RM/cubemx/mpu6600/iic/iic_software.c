/*
 * iic_software.c
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#include "iic_software.h"

#if(MPU_USE_IIC)

static void IIC_Delay(uint32_t ntime);

#define IIC_SCL_PORT		GPIOF
#define IIC_SCL_PIN			GPIO_PIN_7
#define IIC_SDA_PORT		GPIOF
#define IIC_SDA_PIN			GPIO_PIN_9

#define IIC_SCL_H()			HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_L()			HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define IIC_SDA_H()			HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_L()			HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define IIC_SCL_R()			HAL_GPIO_ReadPin(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SDA_R()			HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN)

#define SPEED      IIC_SPEED


void IIC_GPIOInit(void){
	GPIO_InitTypeDef GPIO_InitStruct;
		if(__HAL_RCC_GPIOF_IS_CLK_DISABLED()) __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin = IIC_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}

//400kbit/s,100kbit/s
void IIC_Delay(uint32_t ntime){
	for(uint16_t i = 0; i < ntime; i++){
		for(uint8_t j = 0; j < 100; j++){
			__NOP();__NOP();__NOP();__NOP();
		}
	}
}
bool IIC_Start(void){
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay(SPEED);
	if(!IIC_SDA_R()) return false;
	IIC_SDA_L();
	IIC_Delay(SPEED/2);
	if(IIC_SDA_R()) return false;
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	return true;
}
void IIC_Stop(void){
	IIC_SDA_L();
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	IIC_SCL_H();
	IIC_Delay(SPEED*2);
	IIC_SDA_H();
	IIC_Delay(SPEED/2);
}
void IIC_SendByte(unsigned char chr){
	unsigned char i, _chr=chr;
	for(i=0;i<8;i++){
		if(_chr&0x80) IIC_SDA_H();
		else IIC_SDA_L();
		_chr<<=1;
		IIC_SCL_L();
		IIC_Delay(SPEED/2);
		IIC_SCL_H();
		IIC_Delay(SPEED*2);
		IIC_SCL_L();
		IIC_Delay(SPEED/2);
	}
}
unsigned char IIC_ReadByte(void){
	unsigned char data=0x00, i;
	for(i=0;i<8;i++){
		data<<=1;
		IIC_SCL_L();
		IIC_Delay(SPEED/2);
		IIC_SCL_H();
		IIC_Delay(SPEED);
		if(IIC_SDA_R())
			data++;
		IIC_Delay(SPEED);
		IIC_SCL_L();
		IIC_Delay(SPEED/2);
	}
	return data;
}
void IIC_Ack(void){
	IIC_SCL_L();
	IIC_SDA_L();
	IIC_Delay(SPEED/2);
	IIC_SCL_H();
	IIC_Delay(SPEED*2);
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	IIC_SDA_H();
}
void IIC_NAck(void){
	IIC_SCL_L();
	IIC_SDA_H();
	IIC_Delay(SPEED/2);
	IIC_SCL_H();
	IIC_Delay(SPEED*2);
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	IIC_SDA_H();
}
unsigned char IIC_WaitAck(void){
	unsigned char ack;
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	IIC_SCL_H();
	IIC_Delay(SPEED);
	ack=IIC_SDA_R();
	IIC_Delay(SPEED);
	IIC_SCL_L();
	IIC_Delay(SPEED/2);
	return ack;
}
unsigned char IIC_CheckDevice(unsigned char deviceAddr_NWR){
	unsigned char ack = 1, i=10;
//loop: if(IIC_SDA_R()&&IIC_SCL_R()){
//		IIC_Start();
//		IIC_SendByte((deviceAddr_NWR)&0xFE);
//		ack=IIC_WaitAck();
//		IIC_Stop();
//		return ack;
//	}else{
//		if(i--){
//			IIC_SDA_H();
//			IIC_SCL_H();
//			IIC_Delay(SPEED*2);
//			goto loop;
//		}else return 255;
//	}

	do{
		 if(IIC_SDA_R()&&IIC_SCL_R()){
				IIC_Start();
				IIC_SendByte((deviceAddr_NWR)&0xFE);
				ack=IIC_WaitAck();
				IIC_Stop();
//				return ack;
				break;
			}else{
				if(i--){
					IIC_SDA_H();
					IIC_SCL_H();
					IIC_Delay(SPEED*2);
				}else return 255;
			}
	}while(i);
	if(i == 0) return 255;
	else return ack;
}
/*********************************************************************************************/
#endif
