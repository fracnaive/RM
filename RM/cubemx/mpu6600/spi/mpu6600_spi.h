/*
 * mpu6600_spi.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef MPU6600_SPI_H_
#define MPU6600_SPI_H_

#include "../mpu6600_includes.h"

#if(MPU_USE_SPI)


#define HIGH			GPIO_PIN_SET
#define LOW				GPIO_PIN_RESET

#define SPI_Selete(s)					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, (s))

void spi5Init(void);

void spiWriteByte(uint8_t data);
void spiWrite(uint8_t* buf, uint16_t Size);

uint8_t spiReadByte(void);
void spiRead(uint8_t* buf, uint16_t Size);

void spiRW(uint8_t* txBuf, uint8_t* rxBuf, uint16_t Size);

#endif
#endif /* MPU6600_SPI_H_ */
