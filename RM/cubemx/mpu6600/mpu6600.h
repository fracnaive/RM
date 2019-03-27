/*
 * mpu6600.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef MPU6600_H_
#define MPU6600_H_

#include "mpu6600_includes.h"

#if(MPU_USE_SPI)
#include "spi/mpu6600_spi.h"
#endif

#if(MPU_USE_IIC)
#include "iic/mpu6600_iic.h"
#endif

#include "reg/ist8310_reg.h"
#include "reg/mpu6500_reg.h"

#include "ahrs/ahrs.h"

#define MPU_Selete						SPI_Selete

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))


typedef struct _mpuCalibrate{
	uint8_t gyroFlag;
	uint8_t accFlag;
	uint8_t magFlag;
}mpuCalibrate;

typedef struct _mpuData{
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;

  int16_t mx;
  int16_t my;
  int16_t mz;

  int16_t temp;

  int16_t ax_offset;
  int16_t ay_offset;
  int16_t az_offset;

  int16_t gx_offset;
  int16_t gy_offset;
  int16_t gz_offset;

  int16_t mx_offset;
  int16_t my_offset;
  int16_t mz_offset;
}mpuData;


uint8_t mpuInit(void);
uint8_t mpuReadReg(uint8_t reg);
void mpuReadRegs(uint8_t reg, uint8_t* buf, uint16_t Size);
void mpuWriteReg(uint8_t reg, uint8_t data);
void mpuGetAccOffset(void);
void mpuGetGyroOffset(void);

void mpuGetData(ahrsSensor* sensor);
void mpuGetTempareture(float* temp);

void mpuSetIICSalverAutoRaed(uint8_t deviceAddr, uint8_t baseReg, uint8_t dataSize);


uint8_t istInit(void);
uint8_t istReadRegByMPU(uint8_t reg);
void istWriteRegByMPU(uint8_t reg, uint8_t data);
void istGetMagOffset(void);
void istGetData(uint8_t* buf);


#endif /* MPU6600_H_ */
