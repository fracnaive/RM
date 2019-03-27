/*
 * mpu6600.c
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#include "mpu6600.h"
#include "stdlib.h"

#define MPU_HSPI   hspi5
#define MPU_DELAY		HAL_Delay

extern SPI_HandleTypeDef hspi5;


uint8_t tx, rx;
uint8_t txBuf[14];
uint8_t mpuBuf[14];

mpuData mData;
mpuCalibrate mc = {1, 0, 0};

uint8_t mpuInit(void){

	spi5Init();

	GPIO_InitTypeDef mpuGPIO;
  /*Configure GPIO pin : PB8 */
  mpuGPIO.Pin = GPIO_PIN_8;
  mpuGPIO.Mode = GPIO_MODE_IT_RISING;
  mpuGPIO.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &mpuGPIO);

  mpuWriteReg(MPU6500_PWR_MGMT_1, 0x80);
  MPU_DELAY(100);
  // Reset gyro/accel/temp digital signal path
  mpuWriteReg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  MPU_DELAY(100);

  if (MPU6500_ID != mpuReadReg(MPU6500_WHO_AM_I))
    return 1;
  //0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
  uint8_t MPU6500_Init_Data[7][2] = {
      {MPU6500_PWR_MGMT_1, 0x03},     // Auto selects Clock Source
      {MPU6500_PWR_MGMT_2, 0x00},     // all enable
      {MPU6500_CONFIG, 0x02},         // gyro bandwidth 0x00:250Hz 0x04:20Hz
      {MPU6500_GYRO_CONFIG, 0x18},    // gyro range 0x10:+-1000dps 0x18:+-2000dps
      {MPU6500_ACCEL_CONFIG, 0x10},   // acc range 0x10:+-8G
      {MPU6500_ACCEL_CONFIG_2, 0x00}, // acc bandwidth 0x00:250Hz 0x04:20Hz
      {MPU6500_USER_CTRL, 0x20},      // Enable the I2C Master I/F module
                                      // pins ES_DA and ES_SCL are isolated from
                                      // pins SDA/SDI and SCL/SCLK.
  };

  for (int i = 0; i < 7; i++)
  {
  	mpuWriteReg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    MPU_DELAY(1);
  }

  istInit();

  if (mc.gyroFlag == 1)
  {
    mpuGetGyroOffset();
  }

  if (mc.accFlag == 1)
  {
    mpuGetAccOffset();
  }

  if (mc.magFlag == 1)
  {
    istGetMagOffset();
  }

  return 0;
}
uint8_t mpuReadReg(uint8_t reg){
	MPU_Selete(LOW);
	tx = reg|0x80;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	MPU_Selete(HIGH);
	return rx;
}
void mpuReadRegs(uint8_t reg, uint8_t* buf, uint16_t Size){
	MPU_Selete(LOW);
	tx = reg|0x80;
	txBuf[0] = tx;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	HAL_SPI_TransmitReceive(&MPU_HSPI, txBuf, buf, Size, 55);
	MPU_Selete(HIGH);
}
void mpuWriteReg(uint8_t reg, uint8_t data){
	MPU_Selete(LOW);
	tx = reg|0x7F;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	tx = data;
	HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
	MPU_Selete(HIGH);
}
void mpuGetAccOffset(void){
	 int i;
	  for (i = 0; i < 300; i++)
	  {
	    mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpuBuf, 14);

	    mData.ax_offset += mpuBuf[0] << 8 | mpuBuf[1];
	    mData.ay_offset += mpuBuf[2] << 8 | mpuBuf[3];
	    mData.az_offset += mpuBuf[4] << 8 | (mpuBuf[5] - 4096);

	    MPU_DELAY(2);
	  }

	  mData.ax_offset = mData.ax_offset / 300;
	  mData.ay_offset = mData.ay_offset / 300;
	  mData.az_offset = mData.az_offset / 300;

	  mc.accFlag = 0;
}
void mpuGetGyroOffset(void){
	 int i;
	  for (i = 0; i < 300; i++)
	  {
	    mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpuBuf, 14);

	    mData.gx_offset += mpuBuf[8] << 8 | mpuBuf[9];
	    mData.gy_offset += mpuBuf[10] << 8 | mpuBuf[11];
	    mData.gz_offset += mpuBuf[12] << 8 | mpuBuf[13];

	    MPU_DELAY(2);
	  }

	  mData.gx_offset = mData.gx_offset / 300;
	  mData.gy_offset = mData.gy_offset / 300;
	  mData.gz_offset = mData.gz_offset / 300;
	  mc.gyroFlag = 0;
}

void mpuGetData(ahrsSensor* sensor){
	 mpuReadRegs(MPU6500_ACCEL_XOUT_H, mpuBuf, 14);

	  mData.ax = (mpuBuf[0] << 8 | mpuBuf[1]) - mData.ax_offset;
	  mData.ay = (mpuBuf[2] << 8 | mpuBuf[3]) - mData.ay_offset;
	  mData.az = (mpuBuf[4] << 8 | mpuBuf[5]) - mData.az_offset;
	  mData.temp = mpuBuf[6] << 8 | mpuBuf[7];

	  mData.gx = ((mpuBuf[8] << 8 | mpuBuf[9]) - mData.gx_offset);
	  mData.gy = ((mpuBuf[10] << 8 | mpuBuf[11]) - mData.gy_offset);
	  mData.gz = ((mpuBuf[12] << 8 | mpuBuf[13]) - mData.gz_offset);

	  istGetData((uint8_t *)&mData.mx);

	  sensor->ax = mData.ax / (4096.0f / 9.80665f); //8g -> m/s^2
	  sensor->ay = mData.ay / (4096.0f / 9.80665f); //8g -> m/s^2
	  sensor->az = mData.az / (4096.0f / 9.80665f); //8g -> m/s^2

	  sensor->gx = mData.gx / 16.384f / 57.3f; //2000dps -> rad/s
	  sensor->gy = mData.gy / 16.384f / 57.3f; //2000dps -> rad/s
	  sensor->gz = mData.gz / 16.384f / 57.3f; //2000dps -> rad/s

	  sensor->mx = (mData.my - mData.my_offset);
	  sensor->my = -(mData.mx - mData.mx_offset);
	  sensor->mz = -(mData.mz - mData.mz_offset);

}

void mpuGetTempareture(float* temp){
	*temp = 21 + mData.temp / 333.87f;
}


void mpuSetIICSalverAutoRaed(uint8_t deviceAddr, uint8_t baseReg, uint8_t dataSize){
	 //configure the device address of the IST8310
	  //use slave1,auto transmit single measure mode.
	mpuWriteReg(MPU6500_I2C_SLV1_ADDR, deviceAddr);
	  MPU_DELAY(2);
	  mpuWriteReg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
	  MPU_DELAY(2);
	  mpuWriteReg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
	  MPU_DELAY(2);

	  //use slave0,auto read data
	  mpuWriteReg(MPU6500_I2C_SLV0_ADDR, 0x80 | deviceAddr);
	  MPU_DELAY(2);
	  mpuWriteReg(MPU6500_I2C_SLV0_REG, baseReg);
	  MPU_DELAY(2);

	  //every eight mpu6500 internal samples one i2c master read
	  mpuWriteReg(MPU6500_I2C_SLV4_CTRL, 0x03);
	  MPU_DELAY(2);
	  //enable slave 0 and 1 access delay
	  mpuWriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
	  MPU_DELAY(2);
	  //enable slave 1 auto transmit
	  mpuWriteReg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	  MPU_DELAY(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
	  //enable slave 0 with data_num bytes reading
	  mpuWriteReg(MPU6500_I2C_SLV0_CTRL, 0x80 | dataSize);
	  MPU_DELAY(2);
}


uint8_t istInit(void){
	GPIO_InitTypeDef mpuGPIO;
	mpuGPIO.Pin = GPIO_PIN_3;
	mpuGPIO.Mode = GPIO_MODE_IT_RISING;
	mpuGPIO.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &mpuGPIO);

  /*Configure GPIO pin : PE2 */
  mpuGPIO.Pin = GPIO_PIN_2;
  mpuGPIO.Mode = GPIO_MODE_OUTPUT_PP;
  mpuGPIO.Pull = GPIO_PULLDOWN;
  mpuGPIO.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &mpuGPIO);

  //使能IST8310
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, HIGH);

  //Enable I2C master mode, Reset I2C Slave module
   mpuWriteReg(MPU6500_USER_CTRL, 0x30);
   MPU_DELAY(10);
   //I2C master clock 400kHz
   mpuWriteReg(MPU6500_I2C_MST_CTRL, 0x0d);
   MPU_DELAY(10);

   //turn on slave 1 for ist write and slave 4 for ist read
   mpuWriteReg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //write ist
   MPU_DELAY(10);
   mpuWriteReg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //read ist
   MPU_DELAY(10);

   //reset ist8310
   istWriteRegByMPU(IST8310_R_CONFB, 0x01);
   MPU_DELAY(10);

   if (IST8310_DEVICE_ID_A != istReadRegByMPU(IST8310_WHO_AM_I))
     return 1;

   istWriteRegByMPU(IST8310_R_CONFB, 0x01);
   MPU_DELAY(10);

   //config as ready mode to access reg
   istWriteRegByMPU(IST8310_R_CONFA, 0x00);
   if (istReadRegByMPU(IST8310_R_CONFA) != 0x00)
     return 2;
   MPU_DELAY(10);

   //normal state, no int
   istWriteRegByMPU(IST8310_R_CONFB, 0x00);
   if (istReadRegByMPU(IST8310_R_CONFB) != 0x00)
     return 3;
   MPU_DELAY(10);

   //config  low noise mode, x,y,z axis 16 time 1 avg,
   istWriteRegByMPU(IST8310_AVGCNTL, 0x24); //100100
   if (istReadRegByMPU(IST8310_AVGCNTL) != 0x24)
     return 4;
   MPU_DELAY(10);

   //Set/Reset pulse duration setup, normal mode
   istWriteRegByMPU(IST8310_PDCNTL, 0xc0);
   if (istReadRegByMPU(IST8310_PDCNTL) != 0xc0)
     return 5;
   MPU_DELAY(10);

   //turn off slave1 & slave 4
   mpuWriteReg(MPU6500_I2C_SLV1_CTRL, 0x00);
   MPU_DELAY(10);
   mpuWriteReg(MPU6500_I2C_SLV4_CTRL, 0x00);
   MPU_DELAY(10);

   //configure and turn on slave 0
   mpuSetIICSalverAutoRaed(IST8310_ADDRESS, IST8310_R_XL, 0x06);
   MPU_DELAY(100);
   return 0;

}
uint8_t istReadRegByMPU(uint8_t reg){
	uint8_t retval;
	mpuWriteReg(MPU6500_I2C_SLV4_REG, reg);
	  MPU_DELAY(10);
	  mpuWriteReg(MPU6500_I2C_SLV4_CTRL, 0x80);
	  MPU_DELAY(10);
	  retval = mpuReadReg(MPU6500_I2C_SLV4_DI);
	  //turn off slave4 after read
	  mpuWriteReg(MPU6500_I2C_SLV4_CTRL, 0x00);
	  MPU_DELAY(10);
	  return retval;
}
void istWriteRegByMPU(uint8_t reg, uint8_t data){
  //turn off slave 1 at first
	mpuWriteReg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPU_DELAY(2);
  mpuWriteReg(MPU6500_I2C_SLV1_REG, reg);
  MPU_DELAY(2);
  mpuWriteReg(MPU6500_I2C_SLV1_DO, data);
  MPU_DELAY(2);
  //turn on slave 1 with one byte transmitting
  mpuWriteReg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPU_DELAY(10);
}
void istGetMagOffset(void){
	 int16_t mag_max[3] = {0}, mag_min[3] = {0};
	  int i;
	  for (i = 0; i < 5000; i++)
	  {
	  	istGetData((uint8_t *)&mData.mx);
	    if ((abs(mData.mx) < 400) && (abs(mData.my) < 400) && (abs(mData.mz) < 400))
	    {
	      mag_max[0] = VAL_MAX(mag_max[0], mData.mx);
	      mag_min[0] = VAL_MIN(mag_min[0], mData.mx);

	      mag_max[1] = VAL_MAX(mag_max[1], mData.my);
	      mag_min[1] = VAL_MIN(mag_min[1], mData.my);

	      mag_max[2] = VAL_MAX(mag_max[2], mData.mz);
	      mag_min[2] = VAL_MIN(mag_min[2], mData.mz);
	    }
	    MPU_DELAY(2);
	  }
	  mData.mx_offset = (int16_t)((mag_max[0] + mag_min[0]) * 0.5f);
	  mData.my_offset = (int16_t)((mag_max[1] + mag_min[1]) * 0.5f);
	  mData.mz_offset = (int16_t)((mag_max[2] + mag_min[2]) * 0.5f);

	  mc.magFlag = 0;
}

void istGetData(uint8_t* buf){
	mpuReadRegs(MPU6500_EXT_SENS_DATA_00, buf, 6);
}
