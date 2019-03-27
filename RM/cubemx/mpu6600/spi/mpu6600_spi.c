/*
 * mpu6600_spi.c
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#include "mpu6600_spi.h"

#if(MPU_USE_SPI)

SPI_HandleTypeDef hspi5;

static void spiMspInit(SPI_HandleTypeDef* spiHandle);

void spiMspInit(SPI_HandleTypeDef* spiHandle){
  GPIO_InitTypeDef GPIO_InitStruct;

  if(__HAL_RCC_GPIOF_IS_CLK_DISABLED()) __HAL_RCC_GPIOF_CLK_ENABLE();

  if(spiHandle->Instance==SPI5){
  /* USER CODE BEGIN SPI5_MspInit 0 */

  /* USER CODE END SPI5_MspInit 0 */
    /* SPI5 clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();

    /**SPI5 GPIO Configuration
    PF7     ------> SPI5_SCK
    PF6     ------> SPI5_NSS
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI5_MspInit 1 */

  /* USER CODE END SPI5_MspInit 1 */
  }
}

void spi5Init(void){
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;

  spiMspInit(&hspi5);

  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
  }
}

void spiWriteByte(uint8_t data){
	HAL_StatusTypeDef s = HAL_ERROR;
	uint8_t n = 0;
	do{
		s = HAL_SPI_Transmit(&hspi5, &data, 1, 1000);
		if((++n) > 5) break;
	}while(s != HAL_OK);
}

void spiWrite(uint8_t* buf, uint16_t Size){
	HAL_StatusTypeDef s = HAL_ERROR;
	uint8_t n = 0;
	do{
		s = HAL_SPI_Transmit(&hspi5, buf, Size, 1000);
		if((++n) > 5) break;
	}while(s != HAL_OK);
}

uint8_t spiReadByte(void){
	HAL_StatusTypeDef s = HAL_ERROR;
	uint8_t n = 0, tmp = 0;
	do{
		s = HAL_SPI_Receive(&hspi5, &tmp, 1, 1000);
		if((++n) > 5) break;
	}while(s != HAL_OK);
	return tmp;
}

void spiRead(uint8_t* buf, uint16_t Size){
	HAL_StatusTypeDef s = HAL_ERROR;
	uint8_t n = 0;
	do{
		s = HAL_SPI_Receive(&hspi5, buf, Size, 1000);
		if((++n) > 5) break;
	}while(s != HAL_OK);
}

void spiRW(uint8_t* txBuf, uint8_t* rxBuf, uint16_t Size){
	HAL_StatusTypeDef s = HAL_ERROR;
	uint8_t n = 0;
	do{
		s = HAL_SPI_TransmitReceive(&hspi5, txBuf, rxBuf, Size, 1000);
		if((++n) > 5) break;
	}while(s != HAL_OK);
}

#endif


