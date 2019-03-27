/*
 * tim.c
 *
 *  Created on: 2019年3月24日
 *      Author: XIAOSENLUO
 */


#include "Tim.h"



#if 0
#define TIM_CLOCK_FRE						1000000

TIM_HandleTypeDef htim1;

uint16_t periol = 1;
float duty = 0.05;

static void TIM1_PWM_MspInit(TIM_HandleTypeDef* tim_baseHandle);
static void TIM1_init(void);

void pwmInit(uint32_t frequency, float _duty){
	periol = frequency;
	duty = _duty;
	TIM1_init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void TIM1_init(void){
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (SystemCoreClock) / (TIM_CLOCK_FRE) - 1;			//计数时钟 SystemCoreClock
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = (TIM_CLOCK_FRE) / periol - 1;										//PWM 频率  periol Hz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;

  TIM1_PWM_MspInit(&htim1);

//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)(((TIM_CLOCK_FRE) / periol - 1) * duty / 100);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
//    _Error_Handler(__FILE__, __LINE__);
  }

//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
////    _Error_Handler(__FILE__, __LINE__);
//  }
}



void TIM1_PWM_MspInit(TIM_HandleTypeDef* tim_baseHandle){
	GPIO_InitTypeDef pwmGPIO;
	if(tim_baseHandle->Instance == TIM1){
  /* USER CODE END TIM1_MspPostInit 0 */
    /**TIM1 GPIO Configuration
    PA11     ------> TIM1_CH4
    PA8     ------> TIM1_CH1
    */
		__HAL_RCC_TIM1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();

		pwmGPIO.Pin = GPIO_PIN_8;
		pwmGPIO.Mode = GPIO_MODE_AF_PP;
		pwmGPIO.Pull = GPIO_NOPULL;
		pwmGPIO.Speed = GPIO_SPEED_FREQ_LOW;
		pwmGPIO.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &pwmGPIO);
		pwmGPIO.Pin = GPIO_PIN_14;
		pwmGPIO.Mode = GPIO_MODE_AF_PP;
		pwmGPIO.Pull = GPIO_NOPULL;
		pwmGPIO.Speed = GPIO_SPEED_FREQ_LOW;
		pwmGPIO.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOE, &pwmGPIO);
	}
}


/*
 * @parameters: duty 小数点后一位
 */
void setPWMDuty(float _duty){
	float d;
	if((uint16_t)(_duty*100) > 10000) d = 100.0;
	else d = _duty;
//	TIM4->CCR4 = (uint32_t)(((SystemCoreClock) / periol - 1) * d / 100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(((TIM_CLOCK_FRE) / periol - 1) * d / 100));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(((TIM_CLOCK_FRE) / periol - 1) * d / 100));
}

void setPWMFrequency(uint32_t frequency){
	__HAL_TIM_SET_AUTORELOAD(&htim1, (TIM_CLOCK_FRE) / frequency - 1);
}

void pwmStop(void){
	__HAL_TIM_DISABLE(&htim1);
	__HAL_TIM_MOE_DISABLE(&htim1);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_DISABLE);
}


void pemRestart(void){
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_ENABLE);
	__HAL_TIM_MOE_ENABLE(&htim1);
	__HAL_TIM_ENABLE(&htim1);
}

#endif

