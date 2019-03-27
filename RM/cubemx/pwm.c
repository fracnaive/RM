

#include "pwm.h"


/*
extern TIM_HandleTypeDef htim1;

void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
	
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
}


void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty){
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (20000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (20000*duty) - 1;break;
	}
	
}

void tim_start(void)
{
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

*/

shoot_t fic_shoot;

void fric_set_output(uint16_t  fric_spd1, uint16_t  fric_spd2)
{
  LEFT_FRICTION = fric_spd1;
  RIGHT_FRICTION = fric_spd2;
}

void fric_get_speed(uint16_t  *fric_spd1, uint16_t  *fric_spd2)
{
  *fric_spd1 = LEFT_FRICTION;
  *fric_spd2 = RIGHT_FRICTION;
}

void  shoot_fric_ctrl(shoot_t *shoot)
{
//  if (shoot == NULL)
//    return -RM_INVAL;
 // shoot->target_fric_spd [0]=1250;
	//shoot->target_fric_spd [1]=1250;
  //VAL_LIMIT(1250, FIRC_STOP_SPEED, FIRC_MAX_SPEED);
  //VAL_LIMIT(1250, FIRC_STOP_SPEED, FIRC_MAX_SPEED);

  //shoot_get_fric_speed(shoot, &(shoot->fric_spd[0]), &(shoot->fric_spd[1]));
   fric_get_speed(&shoot->fric_spd[0], &shoot->fric_spd [1]);
  if (shoot->target_fric_spd[0] != shoot->fric_spd[0])
  {
    if (shoot->target_fric_spd[0] < shoot->fric_spd[0])
    {
      shoot->fric_spd[0] -= 1;
    }
    else
    {
      shoot->fric_spd[0] += 1;
    }
  }
  else if (shoot->target_fric_spd[1] != shoot->fric_spd[1])
  {
    if (shoot->target_fric_spd[1] < shoot->fric_spd[1])
    {
      shoot->fric_spd[1] -= 1;
    }
    else
    {
      shoot->fric_spd[1] += 1;
    }
  }

  fric_set_output(shoot->fric_spd[0], shoot->fric_spd[1]);

  //return RM_OK;
}
