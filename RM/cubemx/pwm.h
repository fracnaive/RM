#ifndef _PWM_H_
#define _PWM_H_


#include "stm32f4xx_hal.h"

#define VAL_LIMIT(val, min, max) \
  do                             \
  {                              \
    if ((val) <= (min))          \
    {                            \
      (val) = (min);             \
    }                            \
    else if ((val) >= (max))     \
    {                            \
      (val) = (max);             \
    }                            \
  } while (0)
	
	
#define FIRC_STOP_SPEED 1000u
#define FIRC_MAX_SPEED 1400u
#define FRIC_MIN_SPEED 1180u
	
#define LEFT_FRICTION        TIM1->CCR1
#define RIGHT_FRICTION       TIM1->CCR4

	
typedef struct 
{
uint16_t fric_spd[2];
uint16_t target_fric_spd[2];

} shoot_t;
extern shoot_t fic_shoot;
	
/*
void user_pwm_setvalue(uint16_t value);

extern void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);

extern void tim_start(void);
*/

void fric_set_output(uint16_t  fric_spd1, uint16_t  fric_spd2);
void fric_get_speed(uint16_t  *fric_spd1, uint16_t  *fric_spd2);

void  shoot_fric_ctrl(shoot_t *shoot);


#endif
