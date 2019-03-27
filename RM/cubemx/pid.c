#include "pid.h"
#include "stdint.h"
#include "can.h"

pid_struct_t motor_pid[7];
pid_struct_t usart_pid[7]; //蓝牙接收数据进行pidcanshu
float traget_speed[7]={0}; //各电机目标速度

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}
	

void pid_param_update(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}
/**
  * @brief  pid calculation
  * @param  pid struct
    @param  reference value
    @param  feedback value
  * @retval calculation result
  */
float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * pid->err[0] - pid->err[1];
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
