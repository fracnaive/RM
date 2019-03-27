#ifndef _PID_H_
#define _PID_H_

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
	
	
	int update_flag;  //参数是否更新flag

}pid_struct_t;

	
//typedef struct _angle_pid_struct_
//{
//	float angle_kp;
//  float angle_ki;
//  float angle_kd;
//  float angle_i_max;
//  float angle_out_max;
//  
//  float angle_ref;      // target value
//  float angle_fdb;      // feedback value
//  float angle_err[2];   // error and last error

//  float angle_p_out;
//  float angle_i_out;
//  float angle_d_out;
//  float angle_output;
//	
//}angle_pid_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);			
						
//变量声明
extern pid_struct_t motor_pid[7];
extern pid_struct_t usart_pid[7]; //蓝牙接收数据进行pid参数更新
extern float traget_speed[7]; //目标速度
							
//函数声明
void pid_init(pid_struct_t *pid,float kp,float ki,float kd,float i_max,float out_max);
void pid_param_update(pid_struct_t *pid,float kp,float ki,float kd,float i_max,float out_max);							
float pid_calc(pid_struct_t *pid, float ref, float fdb);
							
#endif
