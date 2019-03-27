#include "angle_gambal_task.h"
#include "stdint.h"
#include "can.h"
#include "pid.h"
#include "cmsis_os.h"
#include "RC_handle.h"

#define  YAW_Clockwise              0x0D1F
#define  YAW_Middle                 0x1569
#define  YAW_Counterclockwise				0x1D78

#define  PITCH_up              0x1BBB
#define  PITCH_down				 		 0x1D64
#define  PITCH_Middle          0x1C77

uint16_t Yaw_angle =0;
uint16_t Pitch_angle=0;
uint16_t Yaw_speed =0;

uint32_t gambal_flagzz=0;
void ANGLE_gambal_task(void const *argument)
{
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	yaw_motor_pid_init();
	pitch_motor_pid_init();
	
	while(1)
	{   
		  gambal_pid_update();
		  yaw_gambal_target();
		  pitch_gambal_target();
		  Yaw_angle_compare();
		  Pitch_angle_compare();		
		  yaw_angle_OUT_PID();
		  pitch_angle_OUT_PID();	
		  gambal_execute();
		gambal_flagzz++;
		osDelayUntil(&xLastWakeTime, 5);//周期5ms
		//osDelay(10);
	}
}


void gambal_pid_update(void )
{ 
	
		 if(usart_pid[5].update_flag !=0)
		 {//usart_pid数据写入motor_pid
		 pid_param_update(&motor_pid[5], usart_pid[5].kp , usart_pid[5].ki , usart_pid[5].kd , usart_pid[5].i_max , usart_pid[5].out_max ); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
	   usart_pid[5].update_flag =0;//清除标志位
		 }
	 
}

void Yaw_angle_compare(void )
{   
	  uint16_t angle=0;
	  angle= motor_info[4].rotor_angle ;
	  if(angle<YAW_Clockwise|angle>YAW_Counterclockwise)  //限制位置
			Yaw_angle=500;
		else 
		  Yaw_angle = ((float)(angle-YAW_Clockwise)/(YAW_Counterclockwise-YAW_Clockwise))*1000;
		
		
	  //Yaw_speed= motor_info[4].rotor_speed ;
	  
}

void Pitch_angle_compare(void )
{   
	  uint16_t angle=0;
	  angle= motor_info[5].rotor_angle ;
	  if(angle<PITCH_up|angle>PITCH_down)  //限制位置
			Pitch_angle=500;
		else 
		  Pitch_angle = ((float)(angle-PITCH_up)/(PITCH_down-PITCH_up))*1000;
		
		
	  //Yaw_speed= motor_info[4].rotor_speed ;
	  
}

void yaw_motor_pid_init(void)
{
	
	pid_init(&motor_pid[4], 10.0f, 0.0f, 2.0f, 30000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
	//pid_init(&motor_pid[5], 8.0f, 0.0f, 0.0f, 30000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
}

void pitch_motor_pid_init(void) //使用同样参数
{
	pid_init(&motor_pid[5], 15.0f, 0.2f, 0.0f, 3000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
}

uint16_t yaw_angle_target=0;
void yaw_gambal_target(void )
{ 
	if(RC_Info.wheel ==0 )
    yaw_angle_target=500;    
	else if(RC_Info.wheel >0)
		yaw_angle_target=(((float)RC_Info.wheel/660)*500)+500;
	else if(RC_Info.wheel <0)
		yaw_angle_target=(((float)RC_Info.wheel/660)*500)+500;
		
	
	
	//angle_target=
}
uint16_t pitch_angle_target=0;
void pitch_gambal_target(void )
{
	if(RC_Info.ch4 ==0 )
		pitch_angle_target=500;
	else if(RC_Info.ch4 >0)
		pitch_angle_target=(((float)RC_Info.ch4/660)*500)+500;
	else if(RC_Info.ch4 <0)
		pitch_angle_target=(((float)RC_Info.ch4/660)*500)+500;

}
int16_t yaw_angle_out=0;
//目标值为middle,实际值为Yaw_angle
void yaw_angle_OUT_PID(void)
{
		yaw_angle_out = pid_calc(&motor_pid[4],  yaw_angle_target , Yaw_angle );
	  
}

int16_t pitch_angle_out=0;
void pitch_angle_OUT_PID(void)
{
	pitch_angle_out = pid_calc(&motor_pid[5],  pitch_angle_target , Pitch_angle );
}


int16_t speed_in=0;
void speed_IN_PID(void)
{
	 
   speed_in = pid_calc(&motor_pid[4],  500 , Yaw_angle );

}

void gambal_execute(void)
{
	set_motor_voltage(0, yaw_angle_out , 
											 pitch_angle_out , 
											 -380,
                       0 );
	
}













