#include "chassis.h"
#include "pid.h"  //PID函数 pid结构体
#include "can.h"  //motor_info[i].rotor_speed
#include "usart.h" //PID参数usart_pid声明
#include "RC_handle.h" //遥控器数据声明
uint16_t _speed=0;
uint16_t gambal_yaw=0;
uint16_t gambal_pitch=0;
/*说明：USART1 三 个通道数据给chassis作为目标值，
*    遥控器通道   chassis变量    |   底盘电机     映射关系                   CAN_Tx/               CAN_Rx/
*
*     ch1          vertical    |   MOTOR_ID2      -vertical-rotate+horizontal    bit：2 3
*     ch2          horizontal      MOTOR_ID1      -vertical-rotate-horizontal   bit：0 1
*     ch3          rotate          MOTOR_ID3      vertical-rotate+horizontal    bit：4 5
*                                  MOTOR_ID4      vertical-rotate-horizontal  bit：6 7
*范围																																				0x200,+-16384/+-20A		
*/

//extern pid_struct_t motor_pid[7];
//extern pid_struct_t usart_pid[7]; //蓝牙接收数据进行pid参数更新

void test_horizontal(void)
{
	traget_speed[1]=RC_Info.ch2 ;   //电机2
	traget_speed[0]=-RC_Info.ch2 ;   //电机1
	traget_speed[2]=RC_Info.ch2 ;  //电机3
	traget_speed[3]=-RC_Info.ch2 ;  //电机4
	//traget_speed[x] = RC_Info.chx / 660 * MAX_CHASSIS_VX_SPEED;
}
void test_vertical(void)
{
	traget_speed[1]=-RC_Info.ch1 ;   //电机2
	traget_speed[0]=-RC_Info.ch1 ;   //电机1
	traget_speed[2]=RC_Info.ch1 ;  //电机3
	traget_speed[3]=RC_Info.ch1 ;  //电机4
}
void test_rotate(void)
{
	traget_speed[1]=-RC_Info.ch3 ;   //电机2
	traget_speed[0]=-RC_Info.ch3 ;   //电机1
	traget_speed[2]=-RC_Info.ch3 ;  //电机3
	traget_speed[3]=-RC_Info.ch3 ;  //电机4
}
void test_chassis_speed(void)
{
	traget_speed[1]=RC_Info.ch2 - RC_Info.ch1 - RC_Info.ch3;   //电机2
	traget_speed[0]=-RC_Info.ch2 - RC_Info.ch1 - RC_Info.ch3;   //电机1
	traget_speed[2]=RC_Info.ch2 + RC_Info.ch1 - RC_Info.ch3;  //电机3
	traget_speed[3]=-RC_Info.ch2 + RC_Info.ch1 - RC_Info.ch3 ;  //电机4

}	


void chassis_pid_init(void )
{ 
	for(int j=0;j<4;j++)
		pid_init(&motor_pid[j], 8.0f, 0.0f, 2.0f, 30000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
	
}
void chassis_pid_update(void )
{ 
	for(int i=0;i<4;i++)
	 {
		 if(usart_pid[i].update_flag !=0)
		 {//usart_pid数据写入motor_pid
		 pid_param_update(&motor_pid[i], usart_pid[i].kp , usart_pid[i].ki , usart_pid[i].kd , usart_pid[i].i_max , usart_pid[i].out_max ); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
	   usart_pid[i].update_flag =0;//清除标志位
		 }
	 }
}
void chassis_traget_speed()
{
	traget_speed[0]=RC_Info.ch1;
  traget_speed[1]=RC_Info.ch2;
	traget_speed[2]=RC_Info.ch3;
	traget_speed[3]=RC_Info.ch4;
}

void chassis_execute(void )
{  
	
	motor_info[0].set_voltage = pid_calc(&motor_pid[0], traget_speed[0]  , motor_info[0].rotor_speed);
	motor_info[1].set_voltage = pid_calc(&motor_pid[1], traget_speed[1]  , motor_info[1].rotor_speed);
	motor_info[2].set_voltage = pid_calc(&motor_pid[2], traget_speed[2]  , motor_info[2].rotor_speed);
	motor_info[3].set_voltage = pid_calc(&motor_pid[3], traget_speed[3]  , motor_info[3].rotor_speed);
	
	set_motor_voltage(1, motor_info[0].set_voltage , 
											 motor_info[1].set_voltage , 
											 motor_info[2].set_voltage ,
                       motor_info[3].set_voltage );
	
//	set_motor_voltage(0, RC_Info.wheel  , 
//											 gambal_pitch, 
//											 -_speed ,
//                       0 );
	
}
