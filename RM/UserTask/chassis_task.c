#include "cmsis_os.h"
#include "chassis_task.h"
#include "chassis.h"
#include "pid.h"

uint32_t task_flag=0;

//osThreadId chassis_task_t;

void chassis_task(void const *argument)
{
	for(int i=0;i<4;i++)
   	pid_init(&motor_pid[i], 10, 0, 0, 30000, 30000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 30000
	
	while(1)
	{
		task_flag++;
		chassis_pid_update();
		//chassis_traget_speed();
		//test_horizontal(); 
		test_chassis_speed();
		chassis_execute();
		osDelay(10);
	}

}

/*说明：USART1 三 个通道数据给chassis作为目标值，
*    遥控器通道   chassis变量    |   底盘电机     映射关系                   CAN_Tx/               CAN_Rx/
*
*     ch1          vertical    |   MOTOR_ID2      vertical+rotate+horizontal    bit：2 3
*     ch2          horizontal      MOTOR_ID1      -vertical+rotate+horizontal   bit：0 1
*     ch3          rotate          MOTOR_ID3      vertical+rotate-horizontal    bit：4 5
*                                  MOTOR_ID4      -vert+ical+rotate-horizontal  bit：6 7
*范围																																				0x200,+-16384/+-20A		
*/
