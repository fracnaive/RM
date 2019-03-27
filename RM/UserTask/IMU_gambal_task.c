#include "cmsis_os.h"
#include "IMU_gambal_task.h"
#include "mpu6600_hal.h"

ahrsSensor mpu_date;
imu_t      imu;
uint32_t gambal_flag;
void IMU_gambal_task(void const *argument)
{
	
	//mpuInit();
	
	
	while(1)
	{
		mpuGetData(&mpu_date);//���ݲɼ�
		mpuMahonyQuaternionUpdateWithFilter(&mpu_date); //��Ԫ��
		imu_attitude_update(&imu); //����
	gambal_flag++;
		osDelay(5);
		
	}
}

