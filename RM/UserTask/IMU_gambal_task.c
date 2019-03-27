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
		mpuGetData(&mpu_date);//数据采集
		mpuMahonyQuaternionUpdateWithFilter(&mpu_date); //四元数
		imu_attitude_update(&imu); //解算
	gambal_flag++;
		osDelay(5);
		
	}
}

