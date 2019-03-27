/*
 * mpu6600_hal.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef MPU6600_HAL_H_
#define MPU6600_HAL_H_

#include "mpu6600.h"


void mpuMahonyQuaternionUpdateWithFilter(ahrsSensor* sensor);

void imu_attitude_update(imu_t *imu);


#endif /* MPU6600_HAL_H_ */
