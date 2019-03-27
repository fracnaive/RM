/*
 * ahrs.h
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#ifndef AHRS_AHRS_H_
#define AHRS_AHRS_H_

typedef struct _ahrsSensor
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float mx;
  float my;
  float mz;
}ahrsSensor;

typedef struct
{
	float rol;
	float pit;
	float yaw;
} imu_t;


#endif /* AHRS_AHRS_H_ */
