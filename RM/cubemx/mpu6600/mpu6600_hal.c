/*
 * mpu6600_hal.c
 *
 *  Created on: 2019年3月18日
 *      Author: XIAOSENLUO
 */

#include "mpu6600_hal.h"
#include "arm_math.h"


#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f


float pitch, yaw, roll;
float deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method


void mpuMahonyQuaternionUpdateWithFilter(ahrsSensor* sensor){
	float ax = sensor->ax, ay = sensor->ay, az = sensor->az,
			gx = sensor->gx, gy = sensor->gy, gz = sensor->gz,
			mx = sensor->mx, my = sensor->my, mz = sensor->mz;
   float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
   float norm;
   float hx, hy, bx, bz;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
   float pa, pb, pc;

   // Auxiliary variables to avoid repeated arithmetic
   float q1q1 = q1 * q1;
   float q1q2 = q1 * q2;
   float q1q3 = q1 * q3;
   float q1q4 = q1 * q4;
   float q2q2 = q2 * q2;
   float q2q3 = q2 * q3;
   float q2q4 = q2 * q4;
   float q3q3 = q3 * q3;
   float q3q4 = q3 * q4;
   float q4q4 = q4 * q4;

   // Normalise accelerometer measurement
   norm = sqrt(ax * ax + ay * ay + az * az);
   if (norm == 0.0f) return; // handle NaN
   norm = 1.0f / norm;        // use reciprocal for division
   ax *= norm;
   ay *= norm;
   az *= norm;

   // Normalise magnetometer measurement
   norm = sqrt(mx * mx + my * my + mz * mz);
   if (norm == 0.0f) return; // handle NaN
   norm = 1.0f / norm;        // use reciprocal for division
   mx *= norm;
   my *= norm;
   mz *= norm;

   // Reference direction of Earth's magnetic field
   hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
   hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
   bx = sqrt((hx * hx) + (hy * hy));
   bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

   // Estimated direction of gravity and magnetic field
   vx = 2.0f * (q2q4 - q1q3);
   vy = 2.0f * (q1q2 + q3q4);
   vz = q1q1 - q2q2 - q3q3 + q4q4;
   wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
   wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
   wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

   // Error is cross product between estimated direction and measured direction of gravity
   ex = (ay * vz - az * vy) + (my * wz - mz * wy);
   ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
   ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
   if (Ki > 0.0f){
       eInt[0] += ex;      // accumulate integral error
       eInt[1] += ey;
       eInt[2] += ez;
   }
   else
   {
       eInt[0] = 0.0f;     // prevent integral wind up
       eInt[1] = 0.0f;
       eInt[2] = 0.0f;
   }

   // Apply feedback terms
   gx = gx + Kp * ex + Ki * eInt[0];
   gy = gy + Kp * ey + Ki * eInt[1];
   gz = gz + Kp * ez + Ki * eInt[2];

   // Integrate rate of change of quaternion
   pa = q2;
   pb = q3;
   pc = q4;
   q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
   q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
   q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
   q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

   // Normalise quaternion
   norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
   norm = 1.0f / norm;
   q[0] = q1 * norm;
   q[1] = q2 * norm;
   q[2] = q3 * norm;
   q[3] = q4 * norm;
}


/**
	* @brief  update imu attitude
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_attitude_update(imu_t *imu)
{
	/* yaw    -pi----pi */
	imu->yaw = -atan2(2*q[1]*q[2] + 2*q[0]*q[3], -2*q[2]*q[2] - 2*q[3]*q[3] + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu->pit = -asin(-2*q[1]*q[3] + 2*q[0]*q[2])* 57.3;   
	/* roll   -pi----pi  */	
	imu->rol =  atan2(2*q[2]*q[3] + 2*q[0]*q[1], -2*q[1]*q[1] - 2*q[2]*q[2] + 1)* 57.3;
}










