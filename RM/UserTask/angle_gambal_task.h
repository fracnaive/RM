#ifndef _ANGLE_GAMBAL_H
#define _ANGLE_GAMBAL_H





void ANGLE_gambal_task(void const *argument);

void gambal_pid_update(void );
	
void Yaw_angle_compare(void );
void Pitch_angle_compare(void );

void yaw_motor_pid_init(void);
void pitch_motor_pid_init(void);

void yaw_gambal_target(void );
void pitch_gambal_target(void );

void yaw_angle_OUT_PID(void);
void pitch_angle_OUT_PID(void);

void speed_IN_PID(void);
void gambal_execute(void);

#endif

