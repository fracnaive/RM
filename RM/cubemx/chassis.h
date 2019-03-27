#ifndef _CHASSIS_H_
#define _CHASSIS_H_
#include "stdint.h"
extern uint16_t _speed;
extern uint16_t gambal_yaw;
extern uint16_t gambal_pitch;
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 3300 //8000rpm
#define MAX_CHASSIS_VY_SPEED 3300
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VW_SPEED 300 //5000rpm

void test_horizontal(void);
void test_vertical(void);
void test_rotate(void);
void test_chassis_speed(void);

void chassis_pid_init(void);
void chassis_pid_update(void);
void chassis_traget_speed(void);
void chassis_execute(void );



#endif
