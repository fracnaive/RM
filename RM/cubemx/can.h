#ifndef _CAN_H_
#define _CAN_H_
#include "stdint.h"
#include "stm32f4xx_hal.h"


typedef struct can_std_msg
{
  uint32_t std_id;
  //uint8_t dlc;
  uint8_t data[8];
}CAN_MsgTypeDef;

/**
  * @brief  CAN发送或是接收的ID
  */
typedef enum
{
    MOTOR_3508 =0x200,
    MOTORA=0x201,   //ID=1
    MOTORB=0x202,
    MOTORC=0x203,
    MOTORD=0x204,
	
	  MOTOR_yaw=0x205,  //0x205=0x204+ID
	  MOTOR_pitch=0x206,
	  MOTOR_shoot=0x207,
    
} CAN_Message_ID;


typedef struct 
{		
	  uint16_t can_id;
		int16_t  set_voltage;
	  uint16_t rotor_angle; //0-8191对应0-360 ，
	  int16_t rotor_speed;  //转速
    int16_t torque_current;  //电流  -16384-16384对应+-20A
	  uint8_t temp;   //温度  bit6
}moto_info_t;


extern CAN_MsgTypeDef   CAN_ReceiveMsg[4];
extern CAN_MsgTypeDef   CAN_TransmitMsg;

//extern CAN_TxHeaderTypeDef Tx_header;
//extern CAN_RxHeaderTypeDef Rx_header;

extern void drv_can_init(void);
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern void CANx_SendNormalData(CAN_HandleTypeDef* hcan,CAN_Message_ID ID,uint8_t *pData);
 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);


//变量声明
extern moto_info_t motor_info[7];

#endif
