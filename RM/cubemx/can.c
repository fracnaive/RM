
#include "can.h"


moto_info_t motor_info[7];

//#include "stm32f4xx_hal.h"
//extern CAN_TxHeaderTypeDef Tx_header;
//extern CAN_RxHeaderTypeDef Rx_header;
extern CAN_HandleTypeDef hcan1;
CAN_MsgTypeDef   CAN_ReceiveMsg[4];
CAN_MsgTypeDef   CAN_TransmitMsg;
uint32_t flagCAN=0;
uint32_t flagMail0=0;
uint32_t flagMail1=0;
uint32_t flagMail2=0;
uint8_t re_date[8]={0};
uint8_t errorflag=0;
uint32_t Rxcan;
uint8_t id=0;
void drv_can_init()
{
	CAN_FilterTypeDef can_filter_st;//筛选器结构体变量
	
	can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;  //32位ID
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;  //32位MASK
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;//过滤器0
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;///过滤器0关联至FIFO0
	
	 HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//挂起中断允许
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING);
 // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
 // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_PASSIVE);
 // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);
	
}


/***************接收回调函数*******************/
/**
  * @brief  RX_handle 将接收的数据进行转换
  * @param  CAN_Message_ID _ID
  * @param  uint8_t aData[]
  * @retval None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef	HAL_RetVal;
	CAN_RxHeaderTypeDef header;
  uint8_t rx_data[8];
	

	if (hcan == &hcan1)
  {
		HAL_RetVal=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
	
    if ( HAL_OK==HAL_RetVal)
		{			
	  uint8_t index = header.StdId-0x201;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
    
			Rxcan++;	 
		}
  }
	 
}

/********************发送函数********************/
/**
* @brief  CANx_SendNormalData 发送数据
  * @param  CAN_Message_ID _ID
  * @param  uint8_t aData[]
  * @retval None
  */
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1ff):(0x200);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

void CANx_SendNormalData(CAN_HandleTypeDef* hcan,CAN_Message_ID ID,uint8_t *pData)
{	
	//HAL_StatusTypeDef	HAL_RetVal;  
	CAN_TxHeaderTypeDef header;
	/*		
	//HAL_Delay(1);   //延时	
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan))
		{
			header.StdId=ID;
			header.IDE = CAN_ID_STD;
			header.RTR = CAN_RTR_DATA;
			header.DLC=8;
			//header.TransmitGlobalTime = DISABLE;
		
			HAL_CAN_AddTxMessage(hcan,&header,pData,(uint32_t*)CAN_TX_MAILBOX0);
		}
		*/
	 header.DLC=8;
   header.StdId=0x1ff;
   header.IDE=CAN_ID_STD;
   header.RTR=CAN_RTR_DATA;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0);
	
	 if(HAL_CAN_AddTxMessage(&hcan1,&header,pData,(uint32_t *)CAN_TX_MAILBOX1)!=HAL_OK)
		 Error_Handler();
}

/*******************发送邮箱回调*******************/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{ 
	
	if (hcan == &hcan1)
  {
	//	can_tx_mailbox_complete_hanle(&hcan1);
		flagMail0++;
  }
 
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
  {
		//can_tx_mailbox_complete_hanle(&hcan1);
  }
  flagMail1++;


}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
  {
		//can_tx_mailbox_complete_hanle(&hcan1);
  }
  flagMail2++;
}
