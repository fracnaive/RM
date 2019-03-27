/**
  ******************************************************************************
  * @file    FreeCars_uart.h
  * @author  FreeCars�۸�
  * @version NULL
  * @date    2014/11/11
  * @brief 	 FreeCars_uart H file.(For XS128)
  *   site:   Http://FreeCars.taobao.com
  *   QQȺ��  384273254��149168724
  *   ��Ȩ��  �˴���ΪFreeCars��λ��Э����룬��������ʹ�ã�Ҳ����������ҵ��;�����뱣���˶����֣�
  *   tips:   ǿ�ҽ���С�����ʹ��FreeCars��Ʒ������ģ����װ��������λ�������￪ʼ��
  ******************************************************************************
  */
#ifndef __FREECARS_USART_H__
#define __FREECARS_USART_H__

#include "stdint.h"


//���²�Ҫ�޸�
#define UartRxBufferLen  100
#define UartRxDataLen    41           //FreeCars��λ�����͸�������MCU���գ���Ҫ��
#define UartRxCmdLen     7	      //FreeCars��λ�������������ݳ��ȣ���Ҫ��

#define UartDataNum      17           //FreeCars��λ������ͨ������������λ�����øı�!!!

#define UartCmdNum  SerialPortRx.Buffer[SerialPortRx.Stack-3]//�����
#define UartCmdData SerialPortRx.Buffer[SerialPortRx.Stack-2]//��������

typedef struct
{
  int32_t Stack;
  uint8_t Data;
  uint8_t PreData;
  uint8_t Buffer[UartRxBufferLen];
  uint8_t Enable;
  uint8_t Check;
}SerialPortType;

//Export Varibales
extern uint8_t uSendBuf[UartDataNum*2];
extern SerialPortType SerialPortRx;
extern double UartData[9];

//Export Functions
void FreeCarsTool_Init(void);
void sendDataToScope(void);
void push(uint8_t adr,uint16_t date);
extern void FreeCars_Send(void);
extern void FreeCars_OnInterrupt(void);

/***********************user***************************/
#define Uartserive   UART4


#endif
