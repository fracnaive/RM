/**
******************************************************************************
* @file    FreeCars_uart.c
* @author  FreeCars�۸�
* @version NULL
* @date    2014/11/11
* @brief 	 FreeCars_uart C file.(For XS128)
*   site:   Http://FreeCars.taobao.com
*   QQȺ��  384273254��149168724
*   ��Ȩ��  �˴���ΪFreeCars��λ��Э����룬��������ʹ�ã�Ҳ����������ҵ��;�����뱣���˶����֣�
*   tips:   ǿ�ҽ���С�����ʹ��FreeCars��Ʒ������ģ����װ��������λ�������￪ʼ��
******************************************************************************
*/
#include "freecars_usart.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "can.h"
#include "chassis.h"

extern UART_HandleTypeDef huart2;

#define RXBUFFERSIZE   1 //����2�����С
extern uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer
//#include "usart.h"
/************�����Ǹ���FreeCars��λ��Э���޸ĵģ�����ı�**************/
uint8_t uSendBuf[UartDataNum*2]={0};
SerialPortType SerialPortRx;

double UartData[9] = {0};

uint8_t FreeCarsDataNum=UartDataNum*2;
/**************************/

void SCI_Put(uint8_t data)
{
	uint8_t Tx_date[1]={data};
	//Tx_date[0]=data;
 //  uart_putchar (Uartserive,(unsigned char)data );//���ݾ���оƬuart�ļ��޸�
	//HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Tx_date,sizeof(Tx_date));
	//HAL_UART_Transmit_IT(&huart2, (uint8_t *)Tx_date,sizeof(Tx_date));
	HAL_UART_Transmit(&huart2, (uint8_t *)Tx_date, sizeof(Tx_date), 10);
};

/*void FreeCarsTool_Init(void)
{
    uart_init (Uartserive, 115200);// ���ڳ�ʼ��(�����ⲿ��ʼ�������δ����)���ݾ���оƬ�޸�
}*/

/*
��ѯ������һ֡����
����ʱ�������ݳ����й�
�����Է����ж����������Ե���
*/
void sendDataToScope(void)
{
    uint16_t i,sum=0;

    SCI_Put(251);
    SCI_Put(109);
    SCI_Put(37);
    sum+=(251);      //ȫ�����ݼ���У��
    sum+=(109);
    sum+=(37);
    for(i=0;i<FreeCarsDataNum;i++)
    {
        SCI_Put(uSendBuf[i]);
        sum+=uSendBuf[i];         //ȫ�����ݼ���У��
    }
    SCI_Put(sum);
}

/*
��ĳ��ͨ���������
adr��ͨ��
date������-32768~32767
*/
void push(uint8_t chanel,uint16_t dat)
{
    uSendBuf[chanel*2]=dat/256;
    uSendBuf[chanel*2+1]=dat%256;
}

void FreeCars_Send()
{ 
#if 1
        push(0,motor_info[0].rotor_angle );
	      push(1,traget_speed[0]);
        push(2,motor_info[0].rotor_speed );
        push(3,motor_info[0].set_voltage );
        push(4,motor_info[0].torque_current );
#endif
	
        
#if 0
        push(0,);
        push(1,0);
           
#endif
   
    sendDataToScope();
}


//�������ݽ��д���
void usart2_get(int pid_ID)
{     
	    
	    usart_pid[pid_ID].kp = UartData[0];
		  usart_pid[pid_ID].ki = UartData[1];
		  usart_pid[pid_ID].kd = UartData[2];
		  usart_pid[pid_ID].i_max = UartData[3];
		  usart_pid[pid_ID].out_max = UartData[4];
	    _speed=UartData[5];
	    gambal_yaw=UartData[6];
	    //gambal_pitch=UartData[6];
	    
		  usart_pid[pid_ID].update_flag  = UartData[7];
	
}
void UartDebug(void)
{
  //push(3,(int16_t)UartData[0]);	//�����ݷ��ͻ�ȥ�۲�
  //push(4,(int16_t)UartData[3]);
	int16_t pid_ID=UartData[8];
  switch(pid_ID)
	{
		case 0:
			usart2_get(pid_ID);
			break;
		case 1:
			usart2_get(pid_ID);
			break;
		case 5:
			usart2_get(pid_ID);
		default :
			break;
		
	}
	
}
/*
������պ���
CmdNum��0~255������
DATA  ��0~255����������Ӧ��������
*/
void UartCmd(uint8_t CmdNum,uint8_t Data)
{
  //push(0,CmdNum);	//�����ݷ��ͻ�ȥ�۲�
  //push(1,Data);
  /*   switch(CmdNum)//cmd number
  {
case UART_IMAG_TX:
  {
  if(Data)
  UART_IMAGE_TRANSFER=true;
        else
  UART_IMAGE_TRANSFER=false;
  break;
	}
 }		 */
}
/**
  * @brief  �˺����ڴ��ڽ����ж��е���
  * @param  None
  * @retval None
  */
void FreeCars_OnInterrupt(void)
{
	int32_t i,b,d1;
	uint32_t d;
  
	SerialPortRx.Data = aRxBuffer[0];  //���ݽ���
	
	if( SerialPortRx.Stack < UartRxBufferLen )
	{
		SerialPortRx.Buffer[SerialPortRx.Stack++] = SerialPortRx.Data;

		if(   SerialPortRx.Stack >= UartRxDataLen  //UartRxDataLen����Ϊһ֡
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen]  ==0xff //У����ͷ
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+1]==0x55
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+2]==0xaa
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen+3]==0x10 )
		
		{
			//double data 9��ͨ������У��
			SerialPortRx.Check = 0;
			b = SerialPortRx.Stack - UartRxDataLen; //��ʼλ
			for(i=b; i<SerialPortRx.Stack-1; i++)  //��У��λ���λ����У��
			{
				SerialPortRx.Check += SerialPortRx.Buffer[i];//У��
			}

			if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
			{
				//У��ɹ����������ݽ���
				for(i = 0; i<9; i++)
				{
					d = SerialPortRx.Buffer[b+i*4+4]*0x1000000L + SerialPortRx.Buffer[b+i*4+5]*0x10000L + SerialPortRx.Buffer[b+i*4+6]*0x100L + SerialPortRx.Buffer[b+i*4+7];
					if(d>0x7FFFFFFF)
					{
						d1 = 0x7FFFFFFF - d;
					}
					else
					{
						d1 = d;
					}
					UartData[i]=d1;
					UartData[i]/=65536.0;
				}
				UartDebug();  //תȥ�������ܵ������ݸ�������
			}
			SerialPortRx.Stack = 0;
		}
		else if(   SerialPortRx.Stack >= UartRxCmdLen //UartRxDataLen����Ϊһ֡
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen]  ==0xff
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+1]==0x55
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+2]==0xaa
		&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen+3]==0x77 )//cmd
		{
			SerialPortRx.Check = 0;
			b = SerialPortRx.Stack - UartRxCmdLen; //��ʼλ
			for(i=b; i<SerialPortRx.Stack-1; i++)  //��У��λ���λ����У��
			{
				SerialPortRx.Check += SerialPortRx.Buffer[i];//У��
			}
			if( SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack-1] )
			{   //У��ɹ�
				UartCmd(UartCmdNum,UartCmdData);//������յ����������MCU�������
			}
			SerialPortRx.Stack = 0;
		}
	}
	else
	{
		SerialPortRx.Stack = 0;
	}
}
