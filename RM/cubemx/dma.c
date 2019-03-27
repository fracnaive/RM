
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "stm32f4xx_hal_dma.h"

//���
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define BUFFER_SIZE  100

volatile uint8_t rx_len = 0;             //һ֡���ݳ���
volatile uint8_t recv_end_flag = 0;    //һ֡���ݽ�����ɱ�־
uint8_t rx_buffer[100]={0};   //�������ݻ���
/*
void mydma_rx_init()
{
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);          //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);  //��DMA����
}


//���ڷ��ͺ���
void DMA_Usart_Send(uint8_t *buf,uint8_t len)
{
   if(HAL_UART_Transmit_DMA(&huart1, buf,len)!= HAL_OK)
        {
            Error_Handler();
        }
    //##-3- Wait for the end of the transfer ###################################  
   // while (UartReady != SET){}
    //Reset transmission flag 
   // UartReady = RESET;
}

//���ڶ�ȡ����
void DMA_Usart1_Read(uint8_t *Data,uint8_t len)
 {
    HAL_UART_Receive_DMA(&huart1,Data,len);//���´�DMA����
 }

*/



