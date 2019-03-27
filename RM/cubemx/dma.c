
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "stm32f4xx_hal_dma.h"

//句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define BUFFER_SIZE  100

volatile uint8_t rx_len = 0;             //一帧数据长度
volatile uint8_t recv_end_flag = 0;    //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};   //接受数据缓存
/*
void mydma_rx_init()
{
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);          //使能IDLE中断
	HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);  //打开DMA接收
}


//串口发送函数
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

//串口读取函数
void DMA_Usart1_Read(uint8_t *Data,uint8_t len)
 {
    HAL_UART_Receive_DMA(&huart1,Data,len);//重新打开DMA接收
 }

*/



