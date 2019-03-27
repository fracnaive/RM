#include "stm32f4xx_hal.h"
#include "usart.h"
#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "cmsis_os.h"
#include "RC_handle.h"

extern void FreeCars_OnInterrupt(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern osSemaphoreId myBinarySem01Handle;

uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
uint8_t TxBuffer[] = "DMA_Semaphore test\n";//发送

float target_speed; //遥控器得出轮子转速

/***************初始化*********/
//usart1数据接收地址
uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];
int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);
//初始化
void dr16_uart_init(void)
{
  UART_Receive_DMA_No_IT(&huart1, dr16_uart_rx_buff, DR16_RX_BUFFER_SIZE);
  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
  uint32_t tmp = 0;

  tmp = huart->RxState;
  if (tmp == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                  (uint32_t)pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/****************USART1中断处理函数(串口空闲中断)******************/


void UART_IdleRxCallback(UART_HandleTypeDef *huart);
uint8_t usart1_rx_handle(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */
    if ((DR16_RX_BUFFER_SIZE - huart->hdmarx->Instance->NDTR) == DR16_DATA_LEN)
    {
      UART_IdleRxCallback(huart);
			
    }

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RX_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
  return 0;
}

/****************USART2_IT  mode**************/
void myRX_usart_IT()
{
	HAL_UART_Receive_IT(&huart2, (uint8_t  *)aRxBuffer, RXBUFFERSIZE);
	//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

void myTX_usart_IT()
{
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)TxBuffer,sizeof(TxBuffer));
}

/****************USART2_DMA  mode**************/

void myUsart_DMA_TX()
{
	
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)TxBuffer,sizeof(TxBuffer));
	
}

void myUsart_DMA_RX() //采用回环模式，持续接收蓝牙数据
{
	
	HAL_UART_Receive_DMA(&huart2, (uint8_t  *)aRxBuffer, RXBUFFERSIZE);
	
}


/****************USART回调函数**************/
//接收回调函数

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
		
	}
	else if(huart->Instance==USART2) //串口2，调试
	{
		
		//osSemaphoreRelease(myBinarySem01Handle);
	 //	flag++;
	
		FreeCars_OnInterrupt();
		
	}
	
	
	
}
//自定义空闲中断回调，数据处理
void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
    {
      //SelfCheck[DEBUS_DATACHECK].devicedata.FPS++;//FPS更新
      get_RC_date_Handle(&RC_Info, dr16_uart_rx_buff);
    }
	
}



//usart2发送回调
//中断触发后就清除发送中断使能，然后调用回调函数，执行完后结束本次发送
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
	//myTX_usart_IT();//开启发送
	// myUsart_DMA_TX();
	
	
}
