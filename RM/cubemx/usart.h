
#ifndef _USART_H_
#define _USART_H_

#include "stdint.h"
#include "stm32f4xx_hal_uart.h"

#define DR16_RX_BUFFER_SIZE      (50u)
#define DR16_DATA_LEN            (18u)
extern uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];

void dr16_uart_init(void);
uint8_t usart1_rx_handle(UART_HandleTypeDef *huart);//中断处理


//typedef int32_t (*dr16_rx_callback_t)(uint8_t *buff, uint16_t len);  //函数指针
//int32_t dr16_rx_uart_callback_register(dr16_rx_callback_t fn);


#define RXBUFFERSIZE   1 //串口2缓存大小
extern uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库USART2接收Buffer

//函数
extern void myRX_usart_IT(void);
extern void myTX_usart_IT(void);
extern void myUsart_DMA_TX(void);
extern void myUsart_DMA_RX(void);


//变量声明
extern float target_speed; //遥控器得出轮子转速
#endif
