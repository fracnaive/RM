
#ifndef _USART_H_
#define _USART_H_

#include "stdint.h"
#include "stm32f4xx_hal_uart.h"

#define DR16_RX_BUFFER_SIZE      (50u)
#define DR16_DATA_LEN            (18u)
extern uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];

void dr16_uart_init(void);
uint8_t usart1_rx_handle(UART_HandleTypeDef *huart);//�жϴ���


//typedef int32_t (*dr16_rx_callback_t)(uint8_t *buff, uint16_t len);  //����ָ��
//int32_t dr16_rx_uart_callback_register(dr16_rx_callback_t fn);


#define RXBUFFERSIZE   1 //����2�����С
extern uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��USART2����Buffer

//����
extern void myRX_usart_IT(void);
extern void myTX_usart_IT(void);
extern void myUsart_DMA_TX(void);
extern void myUsart_DMA_RX(void);


//��������
extern float target_speed; //ң�����ó�����ת��
#endif
