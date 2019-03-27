#ifndef _RC_HANDLE_H_
#define _RC_HANDLE_H_
#include "stdint.h"


//define
#define RC_RX_BUFFER_SIZE      (50u)
#define RC_DATE_LEN            (18U)

/**
  * @brief  遥控数据结构体
  */
typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t left;
  uint8_t right;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
}RC_TypeDef;


//变量
extern RC_TypeDef RC_Info,Last_RC_Info;
//extern uint8_t RC_Buf[];
//函数
void get_RC_date_Handle(RC_TypeDef* rc, uint8_t* buff);


#endif
