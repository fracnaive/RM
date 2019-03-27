#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include "stdint.h"


#if 0
typedef struct
{
  uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~63) Ack */
  uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
  uint8_t res : 2;       /*!< Reserve */
} S_A_R_t;




/* This Struct Is Used To Describe Send Information When Sending */
typedef struct
{
  uint16_t version; /*!< Version */
  uint8_t reciver;  /*!< Receiver Receiver Information */
  union {
    S_A_R_t s_a_r;
    uint8_t S_A_R_c; /*!< Ack And Session */
  };
} send_ctx_t;



#endif



#endif

