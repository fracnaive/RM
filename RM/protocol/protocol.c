
#include "protocol.h"


#if 0

//添加协议帧
uint32_t protocol_s_add_sendnode(uint8_t reciver, uint8_t session, uint8_t pack_type,
                                 void *p_data, uint32_t data_len, uint16_t cmd, uint16_t ack_seq)
{
  send_ctx_t ctx = {0};
  struct perph_interface *int_obj;
  uint32_t status;
  uint32_t malloc_size;
  uint8_t *malloc_zone;
  uint32_t pack_head_offset;
  protocol_pack_desc_t *pack_head;
  send_list_node_t *send_node;
  uint16_t seq;

  status = PROTOCOL_SUCCESS;

  if (data_len > PROTOCOL_MAX_DATA_LEN)
  {
    status = PROTOCOL_ERR_DATA_TOO_LONG;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  //配置发送参数
  ctx.s_a_r.pack_type = pack_type;
  ctx.s_a_r.session = session;
  ctx.s_a_r.res = 0;
  ctx.reciver = reciver;
  ctx.version = PROTOCOL_VERSION;

  //获取路由接口
  int_obj = protocol_s_get_route(reciver);

  if (int_obj == NULL)
  {
    status = PROTOCOL_ERR_ROUTE_NOT_FOUND;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);

    return status;
  }

  if ((pack_type == PROTOCOL_PACK_NOR) && (session != 0))
  {
    if (protocol_s_session_get_node(int_obj, reciver, session) != NULL)
    {
      status = PROTOCOL_ERR_SESSION_IS_USE;
      PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
      return status;
    }
  }

  //分配数据帧所需内存
  if (pack_type == PROTOCOL_PACK_ACK)
  {
    malloc_size = PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_SEND_NODE_SIZE +
                  data_len;
  }
  else
  {
    malloc_size = PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_SEND_NODE_SIZE +
                  data_len + PROTOCOL_PACK_CMD_SIZE;
  }
  malloc_zone = protocol_p_malloc(malloc_size);
  if (malloc_zone == NULL)
  {
    status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  if (pack_type == PROTOCOL_PACK_NOR)
  {
    MUTEX_LOCK(int_obj->send.mutex_lock);
    seq = int_obj->send.send_seq++;
    MUTEX_UNLOCK(int_obj->send.mutex_lock);
  }
  else
  {
    seq = ack_seq;
  }

  pack_head_offset = PROTOCOL_SEND_NODE_SIZE;
  pack_head = (protocol_pack_desc_t *)&malloc_zone[pack_head_offset];
  send_node = (send_list_node_t *)&malloc_zone[0];

  //填充帧数据部分
  protocol_s_fill_pack(&ctx, p_data, data_len, (uint8_t *)(pack_head), seq, cmd);

  //填充send_node
  send_node->session = ctx.s_a_r.session;
  send_node->p_data = &malloc_zone[pack_head_offset];
  send_node->len = malloc_size - PROTOCOL_SEND_NODE_SIZE;
  send_node->pre_timestamp = 0;
  send_node->is_got_ack = 0;
  send_node->is_first_send = 1;
  send_node->address = reciver;
  send_node->pack_type = pack_type;
  send_node->is_ready_realse = 0;
  send_node->cmd = cmd;
  send_node->forward_src_obj = NULL;

  struct send_cmd_info *cmd_info;
  cmd_info = protocol_get_send_cmd_info(cmd);
  if (cmd_info != NULL)
  {
    send_node->rest_cnt = cmd_info->resend_times;
    send_node->timeout = cmd_info->resend_timeout;
    send_node->ack_callback = cmd_info->ack_callback;
    send_node->no_ack_callback = cmd_info->no_ack_callback;
  }
  else
  {
    send_node->rest_cnt = 1;
    send_node->timeout = 0;
    send_node->ack_callback = NULL;
    send_node->no_ack_callback = NULL;
  }

  //添加至发送链表
  MUTEX_LOCK(int_obj->send.mutex_lock);

  if ((pack_type == PROTOCOL_PACK_NOR) && (session != 0))
  {
    if (protocol_s_session_get_node(int_obj, reciver, session) != NULL)
    {
      status = PROTOCOL_ERR_SESSION_IS_USE;
      MUTEX_UNLOCK(int_obj->send.mutex_lock);
      PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
      return status;
    }
  }

  if (pack_type == PROTOCOL_PACK_NOR)
  {
    list_add(&(send_node->send_list), &(int_obj->send.normal_list_header));
    int_obj->send.normal_node_num++;
  }
  else
  {
    list_add(&(send_node->send_list), &(int_obj->send.ack_list_header));
    int_obj->send.ack_node_num++;
  }

  MUTEX_UNLOCK(int_obj->send.mutex_lock);

  if (pack_type == PROTOCOL_PACK_NOR)
  {
      PROTOCOL_SEND_DBG_PRINTF("Send pack, Reciver:0x%02X, Cmd:0x%04X, Session: %d Normal pack.",
                              reciver, cmd, session);
  }
  else
  {
    PROTOCOL_SEND_DBG_PRINTF("Send pack, Reciver:0x%02X, Session: %d Ack pack.",
                              reciver, session);
  }

  return status;
}

/**
  * @brief  协议发送正常帧。
  * @param  reciver 接收设备地址
  *         session 会话号，范围为0~63，当session为0时不要求接收方Ack，否则要求Ack。同一时间内一个接收设备不能有两个
  *         相同的会话号
  *         cmd 命令值
  *         p_data 发送数据指针
  *         data_len 发送数据长度
  * @retval 协议返回状态
  */
uint32_t protocol_send(uint8_t reciver, uint16_t cmd, void *p_data, uint32_t data_len)
{
  uint32_t status;
  uint8_t session = 0;
  uint8_t ack = 0;

  struct send_cmd_info *cmd_info;
  cmd_info = protocol_get_send_cmd_info(cmd);
  
  struct perph_interface *int_obj;
  int_obj = protocol_s_get_route(reciver);
  
  if (cmd_info != NULL)
  {
    ack = cmd_info->ack_enable;
  }

  if (reciver == PROTOCOL_BROADCAST_ADDR)
  {
    status = protocol_s_broadcast_add_node(p_data, data_len, cmd);
  }
  else
  {
    if (ack == 1)
    {
      session = protocol_get_session(int_obj);
    }
    status = protocol_s_add_sendnode(reciver, session, PROTOCOL_PACK_NOR, p_data,
                                     data_len, cmd, 0);
  }
  if (status == PROTOCOL_SUCCESS)
  {
    if (protocol_local_info.send_list_add_callBack != NULL)
    {
      protocol_local_info.send_list_add_callBack();
    }
  }
  else
  {
    if (ack == 1)
    {
      protocol_release_session(int_obj, session);
    }
  }
  return status;
}

#endif