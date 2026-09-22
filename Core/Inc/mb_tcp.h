/**
  ******************************************************************************
  * @file    mb_tcp.h
  * @brief   Modbus TCP 从站（上行侧）：服务器任务
  ******************************************************************************
  */
#ifndef __MB_TCP_H
#define __MB_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Modbus TCP 服务器任务体：监听 502，处理请求（任务永不返回）
  * @param  argument  未使用
  * @retval 无
  */
void mbtcp_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MB_TCP_H */
