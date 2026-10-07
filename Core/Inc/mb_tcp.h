/**
  ******************************************************************************
  * @file    mb_tcp.h
  * @brief   Modbus TCP 从站（上行侧）对外接口：服务器任务
  * @note    地址映射：TCP 单元号 = 从机号；TCP 寄存器地址 = 从机寄存器地址。
  *          读走网关缓存，写走写穿透（mb_gateway）。
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
  * @note   一次只服务一个客户端；客户端非阻塞 + 收发带超时。
  */
void mbtcp_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MB_TCP_H */
