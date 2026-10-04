/**
  ******************************************************************************
  * @file    mb_tcp.c
  * @brief   Modbus TCP 从站（上行侧）：MBAP 解析、功能码分派、数据读写
  * @note    单元号路由：TCP 单元号=从机号，地址=从机寄存器地址；
  *         读走网关缓存，写走"写穿透"（mb_gateway）。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdio.h>
#include "mb_tcp.h"
#include "mb_gateway.h"

/* ================= 常量 ================= */

#define MODBUS_TCP_PORT   502      /* Modbus TCP 标准端口 */
#define MODBUS_MAX_ADU    260      /* Modbus TCP ADU 最大字节数 */

/* 功能码 */
#define MB_FUNC_READ_HOLDING   0x03
#define MB_FUNC_READ_INPUT     0x04
#define MB_FUNC_WRITE_REG      0x06
#define MB_FUNC_WRITE_MULTI    0x10

/* 异常码（应答时功能码 | 0x80，数据段放异常码） */
#define MB_EXC_ILLEGAL_FUNCTION  0x01
#define MB_EXC_ILLEGAL_ADDRESS   0x02
#define MB_EXC_ILLEGAL_VALUE     0x03
#define MB_EXC_SLAVE_FAILURE     0x04

/* 1 = 允许 TCP 写穿透到 RS485；0 = 禁止（排查"谁在写"用） */
#define MB_ALLOW_WRITE   1

/* ================= 工具函数 ================= */

/**
  * @brief  从 2 个字节按大端读出一个 16 位数
  * @param  p  指向两字节（高字节在前）
  * @retval 拼好的 16 位值
  */
static uint16_t rd_u16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
  * @brief  把一个 16 位数按大端写入 2 个字节
  * @param  p  目标缓冲（写入 p[0]=高字节, p[1]=低字节）
  * @param  v  要写的 16 位值
  * @retval 无
  */
static void wr_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

/**
  * @brief  从 TCP 流里精确收满 len 字节（带超时，防止客户端半死把任务卡住）
  * @param  fd         已连接的 socket（非阻塞）
  * @param  buf        接收缓冲
  * @param  len        需要收满的字节数
  * @param  timeout_ms 总超时（毫秒）
  * @retval >0 实际收到的字节数；0 对端关闭；-1 超时/出错
  */
static int recv_exact(int fd, uint8_t *buf, int len, int timeout_ms)
{
  int got = 0;
  uint32_t t0 = HAL_GetTick();

  while (got < len)
  {
    int n = lwip_recv(fd, buf + got, len - got, 0);
    if (n > 0) { got += n; t0 = HAL_GetTick(); continue; }
    if (n == 0) return got;                              /* 对端关闭 */
    if ((HAL_GetTick() - t0) >= (uint32_t)timeout_ms) return -1;  /* 超时 */
    osDelay(2);
  }
  return got;
}

/**
  * @brief  把一段字节发到发完为止（带超时，处理 send 只发了一部分）
  * @param  fd   已连接的 socket（非阻塞）
  * @param  buf  待发送数据
  * @param  len  总长度
  * @retval 0 成功；-1 超时/出错
  */
static int send_all(int fd, const uint8_t *buf, int len)
{
  int sent = 0;
  uint32_t t0 = HAL_GetTick();

  while (sent < len)
  {
    int n = lwip_send(fd, (const void *)(buf + sent), len - sent, 0);
    if (n > 0) { sent += n; t0 = HAL_GetTick(); continue; }
    if ((HAL_GetTick() - t0) >= 5000U) return -1;
    osDelay(2);
  }
  return 0;
}

/**
  * @brief  处理一帧 Modbus TCP 请求，生成应答到 resp
  * @param  req      收到的完整请求帧（MBAP + PDU）
  * @param  req_len  请求帧长度（当前实现未使用）
  * @param  resp     应答缓冲（至少 MODBUS_MAX_ADU 字节）
  * @retval >0 应答帧长度；0 请求应被忽略（如协议号不为 0）
  * @note   单元号路由：TCP 单元号=从机号，地址=从机寄存器地址。
  *         读走 mbgw_read（缓存），写走 mbgw_write_*（写穿透）。
  */
static int modbus_handle(const uint8_t *req, int req_len, uint8_t *resp)
{
  uint16_t tid = rd_u16(req);        /* 事务号 */
  uint16_t pid = rd_u16(req + 2);    /* 协议号 */
  uint8_t  uid = req[6];             /* 单元号 = 从机号 */
  uint8_t  func = req[7];            /* 功能码 */
  uint8_t *pdu;
  uint8_t *p;
  int pdu_len;
  int i;

  (void)req_len;

  if (pid != 0)
  {
    return 0;                        /* 协议号必须为 0 */
  }

  /* ---- 填 MBAP 头，长度稍后补 ---- */
  wr_u16(resp, tid);
  wr_u16(resp + 2, 0);
  resp[6] = uid;
  pdu = resp + 7;
  p = pdu;

  switch (func)
  {
    case MB_FUNC_READ_HOLDING:
    case MB_FUNC_READ_INPUT:
    {
      uint16_t start = rd_u16(req + 8);
      uint16_t qty   = rd_u16(req + 10);
      uint16_t vals[125];

      if (qty < 1 || qty > 125)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_ILLEGAL_VALUE;
        break;
      }
      if (uid < 1 || uid > MB_MAX_SLAVE || (uint32_t)start + qty > MB_MAX_REGS)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_ILLEGAL_ADDRESS;
        break;
      }
      if (!mbgw_is_online(uid))
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_SLAVE_FAILURE;      /* 从机离线/未就绪 */
        break;
      }

      *p++ = func;
      *p++ = (uint8_t)(qty * 2);          /* 字节数 = 寄存器数 × 2 */

      mbgw_read(uid, start, qty, vals);
      for (i = 0; i < (int)qty; i++)
      {
        wr_u16(p, vals[i]);
        p += 2;
      }
      break;
    }

    case MB_FUNC_WRITE_REG:
    {
      uint16_t addr = rd_u16(req + 8);
      uint16_t val  = rd_u16(req + 10);

      if (uid < 1 || uid > MB_MAX_SLAVE || addr >= MB_MAX_REGS)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_ILLEGAL_ADDRESS;
        break;
      }
#if (MB_ALLOW_WRITE == 0)
      *p++ = (uint8_t)(func | 0x80);
      *p++ = MB_EXC_ILLEGAL_FUNCTION;
      break;
#else
      {
        char b[80];
        int n = snprintf(b, sizeof(b), "[MB] WRITE06 uid=%u addr=%u val=%u\r\n",
                         (unsigned)uid, (unsigned)addr, (unsigned)val);
        HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
      }
#endif
      if (mbgw_write_single(uid, addr, val) != 0)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_SLAVE_FAILURE;
        break;
      }
      *p++ = func;                        /* 0x06 应答 = 原样回显 地址+值 */
      wr_u16(p, addr); p += 2;
      wr_u16(p, val);  p += 2;
      break;
    }

    case MB_FUNC_WRITE_MULTI:             /* 0x10 写多个保持寄存器 */
    {
      uint16_t start = rd_u16(req + 8);
      uint16_t qty   = rd_u16(req + 10);
      uint8_t  bc    = req[12];           /* 字节数 */
      uint16_t vals[123];                 /* 0x10 单次最多 123 个 */
      int j;

      if (qty < 1 || qty > 123 || bc != (uint8_t)(qty * 2))
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_ILLEGAL_VALUE;
        break;
      }
      if (uid < 1 || uid > MB_MAX_SLAVE || (uint32_t)start + qty > MB_MAX_REGS)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_ILLEGAL_ADDRESS;
        break;
      }
      for (j = 0; j < (int)qty; j++)
      {
        vals[j] = rd_u16(req + 13 + j * 2);   /* PDU 数据从下标 13 开始 */
      }

#if (MB_ALLOW_WRITE == 0)
      *p++ = (uint8_t)(func | 0x80);
      *p++ = MB_EXC_ILLEGAL_FUNCTION;
      break;
#else
      {
        char b[80];
        int n = snprintf(b, sizeof(b), "[MB] WRITE10 uid=%u start=%u qty=%u\r\n",
                         (unsigned)uid, (unsigned)start, (unsigned)qty);
        HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
      }
#endif
      if (mbgw_write_multiple(uid, start, qty, vals) != 0)
      {
        *p++ = (uint8_t)(func | 0x80);
        *p++ = MB_EXC_SLAVE_FAILURE;
        break;
      }

      *p++ = func;                        /* 0x10 应答 = 功能码 + 起始地址 + 数量 */
      wr_u16(p, start); p += 2;
      wr_u16(p, qty);   p += 2;
      break;
    }

    default:
      *p++ = (uint8_t)(func | 0x80);
      *p++ = MB_EXC_ILLEGAL_FUNCTION;
      break;
  }

  pdu_len = (int)(p - pdu);
  wr_u16(resp + 4, (uint16_t)(pdu_len + 1));  /* 长度 = 单元号 + PDU */
  return 7 + pdu_len;
}

/* ================= 服务器任务 ================= */

/**
  * @brief  Modbus TCP 服务器任务（上行，对电脑）
  * @param  argument  未使用
  * @retval 无（任务永不返回）
  * @note   监听 502，accept 一条连接后反复"收请求 -> modbus_handle -> 发应答"，
  *         直到对端关闭；当前一次只服务一个客户端。
  */
void mbtcp_task(void *argument)
{
  int listen_fd;
  int conn_fd;
  struct sockaddr_in addr;
  struct sockaddr_in cli;
  socklen_t cli_len;
  uint8_t req[MODBUS_MAX_ADU];
  uint8_t resp[MODBUS_MAX_ADU];

  (void)argument;

  /* ---- 服务器五步的前三步（只做一次） ---- */
  listen_fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd < 0)
  {
    for (;;) osDelay(1000);
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(MODBUS_TCP_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (lwip_bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    lwip_close(listen_fd);
    for (;;) osDelay(1000);
  }

  if (lwip_listen(listen_fd, 2) < 0)
  {
    lwip_close(listen_fd);
    for (;;) osDelay(1000);
  }

  HAL_UART_Transmit(&huart1, (uint8_t*)"[MB] Modbus TCP listening on 502\r\n",
                    (uint16_t)strlen("[MB] Modbus TCP listening on 502\r\n"), HAL_MAX_DELAY);

  /* ---- 主循环 ---- */
  for (;;)
  {
    cli_len = sizeof(cli);
    conn_fd = lwip_accept(listen_fd, (struct sockaddr *)&cli, &cli_len);
    if (conn_fd < 0)
    {
      osDelay(10);
      continue;
    }

    /* 打印客户端 IP */
    {
      char line[64];
      uint8_t *ip = (uint8_t *)&cli.sin_addr.s_addr;
      int len = snprintf(line, sizeof(line), "[MB] client %u.%u.%u.%u\r\n",
                         ip[0], ip[1], ip[2], ip[3]);
      HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)len, HAL_MAX_DELAY);
    }

    /* 设为非阻塞：收/发都由上面的带超时循环处理，客户端半死不会卡住本任务 */
    {
      int nb = 1;
      (void)lwip_ioctl(conn_fd, FIONBIO, &nb);
    }

    /* 同一条连接上反复处理请求，直到对端关闭 */
    for (;;)
    {
      uint16_t pid, alen;
      int pdu_len, rlen, r;

      /* 1) 先收 7 字节 MBAP，取出“长度”字段（空闲最多等 30s） */
      r = recv_exact(conn_fd, req, 7, 30000);
      if (r <= 0) break;

      pid  = rd_u16(req + 2);
      alen = rd_u16(req + 4);            /* = 单元号 + PDU */

      if (pid != 0 || alen < 2 || alen > 254)
      {
        break;                           /* 非法帧，断开连接 */
      }

      /* 2) 再收 PDU（alen-1 字节，帧内很快，最多等 2s） */
      pdu_len = (int)alen - 1;
      r = recv_exact(conn_fd, req + 7, pdu_len, 2000);
      if (r <= 0) break;

      /* 3) 解析并生成应答 */
      rlen = modbus_handle(req, 7 + pdu_len, resp);

      /* 4) 发送应答 */
      if (rlen > 0)
      {
        if (send_all(conn_fd, resp, rlen) < 0) break;
      }
    }

    lwip_close(conn_fd);
    HAL_UART_Transmit(&huart1, (uint8_t*)"[MB] closed\r\n",
                      (uint16_t)strlen("[MB] closed\r\n"), HAL_MAX_DELAY);
  }
}
