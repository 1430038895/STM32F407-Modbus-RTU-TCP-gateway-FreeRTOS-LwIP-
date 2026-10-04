/**
  ******************************************************************************
  * @file    mb_rtu.c
  * @brief   Modbus RTU 主站（下行侧）：RS485 收发 + CRC + 主站读写
  * @note    依赖：UART5(RS485) + PD1(收发方向) + 独立总线互斥锁�?  ******************************************************************************
  */

#include "main.h"

#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "mb_rtu.h"

/* ================= RS485 物理层（UART5 + PD1 方向控制�?================= */

#define RS485_EN_PORT   GPIOD
#define RS485_EN_PIN    GPIO_PIN_1

#define RS485_TX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_SET)
#define RS485_RX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_RESET)

/** 保护 RS485 总线的互斥锁（轮询与写转发互斥使用） */
static osMutexId_t s_busMutex;

/* ---- 接收：中断 + 环形缓冲（115200 下也不会因为调度丢字节） ---- */
#define RS485_RXBUFSZ   512                 /* 环形缓冲大小，必须是 2 的幂 */
#define RS485_RXMASK    (RS485_RXBUFSZ - 1)
#define RS485_GAP_MS    0                    /* 帧间静默：响应往返本身就提供了间隔，设为 0 */

static uint8_t          s_rxbuf[RS485_RXBUFSZ];  /* 环形接收缓冲 */
static volatile uint16_t s_rx_head = 0;          /* 中断写入位置 */
static volatile uint16_t s_rx_tail = 0;          /* 任务读取位置 */
static uint8_t          s_rxbyte;                /* 逐字节接收的暂存 */
static volatile uint32_t s_rx_total = 0;         /* 中断累计收到的字节数（诊断用） */

/** @brief 环形缓冲里还有几个字节可读 */
static inline uint16_t rs485_avail(void)
{
  return (uint16_t)((s_rx_head - s_rx_tail) & RS485_RXMASK);
}

/** @brief 中断收到一个字节：入缓冲并重新挂上接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    s_rxbuf[s_rx_head & RS485_RXMASK] = s_rxbyte;
    s_rx_head = (uint16_t)((s_rx_head + 1) & RS485_RXMASK);
    s_rx_total++;
    HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);
  }
}

/** @brief 接收出错（溢出等）：清标志并重新挂上接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);
  }
}

/** @brief UART5 中断入口（代码里自带，不用在 CubeMX 里勾选） */
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

/**
  * @brief   通过 RS485 发送一帧（自动切换收发方向�?  * @param   data  待发送数�?  * @param   len   长度
  * @retval  0  成功�?1 UART 发送失�?  * @note    拉高 EN(发�? -> UART 发�?-> �?TC(发完) -> 拉低 EN(接收)�?  */
static int rs485_send(const uint8_t *data, uint16_t len)
{
  RS485_TX();
  if (HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 100) != HAL_OK)
  {
    RS485_RX();
    return -1;
  }
  {
    uint32_t t0 = HAL_GetTick();
    while (!(UART5->SR & USART_SR_TC))
    {
      if ((HAL_GetTick() - t0) > 20) break;   /* 最多等 20ms，防死等 */
    }
  }
  RS485_RX();
  return 0;
}

/**
  * @brief   �?n 个字节（每字节最多等 to_ms 毫秒�?  * @param   buf  接收缓冲
  * @param   n    要收的字节数
  * @param   to_ms 每字节超时（毫秒�?  * @retval  实际收到的字节数
  */
static int rs485_recv_n(uint8_t *buf, int n, uint32_t to_ms)
{
  int got = 0;
  while (got < n)
  {
    uint32_t t0 = HAL_GetTick();
    while (rs485_avail() == 0)
    {
      if ((HAL_GetTick() - t0) >= to_ms) return got;    /* 每个字节最多等 to_ms */
      osDelay(1);                                        /* 让出 CPU 给屏幕等低优先级任务 */
    }
    buf[got++] = s_rxbuf[s_rx_tail & RS485_RXMASK];     /* 从环形缓冲取一个 */
    s_rx_tail = (uint16_t)((s_rx_tail + 1) & RS485_RXMASK);
  }
  return got;
}

/**
  * @brief   清空 RX 残留（发送新请求前调用，避免上一帧的尾巴污染本帧�?  * @retval  �?  */
static void rs485_flush_rx(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart5);           /* 清溢出等错误标志 */
  s_rx_tail = s_rx_head;                        /* 丢弃环形缓冲里的所有残留 */
#if (RS485_GAP_MS > 0)
  osDelay(RS485_GAP_MS);                        /* 帧间静默（响应往返已提供间隔，默认不延时） */
#endif
}

/**
  * @brief   按“已知长度”读完整一帧应答（先对齐帧头，再按功能码定长）
  * @param   buf       接收缓冲
  * @param   maxlen    缓冲容量
  * @param   slave     期望的从机地址
  * @param   first_to  等第一个字节的最长时间（毫秒�?  * @retval  实际收到的总字节数�? 表示从机没应�?  * @note    第一个字节必须是 slave，否则跳过（引导杂波）；异常�?5 字节�?  *          0x03/0x04 应答 = 3 + 字节�?+ 2�?x06/0x10 应答 = 8 字节�?  */
static int rs485_recv_resp(uint8_t *buf, uint16_t maxlen, uint8_t slave, uint32_t first_to)
{
  int got, skip;

  got = rs485_recv_n(buf, 1, first_to);        /* 等第一个字�?*/
  if (got < 1) return 0;

  for (skip = 0; buf[0] != slave && skip < 8; skip++)
  {
    if (rs485_recv_n(buf, 1, 50) < 1) return 0;  /* 跳过引导杂波 */
  }
  if (buf[0] != slave) return 1;

  if (rs485_recv_n(buf + 1, 1, 50) < 1) return 1;   /* 功能�?*/

  if (buf[1] & 0x80)                                /* 异常应答：还需 3 字节 */
  {
    return 2 + rs485_recv_n(buf + 2, 3, 50);
  }

  if (buf[1] == 0x03 || buf[1] == 0x04)             /* 读应�?*/
  {
    int bc;
    if (rs485_recv_n(buf + 2, 1, 50) < 1) return 2;
    bc = buf[2];
    if (bc > (int)maxlen - 5) bc = (int)maxlen - 5;
    return 3 + rs485_recv_n(buf + 3, bc + 2, 50);
  }

  return 2 + rs485_recv_n(buf + 2, 6, 50);          /* 0x06/0x10 应答 = 8 字节 */
}

/**
  * @brief   把一段字节以 HEX 形式打印到调试串口（USART1�?  * @param   tag   前缀文字
  * @param   data  数据
  * @param   len   长度
  * @retval  �?  */
static void rs485_print(const char *tag, const uint8_t *data, int len)
{
  char line[200];
  int n = snprintf(line, sizeof(line), "%s", tag);
  int i;
  for (i = 0; i < len && n < (int)sizeof(line) - 4; i++)
  {
    n += snprintf(line + n, sizeof(line) - n, " %02X", (unsigned)data[i]);
  }
  n += snprintf(line + n, sizeof(line) - n, "\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);
}

/**
  * @brief   计算 Modbus CRC16（多项式 0xA001，初�?0xFFFF�?  * @param   data  数据
  * @param   len   长度
  * @retval  16 �?CRC 值；发送时“低字节在前”追加到帧尾
  */
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint16_t i;
  int b;
  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (b = 0; b < 8; b++)
    {
      if (crc & 1U) crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      else          crc = (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

/* ================= 对外接口 ================= */

void mbrtu_init(void)
{
  HAL_StatusTypeDef st;
  char b[64];
  int n;

  s_busMutex = osMutexNew(NULL);

  /* 打开 UART5 全局中断，并启动逐字节中断接收 */
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  st = HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);

  n = snprintf(b, sizeof(b), "[RTU] UART5 RX-IT start: st=%d (0=OK)\r\n", (int)st);
  HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, HAL_MAX_DELAY);
}

uint32_t mbrtu_rx_total(void) { return s_rx_total; }

/* 自愈：若 RXNE 中断被关了、或接收状态异常，重新挂上（长时间运行防卡死） */
void mbrtu_rx_heal(void)
{
  if ((UART5->CR1 & USART_CR1_RXNEIE) == 0U || huart5.RxState != HAL_UART_STATE_BUSY_RX)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    (void)HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);
  }
}

int mbrtu_read(uint8_t slave, uint8_t func, uint16_t start, uint16_t count,
               uint16_t *out, uint32_t timeout_ms)
{
  uint8_t req[8];
  uint8_t resp[256];
  uint16_t crc;
  int rxlen;
  uint16_t i;

  /* 1. 组请求帧：[从机][功能码][起始地址][数量][CRC低][CRC高] */
  req[0] = slave;
  req[1] = func;
  req[2] = (uint8_t)(start >> 8);
  req[3] = (uint8_t)(start & 0xFF);
  req[4] = (uint8_t)(count >> 8);
  req[5] = (uint8_t)(count & 0xFF);
  crc = modbus_crc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  /* 2~3. 总线互斥：发�?+ 接收独占总线 */
  osMutexAcquire(s_busMutex, osWaitForever);

  rs485_flush_rx();                             /* 清掉上一次的残留 */
  if (rs485_send(req, 8) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }

  rxlen = rs485_recv_resp(resp, sizeof(resp), slave, timeout_ms);

  osMutexRelease(s_busMutex);

  if (rxlen < 5)
  {
    rs485_print("[RTU] no resp:", resp, (rxlen > 0) ? rxlen : 0);
    return -2;
  }

  /* 4. 从机地址必须匹配 */
  if (resp[0] != slave)
  {
    rs485_print("[RTU] addr mismatch:", resp, rxlen);
    return -3;
  }

  /* 5. CRC 必须�?*/
  crc = modbus_crc16(resp, (uint16_t)(rxlen - 2));
  if ((resp[rxlen - 2] != (uint8_t)(crc & 0xFF)) ||
      (resp[rxlen - 1] != (uint8_t)(crc >> 8)))
  {
    rs485_print("[RTU] crc fail:", resp, rxlen);
    return -4;
  }

  /* 6. 异常应答：功能码 = 原功能码 | 0x80 */
  if (resp[1] == (uint8_t)(func | 0x80))
  {
    rs485_print("[RTU] exception:", resp, rxlen);
    return -5;
  }
  if (resp[1] != func)
  {
    return -6;
  }

  /* 7. 字节数校�?+ 取数据（大端�?*/
  if (resp[2] != (uint8_t)(count * 2))
  {
    return -7;
  }
  for (i = 0; i < count; i++)
  {
    out[i] = (uint16_t)(((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2]);
  }
  return 0;
}

int mbrtu_write_single(uint8_t slave, uint16_t reg, uint16_t value, uint32_t timeout_ms)
{
  uint8_t req[8];
  uint8_t resp[16];
  uint16_t crc;
  int rxlen;

  /* 组请求帧：[从机][0x06][寄存器地址][值][CRC低][CRC高] */
  req[0] = slave;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  crc = modbus_crc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  osMutexAcquire(s_busMutex, osWaitForever);
  rs485_flush_rx();
  if (rs485_send(req, 8) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }
  rxlen = rs485_recv_resp(resp, sizeof(resp), slave, timeout_ms);
  osMutexRelease(s_busMutex);

  if (rxlen < 5)
  {
    return -2;
  }
  if (resp[0] != slave)
  {
    return -3;
  }
  crc = modbus_crc16(resp, (uint16_t)(rxlen - 2));
  if ((resp[rxlen - 2] != (uint8_t)(crc & 0xFF)) ||
      (resp[rxlen - 1] != (uint8_t)(crc >> 8)))
  {
    return -4;
  }
  if (resp[1] == (uint8_t)(0x06 | 0x80))
  {
    return -5;
  }
  if (resp[1] != 0x06)
  {
    return -6;
  }
  return 0;
}

int mbrtu_write_multiple(uint8_t slave, uint16_t reg, uint16_t count,
                         const uint16_t *values, uint32_t timeout_ms)
{
  uint8_t req[260];
  uint8_t resp[16];
  uint16_t crc;
  int rxlen;
  int reqlen;
  int i;

  if (count < 1 || count > 123)
  {
    return -8;
  }

  /* 组请求帧：[从机][0x10][起始地址][数量][字节数][数据...][CRC低][CRC高] */
  req[0] = slave;
  req[1] = 0x10;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(count >> 8);
  req[5] = (uint8_t)(count & 0xFF);
  req[6] = (uint8_t)(count * 2);
  for (i = 0; i < (int)count; i++)
  {
    req[7 + i * 2]     = (uint8_t)(values[i] >> 8);
    req[7 + i * 2 + 1] = (uint8_t)(values[i] & 0xFF);
  }
  reqlen = 7 + count * 2;
  crc = modbus_crc16(req, (uint16_t)reqlen);
  req[reqlen]     = (uint8_t)(crc & 0xFF);
  req[reqlen + 1] = (uint8_t)(crc >> 8);
  reqlen += 2;

  osMutexAcquire(s_busMutex, osWaitForever);
  rs485_flush_rx();
  if (rs485_send(req, (uint16_t)reqlen) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }
  rxlen = rs485_recv_resp(resp, sizeof(resp), slave, timeout_ms);
  osMutexRelease(s_busMutex);

  if (rxlen < 5)
  {
    return -2;
  }
  if (resp[0] != slave)
  {
    return -3;
  }
  crc = modbus_crc16(resp, (uint16_t)(rxlen - 2));
  if ((resp[rxlen - 2] != (uint8_t)(crc & 0xFF)) ||
      (resp[rxlen - 1] != (uint8_t)(crc >> 8)))
  {
    return -4;
  }
  if (resp[1] == (uint8_t)(0x10 | 0x80))
  {
    return -5;
  }
  if (resp[1] != 0x10)
  {
    return -6;
  }
  return 0;
}
