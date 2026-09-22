/**
  ******************************************************************************
  * @file    mb_rtu.c
  * @brief   Modbus RTU 主站（下行侧）：RS485 收发 + CRC + 主站读写
  * @note    依赖：UART5(RS485) + PD1(收发方向) + 独立总线互斥锁。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "mb_rtu.h"

/* ================= RS485 物理层（UART5 + PD1 方向控制） ================= */

#define RS485_EN_PORT   GPIOD
#define RS485_EN_PIN    GPIO_PIN_1

#define RS485_TX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_SET)
#define RS485_RX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_RESET)

/** 保护 RS485 总线的互斥锁（轮询与写转发互斥使用） */
static osMutexId_t s_busMutex;

/**
  * @brief   通过 RS485 发送一帧（自动切换收发方向）
  * @param   data  待发送数据
  * @param   len   长度
  * @retval  0  成功；-1 UART 发送失败
  * @note    拉高 EN(发送) -> UART 发送 -> 等 TC(发完) -> 拉低 EN(接收)。
  */
static int rs485_send(const uint8_t *data, uint16_t len)
{
  RS485_TX();
  if (HAL_UART_Transmit(&huart5, (uint8_t *)data, len, 100) != HAL_OK)
  {
    RS485_RX();
    return -1;
  }
  while (!(UART5->SR & USART_SR_TC)) { }   /* 等最后一字节发完，否则会截断 */
  RS485_RX();
  return 0;
}

/**
  * @brief   按“静默时间”接收一帧（RTU 无长度字段，靠 3.5 字符静默分帧）
  * @param   buf              接收缓冲
  * @param   maxlen           缓冲容量上限（不是期望长度）
  * @param   first_timeout_ms 等第一个字节的最长时间（毫秒）
  * @retval  实际收到的字节数；0 表示超时没收到任何字节
  */
static int rs485_recv(uint8_t *buf, uint16_t maxlen, uint32_t first_timeout_ms)
{
  uint8_t b;
  int len = 0;

  if (HAL_UART_Receive(&huart5, &b, 1, first_timeout_ms) != HAL_OK)
  {
    return 0;
  }
  buf[len++] = b;

  while (len < (int)maxlen)
  {
    if (HAL_UART_Receive(&huart5, &b, 1, 4) != HAL_OK)
    {
      break;   /* 4ms 静默 -> 帧结束（9600 下 3.5 字符约 3.6ms） */
    }
    buf[len++] = b;
  }
  return len;
}

/**
  * @brief   把一段字节以 HEX 形式打印到调试串口（USART1）
  * @param   tag   前缀文字
  * @param   data  数据
  * @param   len   长度
  * @retval  无
  */
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
  * @brief   计算 Modbus CRC16（多项式 0xA001，初值 0xFFFF）
  * @param   data  数据
  * @param   len   长度
  * @retval  16 位 CRC 值；发送时“低字节在前”追加到帧尾
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
  s_busMutex = osMutexNew(NULL);
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

  /* 2~3. 总线互斥：发送 + 接收独占总线 */
  osMutexAcquire(s_busMutex, osWaitForever);

  if (rs485_send(req, 8) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }

  rxlen = rs485_recv(resp, sizeof(resp), timeout_ms);

  osMutexRelease(s_busMutex);

  if (rxlen < 5)
  {
    return -2;
  }

  /* 4. 从机地址必须匹配 */
  if (resp[0] != slave)
  {
    return -3;
  }

  /* 5. CRC 必须对 */
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

  /* 7. 字节数校验 + 取数据（大端） */
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
  if (rs485_send(req, 8) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }
  rxlen = rs485_recv(resp, sizeof(resp), timeout_ms);
  osMutexRelease(s_busMutex);

  if (rxlen < 8)
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
  if (rs485_send(req, (uint16_t)reqlen) != 0)
  {
    osMutexRelease(s_busMutex);
    return -1;
  }
  rxlen = rs485_recv(resp, sizeof(resp), timeout_ms);
  osMutexRelease(s_busMutex);

  if (rxlen < 8)
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
