/**
  ******************************************************************************
  * @file    mb_rtu.c
  * @brief   Modbus RTU 主站（下行侧）：RS485 收发 + CRC + 主站读写
  * @note    依赖硬件：UART5（RS485，PC12=TX / PD2=RX）+ PD1（收发方向 DE/RE）
  *          + 一根总线互斥锁。接收采用"中断 + 环形缓冲"，分帧采用"按长度收"。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "mb_rtu.h"

/* ================= RS485 物理层（UART5 + PD1 方向控制） ================= */

#define RS485_EN_PORT   GPIOD        /**< 收发方向控制所在的 GPIO 端口 */
#define RS485_EN_PIN    GPIO_PIN_1   /**< 方向脚：高=发送(TX)，低=接收(RX) */

#define RS485_TX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_SET)     /**< 切到发送 */
#define RS485_RX()   HAL_GPIO_WritePin(RS485_EN_PORT, RS485_EN_PIN, GPIO_PIN_RESET)   /**< 切到接收 */

/** 保护 RS485 总线的互斥锁：保证任一时刻只有一个事务（轮询读 或 写穿透）占用总线 */
static osMutexId_t s_busMutex;

/* ================= 接收：中断 + 环形缓冲（115200 也不会因调度丢字节） ================= */

#define RS485_RXBUFSZ   512                 /**< 环形缓冲大小（必须是 2 的幂） */
#define RS485_RXMASK    (RS485_RXBUFSZ - 1) /**< 取模掩码（用按位与代替取模） */
#define RS485_GAP_MS    0                   /**< 帧间静默毫秒数；响应的往返本身就提供了间隔，默认 0 */

static uint8_t           s_rxbuf[RS485_RXBUFSZ];   /**< 环形接收缓冲 */
static volatile uint16_t s_rx_head = 0;            /**< 写指针（中断里推进） */
static volatile uint16_t s_rx_tail = 0;            /**< 读指针（任务里推进） */
static uint8_t           s_rxbyte;                 /**< 逐字节接收的单字节暂存（中断用） */
static volatile uint32_t s_rx_total = 0;           /**< 累计收到的字节数（诊断用） */

/**
  * @brief  环形缓冲里当前可读的字节数
  * @param  无
  * @retval 可读字节数
  */
static inline uint16_t rs485_avail(void)
{
  return (uint16_t)((s_rx_head - s_rx_tail) & RS485_RXMASK);
}

/**
  * @brief  UART 接收完成回调（HAL 弱函数，这里重写）
  * @param  huart  产生事件的外设句柄
  * @retval 无
  * @note   仅处理 UART5：把收到的字节压入环形缓冲，然后重新挂上单字节中断接收。
  *         运行在中断上下文，不能调用阻塞式 RTOS API。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    s_rxbuf[s_rx_head & RS485_RXMASK] = s_rxbyte;     /* 入缓冲 */
    s_rx_head = (uint16_t)((s_rx_head + 1) & RS485_RXMASK);
    s_rx_total++;
    HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);       /* 重新挂上，准备收下一个字节 */
  }
}

/**
  * @brief  UART 错误回调（如溢出 ORE）
  * @param  huart  产生错误的外设句柄
  * @retval 无
  * @note   清错误标志并重新挂上接收，避免一次溢出后永久收不到数据。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);
  }
}

/**
  * @brief  UART5 中断入口
  * @param  无
  * @retval 无
  * @note   直接在代码里提供该向量处理函数（就不用在 CubeMX 里勾选 UART5 中断）。
  */
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

/**
  * @brief  通过 RS485 发送一帧（自动切换收发方向）
  * @param  data  待发送数据
  * @param  len   长度
  * @retval 0=成功；-1=UART 发送失败
  * @note   拉高 EN(发送) -> 发送 -> 等 TC(发完，最多20ms) -> 拉低 EN(接收)。
  *         必须等 TC，否则最后一字节可能被截断。
  */
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
  * @brief  从环形缓冲取 n 个字节（每个字节最多等 to_ms）
  * @param  buf    接收缓冲
  * @param  n      需要取的字节数
  * @param  to_ms  每个字节的超时（毫秒）
  * @retval 实际取到的字节数
  * @note   等待时用 osDelay(1) 让出 CPU，给屏幕等低优先级任务运行。
  */
static int rs485_recv_n(uint8_t *buf, int n, uint32_t to_ms)
{
  int got = 0;   /* 已取字节数 */

  while (got < n)
  {
    uint32_t t0 = HAL_GetTick();
    while (rs485_avail() == 0)
    {
      if ((HAL_GetTick() - t0) >= to_ms) return got;    /* 等到超时 */
      osDelay(1);                                        /* 让出 CPU */
    }
    buf[got++] = s_rxbuf[s_rx_tail & RS485_RXMASK];     /* 从环形缓冲取一个 */
    s_rx_tail = (uint16_t)((s_rx_tail + 1) & RS485_RXMASK);
  }
  return got;
}

/**
  * @brief  清空 RX 残留（发送新请求前调用）
  * @param  无
  * @retval 无
  * @note   清错误标志 + 丢弃环形缓冲里所有残留，避免上一帧的尾巴污染本帧。
  */
static void rs485_flush_rx(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart5);            /* 清溢出等错误标志 */
  s_rx_tail = s_rx_head;                        /* 丢弃所有残留（读指针追上写指针） */
#if (RS485_GAP_MS > 0)
  osDelay(RS485_GAP_MS);                        /* 帧间静默（默认不延时） */
#endif
}

/**
  * @brief  按"已知长度"读一整帧应答（先对齐帧头，再按功能码定长）
  * @param  buf       接收缓冲
  * @param  maxlen    缓冲容量
  * @param  slave     期望的从机地址
  * @param  first_to  等第一个字节的最长时间（毫秒）
  * @retval 实际收到的总字节数；0 表示从机没应答
  * @note   第一个字节必须是 slave，否则跳过（引导杂波），最多跳 8 个字节。
  *         长度规则：异常帧 = 5 字节；0x03/0x04 应答 = 3 + 字节数 + 2；
  *         0x06/0x10 应答 = 8 字节。
  */
static int rs485_recv_resp(uint8_t *buf, uint16_t maxlen, uint8_t slave, uint32_t first_to)
{
  int got, skip;

  got = rs485_recv_n(buf, 1, first_to);        /* 等第一个字节 */
  if (got < 1) return 0;

  for (skip = 0; buf[0] != slave && skip < 8; skip++)
  {
    if (rs485_recv_n(buf, 1, 50) < 1) return 0;  /* 对齐帧头：跳过非本机的杂波字节 */
  }
  if (buf[0] != slave) return 1;

  if (rs485_recv_n(buf + 1, 1, 50) < 1) return 1;   /* 功能码 */

  if (buf[1] & 0x80)                                /* 异常应答：还需 3 字节 */
  {
    return 2 + rs485_recv_n(buf + 2, 3, 50);
  }

  if (buf[1] == 0x03 || buf[1] == 0x04)             /* 读应答：还需 1(字节数) + n + 2(CRC) */
  {
    int bc;                                          /* 字节数 */
    if (rs485_recv_n(buf + 2, 1, 50) < 1) return 2;
    bc = buf[2];
    if (bc > (int)maxlen - 5) bc = (int)maxlen - 5;  /* 防越界 */
    return 3 + rs485_recv_n(buf + 3, bc + 2, 50);
  }

  return 2 + rs485_recv_n(buf + 2, 6, 50);          /* 0x06/0x10 应答 = 8 字节 */
}

/**
  * @brief  把一段字节以 HEX 形式打印到调试串口 USART1（仅调试用）
  * @param  tag   前缀文字
  * @param  data  数据
  * @param  len   长度
  * @retval 无
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
  * @brief  计算 Modbus CRC16（多项式 0xA001，初值 0xFFFF）
  * @param  data  数据
  * @param  len   长度
  * @retval 16 位 CRC；发送时按"低字节在前"追加到帧尾
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

/**
  * @brief  初始化 RTU 模块：创建总线互斥锁 + 打开 UART5 中断并启动单字节接收
  * @param  无
  * @retval 无
  * @note   必须在创建任务前调用（MX_FREERTOS_Init 里）。
  */
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

/**
  * @brief  取中断累计收到的字节数（诊断用）
  * @param  无
  * @retval 累计字节数
  */
uint32_t mbrtu_rx_total(void) { return s_rx_total; }

/**
  * @brief  接收自愈：若 RXNE 中断被关了、或接收状态异常，就重新挂上（长时间运行防卡死）
  * @param  无
  * @retval 无
  * @note   每轮轮询调用一次。
  */
void mbrtu_rx_heal(void)
{
  if ((UART5->CR1 & USART_CR1_RXNEIE) == 0U || huart5.RxState != HAL_UART_STATE_BUSY_RX)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    (void)HAL_UART_Receive_IT(&huart5, &s_rxbyte, 1);
  }
}

/**
  * @brief  RTU 主站读寄存器（0x03 保持 / 0x04 输入）
  * @param  slave       从机地址
  * @param  func        功能码：0x03 或 0x04
  * @param  start       起始寄存器地址
  * @param  count       读取寄存器个数（1..125）
  * @param  out         输出缓冲（长度 >= count）
  * @param  timeout_ms  等应答首字节的超时（毫秒）
  * @retval 0=成功；-1=发送失败；-2=无/过短应答(超时)；-3=地址不符；-4=CRC 错；
  *         -5=从机异常应答；-6=功能码不符；-7=字节数不符
  * @note   全程持有 s_busMutex，保证与写事务互斥。
  */
int mbrtu_read(uint8_t slave, uint8_t func, uint16_t start, uint16_t count,
               uint16_t *out, uint32_t timeout_ms)
{
  uint8_t  req[8];      /* 请求帧：从机+功能码+起始地址+数量+CRC */
  uint8_t  resp[256];   /* 应答帧缓冲 */
  uint16_t crc;
  int      rxlen;       /* 实收应答字节数 */
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

/**
  * @brief  RTU 主站写单个保持寄存器（0x06）
  * @param  slave       从机地址
  * @param  reg         寄存器地址
  * @param  value       要写的值
  * @param  timeout_ms  等应答超时（毫秒）
  * @retval 0=成功；负值同 mbrtu_read（-1..-6）
  */
int mbrtu_write_single(uint8_t slave, uint16_t reg, uint16_t value, uint32_t timeout_ms)
{
  uint8_t  req[8];
  uint8_t  resp[16];
  uint16_t crc;
  int      rxlen;

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

  if (rxlen < 5) return -2;
  if (resp[0] != slave) return -3;
  crc = modbus_crc16(resp, (uint16_t)(rxlen - 2));
  if ((resp[rxlen - 2] != (uint8_t)(crc & 0xFF)) ||
      (resp[rxlen - 1] != (uint8_t)(crc >> 8))) return -4;
  if (resp[1] == (uint8_t)(0x06 | 0x80)) return -5;
  if (resp[1] != 0x06) return -6;
  return 0;
}

/**
  * @brief  RTU 主站写多个保持寄存器（0x10）
  * @param  slave       从机地址
  * @param  reg         起始寄存器地址
  * @param  count       寄存器个数（1..123）
  * @param  values      值数组（长度 >= count）
  * @param  timeout_ms  等应答超时（毫秒）
  * @retval 0=成功；负值同 mbrtu_read（-1..-6）；-8=参数非法
  */
int mbrtu_write_multiple(uint8_t slave, uint16_t reg, uint16_t count,
                         const uint16_t *values, uint32_t timeout_ms)
{
  uint8_t  req[260];    /* 最大：7 + 123*2 + 2 = 255 字节 */
  uint8_t  resp[16];
  uint16_t crc;
  int      rxlen;
  int      reqlen;      /* 请求总长度 */
  int      i;

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

  if (rxlen < 5) return -2;
  if (resp[0] != slave) return -3;
  crc = modbus_crc16(resp, (uint16_t)(rxlen - 2));
  if ((resp[rxlen - 2] != (uint8_t)(crc & 0xFF)) ||
      (resp[rxlen - 1] != (uint8_t)(crc >> 8))) return -4;
  if (resp[1] == (uint8_t)(0x10 | 0x80)) return -5;
  if (resp[1] != 0x10) return -6;
  return 0;
}
