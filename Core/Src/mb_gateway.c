/**
  ******************************************************************************
  * @file    mb_gateway.c
  * @brief   网关应用层：寄存器缓存 + 设备点表 + 轮询/写穿透/单元号路由
  * @note    上行(TCP)与下行(RTU)在此解耦：TCP 读缓存、写穿透；轮询任务填缓存。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "mb_gateway.h"
#include "mb_rtu.h"

/* ================= 缓存与状态 ================= */

/** 按从机号分区的寄存器缓存：g_slave_regs[slave-1][reg]（访问需持 regMutex） */
static uint16_t g_slave_regs[MB_MAX_SLAVE][MB_MAX_REGS];

/** 各从机在线标志（下标 1..MB_MAX_SLAVE），1=在线 */
static uint8_t g_slave_online[MB_MAX_SLAVE + 1];

/** 保护缓存分区的互斥锁 */
static osMutexId_t s_regMutex;

/** 轮询任务心跳时间戳（每轮刷新），供看门狗判断其是否卡死 */
static volatile uint32_t s_alive_ms = 0;

/* ================= 设备点表（增删设备只改这里，不用动代码） ================= */

/**
  * @brief  一条“轮询读任务”的描述（改表即可增删采集点）
  */
typedef struct
{
  uint8_t    slave;       /**< 从机地址 1..MB_MAX_SLAVE */
  uint8_t    func;        /**< 功能码：0x03 保持寄存器 / 0x04 输入寄存器 */
  uint16_t   reg;         /**< 从机寄存器起始地址 0..MB_MAX_REGS-1 */
  uint16_t   count;       /**< 读取寄存器个数（浮点要 >=2 且为偶数） */
  mb_dtype_t type;        /**< 数据类型 */
  uint8_t    online;      /**< 在线状态：1=在线 0=离线（运行时更新） */
  uint32_t   last_ok_ms;  /**< 最近一次采集成功的时刻 */
} mb_dev_t;

static mb_dev_t g_devs[] =
{
  /* 从机1：保持寄存器 地址0 起 2 个，按 U16 解码 -> g_slave_regs[0][0..1] */
  { .slave = 1, .func = 0x03, .reg = 0, .count = 2, .type = DT_U16 },

  /* 从机2：保持寄存器 地址0 起 2 个 = 1 个 float(高字在前) -> g_slave_regs[1][0..1] */
  { .slave = 2, .func = 0x03, .reg = 0, .count = 2, .type = DT_F32_ABCD },

  /* 继续加设备照上面写；reg + count 不要超过 MB_MAX_REGS */
};
#define DEV_COUNT  (sizeof(g_devs) / sizeof(g_devs[0]))

/* 轮询重试策略 */
#define RETRY_MAX       3    /**< 单次轮询最多尝试次数 */
#define RETRY_DELAY_MS  50   /**< 两次尝试之间的间隔（毫秒） */

/**
  * @brief  按数据类型把从机读回的寄存器写进该从机的缓存分区
  * @param  dev  设备描述
  * @param  raw  从机读回的原始寄存器数组（长度 >= dev->count）
  * @retval 无
  * @note   调用者必须已持有 regMutex。
  *         整数：每个寄存器 -> 一个单元（S16 符号扩展）。
  *         32 位：占两个寄存器，统一“高字在前”存入；是否浮点交给上位机。
  */
static void decode_into_regs(const mb_dev_t *dev, const uint16_t *raw)
{
  uint16_t *dst = &g_slave_regs[dev->slave - 1][dev->reg];
  uint16_t k;

  if (dev->type == DT_U16 || dev->type == DT_S16)
  {
    for (k = 0; k < dev->count; k++)
    {
      dst[k] = (dev->type == DT_S16) ? (uint16_t)(int16_t)raw[k] : raw[k];
    }
  }
  else /* 32 位：ABCD 原样，CDAB 交换两个字（只归一化字序，不解释浮点） */
  {
    for (k = 0; k + 1 < dev->count; k += 2)
    {
      uint16_t a = raw[k];
      uint16_t b = raw[k + 1];
      dst[k]     = (dev->type == DT_F32_ABCD) ? a : b;
      dst[k + 1] = (dev->type == DT_F32_ABCD) ? b : a;
    }
  }
}

/* ================= 对外接口 ================= */

void mbgw_init(void)
{
  s_regMutex = osMutexNew(NULL);
}

uint8_t mbgw_is_online(uint8_t slave)
{
  if (slave < 1 || slave > MB_MAX_SLAVE) return 0;
  return g_slave_online[slave];
}

void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out)
{
  uint16_t i;
  osMutexAcquire(s_regMutex, osWaitForever);
  for (i = 0; i < qty; i++)
  {
    out[i] = g_slave_regs[slave - 1][start + i];
  }
  osMutexRelease(s_regMutex);
}

int mbgw_write_single(uint8_t slave, uint16_t addr, uint16_t val)
{
  if (mbrtu_write_single(slave, addr, val, 200) != 0)
  {
    return -1;                                /* 转发失败 */
  }
  osMutexAcquire(s_regMutex, osWaitForever);
  g_slave_regs[slave - 1][addr] = val;
  osMutexRelease(s_regMutex);
  g_slave_online[slave] = 1;
  return 0;
}

int mbgw_write_multiple(uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals)
{
  uint16_t j;
  if (mbrtu_write_multiple(slave, start, qty, vals, 200) != 0)
  {
    return -1;
  }
  osMutexAcquire(s_regMutex, osWaitForever);
  for (j = 0; j < qty; j++)
  {
    g_slave_regs[slave - 1][start + j] = vals[j];
  }
  osMutexRelease(s_regMutex);
  g_slave_online[slave] = 1;
  return 0;
}

uint32_t mbgw_last_alive_ms(void)
{
  return s_alive_ms;
}

/**
  * @brief  RS485 / Modbus RTU 主站轮询任务
  * @param  argument  未使用
  * @retval 无（任务永不返回）
  * @note   每秒把 g_devs[] 里每台设备读一遍：
  *         读成功 -> 按类型写入该从机缓存分区并标在线；
  *         读失败 -> 标离线并打印错误码。
  */
void mbgw_poll_task(void *argument)
{
  uint16_t tmp[16] = {0};
  uint32_t i;
  int r;

  (void)argument;

  HAL_UART_Transmit(&huart1, (uint8_t*)"[RTU] task started\r\n",
                    (uint16_t)strlen("[RTU] task started\r\n"), HAL_MAX_DELAY);

  for (;;)
  {
    s_alive_ms = HAL_GetTick();                /* 心跳：轮询任务还活着 */

    for (i = 0; i < DEV_COUNT; i++)
    {
      int attempt;

      if (g_devs[i].slave < 1 || g_devs[i].slave > MB_MAX_SLAVE ||
          g_devs[i].count > 16 ||
          (uint32_t)g_devs[i].reg + g_devs[i].count > MB_MAX_REGS)
      {
        continue;                                /* 防越界，跳过非法配置 */
      }

      /* 带重试地读：最多尝试 RETRY_MAX 次 */
      r = -1;
      for (attempt = 0; attempt < RETRY_MAX; attempt++)
      {
        r = mbrtu_read(g_devs[i].slave, g_devs[i].func,
                       g_devs[i].reg, g_devs[i].count, tmp, 200);
        if (r == 0)
        {
          break;                                 /* 成功，不再重试 */
        }
        osDelay(RETRY_DELAY_MS);                 /* 失败：歇一下再试 */
      }

      if (r == 0)
      {
        osMutexAcquire(s_regMutex, osWaitForever);
        decode_into_regs(&g_devs[i], tmp);
        osMutexRelease(s_regMutex);

        g_devs[i].last_ok_ms = HAL_GetTick();
        g_slave_online[g_devs[i].slave] = 1;

        if (!g_devs[i].online)
        {
          g_devs[i].online = 1;
          HAL_UART_Transmit(&huart1, (uint8_t*)"[RTU] slave back ONLINE\r\n",
                            (uint16_t)strlen("[RTU] slave back ONLINE\r\n"), HAL_MAX_DELAY);
        }

        char b[96];
        int n = snprintf(b, sizeof(b), "[RTU] slave=%u reg=%u ok: %u,%u\r\n",
                         g_devs[i].slave, g_devs[i].reg,
                         (unsigned)tmp[0], (unsigned)tmp[1]);
        HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
      }
      else
      {
        if (g_devs[i].online)
        {
          g_devs[i].online = 0;
          g_slave_online[g_devs[i].slave] = 0;
          char b[80];
          int n = snprintf(b, sizeof(b),
                           "[RTU] slave=%u OFFLINE (err=%d, retried %d)\r\n",
                           g_devs[i].slave, r, RETRY_MAX);
          HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
        }
      }
    }

    osDelay(1000);
  }
}
