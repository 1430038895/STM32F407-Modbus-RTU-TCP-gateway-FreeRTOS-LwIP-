/**
  ******************************************************************************
  * @file    mb_gateway.c
  * @brief   网关应用层：全量寄存器镜像 + 点表 + 轮询更新 + 出口
  * @note    B 方案：
  *            ① 保留 g_slave_regs[从机][寄存器] 全量镜像，轮询刷新（TCP 透传照旧）；
  *            ② 描述表逐条列出（含"参与组合、不单独输出"的寄存器）；
  *            ③ 对外：mbgw_*（按从机+地址）与 dp_*（按点）。
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdio.h>
#include "mb_gateway.h"
#include "mb_rtu.h"

/* ================= 导出服务参数 ================= */

#define DUMP_TCP_PORT   5000     /**< Python 连这个端口读全部点 */
#ifndef DUMP_BUF_SIZE
#define DUMP_BUF_SIZE   8192     /**< 单设备文本缓冲（几百字节足够） */
#endif

/* ================= 描述表类型 ================= */

typedef struct { uint8_t start, count; uint16_t period_ms; } seg_def_t;

typedef struct pt_def {
    uint8_t     seg;      /**< 属于哪段（本从机 segs[] 的下标） */
    uint8_t     offset;   /**< 段内偏移（寄存器） */
    uint8_t     kind;     /**< 原始类型 */
    uint8_t     dec;      /**< 呈现小数位（T_U32_DEC 时忽略） */
    uint8_t     hide;     /**< 1 = 参与组合计算，不单独输出 */
    const char *name;     /**< 点名 */
} pt_def_t;

typedef struct {
    uint8_t          addr;    /**< 从机地址 */
    const char      *name;    /**< 设备名 */
    uint8_t          seg_num; const seg_def_t *segs;
    uint8_t          pt_num;  const pt_def_t  *pts;
} slave_def_t;

/* ================= 运行期点数据（由描述表生成；只生成一次，之后只操作它） ================= */

typedef struct {
    uint16_t        r[3];    /* 原始寄存器：U16 用 r[0]；32位用 r[0]r[1]；U32_DEC 用 r[0..2] */
    uint8_t         q;       /* 质量：0=没读过 1=好 2=通信失败 */
    uint32_t        ts;      /* 最后一次成功的时间戳(ms) */
    const pt_def_t *def;     /* 指向描述表那一行（类型/小数位/名字都从这里取） */
    uint8_t         dev;     /* 属于哪台从机（slv 下标） */
    uint8_t         changed; /* 本次轮询该点值是否变化（给屏幕/通知层用） */
} point_t;

static point_t  pt[MAX_PT];  /* 所有点（连续数组，按点号 0..pt_used-1 索引） */
static uint16_t pt_used;     /* 实际生成了几个点（=78） */

/* ================= 镜像 + 状态 ================= */

/** 按从机分区的寄存器镜像：g_slave_regs[从机号-1][寄存器地址]（访问需持 s_regMutex） */
static uint16_t g_slave_regs[MB_MAX_SLAVE][MB_MAX_REGS];
static uint8_t  g_slave_online[MB_MAX_SLAVE + 1];
static osMutexId_t s_regMutex;
static volatile uint32_t s_alive_ms = 0;
static volatile uint32_t s_cycle_ms = 0;   /* 最近一轮完整轮询耗时(ms) */

typedef struct { uint8_t addr; const slave_def_t *def; uint16_t pt_base; uint8_t online; uint8_t fail_cnt;
                 uint16_t base; uint16_t span; } gw_slave_t;
static gw_slave_t slv[MAX_SLAVES];

/* ================= 变更通知（给屏幕：值一变就通知，屏幕不用轮询） ================= */
static osEventFlagsId_t s_chgEvt;                    /* 事件标志句柄 */
#define GW_CHG_BIT      0x0001u                      /* "有点变了"这个位 */
static uint8_t  s_dirty[(MAX_PT + 7) / 8];           /* 脏位图：哪几个点变了 */

/** @brief 标记某个点的值变了：置脏位 + 置事件（线程安全） */
static void chg_mark(uint16_t idx)
{
  taskENTER_CRITICAL();
  s_dirty[idx >> 3] |= (uint8_t)(1u << (idx & 7));
  taskEXIT_CRITICAL();
  (void)osEventFlagsSet(s_chgEvt, GW_CHG_BIT);
}

/* ================= 设备描述表（日常只改这里） ================= */
/* 字段：{段, 段内偏移, 类型, 小数位, 是否隐藏(参与组合不单独输出), 点名} */

/* ---------------- 设备1：COM10-001 流量计（14 条） ---------------- */
static const seg_def_t SEG_D1[] = { {0x00, 8, 1000}, {0x0D, 6, 1000} };
static const pt_def_t  PT_D1[]  = {
  { 0, 0, T_U16,     0, 0, "baud"              },   /* 0  波特率 */
  { 0, 1, T_U16,     0, 0, "station"           },   /* 1  站点号 */
  { 0, 2, T_U16,     0, 0, "decimals"          },   /* 2  小数位 */
  { 0, 3, T_U16,     2, 0, "K"                 },   /* 3  K (/100) */
  { 0, 4, T_U32_DEC, 0, 0, "flow_total"        },   /* 4~6 累积流量(组合) */
  { 0, 5, T_U16,     0, 1, "flow_total_lo"     },   /* 5  低字(隐藏) */
  { 0, 6, T_U16,     0, 1, "flow_total_dec"    },   /* 6  小数位(隐藏) */
  { 0, 7, T_U16,     0, 0, "flow_total_pulse"  },   /* 7  脉冲 */
  { 1, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 13~15 临时累积(组合) */
  { 1, 1, T_U16,     0, 1, "flow_temp_lo"      },   /* 14 低字(隐藏) */
  { 1, 2, T_U16,     0, 1, "flow_temp_dec"     },   /* 15 小数位(隐藏) */
  { 1, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 16 脉冲 */
  { 1, 4, T_U16,     2, 0, "flow_instant"      },   /* 17 瞬时流量 (/100) */
  { 1, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 18 瞬时脉冲 */
};

/* ---------------- 设备2：COM10-002（16 条） ---------------- */
static const seg_def_t SEG_D2[] = { {0x00, 8, 1000}, {0x09, 2, 1000}, {0x0D, 6, 1000} };
static const pt_def_t  PT_D2[]  = {
  { 0, 0, T_U16,     0, 0, "baud"              },
  { 0, 1, T_U16,     0, 0, "station"           },
  { 0, 2, T_U16,     0, 0, "decimals"          },
  { 0, 3, T_U16,     2, 0, "K"                 },
  { 0, 4, T_U32_DEC, 0, 0, "flow_total"        },
  { 0, 5, T_U16,     0, 1, "flow_total_lo"     },
  { 0, 6, T_U16,     0, 1, "flow_total_dec"    },
  { 0, 7, T_U16,     0, 0, "flow_total_pulse"  },
  { 1, 0, T_U16,     2, 0, "ma_4ma"            },   /* 9  4mA 下限 (/100) */
  { 1, 1, T_U16,     2, 0, "ma_20ma"           },   /* 10 20mA 上限 (/100) */
  { 2, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 13~15 */
  { 2, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 2, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 2, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 16 */
  { 2, 4, T_U16,     2, 0, "flow_instant"      },   /* 17 */
  { 2, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 18 */
};

/* ---------------- 设备3：COM10-003 气象站（9 条，全 F32） ---------------- */
static const seg_def_t SEG_D3[] = { {0x00, 18, 1000} };
static const pt_def_t  PT_D3[]  = {
  { 0, 0,  T_F32, 0, 0, "wind_speed" },
  { 0, 2,  T_F32, 0, 0, "wind_dir"   },
  { 0, 4,  T_F32, 0, 0, "air_temp"   },
  { 0, 6,  T_F32, 0, 0, "humidity"   },
  { 0, 8,  T_F32, 0, 0, "pressure"   },
  { 0, 10, T_F32, 0, 0, "rain_min"   },
  { 0, 12, T_F32, 0, 0, "rain_hour"  },
  { 0, 14, T_F32, 0, 0, "rain_day"   },
  { 0, 16, T_F32, 0, 0, "rain_total" },
};

/* ---------------- 设备4/6：COM10-004/006 空气环境（10 条） ---------------- */
static const seg_def_t SEG_D4[] = { {0x01, 2, 1000}, {0x0A, 6, 1000}, {0x1A, 2, 1000} };
static const pt_def_t  PT_D4[]  = {
  { 0, 0, T_U16, 0, 0, "dev_addr" },   /* 1 */
  { 0, 1, T_U16, 0, 0, "baud"     },   /* 2 */
  { 1, 0, T_I16, 1, 0, "temp"     },   /* 10 (/10, 有符号) */
  { 1, 1, T_I16, 1, 0, "humidity" },   /* 11 (/10, 有符号) */
  { 1, 2, T_U16, 0, 0, "pm1"      },   /* 12 */
  { 1, 3, T_U16, 0, 0, "pm25"     },   /* 13 */
  { 1, 4, T_U16, 0, 0, "pm10"     },   /* 14 */
  { 1, 5, T_U16, 0, 0, "co2"      },   /* 15 */
  { 2, 0, T_U16, 2, 0, "hcho"     },   /* 26 (/100) */
  { 2, 1, T_U16, 1, 0, "voc"      },   /* 27 (/10) */
};

/* ---------------- 设备5：COM10-005 流量+压力（17 条） ---------------- */
static const seg_def_t SEG_D5[] = { {0x00, 8, 1000}, {0x0B, 2, 1000}, {0x0D, 6, 1000}, {0x14, 1, 1000} };
static const pt_def_t  PT_D5[]  = {
  { 0, 0, T_U16,     0, 0, "baud"              },
  { 0, 1, T_U16,     0, 0, "station"           },
  { 0, 2, T_U16,     0, 0, "decimals"          },
  { 0, 3, T_U16,     2, 0, "K"                 },
  { 0, 4, T_U32_DEC, 0, 0, "flow_total"        },
  { 0, 5, T_U16,     0, 1, "flow_total_lo"     },
  { 0, 6, T_U16,     0, 1, "flow_total_dec"    },
  { 0, 7, T_U16,     0, 0, "flow_total_pulse"  },
  { 1, 0, T_U16,     0, 0, "press_A"           },   /* 11 */
  { 1, 1, T_U16,     0, 0, "press_B"           },   /* 12 */
  { 2, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 13~15 */
  { 2, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 2, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 2, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 16 */
  { 2, 4, T_U16,     2, 0, "flow_instant"      },   /* 17 */
  { 2, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 18 */
  { 3, 0, T_U16,     2, 0, "pressure_kpa"      },   /* 20 (/100) */
};

/* ---------------- 设备7：COM10-007 流量+NTC（16 条） ---------------- */
static const seg_def_t SEG_D7[] = { {0x00, 9, 1000}, {0x0D, 7, 1000} };
static const pt_def_t  PT_D7[]  = {
  { 0, 0, T_U16,     0, 0, "baud"              },
  { 0, 1, T_U16,     0, 0, "station"           },
  { 0, 2, T_U16,     0, 0, "decimals"          },
  { 0, 3, T_U16,     2, 0, "K"                 },
  { 0, 4, T_U32_DEC, 0, 0, "flow_total"        },
  { 0, 5, T_U16,     0, 1, "flow_total_lo"     },
  { 0, 6, T_U16,     0, 1, "flow_total_dec"    },
  { 0, 7, T_U16,     0, 0, "flow_total_pulse"  },
  { 0, 8, T_U16,     0, 0, "ntc_res"           },   /* 8  NTC 电阻 */
  { 1, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 13~15 */
  { 1, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 1, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 1, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 16 */
  { 1, 4, T_U16,     2, 0, "flow_instant"      },   /* 17 */
  { 1, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 18 */
  { 1, 6, T_U16,     1, 0, "temp_c"            },   /* 19 温度 (/10) */
};

/* ---------------- 设备8：COM10-008 简易温湿度（2 条） ---------------- */
static const seg_def_t SEG_D8[] = { {0x00, 2, 1000} };
static const pt_def_t  PT_D8[]  = {
  { 0, 0, T_I16, 1, 0, "humidity" },   /* 0 (/10, 有符号) */
  { 0, 1, T_U16, 1, 0, "temp"     },   /* 1 (/10) */
};

#define PTS(x)  ((uint8_t)(sizeof(x)/sizeof((x)[0])))

static const slave_def_t SLAVES[] = {
  { 1, "COM10-001", 2, SEG_D1, PTS(PT_D1), PT_D1 },
  { 2, "COM10-002", 3, SEG_D2, PTS(PT_D2), PT_D2 },
  { 3, "COM10-003", 1, SEG_D3, PTS(PT_D3), PT_D3 },
  { 4, "COM10-004", 3, SEG_D4, PTS(PT_D4), PT_D4 },
  { 5, "COM10-005", 4, SEG_D5, PTS(PT_D5), PT_D5 },
  { 6, "COM10-006", 3, SEG_D4, PTS(PT_D4), PT_D4 },   /* 同 004 */
  { 7, "COM10-007", 2, SEG_D7, PTS(PT_D7), PT_D7 },
  { 8, "COM10-008", 1, SEG_D8, PTS(PT_D8), PT_D8 },
};
#define SLAVE_NUM  (sizeof(SLAVES)/sizeof(SLAVES[0]))

/* 轮询重试策略 */
#define RETRY_MAX       2
#define RETRY_DELAY_MS  10

/* 串口调试打印（屏幕已能显示，默认关掉以减小轮询耗时） */
#define GW_DEBUG_SERIAL   0

/* 调试：只轮询这一台从机（0 = 轮询全部）。改这里可单台排查。 */
#define DEBUG_ONLY_SLAVE   0

/* 轮询节奏（毫秒）：为在 200ms 内完成一轮，都设为 0 */
#define POLL_SLAVE_GAP_MS   0
#define POLL_CYCLE_GAP_MS   5

/* ================= 初始化 ================= */

static bool cache_init(void)
{
  uint16_t pcur = 0;

  for (uint8_t s = 0; s < SLAVE_NUM; s++)
  {
    const slave_def_t *d = &SLAVES[s];

    for (uint8_t k = 0; k < s; k++)
      if (slv[k].addr == d->addr) return false;
    if (d->addr < 1 || d->addr > MB_MAX_SLAVE) return false;

    slv[s].addr    = d->addr;
    slv[s].def     = d;
    slv[s].pt_base = pcur;
    slv[s].online  = 0;
    slv[s].fail_cnt = 0;

    /* 合并段：算出这台设备要读的"最小地址 ~ 最大地址"一整段 */
    {
      uint16_t b = d->segs[0].start;
      uint16_t e = (uint16_t)(d->segs[0].start + d->segs[0].count - 1);
      for (uint8_t t = 1; t < d->seg_num; t++)
      {
        uint16_t s0 = d->segs[t].start;
        uint16_t s1 = (uint16_t)(d->segs[t].start + d->segs[t].count - 1);
        if (s0 < b) b = s0;
        if (s1 > e) e = s1;
      }
      if ((uint16_t)(e + 1) > MB_MAX_REGS) return false;   /* 合并后超出缓存 */
      slv[s].base = b;
      slv[s].span = (uint16_t)(e - b + 1);
    }

    for (uint8_t i = 0; i < d->pt_num; i++)
    {
      const pt_def_t  *p = &d->pts[i];
      const seg_def_t *g = &d->segs[p->seg];
      uint8_t regs;

      if (p->hide) continue;                       /* 组合用的寄存器不单独输出 */

      regs = (p->kind == T_U32 || p->kind == T_F32) ? 2 :
             (p->kind == T_U32_DEC) ? 3 : 1;
      if ((uint16_t)g->start + g->count > MB_MAX_REGS) return false;
      if ((uint16_t)p->offset + regs > g->count) return false;
      if (pcur >= MAX_PT) return false;

      pt[pcur].r[0] = pt[pcur].r[1] = pt[pcur].r[2] = 0;
      pt[pcur].def  = p;
      pt[pcur].dev  = s;
      pt[pcur].q    = 0;
      pt[pcur].ts   = 0;
      pt[pcur].changed = 0;
      pcur++;
    }
  }

  pt_used = pcur;
  return (pt_used <= MAX_PT);
}

/* ================= 解码一台设备的原始寄存器到点 ================= */

/* 调用者必须已持有 s_regMutex
 * raw = 从 base 地址开始一次性读回的整段寄存器（含中间的空洞） */
static void decode_dev(uint8_t s, const uint16_t *raw, uint16_t base)
{
  const slave_def_t *d = slv[s].def;
  uint32_t now = HAL_GetTick();

  for (uint16_t i = 0; i < pt_used; i++)
  {
    const pt_def_t *p;
    uint16_t a, off, o0, o1, o2;

    if (pt[i].dev != s) continue;
    p = pt[i].def;
    a = (uint16_t)(d->segs[p->seg].start + p->offset);   /* 该点的绝对地址 */
    off = (uint16_t)(a - base);

    o0 = pt[i].r[0]; o1 = pt[i].r[1]; o2 = pt[i].r[2];

    switch (p->kind)
    {
      case T_U16:
      case T_I16:
        pt[i].r[0] = raw[off];
        break;
      case T_U32:
      case T_F32:
        pt[i].r[0] = raw[off];
        pt[i].r[1] = raw[off + 1];
        break;
      case T_U32_DEC:
        pt[i].r[0] = raw[off];
        pt[i].r[1] = raw[off + 1];
        pt[i].r[2] = raw[off + 2];
        break;
      default: break;
    }

    if (o0 != pt[i].r[0] || o1 != pt[i].r[1] || o2 != pt[i].r[2])
    {
      pt[i].changed = 1;
      chg_mark((uint16_t)i);
    }

    pt[i].q  = 1;
    pt[i].ts = now;
  }
}

/* 标记一台设备的所有点为通信失败（值保持旧值，累积量不清零） */
static void mark_dev_bad(uint8_t s)
{
  for (uint16_t i = 0; i < pt_used; i++)
  {
    if (pt[i].dev == s)
      pt[i].q = 2;
  }
}

/* ================= 对外：生命周期 ================= */

void mbgw_init(void)
{
  s_regMutex = osMutexNew(NULL);
  s_chgEvt   = osEventFlagsNew(NULL);
  (void)cache_init();
}

uint8_t mbgw_is_online(uint8_t slave)
{
  if (slave < 1 || slave > MB_MAX_SLAVE) return 0;
  return g_slave_online[slave];
}

uint32_t mbgw_last_alive_ms(void)
{
  return s_alive_ms;
}

/* ================= 对外：变更通知（给屏幕） ================= */

/* 阻塞等待“有点变了”。返回事件标志（含 GW_CHG_BIT）或错误码（含 osErrorTimeout） */
uint32_t gw_wait_changed(uint32_t timeout_ms)
{
  return osEventFlagsWait(s_chgEvt, GW_CHG_BIT, osFlagsWaitAny, timeout_ms);
}

/* 取走当前所有“变了的点号”到 out（最多 max 个），并清零脏位。返回个数 */
uint16_t gw_take_changed(uint16_t *out, uint16_t max)
{
  uint16_t n = 0;
  taskENTER_CRITICAL();
  for (uint16_t i = 0; i < pt_used && n < max; i++)
  {
    if (s_dirty[i >> 3] & (uint8_t)(1u << (i & 7)))
    {
      s_dirty[i >> 3] &= (uint8_t)~(1u << (i & 7));
      out[n++] = i;
    }
  }
  taskEXIT_CRITICAL();
  return n;
}

/* ================= 对外：按 从机+寄存器（TCP 透传） ================= */

void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out)
{
  uint16_t i;
  if (slave < 1 || slave > MB_MAX_SLAVE) { for (i = 0; i < qty; i++) out[i] = 0; return; }

  osMutexAcquire(s_regMutex, osWaitForever);
  for (i = 0; i < qty; i++)
    out[i] = g_slave_regs[slave - 1][start + i];
  osMutexRelease(s_regMutex);
}

int mbgw_write_single(uint8_t slave, uint16_t addr, uint16_t val)
{
  if (slave < 1 || slave > MB_MAX_SLAVE || addr >= MB_MAX_REGS) return -1;
  if (mbrtu_write_single(slave, addr, val, 200) != 0) return -1;

  osMutexAcquire(s_regMutex, osWaitForever);
  g_slave_regs[slave - 1][addr] = val;
  osMutexRelease(s_regMutex);
  g_slave_online[slave] = 1;
  return 0;
}

int mbgw_write_multiple(uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals)
{
  uint16_t j;
  if (slave < 1 || slave > MB_MAX_SLAVE || (uint32_t)start + qty > MB_MAX_REGS) return -1;
  if (mbrtu_write_multiple(slave, start, qty, vals, 200) != 0) return -1;

  osMutexAcquire(s_regMutex, osWaitForever);
  for (j = 0; j < qty; j++)
    g_slave_regs[slave - 1][start + j] = vals[j];
  osMutexRelease(s_regMutex);
  g_slave_online[slave] = 1;
  return 0;
}

/* ================= 对外：按点 ================= */

int dp_find(uint8_t addr, const char *name)
{
  for (uint16_t i = 0; i < pt_used; i++)
  {
    if (slv[pt[i].dev].addr != addr) continue;
    if (strcmp(name, pt[i].def->name) == 0) return (int)i;
  }
  return -1;
}

uint16_t dp_count(void)           { return pt_used; }
uint8_t  dp_quality(uint16_t idx) { return (idx < pt_used) ? pt[idx].q : 2; }
uint32_t dp_age_ms(uint16_t idx)  { return (idx < pt_used) ? (HAL_GetTick() - pt[idx].ts) : 0xFFFFFFFF; }
uint8_t  dp_kind(uint16_t idx)    { return (idx < pt_used) ? pt[idx].def->kind : 0xFF; }

const char *dp_name(uint16_t idx)
{
  return (idx < pt_used) ? pt[idx].def->name : "";
}
const char *dp_dev_name(uint16_t idx)
{
  return (idx < pt_used) ? slv[pt[idx].dev].def->name : "";
}
uint8_t dp_dev_addr(uint16_t idx)
{
  return (idx < pt_used) ? slv[pt[idx].dev].addr : 0;
}

/* ---- 按从机序号访问（给屏幕用） ---- */
uint8_t     gw_slave_count(void)          { return (uint8_t)SLAVE_NUM; }
const char *gw_slave_name(uint8_t i)      { return (i < SLAVE_NUM) ? slv[i].def->name : ""; }
uint8_t     gw_slave_addr(uint8_t i)      { return (i < SLAVE_NUM) ? slv[i].addr : 0; }
uint8_t     gw_slave_online(uint8_t i)    { return (i < SLAVE_NUM) ? slv[i].online : 0; }
uint8_t     gw_slave_fail(uint8_t i)      { return (i < SLAVE_NUM) ? slv[i].fail_cnt : 0; }
uint32_t    gw_cycle_ms(void)             { return s_cycle_ms; }

/* ================= F32 解码（不用 float：IEEE754 -> 定点整数） =================
 *
 * 一个 32 位浮点数的二进制布局：  [符号1][指数8][尾数23]
 *   值 = (-1)^sign * 2^(exp-127) * (1 + 尾数/2^23)
 * 这里把它换算成“定点整数”：返回 = 真值 × 10^decimals，方便整数显示。
 * 例：raw=40 48 F5 C3 (即 3.14)，decimals=3 -> 返回 3140。
 */
static int64_t f32_to_fixed(const uint8_t *raw, uint8_t decimals)
{
  uint32_t bits = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16) |
                  ((uint32_t)raw[2] << 8)  | (uint32_t)raw[3];
  uint32_t sign = bits >> 31;                 /* 符号位 */
  int32_t  exp  = (int32_t)((bits >> 23) & 0xFF) - 127;   /* 真实指数 */
  uint32_t frac = bits & 0x7FFFFFu;           /* 尾数（23 位） */
  int64_t  m, numer, result;
  int32_t  shift;

  if (exp == 128) return 0;                   /* 无穷/NaN -> 0 */
  if (exp == -127 && frac == 0) return 0;     /* ±0 */

  if (exp == -127) { m = frac;          shift = -149; }    /* 非规格化小数 */
  else             { m = (1u << 23) | frac; shift = exp - 23; }

  numer = m;                                  /* 真值 = numer * 2^shift */
  for (uint8_t i = 0; i < decimals; i++) numer *= 10;      /* 再 ×10^decimals */

  if (shift >= 0)
  {
    result = numer << shift;                  /* 放大 */
  }
  else
  {
    int32_t sh = -shift;                      /* 缩小（带四舍五入） */
    result = (sh > 62) ? 0 : (numer + ((int64_t)1 << (sh - 1))) >> sh;
  }
  return sign ? -result : result;
}

bool dp_read(uint16_t idx, dp_out_t *out)
{
  if (idx >= pt_used) return false;

  memset(out, 0, sizeof(*out));

  switch (pt[idx].def->kind)
  {
    case T_U16:
      out->scaled = pt[idx].r[0];
      out->decimals = pt[idx].def->dec;
      break;
    case T_I16:
      out->scaled = (int16_t)pt[idx].r[0];
      out->decimals = pt[idx].def->dec;
      break;
    case T_U32:
      out->scaled = ((uint32_t)pt[idx].r[0] << 16) | pt[idx].r[1];
      out->decimals = pt[idx].def->dec;
      break;
    case T_U32_DEC:
    {
      /* 高字/低字 = 整数部分；第 3 个寄存器 = 小数部分，单位 0.01（25 -> 0.25） */
      uint32_t ip = ((uint32_t)pt[idx].r[0] << 16) | pt[idx].r[1];
      out->scaled   = (int64_t)ip * 100 + pt[idx].r[2];
      out->decimals = 2;
      break;
    }
    case T_F32:
      out->is_f32 = 1;
      out->raw[0] = (uint8_t)(pt[idx].r[0] >> 8);
      out->raw[1] = (uint8_t)(pt[idx].r[0]);
      out->raw[2] = (uint8_t)(pt[idx].r[1] >> 8);
      out->raw[3] = (uint8_t)(pt[idx].r[1]);
      break;
    default:
      return false;
  }
  return true;
}

/* 把任意点取成“十进制定点值”：真值 = *value / 10^decimals（F32 也适用）
 * —— 屏幕/LVGL 以后直接调它（例如 decimals=2 就拿到 ×100 的整数）。 */
bool dp_read_num(uint16_t idx, int64_t *value, uint8_t decimals)
{
  dp_out_t o;

  if (!dp_read(idx, &o)) return false;

  if (o.is_f32)
  {
    *value = f32_to_fixed(o.raw, decimals);   /* F32：解码成定点 */
    return true;
  }

  /* 非 F32：把 scaled/decimals 调整到目标小数位 */
  if (decimals >= o.decimals)
  {
    for (uint8_t i = o.decimals; i < decimals; i++) o.scaled *= 10;
  }
  else
  {
    for (uint8_t i = decimals; i < o.decimals; i++) o.scaled /= 10;
  }
  *value = o.scaled;
  return true;
}

/* 把“定点整数 + 小数位”格式化成字符串（内部用） */
static int fmt_fixed(int64_t s, uint8_t decimals, char *buf)
{
  uint64_t a = (s < 0) ? (uint64_t)(-s) : (uint64_t)s;
  uint64_t p = 1;
  for (uint8_t i = 0; i < decimals; i++) p *= 10;
  uint64_t ip = a / p;
  uint64_t fp = a % p;

  char ib[24]; int ii = 0;
  do { ib[ii++] = (char)('0' + (ip % 10)); ip /= 10; } while (ip);

  int m = 0;
  if (s < 0) buf[m++] = '-';
  while (ii) buf[m++] = ib[--ii];

  if (decimals)
  {
    char fb[12];
    for (int i = decimals - 1; i >= 0; i--) { fb[i] = (char)('0' + (fp % 10)); fp /= 10; }
    buf[m++] = '.';
    for (uint8_t i = 0; i < decimals; i++) buf[m++] = fb[i];
  }
  buf[m] = 0;
  return m;
}

int dp_format(const dp_out_t *v, char *buf)
{
  if (v->is_f32)
  {
    return sprintf(buf, "F32:%02X%02X%02X%02X",
                   v->raw[0], v->raw[1], v->raw[2], v->raw[3]);
  }
  return fmt_fixed(v->scaled, v->decimals, buf);
}

/* 把任意点直接格式化成十进制字符串（F32 也换算成小数），给屏幕/调试用 */
int dp_format_num(uint16_t idx, char *buf, uint8_t decimals)
{
  int64_t v;
  if (!dp_read_num(idx, &v, decimals)) { buf[0] = 0; return 0; }
  return fmt_fixed(v, decimals, buf);
}

/* 把一个设备的点写进 buf，返回长度 */
static int cache_dump_dev(char *buf, int cap, uint8_t dev)
{
  int n = 0;

  for (uint16_t i = 0; i < pt_used; i++)
  {
    dp_out_t o;
    char val[40];
    int w;

    if (pt[i].dev != dev) continue;
    if (!dp_read(i, &o)) continue;
    dp_format(&o, val);

    {
      const pt_def_t *p = pt[i].def;
      uint16_t reg = (uint16_t)(slv[pt[i].dev].def->segs[p->seg].start + p->offset);
      w = snprintf(buf + n, (cap - n > 0) ? (size_t)(cap - n) : 0,
                   "%u,%s,%u,%s,%s,q=%u,age=%lu,reg=%u,k=%u,d=%u\r\n",
                   (unsigned)i, dp_dev_name(i), (unsigned)dp_dev_addr(i),
                   dp_name(i), val,
                   (unsigned)dp_quality(i), (unsigned long)dp_age_ms(i),
                   (unsigned)reg, (unsigned)p->kind, (unsigned)p->dec);
    }
    if (w <= 0) break;
    n += w;
    if (n >= cap - 1) break;
  }
  if (cap > 0) buf[(n < cap) ? n : cap - 1] = 0;
  return n;
}

int cache_dump(char *buf, int cap)
{
  int n = 0;
  for (uint16_t i = 0; i < pt_used; i++)
  {
    dp_out_t o;
    char val[40];
    int w;
    if (!dp_read(i, &o)) continue;
    dp_format(&o, val);
    w = snprintf(buf + n, (cap - n > 0) ? (size_t)(cap - n) : 0,
                 "%u,%s,%u,%s,%s,q=%u,age=%lu\r\n",
                 (unsigned)i, dp_dev_name(i), (unsigned)dp_dev_addr(i),
                 dp_name(i), val,
                 (unsigned)dp_quality(i), (unsigned long)dp_age_ms(i));
    if (w <= 0) break;
    n += w;
    if (n >= cap) { n = cap - 1; break; }
  }
  if (cap > 0) buf[(n < cap) ? n : cap - 1] = 0;
  return n;
}

/* ================= 调试：把一台从机的缓存打到串口 ================= */

#if GW_DEBUG_SERIAL
static void gw_print_slave(uint8_t s)
{
  char line[100];
  int n;

  n = snprintf(line, sizeof(line), "\r\n[%s] addr=%u online=%u\r\n",
               slv[s].def->name, (unsigned)slv[s].addr, (unsigned)slv[s].online);
  HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, HAL_MAX_DELAY);

  for (uint16_t i = 0; i < pt_used; i++)
  {
    char val[40];
    if (pt[i].dev != s) continue;
    if (!dp_format_num(i, val, 3)) continue;   /* F32 也换算成十进制 */
    n = snprintf(line, sizeof(line), "   %s = %s  q=%u age=%lu\r\n",
                 pt[i].def->name, val, (unsigned)pt[i].q, (unsigned long)dp_age_ms(i));
    HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)n, HAL_MAX_DELAY);
  }
}

#endif  /* GW_DEBUG_SERIAL */

/* ================= 轮询任务（全量镜像 + 点，一起刷新） ================= */

void mbgw_poll_task(void *argument)
{
  uint16_t tmp[MB_MAX_REGS];
  (void)argument;

  HAL_UART_Transmit(&huart1, (uint8_t*)"[RTU] poll task started\r\n",
                    (uint16_t)strlen("[RTU] poll task started\r\n"), HAL_MAX_DELAY);

  for (;;)
  {
    uint32_t t0 = HAL_GetTick();
    s_alive_ms = t0;
    mbrtu_rx_heal();          /* 每轮自愈一次 UART 接收 */

#if GW_DEBUG_SERIAL
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n===== POLL START =====\r\n",
                      (uint16_t)strlen("\r\n===== POLL START =====\r\n"), HAL_MAX_DELAY);
    {
      char b[48];
      int n;
      n = snprintf(b, sizeof(b), "[RTU] rx_total=%lu\r\n", (unsigned long)mbrtu_rx_total());
      HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)n, HAL_MAX_DELAY);
    }
#endif

    for (uint8_t s = 0; s < SLAVE_NUM; s++)
    {
      uint16_t base = slv[s].base;
      uint16_t span = slv[s].span;
      uint8_t  ok_all = 0;
      int r = -1, a;

#if (DEBUG_ONLY_SLAVE > 0)
      if (slv[s].addr != (DEBUG_ONLY_SLAVE)) continue;   /* 只测指定从机 */
#endif

      s_alive_ms = HAL_GetTick();

      /* 一次读完这台设备的整段（含中间空洞） */
      for (a = 0; a < RETRY_MAX; a++)
      {
        r = mbrtu_read(slv[s].addr, 0x03, base, span, tmp, 50);
        if (r == 0) break;
        osDelay(RETRY_DELAY_MS);
      }

      if (r == 0)
      {
        osMutexAcquire(s_regMutex, osWaitForever);
        memcpy(&g_slave_regs[slv[s].addr - 1][base], tmp, (size_t)span * 2);
        decode_dev(s, tmp, base);
        osMutexRelease(s_regMutex);
        ok_all = 1;
      }
      else
      {
        mark_dev_bad(s);
      }

      if (ok_all) { slv[s].online = 1; slv[s].fail_cnt = 0; g_slave_online[slv[s].addr] = 1; }
      else        { slv[s].online = 0; if (slv[s].fail_cnt < 255) slv[s].fail_cnt++; g_slave_online[slv[s].addr] = 0; }

#if GW_DEBUG_SERIAL
      osMutexAcquire(s_regMutex, osWaitForever);
      gw_print_slave(s);
      osMutexRelease(s_regMutex);
#endif
      osDelay(POLL_SLAVE_GAP_MS);
    }

    s_cycle_ms = HAL_GetTick() - t0;

#if GW_DEBUG_SERIAL
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n===== POLL DONE =====\r\n",
                      (uint16_t)strlen("\r\n===== POLL DONE =====\r\n"), HAL_MAX_DELAY);
#endif
    osDelay(POLL_CYCLE_GAP_MS);
  }
}

/* ================= 导出服务：TCP 5000（按设备分次读，避免大包） ================= */

/*
  协议（端口 5000）：
    连接后服务器先发 1 字节“设备数 N”；
    之后客户端每发 1 字节“设备序号 d(0..N-1)”，服务器回：
      4 字节长度(大端) + 该设备全部点的文本(每行一条)。
*/

static int dump_send_all(int fd, const uint8_t *buf, int len)
{
  int sent = 0, spin = 0;
  while (sent < len)
  {
    int w = lwip_send(fd, buf + sent, (size_t)(len - sent), 0);
    if (w > 0) { sent += w; spin = 0; continue; }
    if (w == 0) break;
    if (++spin > 1000) break;
    osDelay(5);
  }
  return sent;
}

static int dump_recv_byte(int fd, uint8_t *b, int timeout_ms)
{
  int waited = 0;
  while (waited < timeout_ms)
  {
    int r = lwip_recv(fd, b, 1, 0);
    if (r == 1) return 1;
    if (r == 0) return 0;
    osDelay(5);
    waited += 5;
  }
  return -1;
}

void mbgw_dump_task(void *argument)
{
  static char dumpbuf[DUMP_BUF_SIZE];
  int listen_fd = -1;
  struct sockaddr_in a;

  (void)argument;

  for (;;)
  {
    listen_fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { osDelay(500); continue; }

    memset(&a, 0, sizeof(a));
    a.sin_family      = AF_INET;
    a.sin_port        = htons(DUMP_TCP_PORT);
    a.sin_addr.s_addr = htonl(INADDR_ANY);

    if (lwip_bind(listen_fd, (struct sockaddr *)&a, sizeof(a)) != 0 ||
        lwip_listen(listen_fd, 2) != 0)
    {
      lwip_close(listen_fd);
      listen_fd = -1;
      osDelay(500);
      continue;
    }
    break;
  }

  HAL_UART_Transmit(&huart1, (uint8_t*)"[DUMP] listening on 5000\r\n",
                    (uint16_t)strlen("[DUMP] listening on 5000\r\n"), HAL_MAX_DELAY);

  for (;;)
  {
    struct sockaddr_in cli;
    socklen_t cl = sizeof(cli);
    int c, fl;
    uint8_t ndev;

    c = lwip_accept(listen_fd, (struct sockaddr *)&cli, &cl);
    if (c < 0) { osDelay(10); continue; }

    fl = 1;
    lwip_ioctl(c, FIONBIO, &fl);

    HAL_UART_Transmit(&huart1, (uint8_t*)"[DUMP] client connected\r\n",
                      (uint16_t)strlen("[DUMP] client connected\r\n"), HAL_MAX_DELAY);

    ndev = (uint8_t)SLAVE_NUM;
    dump_send_all(c, &ndev, 1);

    for (;;)
    {
      uint8_t req;
      int r, n;

      r = dump_recv_byte(c, &req, 5000);
      if (r != 1) break;
      if (req >= SLAVE_NUM) break;

      osMutexAcquire(s_regMutex, osWaitForever);
      n = cache_dump_dev(dumpbuf, sizeof(dumpbuf), req);
      osMutexRelease(s_regMutex);

      {
        uint8_t hdr[4];
        hdr[0] = (uint8_t)((uint32_t)n >> 24);
        hdr[1] = (uint8_t)((uint32_t)n >> 16);
        hdr[2] = (uint8_t)((uint32_t)n >> 8);
        hdr[3] = (uint8_t)((uint32_t)n);

        dump_send_all(c, hdr, 4);
        if (n > 0) dump_send_all(c, (const uint8_t *)dumpbuf, n);
      }
    }

    lwip_close(c);
  }
}
