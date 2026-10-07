/**
  ******************************************************************************
  * @file    mb_gateway.c
  * @brief   网关应用层（核心）：描述表 -> 运行期点表 -> 轮询刷新 -> 三种出口
  *
  * @note    本文件是三张"视图"的中心：
  *            ① 描述表（SEG_Dx / PT_Dx / SLAVES，常量，放 Flash）—— 你每天改的就是它；
  *            ② 点表  pt[]      （RAM，按"点"组织：值/质量/时间）—— 屏幕、导出用；
  *            ③ 寄存器镜像 g_slave_regs[][]（RAM，按"从机+寄存器地址"）—— Modbus TCP 透传用。
  *
  *          轮询任务一次读回整台设备的寄存器，就同时更新 ②（decode_dev）和 ③（memcpy）。
  *          对外接口：
  *            - mbgw_*（按 从机 + 寄存器地址）  ：给 Modbus TCP 透传；
  *            - dp_*  （按 点号）               ：给屏幕/调试；
  *            - gw_*  （生命周期/从机信息/变更通知）。
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

#define DUMP_TCP_PORT   5000      /**< 文本导出服务监听的端口（PC 端桥/脚本连它读全部点） */
#ifndef DUMP_BUF_SIZE
#define DUMP_BUF_SIZE   8192      /**< 单台设备文本缓冲大小（一台几百字节，够用） */
#endif

/* ================= 描述表类型 ================= */

/**
  * @brief 一段"连续的寄存器"描述
  * @note  一台设备的寄存器地址可能不连续（有空洞），用一段表示"从 start 起读 count 个"。
  *        注意：当前轮询已改为"每台设备把段合并成一段整体读"，
  *        因此 period_ms 暂时未使用（保留字段，勿删）。
  */
typedef struct
{
  uint8_t  start;      /**< 起始寄存器地址（0..MB_MAX_REGS-1） */
  uint8_t  count;      /**< 该段寄存器个数 */
  uint16_t period_ms;  /**< 预留的轮询周期（当前未使用，保留） */
} seg_def_t;

/**
  * @brief 一个"数据点"的描述（描述表的一行）
  * @note  模板：说明"某个寄存器/某组寄存器"是什么、怎么解释。
  */
typedef struct pt_def
{
  uint8_t     seg;      /**< 属于本从机 segs[] 里的第几段（下标） */
  uint8_t     offset;   /**< 段内偏移（寄存器个数） */
  uint8_t     kind;     /**< 原始类型：T_U16/T_I16/T_U32/T_U32_DEC/T_F32 */
  uint8_t     dec;      /**< 呈现小数位：真值 = 寄存器值 / 10^dec（T_U32_DEC 时忽略） */
  uint8_t     hide;     /**< 1 = 只参与组合计算、不单独输出（如累积量的低字/小数位） */
  const char *name;     /**< 点名（唯一，如 "flow_total"） */
} pt_def_t;

/**
  * @brief 一台"从机设备"的描述
  * @note  指向它自己的段表与点表；改设备只改这里。
  */
typedef struct
{
  uint8_t          addr;      /**< 从机地址（1..MB_MAX_SLAVE） */
  const char      *name;      /**< 设备名（如 "COM10-001"） */
  uint8_t          seg_num;   /**< 段数组元素个数 */
  const seg_def_t *segs;      /**< 指向段数组 */
  uint8_t          pt_num;    /**< 点数组元素个数 */
  const pt_def_t  *pts;       /**< 指向点数组 */
} slave_def_t;

/* ================= 运行期点数据 =================
 * 说明：由描述表在 cache_init() 里"生成一次"，之后更新/读取都只操作这个数组。
 */

/**
  * @brief 运行期的"一个点"（点表的元素）
  */
typedef struct
{
  uint16_t        r[3];      /**< 原始寄存器值：U16/I16 用 r[0]；U32/F32 用 r[0]r[1]；U32_DEC 用 r[0..2] */
  uint8_t         q;         /**< 质量：0=没读过；1=好；2=通信失败 */
  uint32_t        ts;        /**< 最后一次成功读到的时间戳（HAL_GetTick，单位 ms） */
  const pt_def_t *def;       /**< 指回描述表那一行（类型/小数位/名字都从这里取） */
  uint8_t         dev;       /**< 属于哪台设备（slv[] 下标） */
  uint8_t         changed;   /**< 本次轮询该点值是否发生变化（1=变了，给屏幕/通知用） */
} point_t;

static point_t  pt[MAX_PT];    /**< 所有点（连续数组，按点号 0..pt_used-1 索引） */
static uint16_t pt_used;       /**< 实际生成的点数（hide 的行不生成；当前=78） */

/* ================= 镜像 + 状态 ================= */

/** 寄存器镜像：g_slave_regs[从机号-1][寄存器地址]。访问前必须持有 s_regMutex。 */
static uint16_t g_slave_regs[MB_MAX_SLAVE][MB_MAX_REGS];
/** 各从机在线标志，下标 1..MB_MAX_SLAVE（1=在线）。 */
static uint8_t  g_slave_online[MB_MAX_SLAVE + 1];
/** 保护 pt[] 与镜像的互斥锁（轮询写、TCP 读、导出读、屏幕读都要用它）。 */
static osMutexId_t s_regMutex;
/** 轮询任务心跳（最近一次进入轮询循环的时间），供看门狗判断其是否卡死。 */
static volatile uint32_t s_alive_ms = 0;
/** 最近一轮"完整轮询"的耗时（从第一台开始到最后一台结束），单位 ms，给屏幕显示。 */
static volatile uint32_t s_cycle_ms = 0;   /**< 最近一轮"完整轮询"的耗时（毫秒），给屏幕显示 */
static uint32_t s_cycle_cnt = 0;           /**< 轮询轮次计数（每轮 +1），用于降级退避调度 */

/**
  * @brief 运行期的一台"从机"（由描述表生成）
  */
typedef struct
{
  uint8_t          addr;      /**< 从机地址 */
  const slave_def_t *def;     /**< 指回描述表那台 */
  uint16_t         pt_base;   /**< 这台设备的第一个点在 pt[] 里的下标 */
  uint8_t          online;    /**< 最近一轮是否读到数据（1=在线） */
  uint8_t          fail_cnt;  /**< 连续失败次数（成功清零；屏幕用它画红/黑点） */
  uint16_t         base;      /**< 合并段后的起始地址（min） */
  uint16_t         span;      /**< 合并段后的寄存器个数（max - min + 1） */
  uint32_t         next_cycle;/**< 下一次允许被轮询的“轮次号”（降级退避用） */
} gw_slave_t;

static gw_slave_t slv[MAX_SLAVES];  /**< 运行期从机数组（下标 0..SLAVE_NUM-1） */

/* ================= 变更通知（给屏幕：值一变就通知，屏幕不用轮询） ================= */

static osEventFlagsId_t s_chgEvt;              /**< 事件标志组句柄（"有点变了"） */
#define GW_CHG_BIT      0x0001u                /**< 事件标志里表示"有点变了"的那一位 */
static uint8_t  s_dirty[(MAX_PT + 7) / 8];     /**< 脏位图：每个点一位，1=该点值变过 */

/**
  * @brief  标记某个点的值变了：置脏位 + 置事件
  * @param  idx  点号（0..pt_used-1）
  * @retval 无
  * @note   临界区保护脏位图（与 gw_take_changed 的清除互斥）；置位后再置事件唤醒屏幕。
  */
static void chg_mark(uint16_t idx)
{
  taskENTER_CRITICAL();
  s_dirty[idx >> 3] |= (uint8_t)(1u << (idx & 7));   /* idx>>3=第几字节；idx&7=字节内第几位 */
  taskEXIT_CRITICAL();
  (void)osEventFlagsSet(s_chgEvt, GW_CHG_BIT);
}

/* ================= 设备描述表（日常只改这里） ================= */
/* PT 行字段：{段, 段内偏移, 类型, 小数位, 是否隐藏(参与组合不单独输出), 点名} */

/* ---------------- 设备1：COM10-001 流量计（14 条） ----------------
 * 段0 地址0..7；段1 地址13..18（合并后整体读 0..18） */
static const seg_def_t SEG_D1[] = { {0x00, 8, 1000}, {0x0D, 6, 1000} };
static const pt_def_t  PT_D1[]  = {
  { 0, 0, T_U16,     0, 0, "baud"              },   /* 地址0  波特率 */
  { 0, 1, T_U16,     0, 0, "station"           },   /* 地址1  站点号 */
  { 0, 2, T_U16,     0, 0, "decimals"          },   /* 地址2  小数位 */
  { 0, 3, T_U16,     2, 0, "K"                 },   /* 地址3  K 值(/100) */
  { 0, 4, T_U32_DEC, 0, 0, "flow_total"        },   /* 地址4~6 永久累积流量(组合) */
  { 0, 5, T_U16,     0, 1, "flow_total_lo"     },   /* 地址5  低字(隐藏) */
  { 0, 6, T_U16,     0, 1, "flow_total_dec"    },   /* 地址6  小数位(隐藏) */
  { 0, 7, T_U16,     0, 0, "flow_total_pulse"  },   /* 地址7  脉冲 */
  { 1, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 地址13~15 临时累积流量(组合) */
  { 1, 1, T_U16,     0, 1, "flow_temp_lo"      },   /* 地址14 低字(隐藏) */
  { 1, 2, T_U16,     0, 1, "flow_temp_dec"     },   /* 地址15 小数位(隐藏) */
  { 1, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 地址16 脉冲 */
  { 1, 4, T_U16,     2, 0, "flow_instant"      },   /* 地址17 瞬时流量(/100) */
  { 1, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 地址18 瞬时脉冲 */
};

/* ---------------- 设备2：COM10-002（16 条）
 * 段0 地址0..7；段1 地址9..10；段2 地址13..18（合并后整体读 0..18） ---------------- */
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
  { 1, 0, T_U16,     2, 0, "ma_4ma"            },   /* 地址9  4mA 下限(/100) */
  { 1, 1, T_U16,     2, 0, "ma_20ma"           },   /* 地址10 20mA 上限(/100) */
  { 2, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 地址13~15 */
  { 2, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 2, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 2, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 地址16 */
  { 2, 4, T_U16,     2, 0, "flow_instant"      },   /* 地址17 */
  { 2, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 地址18 */
};

/* ---------------- 设备3：COM10-003 气象站（9 条，全 F32）
 * 段0 地址0..17（每个 float 占 2 个寄存器） ---------------- */
static const seg_def_t SEG_D3[] = { {0x00, 18, 1000} };
static const pt_def_t  PT_D3[]  = {
  { 0, 0,  T_F32, 0, 0, "wind_speed" },   /* 地址0  瞬时风速 */
  { 0, 2,  T_F32, 0, 0, "wind_dir"   },   /* 地址2  瞬时风向 */
  { 0, 4,  T_F32, 0, 0, "air_temp"   },   /* 地址4  气温 */
  { 0, 6,  T_F32, 0, 0, "humidity"   },   /* 地址6  相对湿度 */
  { 0, 8,  T_F32, 0, 0, "pressure"   },   /* 地址8  气压 */
  { 0, 10, T_F32, 0, 0, "rain_min"   },   /* 地址10 分钟雨量 */
  { 0, 12, T_F32, 0, 0, "rain_hour"  },   /* 地址12 小时雨量 */
  { 0, 14, T_F32, 0, 0, "rain_day"   },   /* 地址14 天雨量 */
  { 0, 16, T_F32, 0, 0, "rain_total" },   /* 地址16 累计雨量 */
};

/* ---------------- 设备4/6：COM10-004/006 空气环境（10 条，两台共用此表）
 * 段0 地址1..2；段1 地址10..15；段2 地址26..27 ---------------- */
static const seg_def_t SEG_D4[] = { {0x01, 2, 1000}, {0x0A, 6, 1000}, {0x1A, 2, 1000} };
static const pt_def_t  PT_D4[]  = {
  { 0, 0, T_U16, 0, 0, "dev_addr" },   /* 地址1  设备地址 */
  { 0, 1, T_U16, 0, 0, "baud"     },   /* 地址2  波特率 */
  { 1, 0, T_I16, 1, 0, "temp"     },   /* 地址10 温度(/10, 有符号) */
  { 1, 1, T_I16, 1, 0, "humidity" },   /* 地址11 湿度(/10, 有符号) */
  { 1, 2, T_U16, 0, 0, "pm1"      },   /* 地址12 PM1.0 */
  { 1, 3, T_U16, 0, 0, "pm25"     },   /* 地址13 PM2.5 */
  { 1, 4, T_U16, 0, 0, "pm10"     },   /* 地址14 PM10 */
  { 1, 5, T_U16, 0, 0, "co2"      },   /* 地址15 二氧化碳 */
  { 2, 0, T_U16, 2, 0, "hcho"     },   /* 地址26 甲醛(/100) */
  { 2, 1, T_U16, 1, 0, "voc"      },   /* 地址27 VOCs(/10) */
};

/* ---------------- 设备5：COM10-005 流量+压力（17 条）
 * 段0 0..7；段1 11..12；段2 13..18；段3 20 ---------------- */
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
  { 1, 0, T_U16,     0, 0, "press_A"           },   /* 地址11 压力常数A */
  { 1, 1, T_U16,     0, 0, "press_B"           },   /* 地址12 压力常数B */
  { 2, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 地址13~15 */
  { 2, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 2, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 2, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 地址16 */
  { 2, 4, T_U16,     2, 0, "flow_instant"      },   /* 地址17 */
  { 2, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 地址18 */
  { 3, 0, T_U16,     2, 0, "pressure_kpa"      },   /* 地址20 压力(kPa)(/100) */
};

/* ---------------- 设备7：COM10-007 流量+NTC（16 条）
 * 段0 0..8；段1 13..19 ---------------- */
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
  { 0, 8, T_U16,     0, 0, "ntc_res"           },   /* 地址8  NTC 电阻阻值 */
  { 1, 0, T_U32_DEC, 0, 0, "flow_temp"         },   /* 地址13~15 */
  { 1, 1, T_U16,     0, 1, "flow_temp_lo"      },
  { 1, 2, T_U16,     0, 1, "flow_temp_dec"     },
  { 1, 3, T_U16,     0, 0, "flow_temp_pulse"   },   /* 地址16 */
  { 1, 4, T_U16,     2, 0, "flow_instant"      },   /* 地址17 */
  { 1, 5, T_U16,     0, 0, "flow_instant_pulse"},   /* 地址18 */
  { 1, 6, T_U16,     1, 0, "temp_c"            },   /* 地址19 摄氏温度(/10) */
};

/* ---------------- 设备8：COM10-008 简易温湿度（2 条）
 * 段0 地址0..1 ---------------- */
static const seg_def_t SEG_D8[] = { {0x00, 2, 1000} };
static const pt_def_t  PT_D8[]  = {
  { 0, 0, T_I16, 1, 0, "humidity" },   /* 地址0 湿度(/10, 有符号) */
  { 0, 1, T_U16, 1, 0, "temp"     },   /* 地址1 温度(/10) */
};

/** 取数组元素个数（编译期常量） */
#define PTS(x)  ((uint8_t)(sizeof(x)/sizeof((x)[0])))

/** 从机总表：每行 = {地址, 设备名, 段数, 段表, 点数, 点表} */
static const slave_def_t SLAVES[] = {
  { 1, "COM10-001", 2, SEG_D1, PTS(PT_D1), PT_D1 },
  { 2, "COM10-002", 3, SEG_D2, PTS(PT_D2), PT_D2 },
  { 3, "COM10-003", 1, SEG_D3, PTS(PT_D3), PT_D3 },
  { 4, "COM10-004", 3, SEG_D4, PTS(PT_D4), PT_D4 },
  { 5, "COM10-005", 4, SEG_D5, PTS(PT_D5), PT_D5 },
  { 6, "COM10-006", 3, SEG_D4, PTS(PT_D4), PT_D4 },   /* 设备6 与设备4 描述相同 */
  { 7, "COM10-007", 2, SEG_D7, PTS(PT_D7), PT_D7 },
  { 8, "COM10-008", 1, SEG_D8, PTS(PT_D8), PT_D8 },
};
/** 从机台数 */
#define SLAVE_NUM  (sizeof(SLAVES)/sizeof(SLAVES[0]))

/* ---- 轮询策略 ---- */
#define RETRY_MAX       2     /**< 单台设备读失败时最多重试次数 */
#define RETRY_DELAY_MS  10    /**< 两次重试之间的间隔（毫秒） */
#define READ_TIMEOUT_MS 20    /**< 单次读等应答的超时（毫秒）；掉线时靠它尽快跳过 */

/* ---- 降级退避：连续失败越多，轮询得越稀，避免一台坏从机拖慢整轮 ----
 * 间隔含义：连续失败达到阈值后，改为“每 N 轮才轮询一次”。 */
#define BACKOFF_L1      3     /**< 失败 >= 3 起：每 2 轮轮询一次 */
#define BACKOFF_L2      6     /**< 失败 >= 6 起：每 5 轮轮询一次 */
#define BACKOFF_L3      12    /**< 失败 >= 12 起：每 12 轮轮询一次 */

/* ---- 调试开关 ---- */
#define GW_DEBUG_SERIAL   0   /**< 1=把每台设备缓存打到串口(USART1)，会明显变慢；正式版=0 */
#define DEBUG_ONLY_SLAVE  0   /**< 0=轮询全部；填某从机地址则只轮询它（单台排查用） */

/* ---- 轮询节奏（毫秒）---- */
#define POLL_SLAVE_GAP_MS   0   /**< 两台设备之间的间隔 */
#define POLL_CYCLE_GAP_MS   5   /**< 每一轮之间的间隔（留给屏幕等低优先级任务） */

/* ================= 初始化 ================= */

/**
  * @brief  把描述表（SLAVES/SEG/PT）解析成运行期表（slv[] 与 pt[]）
  * @param  无
  * @retval true=成功；false=描述表有误（地址重复/越界、点跨段、超容量等）
  * @note   只在启动时调用一次；之后不再遍历描述表。
  *         流程：遍历每台设备 -> 登记 slv[s]（含合并段 base/span）-> 遍历每个点生成 pt[]。
  */
static bool cache_init(void)
{
  uint16_t pcur = 0;   /* 下一个要写入 pt[] 的下标（跑完即为总点数） */

  for (uint8_t s = 0; s < SLAVE_NUM; s++)
  {
    const slave_def_t *d = &SLAVES[s];   /* d = 第 s 台设备的描述 */

    /* 从机地址查重 + 范围检查 */
    for (uint8_t k = 0; k < s; k++)
      if (slv[k].addr == d->addr) return false;
    if (d->addr < 1 || d->addr > MB_MAX_SLAVE) return false;

    slv[s].addr     = d->addr;
    slv[s].def      = d;
    slv[s].pt_base  = pcur;    /* 这台设备的点从 pt[pcur] 开始 */
    slv[s].online   = 0;
    slv[s].fail_cnt = 0;
    slv[s].next_cycle = 0;   /* 首轮即可被轮询 */

    /* 合并段：算出这台设备要读的"最小地址 ~ 最大地址"一整段 */
    {
      uint16_t b = d->segs[0].start;                                       /* 最小地址 */
      uint16_t e = (uint16_t)(d->segs[0].start + d->segs[0].count - 1);    /* 最大地址 */
      for (uint8_t t = 1; t < d->seg_num; t++)
      {
        uint16_t s0 = d->segs[t].start;
        uint16_t s1 = (uint16_t)(d->segs[t].start + d->segs[t].count - 1);
        if (s0 < b) b = s0;
        if (s1 > e) e = s1;
      }
      if ((uint16_t)(e + 1) > MB_MAX_REGS) return false;   /* 合并后超出镜像容量 */
      slv[s].base = b;
      slv[s].span = (uint16_t)(e - b + 1);
    }

    /* 遍历这台设备的每个点，生成运行期点 */
    for (uint8_t i = 0; i < d->pt_num; i++)
    {
      const pt_def_t  *p = &d->pts[i];        /* 第 i 行点描述 */
      const seg_def_t *g = &d->segs[p->seg];   /* 该点所属的段 */
      uint8_t regs;                            /* 该点占用的寄存器个数 */

      if (p->hide) continue;                   /* 组合用的行不单独生成点 */

      regs = (p->kind == T_U32 || p->kind == T_F32) ? 2 :
             (p->kind == T_U32_DEC) ? 3 : 1;

      /* 校验：段不越界、点不跨段、容量足够 */
      if ((uint16_t)g->start + g->count > MB_MAX_REGS) return false;
      if ((uint16_t)p->offset + regs > g->count) return false;
      if (pcur >= MAX_PT) return false;

      pt[pcur].r[0] = pt[pcur].r[1] = pt[pcur].r[2] = 0;
      pt[pcur].def     = p;
      pt[pcur].dev     = s;
      pt[pcur].q       = 0;
      pt[pcur].ts      = 0;
      pt[pcur].changed = 0;
      pcur++;
    }
  }

  pt_used = pcur;
  return (pt_used <= MAX_PT);
}

/* ================= 解码一台设备的原始寄存器到点 ================= */

/**
  * @brief  把"一台设备一次读回的整段寄存器"解码进属于它的每个点
  * @param  s    设备序号（slv[] 下标）
  * @param  raw  从地址 base 开始、连续读回的寄存器数组（含中间空洞）
  * @param  base raw[0] 对应的绝对寄存器地址
  * @retval 无
  * @note   调用者必须已持有 s_regMutex。
  *         对每个点：绝对地址 a = segs[p->seg].start + p->offset；
  *         在 raw 里的下标 off = a - base。写前先比旧值，变了就 chg_mark()。
  */
static void decode_dev(uint8_t s, const uint16_t *raw, uint16_t base)
{
  const slave_def_t *d = slv[s].def;
  uint32_t now = HAL_GetTick();

  for (uint16_t i = 0; i < pt_used; i++)
  {
    const pt_def_t *p;
    uint16_t a, off, o0, o1, o2;

    if (pt[i].dev != s) continue;               /* 只解本设备的点 */
    p   = pt[i].def;
    a   = (uint16_t)(d->segs[p->seg].start + p->offset);   /* 该点的绝对地址 */
    off = (uint16_t)(a - base);                            /* 在 raw 里的下标 */

    o0 = pt[i].r[0]; o1 = pt[i].r[1]; o2 = pt[i].r[2];      /* 先留旧值，用于比较 */

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
      pt[i].changed = 1;         /* 值变了 */
      chg_mark((uint16_t)i);     /* 通知（置脏位+置事件） */
    }

    pt[i].q  = 1;                /* 质量=好 */
    pt[i].ts = now;              /* 记录成功时刻 */
  }
}

/**
  * @brief  把一台设备的所有点标记为"通信失败"（q=2）
  * @param  s  设备序号
  * @retval 无
  * @note   值保持旧值（不清零），累积量尤其不能清（否则统计会倒流）。
  */
static void mark_dev_bad(uint8_t s)
{
  for (uint16_t i = 0; i < pt_used; i++)
  {
    if (pt[i].dev == s)
      pt[i].q = 2;
  }
}

/* ================= 对外：生命周期 ================= */

/**
  * @brief  初始化网关模块：创建互斥锁/事件标志，并解析描述表
  * @param  无
  * @retval 无
  * @note   必须在创建任务之前调用（由 freertos.c 的 MX_FREERTOS_Init 调用）。
  */
void mbgw_init(void)
{
  s_regMutex = osMutexNew(NULL);
  s_chgEvt   = osEventFlagsNew(NULL);
  (void)cache_init();
}

/**
  * @brief  查询某从机当前是否在线（最近一轮是否读到数据）
  * @param  slave  从机地址（1..MB_MAX_SLAVE）
  * @retval 1=在线；0=离线/地址非法
  */
uint8_t mbgw_is_online(uint8_t slave)
{
  if (slave < 1 || slave > MB_MAX_SLAVE) return 0;
  return g_slave_online[slave];
}

/**
  * @brief  取轮询任务的最近心跳时间
  * @param  无
  * @retval HAL_GetTick 时间戳（毫秒）
  */
uint32_t mbgw_last_alive_ms(void)
{
  return s_alive_ms;
}

/* ================= 对外：变更通知（给屏幕） ================= */

/**
  * @brief  阻塞等待"有点变了"事件
  * @param  timeout_ms  最长等待（毫秒），osWaitForever 表示一直等
  * @retval 事件标志值（含 GW_CHG_BIT=等到；否则为超时/错误码）
  */
uint32_t gw_wait_changed(uint32_t timeout_ms)
{
  return osEventFlagsWait(s_chgEvt, GW_CHG_BIT, osFlagsWaitAny, timeout_ms);
}

/**
  * @brief  取走当前所有"变了的点号"，并清脏位
  * @param  out  输出缓冲（点号数组）
  * @param  max  最多取多少个
  * @retval 实际取出的点号个数
  */
uint16_t gw_take_changed(uint16_t *out, uint16_t max)
{
  uint16_t n = 0;   /* 已取出个数 */

  taskENTER_CRITICAL();
  for (uint16_t i = 0; i < pt_used && n < max; i++)
  {
    if (s_dirty[i >> 3] & (uint8_t)(1u << (i & 7)))
    {
      s_dirty[i >> 3] &= (uint8_t)~(1u << (i & 7));   /* 清该点脏位 */
      out[n++] = i;
    }
  }
  taskEXIT_CRITICAL();
  return n;
}

/* ================= 对外：按 从机+寄存器（TCP 透传） ================= */

/**
  * @brief  读某从机的一段寄存器（从镜像里取，供 Modbus TCP 透传）
  * @param  slave  从机地址
  * @param  start  起始寄存器地址
  * @param  qty    读取个数
  * @param  out    输出缓冲（长度 >= qty）
  * @retval 无
  */
void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out)
{
  uint16_t i;
  if (slave < 1 || slave > MB_MAX_SLAVE) { for (i = 0; i < qty; i++) out[i] = 0; return; }

  osMutexAcquire(s_regMutex, osWaitForever);
  for (i = 0; i < qty; i++)
    out[i] = g_slave_regs[slave - 1][start + i];
  osMutexRelease(s_regMutex);
}

/**
  * @brief  写单个寄存器（写穿透：先写给从机，成功后才更新镜像）
  * @param  slave  从机地址
  * @param  addr   寄存器地址
  * @param  val    要写的值
  * @retval 0=成功；-1=参数非法或 RTU 写失败
  * @note   顺序很重要：设备是"真相"，先写设备；失败就不动镜像。
  */
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

/**
  * @brief  写多个寄存器（写穿透）
  * @param  slave  从机地址
  * @param  start  起始寄存器地址
  * @param  qty    个数
  * @param  vals   值数组（长度 >= qty）
  * @retval 0=成功；-1=参数非法或 RTU 写失败
  */
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

/**
  * @brief  按"设备地址 + 点名"查找点号
  * @param  addr  从机地址
  * @param  name  点名（如 "flow_total"）
  * @retval 点号（>=0）；-1=没找到
  */
int dp_find(uint8_t addr, const char *name)
{
  for (uint16_t i = 0; i < pt_used; i++)
  {
    if (slv[pt[i].dev].addr != addr) continue;
    if (strcmp(name, pt[i].def->name) == 0) return (int)i;
  }
  return -1;
}

/** @brief 总点数 */
uint16_t dp_count(void)           { return pt_used; }
/** @brief 质量：0=没读过 1=好 2=失败（越界按 2 处理） */
uint8_t  dp_quality(uint16_t idx) { return (idx < pt_used) ? pt[idx].q : 2; }
/** @brief 距最近一次成功读到的毫秒数（越界返回 0xFFFFFFFF） */
uint32_t dp_age_ms(uint16_t idx)  { return (idx < pt_used) ? (HAL_GetTick() - pt[idx].ts) : 0xFFFFFFFF; }
/** @brief 点的原始类型（越界返回 0xFF） */
uint8_t  dp_kind(uint16_t idx)    { return (idx < pt_used) ? pt[idx].def->kind : 0xFF; }

/** @brief 点名（越界返回空串） */
const char *dp_name(uint16_t idx)
{
  return (idx < pt_used) ? pt[idx].def->name : "";
}
/** @brief 点所属的设备名（越界返回空串） */
const char *dp_dev_name(uint16_t idx)
{
  return (idx < pt_used) ? slv[pt[idx].dev].def->name : "";
}
/** @brief 点所属的设备地址（越界返回 0） */
uint8_t dp_dev_addr(uint16_t idx)
{
  return (idx < pt_used) ? slv[pt[idx].dev].addr : 0;
}

/* ---- 按从机序号访问（给屏幕用） ---- */
/** @brief 从机台数 */
uint8_t     gw_slave_count(void)       { return (uint8_t)SLAVE_NUM; }
/** @brief 第 i 台的设备名 */
const char *gw_slave_name(uint8_t i)   { return (i < SLAVE_NUM) ? slv[i].def->name : ""; }
/** @brief 第 i 台的从机地址 */
uint8_t     gw_slave_addr(uint8_t i)   { return (i < SLAVE_NUM) ? slv[i].addr : 0; }
/** @brief 第 i 台是否在线 */
uint8_t     gw_slave_online(uint8_t i) { return (i < SLAVE_NUM) ? slv[i].online : 0; }
/** @brief 第 i 台的连续失败次数 */
uint8_t     gw_slave_fail(uint8_t i)   { return (i < SLAVE_NUM) ? slv[i].fail_cnt : 0; }
/** @brief 最近一轮完整轮询耗时（毫秒） */
uint32_t    gw_cycle_ms(void)          { return s_cycle_ms; }

/* ================= F32 解码（不用 float：IEEE754 -> 定点整数） =================
 *
 * 一个 32 位浮点数的二进制布局：  [符号1][指数8][尾数23]
 *   值 = (-1)^sign * 2^(exp-127) * (1 + 尾数/2^23)
 * 这里把它换算成"定点整数"：返回 = 真值 × 10^decimals，便于整数显示。
 * 例：raw=40 48 F5 C3 (即 3.14)，decimals=3 -> 返回 3140。
 */

/**
  * @brief  把 IEEE754 的 4 字节（大端）转成"真值 × 10^decimals"的整数
  * @param  raw       4 个原始字节（高字在前）
  * @param  decimals  要保留的小数位数
  * @retval 定点整数（纯整数运算，不引入浮点）
  */
static int64_t f32_to_fixed(const uint8_t *raw, uint8_t decimals)
{
  uint32_t bits = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16) |
                  ((uint32_t)raw[2] << 8)  | (uint32_t)raw[3];   /* 4 字节拼成 32 位 */
  uint32_t sign = bits >> 31;                              /* 第31位=符号 */
  int32_t  exp  = (int32_t)((bits >> 23) & 0xFF) - 127;    /* 第30..23位=指数，减偏移127 */
  uint32_t frac = bits & 0x7FFFFFu;                        /* 第22..0位=尾数(23位) */
  int64_t  m, numer, result;
  int32_t  shift;

  if (exp == 128) return 0;                   /* 指数全1：无穷/NaN -> 当 0 */
  if (exp == -127 && frac == 0) return 0;     /* 指数全0且尾数0：±0 */

  if (exp == -127) { m = frac;               shift = -149; }   /* 非规格化数 */
  else             { m = (1u << 23) | frac;  shift = exp - 23; } /* 规格化：补隐含的 1 */

  numer = m;                                  /* 真值 = numer × 2^shift */
  for (uint8_t i = 0; i < decimals; i++) numer *= 10;   /* 再 ×10^decimals */

  if (shift >= 0)
  {
    result = numer << shift;                  /* shift>=0：左移放大 */
  }
  else
  {
    int32_t sh = -shift;                      /* shift<0：右移缩小（带四舍五入） */
    result = (sh > 62) ? 0 : (numer + ((int64_t)1 << (sh - 1))) >> sh;
  }
  return sign ? -result : result;             /* 负数取反 */
}

/**
  * @brief  取某个点的"原始读数"
  * @param  idx  点号
  * @param  out  输出（整数给 scaled/decimals；F32 给 raw[4]）
  * @retval true=成功；false=点号越界/类型未知
  * @note   U32_DEC：高字/低字=整数部分，第 3 个寄存器=小数部分(单位0.01)。
  */
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

/**
  * @brief  取某个点的"十进制定点值"（统一接口，F32 也解码）
  * @param  idx       点号
  * @param  value     输出：真值 = (*value) / 10^decimals
  * @param  decimals  目标小数位
  * @retval true=成功；false=点号越界
  */
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
    for (uint8_t i = o.decimals; i < decimals; i++) o.scaled *= 10;   /* 目标位数更多：×10 */
  }
  else
  {
    for (uint8_t i = decimals; i < o.decimals; i++) o.scaled /= 10;   /* 目标位数更少：÷10 */
  }
  *value = o.scaled;
  return true;
}

/**
  * @brief  把"定点整数 + 小数位"格式化成字符串（内部用，不用 %f）
  * @param  s         定点整数（真值 = s / 10^decimals）
  * @param  decimals  小数位
  * @param  buf       输出缓冲（调用者保证足够大，建议 >= 24）
  * @retval 写入的字符数
  */
static int fmt_fixed(int64_t s, uint8_t decimals, char *buf)
{
  uint64_t a = (s < 0) ? (uint64_t)(-s) : (uint64_t)s;   /* 绝对值（避免负数取模出问题） */
  uint64_t p = 1;
  for (uint8_t i = 0; i < decimals; i++) p *= 10;        /* p = 10^decimals */
  uint64_t ip = a / p;                                    /* 整数部分 */
  uint64_t fp = a % p;                                    /* 小数部分 */

  char ib[24]; int ii = 0;
  do { ib[ii++] = (char)('0' + (ip % 10)); ip /= 10; } while (ip);   /* 整数部分逐位取（反序） */

  int m = 0;
  if (s < 0) buf[m++] = '-';
  while (ii) buf[m++] = ib[--ii];                        /* 填回成正序 */

  if (decimals)
  {
    char fb[12];
    for (int i = decimals - 1; i >= 0; i--) { fb[i] = (char)('0' + (fp % 10)); fp /= 10; }
    buf[m++] = '.';
    for (uint8_t i = 0; i < decimals; i++) buf[m++] = fb[i];
  }
  buf[m] = 0;                                            /* 结束符 */
  return m;
}

/**
  * @brief  把 dp_out_t 格式化成字符串（F32 -> "F32:xxxxxxxx"，其余 -> 十进制）
  * @param  v    dp_read 的输出
  * @param  buf  输出缓冲
  * @retval 写入长度
  */
int dp_format(const dp_out_t *v, char *buf)
{
  if (v->is_f32)
  {
    return sprintf(buf, "F32:%02X%02X%02X%02X",
                   v->raw[0], v->raw[1], v->raw[2], v->raw[3]);
  }
  return fmt_fixed(v->scaled, v->decimals, buf);
}

/**
  * @brief  直接把某个点格式化成十进制字符串（F32 也换算成小数）
  * @param  idx       点号
  * @param  buf       输出缓冲
  * @param  decimals  小数位
  * @retval 写入长度（0=点号非法）
  */
int dp_format_num(uint16_t idx, char *buf, uint8_t decimals)
{
  int64_t v;
  if (!dp_read_num(idx, &v, decimals)) { buf[0] = 0; return 0; }
  return fmt_fixed(v, decimals, buf);
}

/**
  * @brief  把"一台设备的所有点"拼成 CSV 文本（导出协议用）
  * @param  buf  输出缓冲
  * @param  cap  缓冲容量
  * @param  dev  设备序号（slv[] 下标）
  * @retval 写入长度
  * @note   每行：点号,设备名,从机号,点名,值,q=质量,age=龄期,reg=寄存器地址,k=类型,d=小数位
  */
static int cache_dump_dev(char *buf, int cap, uint8_t dev)
{
  int n = 0;   /* 已写入长度 */

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
      uint16_t reg = (uint16_t)(slv[pt[i].dev].def->segs[p->seg].start + p->offset);   /* 该点的绝对寄存器地址 */
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

/**
  * @brief  把所有点拼成一个 CSV（不按设备分）
  * @param  buf  输出缓冲
  * @param  cap  缓冲容量
  * @retval 写入长度
  * @note   每行：点号,设备名,从机号,点名,值,q=质量,age=龄期
  */
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

/* ================= 调试：把一台从机的缓存打到串口（GW_DEBUG_SERIAL=1 时才编译） ================= */

#if GW_DEBUG_SERIAL
/**
  * @brief  把一台设备的所有点打到 USART1（仅调试）
  * @param  s  设备序号
  * @retval 无
  */
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

/**
  * @brief  RS485 Modbus RTU 主站轮询任务（任务永不返回）
  * @param  argument  未使用
  * @retval 无
  * @note   每轮：对每台设备"整段读一次"；成功->解码进 pt[] 并写镜像、在线清零失败计数；
  *         失败->标 q=2、失败计数+1。每台/每段刷新心跳，防止看门狗误判。
  *         每轮开始调用 mbrtu_rx_heal() 自愈 UART 接收。
  */
void mbgw_poll_task(void *argument)
{
  uint16_t tmp[MB_MAX_REGS];   /* 一次性读回的寄存器缓冲（最大一台的一段） */
  (void)argument;

  HAL_UART_Transmit(&huart1, (uint8_t*)"[RTU] poll task started\r\n",
                    (uint16_t)strlen("[RTU] poll task started\r\n"), HAL_MAX_DELAY);

  for (;;)
  {
    uint32_t t0 = HAL_GetTick();   /* 本轮开始时刻（用于统计整轮耗时） */
    s_alive_ms = t0;
    s_cycle_cnt++;                 /* 轮次 +1（降级退避按轮次调度） */
    mbrtu_rx_heal();               /* 每轮自愈一次 UART 接收 */

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
      uint16_t base = slv[s].base;   /* 合并段的起始地址 */
      uint16_t span = slv[s].span;   /* 合并段的寄存器个数 */
      uint8_t  ok_all = 0;           /* 本台是否读到（1=成功） */
      int r = -1, a;                 /* r=读结果；a=重试计数 */
      uint32_t gap = 1;              /* 失败后下次轮询的间隔轮数（退避用） */

#if (DEBUG_ONLY_SLAVE > 0)
      if (slv[s].addr != (DEBUG_ONLY_SLAVE)) continue;   /* 只测指定从机 */
#endif

      /* 降级退避：未到这台设备的下次轮询轮次就跳过，避免坏从机拖慢整轮 */
      if ((int32_t)(s_cycle_cnt - slv[s].next_cycle) < 0) continue;

      s_alive_ms = HAL_GetTick();

      /* 一次读完这台设备的整段（含中间空洞） */
      for (a = 0; a < RETRY_MAX; a++)
      {
        r = mbrtu_read(slv[s].addr, 0x03, base, span, tmp, READ_TIMEOUT_MS);
        if (r == 0) break;
        osDelay(RETRY_DELAY_MS);
      }

      if (r == 0)
      {
        osMutexAcquire(s_regMutex, osWaitForever);
        memcpy(&g_slave_regs[slv[s].addr - 1][base], tmp, (size_t)span * 2);   /* 更新镜像 */
        decode_dev(s, tmp, base);                                              /* 更新点表 */
        osMutexRelease(s_regMutex);
        ok_all = 1;
      }
      else
      {
        mark_dev_bad(s);                                                       /* 标失败 */
      }

      /* 更新在线/失败计数 + 安排下次轮询轮次 */
      if (ok_all)
      {
        slv[s].online   = 1;
        slv[s].fail_cnt = 0;
        slv[s].next_cycle = s_cycle_cnt + 1;                 /* 正常：每轮都轮询 */
        g_slave_online[slv[s].addr] = 1;
      }
      else
      {
        slv[s].online = 0;
        if (slv[s].fail_cnt < 255) slv[s].fail_cnt++;        /* 连续失败 +1 */
        /* 分级退避：失败越多，下次轮询间隔越大 */
        gap = (slv[s].fail_cnt >= BACKOFF_L3) ? 12 :
              (slv[s].fail_cnt >= BACKOFF_L2) ? 5  :
              (slv[s].fail_cnt >= BACKOFF_L1) ? 2  : 1;
        slv[s].next_cycle = s_cycle_cnt + gap;
        g_slave_online[slv[s].addr] = 0;
      }

#if GW_DEBUG_SERIAL
      osMutexAcquire(s_regMutex, osWaitForever);
      gw_print_slave(s);
      osMutexRelease(s_regMutex);
#endif
      osDelay(POLL_SLAVE_GAP_MS);
    }

    s_cycle_ms = HAL_GetTick() - t0;   /* 记录整轮耗时 */

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
    说明：一次应答只有几百字节，稳稳小于 TCP 发送缓冲，不会卡。
*/

/**
  * @brief  非阻塞发送，直到发完或超时
  * @param  fd   已连接的 socket（非阻塞）
  * @param  buf  数据
  * @param  len  长度
  * @retval 实际发送字节数
  */
static int dump_send_all(int fd, const uint8_t *buf, int len)
{
  int sent = 0, spin = 0;
  while (sent < len)
  {
    int w = lwip_send(fd, buf + sent, (size_t)(len - sent), 0);
    if (w > 0) { sent += w; spin = 0; continue; }
    if (w == 0) break;
    if (++spin > 1000) break;   /* 约 5s 仍发不完则放弃 */
    osDelay(5);
  }
  return sent;
}

/**
  * @brief  非阻塞收 1 字节，带超时
  * @param  fd          已连接的 socket
  * @param  b           输出
  * @param  timeout_ms  超时（毫秒）
  * @retval 1=收到 0=对端关闭 -1=超时
  */
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

/**
  * @brief  文本导出任务：监听 5000，按设备序号把该设备的点发出去（任务永不返回）
  * @param  argument  未使用
  * @retval 无
  * @note   一次只服务一个客户端；同一连接可连续请求多台设备；空闲 5s 自动断开。
  */
void mbgw_dump_task(void *argument)
{
  static char dumpbuf[DUMP_BUF_SIZE];   /* 单台设备的文本缓冲（静态，别放栈上） */
  int listen_fd = -1;
  struct sockaddr_in a;

  (void)argument;

  /* 建监听套接字（端口可能还没就绪，失败就重试） */
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
    lwip_ioctl(c, FIONBIO, &fl);   /* 客户端套接字设为非阻塞 */

    ndev = (uint8_t)SLAVE_NUM;
    dump_send_all(c, &ndev, 1);    /* 先告诉客户端有几台设备 */

    for (;;)
    {
      uint8_t req;
      int r, n;

      r = dump_recv_byte(c, &req, 5000);   /* 等客户端要哪台设备 */
      if (r != 1) break;                   /* 超时或对端关闭 */
      if (req >= SLAVE_NUM) break;

      osMutexAcquire(s_regMutex, osWaitForever);
      n = cache_dump_dev(dumpbuf, sizeof(dumpbuf), req);
      osMutexRelease(s_regMutex);

      {
        uint8_t hdr[4];
        hdr[0] = (uint8_t)((uint32_t)n >> 24);   /* 先发 4 字节长度(大端) */
        hdr[1] = (uint8_t)((uint32_t)n >> 16);
        hdr[2] = (uint8_t)((uint32_t)n >> 8);
        hdr[3] = (uint8_t)(n);

        dump_send_all(c, hdr, 4);
        if (n > 0) dump_send_all(c, (const uint8_t *)dumpbuf, n);
      }
    }

    lwip_close(c);
  }
}
