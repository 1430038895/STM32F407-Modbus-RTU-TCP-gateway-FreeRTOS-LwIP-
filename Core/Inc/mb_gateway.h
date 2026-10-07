/**
  ******************************************************************************
  * @file    mb_gateway.h
  * @brief   网关应用层对外接口：全量寄存器镜像 + 点表 + 质量/时间戳 + 三种出口
  *
  * @note    数据结构（描述表/点表/镜像）都在 mb_gateway.c 内部，本头文件只暴露接口：
  *            - 生命周期：mbgw_init / mbgw_poll_task / mbgw_dump_task
  *            - 按“从机+寄存器”  ：mbgw_read / mbgw_write_single / mbgw_write_multiple（TCP 透传）
  *            - 按“点”            ：dp_*（给屏幕/调试/导出）
  *            - 从机信息/变更通知 ：gw_*
  ******************************************************************************
  */
#ifndef __MB_GATEWAY_H
#define __MB_GATEWAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---------- 原始数据类型（决定一个点占几个寄存器、怎么解释） ---------- */
enum
{
  T_U16 = 0,   /**< 无符号 16 位（占 1 个寄存器） */
  T_I16,       /**< 有符号 16 位（占 1 个寄存器） */
  T_U32,       /**< 无符号 32 位（占 2 个寄存器，高字在前） */
  T_U32_DEC,   /**< 累积量：高字/低字/小数部分(单位0.01)（占 3 个寄存器） */
  T_F32,       /**< 浮点原始字节（占 2 个寄存器；只存原始字节，不做 float 运算） */
};

/* ---------- 容量上限 ---------- */
#define MB_MAX_SLAVE   8     /**< 从机号 1..8：镜像按此分区 */
#define MB_MAX_REGS    64    /**< 每个从机的寄存器地址范围 0..63 */
#define MAX_SLAVES     8     /**< 描述表里从机数上限 */
#define MAX_PT         96    /**< 数据点总上限（当前用了 78） */

/* ---------- 读出口：整数/定点放在这里；F32 只给原始字节 ---------- */
typedef struct
{
  int64_t scaled;      /**< 已乘倍率的十进制整数值（真值 = scaled / 10^decimals） */
  uint8_t decimals;    /**< 小数位数 */
  uint8_t is_f32;      /**< 1 = raw[] 里是 4 字节浮点原始值（scaled 无效） */
  uint8_t raw[4];      /**< F32 的原始字节（高字在前） */
} dp_out_t;

/* ---------- 生命周期 ---------- */

/**
  * @brief  初始化网关：创建互斥锁/事件标志 + 解析描述表生成点表
  * @param  无
  * @retval 无
  * @note   必须在创建任务前调用（放在 MX_FREERTOS_Init 里）。
  */
void mbgw_init(void);

/**
  * @brief  RS485 轮询任务体（任务永不返回）
  * @param  argument  未使用
  * @retval 无
  */
void mbgw_poll_task(void *argument);

/**
  * @brief  文本导出任务体（TCP 5000，任务永不返回）
  * @param  argument  未使用
  * @retval 无
  */
void mbgw_dump_task(void *argument);

/**
  * @brief  取轮询任务的最近心跳时间（给看门狗判活）
  * @param  无
  * @retval HAL_GetTick 时间戳（毫秒）
  */
uint32_t mbgw_last_alive_ms(void);

/**
  * @brief  查询某从机是否在线
  * @param  slave  从机地址（1..MB_MAX_SLAVE）
  * @retval 1=在线；0=离线/地址非法
  */
uint8_t  mbgw_is_online(uint8_t slave);

/* ---------- 变更通知（给屏幕：值一变就通知，不用轮询） ---------- */

/**
  * @brief  阻塞等待"有点变了"事件
  * @param  timeout_ms  最长等待（毫秒），osWaitForever 表示一直等
  * @retval 事件标志（含 GW_CHG_BIT=等到）或超时/错误码
  */
uint32_t gw_wait_changed(uint32_t timeout_ms);

/**
  * @brief  取走当前所有"变了的点号"，并清脏位
  * @param  out  输出点号数组
  * @param  max  最多取多少个
  * @retval 实际取出的个数
  */
uint16_t gw_take_changed(uint16_t *out, uint16_t max);

/* ---------- 按"从机号 + 寄存器地址"（Modbus TCP 透传用） ---------- */

/**
  * @brief  读一段寄存器（从镜像取）
  * @param  slave  从机地址
  * @param  start  起始寄存器地址
  * @param  qty    数量
  * @param  out    输出缓冲（长度 >= qty）
  * @retval 无
  */
void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out);

/**
  * @brief  写穿透：写单个寄存器（先写设备，成功后更新镜像）
  * @param  slave  从机地址
  * @param  addr   寄存器地址
  * @param  val    值
  * @retval 0=成功；-1=参数非法或 RTU 写失败
  */
int  mbgw_write_single(uint8_t slave, uint16_t addr, uint16_t val);

/**
  * @brief  写穿透：写多个寄存器
  * @param  slave  从机地址
  * @param  start  起始寄存器地址
  * @param  qty    数量
  * @param  vals   值数组（长度 >= qty）
  * @retval 0=成功；-1=参数非法或 RTU 写失败
  */
int  mbgw_write_multiple(uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals);

/* ---------- 按"点"（给屏幕/调试/导出） ---------- */

/**
  * @brief  按"设备地址 + 点名"查点号
  * @param  addr  从机地址
  * @param  name  点名
  * @retval 点号(>=0)；-1=没找到
  */
int         dp_find(uint8_t addr, const char *name);

/** @brief 总点数 */
uint16_t    dp_count(void);

/** @brief 某点质量：0=没读过 1=好 2=失败（越界按 2） */
uint8_t     dp_quality(uint16_t idx);

/** @brief 距最近一次成功读到的毫秒数（越界返回 0xFFFFFFFF） */
uint32_t    dp_age_ms(uint16_t idx);

/** @brief 某点原始类型 T_xxx（越界返回 0xFF） */
uint8_t     dp_kind(uint16_t idx);

/** @brief 点名（越界返回空串） */
const char *dp_name(uint16_t idx);

/** @brief 点所属设备名（越界返回空串） */
const char *dp_dev_name(uint16_t idx);

/** @brief 点所属设备地址（越界返回 0） */
uint8_t     dp_dev_addr(uint16_t idx);

/**
  * @brief  取某点原始读数
  * @param  idx  点号
  * @param  out  输出（整数给 scaled/decimals；F32 给 raw[4]）
  * @retval true=成功；false=越界/类型未知
  */
bool        dp_read(uint16_t idx, dp_out_t *out);

/**
  * @brief  把 dp_out_t 格式化成字符串（F32 -> "F32:xxxxxxxx"，其余 -> 十进制）
  * @param  v    dp_read 的输出
  * @param  buf  输出缓冲
  * @retval 写入长度
  */
int         dp_format(const dp_out_t *v, char *buf);

/**
  * @brief  取某点的十进制定点值（统一接口，F32 也解码）
  * @param  idx       点号
  * @param  value     输出：真值 = (*value) / 10^decimals
  * @param  decimals  目标小数位
  * @retval true=成功；false=越界
  */
bool        dp_read_num(uint16_t idx, int64_t *value, uint8_t decimals);

/**
  * @brief  直接把某点格式化成十进制字符串（F32 也换算）
  * @param  idx       点号
  * @param  buf       输出缓冲
  * @param  decimals  小数位
  * @retval 写入长度（0=点号非法）
  */
int         dp_format_num(uint16_t idx, char *buf, uint8_t decimals);

/**
  * @brief  把所有点拼成 CSV 文本
  * @param  buf  输出缓冲
  * @param  cap  缓冲容量
  * @retval 写入长度
  */
int         cache_dump(char *buf, int cap);

/* ---------- 按从机序号访问（给屏幕用） ---------- */

/** @brief 从机台数 */
uint8_t     gw_slave_count(void);

/** @brief 第 i 台的设备名 */
const char *gw_slave_name(uint8_t i);

/** @brief 第 i 台的从机地址 */
uint8_t     gw_slave_addr(uint8_t i);

/** @brief 第 i 台是否在线（1=在线） */
uint8_t     gw_slave_online(uint8_t i);

/** @brief 第 i 台的连续失败次数（成功清零） */
uint8_t     gw_slave_fail(uint8_t i);

/** @brief 最近一轮完整轮询耗时（毫秒） */
uint32_t    gw_cycle_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __MB_GATEWAY_H */
