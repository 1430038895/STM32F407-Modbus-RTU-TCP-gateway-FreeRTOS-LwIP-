/**
  ******************************************************************************
  * @file    mb_gateway.h
  * @brief   网关应用层：全量寄存器镜像 + 点表(描述表) + 质量/时间戳 + 出口
  * @note    B 方案：照旧把所有寄存器存下来、轮询更新（TCP 透传不变）；
  *          在上面再加一层"按点"的接口（dp_*）供屏幕/导出使用。
  ******************************************************************************
  */
#ifndef __MB_GATEWAY_H
#define __MB_GATEWAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---------- 原始数据类型（放进 pt[] 存的内容） ---------- */
enum {
  T_U16 = 0,   /**< 无符号 16 位（1 寄存器） */
  T_I16,       /**< 有符号 16 位（1 寄存器） */
  T_U32,       /**< 无符号 32 位（2 寄存器，高字在前） */
  T_U32_DEC,   /**< 累积量：高字/低字/小数部分(0.01)（3 寄存器） */
  T_F32,       /**< 浮点原始字节（2 寄存器，不做 float 运算） */
};

/* ---------- 容量 ---------- */
#define MB_MAX_SLAVE   8     /**< 从机号 1..8：缓存/镜像按此分区 */
#define MB_MAX_REGS    64    /**< 每个从机缓存寄存器地址 0..63 */
#define MAX_SLAVES     8     /**< 描述表里从机数上限 */
#define MAX_PT         96    /**< 数据点上限 */

/* ---------- 读出口：整数/定点在这里，F32 只给原始字节 ---------- */
typedef struct {
  int64_t scaled;      /**< 已乘倍率的十进制整数值 */
  uint8_t decimals;    /**< 小数位数 */
  uint8_t is_f32;      /**< 1 = raw 里是 4 字节浮点原始值 */
  uint8_t raw[4];      /**< F32 的原始字节（高字在前） */
} dp_out_t;

/* ---------- 生命周期 ---------- */
void     mbgw_init(void);
void     mbgw_poll_task(void *argument);
void     mbgw_dump_task(void *argument);
uint32_t mbgw_last_alive_ms(void);
uint8_t  mbgw_is_online(uint8_t slave);

/* ---------- 变更通知（给屏幕：值一变就通知，不用轮询） ---------- */
uint32_t gw_wait_changed(uint32_t timeout_ms);            /**< 阻塞等“有点变了” */
uint16_t gw_take_changed(uint16_t *out, uint16_t max);    /**< 取走变了的点号并清零 */

/* ---------- 旧接口：按“从机号 + 寄存器地址”（TCP 透传用，保持不变） ---------- */
void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out);
int  mbgw_write_single(uint8_t slave, uint16_t addr, uint16_t val);
int  mbgw_write_multiple(uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals);

/* ---------- 新接口：按“点” ---------- */
int         dp_find(uint8_t addr, const char *name);
uint16_t    dp_count(void);
uint8_t     dp_quality(uint16_t idx);
uint32_t    dp_age_ms(uint16_t idx);
uint8_t     dp_kind(uint16_t idx);
const char *dp_name(uint16_t idx);
const char *dp_dev_name(uint16_t idx);
uint8_t     dp_dev_addr(uint16_t idx);
bool        dp_read(uint16_t idx, dp_out_t *out);
int         dp_format(const dp_out_t *v, char *buf);
bool        dp_read_num(uint16_t idx, int64_t *value, uint8_t decimals);
int         dp_format_num(uint16_t idx, char *buf, uint8_t decimals);
int         cache_dump(char *buf, int cap);

/* ---------- 按从机序号访问（给屏幕用） ---------- */
uint8_t     gw_slave_count(void);
const char *gw_slave_name(uint8_t i);
uint8_t     gw_slave_addr(uint8_t i);
uint8_t     gw_slave_online(uint8_t i);
uint8_t     gw_slave_fail(uint8_t i);
uint32_t    gw_cycle_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __MB_GATEWAY_H */
