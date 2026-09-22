/**
  ******************************************************************************
  * @file    mb_gateway.h
  * @brief   网关应用层：寄存器缓存 + 设备点表 + 轮询/写穿透/单元号路由
  ******************************************************************************
  */
#ifndef __MB_GATEWAY_H
#define __MB_GATEWAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MB_MAX_SLAVE   8     /**< 单元号路由：支持从机号 1..8 */
#define MB_MAX_REGS    64    /**< 每个从机缓存寄存器地址 0..63 */

/** 采集数据的类型（决定占几个寄存器、如何解码） */
typedef enum
{
  DT_U16 = 0,    /**< 无符号 16 位，占 1 个寄存器 */
  DT_S16,        /**< 有符号 16 位，占 1 个寄存器 */
  DT_F32_ABCD,   /**< 32 位浮点，高字在前，占 2 个寄存器 */
  DT_F32_CDAB,   /**< 32 位浮点，低字在前，占 2 个寄存器 */
} mb_dtype_t;

/**
  * @brief  初始化网关模块（创建缓存互斥锁）
  * @param  无
  * @retval 无
  * @note   必须在创建任务前调用。
  */
void mbgw_init(void);

/**
  * @brief  轮询任务体：按点表采集所有从机（任务永不返回）
  * @param  argument  未使用
  * @retval 无
  */
void mbgw_poll_task(void *argument);

/**
  * @brief  查询某从机是否在线
  * @param  slave  从机号（1..MB_MAX_SLAVE）
  * @retval 1=在线；0=离线
  */
uint8_t mbgw_is_online(uint8_t slave);

/**
  * @brief  读取某从机的寄存器缓存（调用者须先确保地址范围合法）
  * @param  slave  从机号
  * @param  start  起始寄存器地址
  * @param  qty    数量
  * @param  out    输出缓冲（长度 >= qty）
  * @retval 无
  */
void mbgw_read(uint8_t slave, uint16_t start, uint16_t qty, uint16_t *out);

/**
  * @brief  写穿透：写单个寄存器到从机，成功后更新缓存
  * @param  slave  从机号
  * @param  addr   寄存器地址
  * @param  val    写入值
  * @retval 0 成功；负值=RTU 写失败
  */
int mbgw_write_single(uint8_t slave, uint16_t addr, uint16_t val);

/**
  * @brief  写穿透：写多个寄存器到从机，成功后更新缓存
  * @param  slave  从机号
  * @param  start  起始寄存器地址
  * @param  qty    数量
  * @param  vals   值数组
  * @retval 0 成功；负值=RTU 写失败
  */
int mbgw_write_multiple(uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals);

/**
  * @brief  获取轮询任务最近一次的心跳时间戳（供看门狗判断其是否卡死）
  * @param  无
  * @retval HAL_GetTick() 时间戳
  */
uint32_t mbgw_last_alive_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __MB_GATEWAY_H */
