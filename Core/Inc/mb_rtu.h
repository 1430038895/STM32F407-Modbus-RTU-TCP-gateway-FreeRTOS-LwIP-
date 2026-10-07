/**
  ******************************************************************************
  * @file    mb_rtu.h
  * @brief   Modbus RTU 主站（下行侧）对外接口：RS485 收发 + CRC + 主站读写
  * @note    面向对象是"从机"；对外只暴露：初始化、主站读/写、诊断/自愈。
  ******************************************************************************
  */
#ifndef __MB_RTU_H
#define __MB_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  初始化 RTU 模块
  * @param  无
  * @retval 无
  * @note   创建总线互斥锁，打开 UART5 全局中断并启动单字节中断接收。
  *         必须在创建任务之前调用。
  */
void mbrtu_init(void);

/**
  * @brief  Modbus RTU 主站：读寄存器（0x03 保持 / 0x04 输入）
  * @param  slave       从机地址（1~247）
  * @param  func        功能码：0x03=保持寄存器，0x04=输入寄存器
  * @param  start       起始寄存器地址
  * @param  count       读取的寄存器个数（1~125）
  * @param  out         输出缓冲，长度至少 count，存放读到的 16 位数
  * @param  timeout_ms  等待应答首个字节的超时（毫秒）
  * @retval 0   成功
  *         -1  发送失败
  *         -2  应答太短 / 超时（从机没应答）
  *         -3  从机地址不符
  *         -4  CRC 错误
  *         -5  从机返回异常应答
  *         -6  功能码不符
  *         -7  字节数不符
  */
int mbrtu_read(uint8_t slave, uint8_t func, uint16_t start, uint16_t count,
               uint16_t *out, uint32_t timeout_ms);

/**
  * @brief  Modbus RTU 主站：写单个保持寄存器（功能码 0x06）
  * @param  slave       从机地址（1~247）
  * @param  reg         寄存器地址
  * @param  value       要写入的 16 位值
  * @param  timeout_ms  等待应答超时（毫秒）
  * @retval 0=成功；负值同 mbrtu_read（-1..-6）
  */
int mbrtu_write_single(uint8_t slave, uint16_t reg, uint16_t value, uint32_t timeout_ms);

/**
  * @brief  Modbus RTU 主站：写多个保持寄存器（功能码 0x10）
  * @param  slave       从机地址（1~247）
  * @param  reg         起始寄存器地址
  * @param  count       寄存器个数（1~123）
  * @param  values      要写入的值数组（长度 >= count）
  * @param  timeout_ms  等待应答超时（毫秒）
  * @retval 0=成功；负值同 mbrtu_read（-1..-6）；-8=参数非法
  */
int mbrtu_write_multiple(uint8_t slave, uint16_t reg, uint16_t count,
                         const uint16_t *values, uint32_t timeout_ms);

/**
  * @brief  诊断：中断累计收到的字节数
  * @param  无
  * @retval 累计字节数
  */
uint32_t mbrtu_rx_total(void);

/**
  * @brief  接收自愈：长时间运行后重新挂上 UART 接收（防卡死）
  * @param  无
  * @retval 无
  */
void mbrtu_rx_heal(void);

#ifdef __cplusplus
}
#endif

#endif /* __MB_RTU_H */
