/**
  ******************************************************************************
  * @file    joystick.h
  * @brief   双轴 XY 摇杆（X/Y 模拟 + 中心按压开关）对外接口
  * @note    依赖：CubeMX 的 hadc1（ADC1）+ SW 输入脚。
  *          原始值范围 0..4095（12 位），中点约 2048。
  ******************************************************************************
  */
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 方向取值 */
#define JOY_NONE    0   /**< 居中/无方向 */
#define JOY_UP      1   /**< 上 */
#define JOY_DOWN    2   /**< 下 */
#define JOY_LEFT    3   /**< 左 */
#define JOY_RIGHT   4   /**< 右 */

/**
  * @brief  初始化摇杆（ADC 由 CubeMX 初始化，本函数基本为空，占位）
  * @param  无
  * @retval 无
  */
void joystick_init(void);

/**
  * @brief  读 X/Y（已做多次平均）
  * @param  x,y  输出：原始值 0..4095
  * @retval 无
  */
void joystick_read(uint16_t *x, uint16_t *y);

/**
  * @brief  立即判方向（已含平均 + 死区）
  * @param  无
  * @retval JOY_xxx
  */
uint8_t joystick_dir_raw(void);

/**
  * @brief  稳定方向：连续 2 次同方向才算数
  * @param  无
  * @retval JOY_xxx（含 JOY_NONE）
  */
uint8_t joystick_dir_stable(void);

/**
  * @brief  方向"事件"：方向第一次稳定时返回一次该方向，其余返回 NONE
  * @param  无
  * @retval JOY_xxx（新方向）或 JOY_NONE
  */
uint8_t joystick_dir_event(void);

/**
  * @brief  读中心按键
  * @param  无
  * @retval true=按下；false=松开
  */
bool joystick_sw(void);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
