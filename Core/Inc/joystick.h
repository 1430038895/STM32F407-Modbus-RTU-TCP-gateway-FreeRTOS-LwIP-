/**
  ******************************************************************************
  * @file    joystick.h
  * @brief   双轴 XY 摇杆（X/Y 模拟 + 中心按压开关）驱动
  * @note    依赖：CubeMX 的 hadc1（ADC1，1通道，单次）+ SW 输入脚
  ******************************************************************************
  */
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 方向 */
#define JOY_NONE    0
#define JOY_UP      1
#define JOY_DOWN    2
#define JOY_LEFT    3
#define JOY_RIGHT   4

/** @brief 初始化（ADC 由 CubeMX 初始化，这里基本不用做事；占位） */
void joystick_init(void);

/**
  * @brief  读 X/Y（已做多次平均），原始值 0..4095（12 位）
  * @param  x,y  输出
  */
void joystick_read(uint16_t *x, uint16_t *y);

/** @brief 立即判方向（已含多次平均 + 死区），返回 JOY_xxx */
uint8_t joystick_dir_raw(void);

/** @brief 稳定方向：连续 2 次同方向才算数，返回 JOY_xxx（含 NONE） */
uint8_t joystick_dir_stable(void);

/** @brief 方向"事件"：方向第一次稳定时返回一次该方向，其余返回 NONE（适合切换列表） */
uint8_t joystick_dir_event(void);

/** @brief 中心按压：按下返回 true（低电平有效） */
bool joystick_sw(void);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
