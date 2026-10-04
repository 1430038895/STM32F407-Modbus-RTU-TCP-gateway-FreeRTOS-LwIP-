/**
  ******************************************************************************
  * @file    ui.h
  * @brief   自绘界面（不依赖 LVGL）：左 8 从机列表 + 右数据卡片
  ******************************************************************************
  */
#ifndef __UI_H
#define __UI_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化并画出整个界面 */
void ui_init(void);

/** @brief 处理一次摇杆事件（在主循环里周期性调用） */
void ui_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __UI_H */
