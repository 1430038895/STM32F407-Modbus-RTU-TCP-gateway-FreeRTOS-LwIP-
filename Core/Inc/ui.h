/**
  ******************************************************************************
  * @file    ui.h
  * @brief   自绘界面（不依赖 LVGL）对外接口
  * @note    左 8 从机列表 + 右数据（中文标签 + 数值）+ 右上角轮询耗时。
  ******************************************************************************
  */
#ifndef __UI_H
#define __UI_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化并整屏画一遍（清屏 + 标题 + 列表 + 数据）
  * @param  无
  * @retval 无
  */
void ui_init(void);

/**
  * @brief  处理一次摇杆 + 周期刷新（在主循环里周期性调用）
  * @param  无
  * @retval 无
  * @note   不要频繁调用；一般在任务里 osDelay(20) 调一次。
  */
void ui_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __UI_H */
