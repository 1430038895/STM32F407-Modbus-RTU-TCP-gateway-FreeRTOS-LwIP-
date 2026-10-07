/**
  ******************************************************************************
  * @file    st7789.h
  * @brief   ST7789 SPI TFT 驱动（240x320，4 线 SPI，只写）对外接口
  * @note    依赖：CubeMX 生成的 hspi1（SPI1）+ 4 个控制脚（CS/DC/RST/BLK）。
  *          颜色统一用 RGB565（16 位：R5 G6 B5）。
  ******************************************************************************
  */
#ifndef __ST7789_H
#define __ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ST7789_WIDTH    240    /**< 屏幕宽（像素） */
#define ST7789_HEIGHT   320    /**< 屏幕高（像素） */

/* 常用颜色（RGB565） */
#define ST7789_BLACK    0x0000 /**< 黑 */
#define ST7789_WHITE    0xFFFF /**< 白 */
#define ST7789_RED      0xF800 /**< 红 */
#define ST7789_GREEN    0x07E0 /**< 绿 */
#define ST7789_BLUE     0x001F /**< 蓝 */

/**
  * @brief 初始化 ST7789：硬件复位 + 上电寄存器序列 + 开背光
  * @param  无
  * @retval 无
  */
void st7789_init(void);

/**
  * @brief 初始化 SPI 发送 DMA（须在 RTOS 内核启动后调用一次）
  * @param  无
  * @retval 无
  */
void st7789_dma_init(void);

/**
  * @brief  设置显示方向
  * @param  rot  0=竖屏(240x320), 1=横屏(320x240), 2=竖屏翻转, 3=横屏翻转
  * @retval 无
  */
void st7789_set_rotation(uint8_t rot);

/**
  * @brief  背光开关
  * @param  on  true=亮，false=灭
  * @retval 无
  */
void st7789_backlight(bool on);

/**
  * @brief  整屏填充一个颜色
  * @param  color  RGB565 颜色
  * @retval 无
  */
void st7789_fill(uint16_t color);

/**
  * @brief  填充一块矩形区域
  * @param  x,y    左上角坐标
  * @param  w,h    宽、高（像素）
  * @param  color  RGB565 颜色
  * @retval 无
  */
void st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
  * @brief  画一块 RGB565 像素（以后 LVGL 的 flush 回调就用它）
  * @param  x,y  左上角坐标
  * @param  w,h  宽、高（像素）
  * @param  pix  像素数组（RGB565，长度 = w*h）
  * @retval 无
  */
void st7789_draw_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pix);

/**
  * @brief  画一个字符（8x16 字模，可放大）
  * @param  x,y    左上角
  * @param  c      字符(ASCII 32..126，其它按空格)
  * @param  fg,bg  前景/背景色
  * @param  scale  放大倍数(1..3)
  * @retval 无
  */
void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);

/**
  * @brief  画字符串（不换行）
  * @param  x,y    左上角
  * @param  s      字符串
  * @param  fg,bg  前景/背景色
  * @param  scale  放大倍数(1..3)
  * @retval 无
  */
void st7789_draw_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);

/**
  * @brief  画一串 Unicode 码点（ASCII 走 8x16，中文走 16x16 字模）
  * @param  x,y    左上角
  * @param  codes  码点数组
  * @param  n      个数
  * @param  fg,bg  前景/背景色
  * @retval 无
  */
void st7789_draw_codes(uint16_t x, uint16_t y, const uint16_t *codes, uint8_t n, uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* __ST7789_H */
