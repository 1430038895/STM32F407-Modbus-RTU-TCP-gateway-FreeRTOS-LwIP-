/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   双轴 XY 摇杆驱动（ADC 采样 + 多次平均 + 死区 + 方向判定）
  * @note    接线：X=PA0(ADC1_IN0)  Y=PA3(ADC1_IN3)  SW=PB8(输入+内部上拉)
  *          模块极性：左推 X 电压升高、下推 Y 电压降低 -> 这里已翻转成自然方向。
  *          读取流程：多次采样求平均 -> 与中点比较 -> 死区过滤 -> 取偏离大的轴作方向。
  ******************************************************************************
  */

#include "main.h"
#include "adc.h"
#include "joystick.h"

/* ================= 接线定义 ================= */
#define JOY_SW_PORT   GPIOB          /**< 中心按键 SW 所在端口 */
#define JOY_SW_PIN    GPIO_PIN_8     /**< SW 引脚（按下=低电平） */

#define JOY_CH_X      ADC_CHANNEL_0  /**< X 轴通道：PA0 = ADC1_IN0 */
#define JOY_CH_Y      ADC_CHANNEL_3  /**< Y 轴通道：PA3 = ADC1_IN3 */

/* ================= 参数 ================= */
#define JOY_SAMPLES   8         /**< 每次读的采样次数（求平均，抑制毛刺） */
#define JOY_CENTER    2048      /**< 12 位 ADC 的理论中点（约 VCC/2） */
#define JOY_DEADZONE  500       /**< 死区：|偏移| 小于它就算"居中"，避免抖动乱跳 */

/**
  * @brief  读一个 ADC 通道（单次转换）
  * @param  ch  ADC 通道（如 ADC_CHANNEL_0）
  * @retval 12 位 ADC 原始值（0..4095）
  * @note   每次读前重设通道（CubeMX 只配了一个通道，这里动态切换）。
  */
static uint16_t adc_read(uint32_t ch)
{
  ADC_ChannelConfTypeDef cfg = {0};   /* 通道配置结构（清零） */
  uint16_t v;

  cfg.Channel      = ch;
  cfg.Rank         = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &cfg);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

/**
  * @brief  初始化摇杆（ADC 已由 CubeMX 的 MX_ADC1_Init 初始化，此函数留空占位）
  * @param  无
  * @retval 无
  */
void joystick_init(void)
{
  /* ADC 在 CubeMX 生成的 MX_ADC1_Init 里已初始化，这里无需额外动作 */
}

/**
  * @brief  读摇杆 X/Y（已做 JOY_SAMPLES 次平均）
  * @param  x,y  输出：X、Y 的原始值 0..4095
  * @retval 无
  */
void joystick_read(uint16_t *x, uint16_t *y)
{
  uint32_t sx = 0, sy = 0;   /* X/Y 累加和 */

  for (uint8_t i = 0; i < JOY_SAMPLES; i++)
  {
    sx += adc_read(JOY_CH_X);
    sy += adc_read(JOY_CH_Y);
  }
  *x = (uint16_t)(sx / JOY_SAMPLES);
  *y = (uint16_t)(sy / JOY_SAMPLES);
}

/** @brief 整数取绝对值（内部用） */
static int joy_abs(int v) { return (v < 0) ? -v : v; }

/**
  * @brief  立即判定方向（已做平均 + 死区）
  * @param  无
  * @retval JOY_UP / JOY_DOWN / JOY_LEFT / JOY_RIGHT / JOY_NONE
  */
uint8_t joystick_dir_raw(void)
{
  uint16_t x, y;
  int dx, dy;

  joystick_read(&x, &y);

  /* 模块极性：左推 X 升、下推 Y 降。
     换算成"自然方向"：右/上 为正 —— 所以 X 取反、Y 不取反。 */
  dx = (int)JOY_CENTER - (int)x;    /* 右为正 */
  dy = (int)y - (int)JOY_CENTER;    /* 上为正 */

  if (joy_abs(dx) < JOY_DEADZONE && joy_abs(dy) < JOY_DEADZONE)
    return JOY_NONE;                /* 在死区内 = 居中 */

  if (joy_abs(dx) > joy_abs(dy))    /* 取偏离更大的那个轴 */
    return (dx > 0) ? JOY_RIGHT : JOY_LEFT;
  return (dy > 0) ? JOY_UP : JOY_DOWN;
}

/**
  * @brief  稳定方向：连续 2 次同方向才算数
  * @param  无
  * @retval JOY_xxx（含 JOY_NONE）
  * @note   用静态变量记录上次方向与连续次数；不足 2 次时返回 NONE。
  */
uint8_t joystick_dir_stable(void)
{
  static uint8_t last = JOY_NONE;   /* 上一次的原始方向 */
  static uint8_t cnt  = 0;          /* 与上次相同的连续次数 */
  uint8_t d = joystick_dir_raw();

  if (d == last) { if (cnt < 255) cnt++; }
  else           { last = d; cnt = 1; }

  return (cnt >= 2) ? d : JOY_NONE;   /* 连续 2 次同向才认 */
}

/**
  * @brief  方向"事件"：方向第一次稳定时返回一次，之后保持不动只返回 NONE
  * @param  无
  * @retval JOY_xxx（只在"新方向"时给出一次；其余为 JOY_NONE）
  * @note   适合"切换列表选择"这类每次动作只触发一次的场景。
  */
uint8_t joystick_dir_event(void)
{
  static uint8_t reported = JOY_NONE;   /* 已上报过的方向 */
  uint8_t d = joystick_dir_stable();

  if (d == JOY_NONE) { reported = JOY_NONE; return JOY_NONE; }
  if (d != reported) { reported = d; return d; }   /* 新方向：只报一次 */
  return JOY_NONE;
}

/**
  * @brief  读中心按键
  * @param  无
  * @retval true=按下；false=松开
  * @note   按下时 SW 导通到 GND（低电平），依赖内部上拉。
  */
bool joystick_sw(void)
{
  return (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_RESET);
}
