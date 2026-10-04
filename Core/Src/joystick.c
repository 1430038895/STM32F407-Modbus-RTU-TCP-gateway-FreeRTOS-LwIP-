/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   双轴 XY 摇杆驱动（ADC 采样 + 平均 + 死区 + 方向判定）
  * @note    X=PA0(ADC1_IN0)  Y=PA3(ADC1_IN3)  SW=PB8(输入+上拉)
  *          模块极性：左推 X 升、下推 Y 降 -> 已在这里翻转成自然方向
  ******************************************************************************
  */

#include "main.h"
#include "adc.h"
#include "joystick.h"

/* ================= 接线定义 ================= */
#define JOY_SW_PORT   GPIOB
#define JOY_SW_PIN    GPIO_PIN_8

#define JOY_CH_X      ADC_CHANNEL_0     /* PA0 */
#define JOY_CH_Y      ADC_CHANNEL_3     /* PA3 */

/* ================= 参数 ================= */
#define JOY_SAMPLES   8         /* 每次读的采样次数（求平均，去毛刺） */
#define JOY_CENTER    2048      /* 12 位中点 */
#define JOY_DEADZONE  500       /* 死区：偏移小于它算"居中" */

/** @brief 读一个 ADC 通道（单次转换） */
static uint16_t adc_read(uint32_t ch)
{
  ADC_ChannelConfTypeDef cfg = {0};
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

void joystick_init(void)
{
  /* ADC 在 CubeMX 生成的 MX_ADC1_Init 里已初始化，这里无需额外动作 */
}

void joystick_read(uint16_t *x, uint16_t *y)
{
  uint32_t sx = 0, sy = 0;

  for (uint8_t i = 0; i < JOY_SAMPLES; i++)
  {
    sx += adc_read(JOY_CH_X);
    sy += adc_read(JOY_CH_Y);
  }
  *x = (uint16_t)(sx / JOY_SAMPLES);
  *y = (uint16_t)(sy / JOY_SAMPLES);
}

static int joy_abs(int v) { return (v < 0) ? -v : v; }

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
    return JOY_NONE;

  if (joy_abs(dx) > joy_abs(dy))
    return (dx > 0) ? JOY_RIGHT : JOY_LEFT;
  return (dy > 0) ? JOY_UP : JOY_DOWN;
}

uint8_t joystick_dir_stable(void)
{
  static uint8_t last = JOY_NONE;
  static uint8_t cnt  = 0;
  uint8_t d = joystick_dir_raw();

  if (d == last) { if (cnt < 255) cnt++; }
  else           { last = d; cnt = 1; }

  return (cnt >= 2) ? d : JOY_NONE;   /* 连续 2 次同向才认 */
}

uint8_t joystick_dir_event(void)
{
  static uint8_t reported = JOY_NONE;
  uint8_t d = joystick_dir_stable();

  if (d == JOY_NONE) { reported = JOY_NONE; return JOY_NONE; }
  if (d != reported) { reported = d; return d; }   /* 新方向：只报一次 */
  return JOY_NONE;
}

bool joystick_sw(void)
{
  /* 按下导通到 GND -> 低电平；内部上拉 */
  return (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_RESET);
}
