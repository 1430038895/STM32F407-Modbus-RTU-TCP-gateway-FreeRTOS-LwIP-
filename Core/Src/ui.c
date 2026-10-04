/**
  ******************************************************************************
  * @file    ui.c
  * @brief   自绘界面：左侧 8 从机（摇杆上下切换），右侧一行一个点（中文标签+数值）
  * @note    不依赖 LVGL；右侧只重画"变了的值"。
  ******************************************************************************
  */

#include "main.h"
#include "st7789.h"
#include "joystick.h"
#include "mb_gateway.h"
#include "zh_labels.h"
#include <string.h>
#include <stdio.h>

/* ================= 配色（RGB565） ================= */
#define C_BG      0x10A3   /* #101418 */
#define C_PANEL   0x1904   /* #1B2026 */
#define C_BORDER  0x2966   /* #2A2F36 */
#define C_TEXT    0xEF7D   /* #EFEFEF */
#define C_TEXT2   0xAD55   /* #AAAAAA */
#define C_BLUE    0x453E   /* #42A5F5 */
#define C_GREEN   0x4D6A   /* #4CAF50 */
#define C_RED     0xF206   /* #F44336 */
#define C_BLACK   0x0000   /* 黑点（失败过多） */
#define C_SEL     0x1A4E   /* #1E4976 */

/* ================= 布局 ================= */
#define SCR_W       240
#define SCR_H       320
#define TITLE_H     28

#define LIST_X      0
#define LIST_Y      30
#define LIST_W      100
#define ROW_H       34

#define RIGHT_X     104
#define RIGHT_Y     30
#define VALUE_X     (RIGHT_X + 70)     /* 数值起始 x */
#define ROW_STEP    20                 /* 每个点的行高 */

#define UI_MAX_SLAVES  8
#define UI_MAX_ROWS    16

/* ================= 运行期状态 ================= */
static uint8_t  s_sel       = 0;
static uint8_t  s_drawn_sel = 0xFF;
static uint8_t  s_drawn_online[UI_MAX_SLAVES];
static uint16_t s_row_idx[UI_MAX_ROWS];
static uint8_t  s_row_cnt   = 0;
static char     s_row_val[UI_MAX_ROWS][10];

/* ================= 小工具 ================= */

/* 收集当前从机所有点（含配置项） */
static uint8_t collect_rows(uint16_t *out)
{
  uint8_t addr = gw_slave_addr(s_sel);
  uint8_t n = 0;
  for (uint16_t idx = 0; idx < dp_count() && n < UI_MAX_ROWS; idx++)
  {
    if (dp_dev_addr(idx) != addr) continue;
    out[n++] = idx;
  }
  return n;
}

static void draw_cycle(void)
{
  char num[12];
  char b[16];
  snprintf(num, sizeof(num), "%lums", (unsigned long)gw_cycle_ms());
  snprintf(b, sizeof(b), "%-7s", num);          /* 固定宽度，原地覆盖，不闪 */
  st7789_draw_text(156, 6, b, C_GREEN, C_PANEL, 1);
}

static void draw_title(void)
{
  st7789_fill_rect(0, 0, SCR_W, TITLE_H, C_PANEL);
  st7789_draw_text(8, 6, "Modbus Gateway", C_TEXT, C_PANEL, 1);
  st7789_fill_rect(0, TITLE_H, SCR_W, 1, C_BORDER);
  draw_cycle();
}

/* ================= 左侧列表 ================= */
static void draw_row(uint8_t i, uint8_t selected)
{
  uint16_t y  = (uint16_t)(LIST_Y + i * ROW_H);
  uint16_t bg = selected ? C_SEL  : C_BG;
  uint16_t fg = selected ? C_TEXT : C_TEXT2;

  st7789_fill_rect(LIST_X, y, LIST_W, ROW_H, bg);
  if (selected)
    st7789_fill_rect(LIST_X, y, 4, ROW_H, C_BLUE);
  st7789_draw_text(LIST_X + 10, (uint16_t)(y + 9), gw_slave_name(i), fg, bg, 1);

  /* 右下角状态点：绿=在线；离线显示 N 个红点(1~4)；连续失败≥5 显示黑点 */
  {
    uint16_t doty = (uint16_t)(y + ROW_H - 9);
    uint16_t dotx = (uint16_t)(LIST_X + LIST_W - 9);
    uint8_t  fail = gw_slave_fail(i);

    if (gw_slave_online(i))
      st7789_fill_rect(dotx, doty, 6, 6, C_GREEN);
    else if (fail >= 5)
      st7789_fill_rect(dotx, doty, 6, 6, C_BLACK);
    else
    {
      uint8_t n = (fail == 0) ? 1 : fail;
      for (uint8_t k = 0; k < n; k++)
        st7789_fill_rect((uint16_t)(dotx - k * 7), doty, 5, 5, C_RED);
    }
  }
  st7789_fill_rect(LIST_X, (uint16_t)(y + ROW_H - 1), LIST_W, 1, C_BORDER);
}

static void draw_list(void)
{
  for (uint8_t i = 0; i < gw_slave_count(); i++)
  {
    draw_row(i, (uint8_t)(i == s_sel));
    s_drawn_online[i] = gw_slave_online(i);
  }
}

static void list_update(void)
{
  for (uint8_t i = 0; i < gw_slave_count(); i++)
  {
    uint8_t on = gw_slave_online(i);
    if (on != s_drawn_online[i])
    {
      draw_row(i, (uint8_t)(i == s_sel));
      s_drawn_online[i] = on;
    }
  }
}

/* ================= 右侧数据 ================= */
static void draw_label(uint16_t idx, uint16_t y)
{
  uint8_t len = 0;
  const uint16_t *codes = zh_label_lookup(dp_name(idx), &len);
  char nm[10];

  if (codes && len)
  {
    st7789_draw_codes(RIGHT_X + 4, y, codes, len, C_TEXT2, C_BG);
  }
  else
  {
    strncpy(nm, dp_name(idx), 9); nm[9] = 0;
    st7789_draw_text(RIGHT_X + 4, y, nm, C_TEXT2, C_BG, 1);
  }
}

static void draw_value(uint16_t idx, uint16_t slot, uint16_t y, uint8_t force)
{
  char val[16];
  char pad[12];
  dp_format_num(idx, val, 2); val[8] = 0;
  snprintf(pad, sizeof(pad), "%-8s", val);   /* 固定宽度，原地覆盖，不闪 */
  if (force || strcmp(pad, s_row_val[slot]) != 0)
  {
    st7789_draw_text(VALUE_X, y, pad, C_GREEN, C_BG, 1);
    strncpy(s_row_val[slot], pad, 9); s_row_val[slot][9] = 0;
  }
}

static void data_full(void)
{
  uint16_t y = (uint16_t)(RIGHT_Y + 22);

  s_row_cnt = collect_rows(s_row_idx);
  st7789_fill_rect(RIGHT_X, RIGHT_Y, (uint16_t)(SCR_W - RIGHT_X), (uint16_t)(SCR_H - RIGHT_Y), C_BG);
  st7789_draw_text(RIGHT_X + 4, RIGHT_Y + 2, gw_slave_name(s_sel), C_BLUE, C_BG, 1);
  st7789_fill_rect(RIGHT_X + 2, (uint16_t)(RIGHT_Y + 20), (uint16_t)(SCR_W - RIGHT_X - 4), 1, C_BORDER);

  for (uint8_t k = 0; k < s_row_cnt; k++)
  {
    draw_label(s_row_idx[k], y);
    draw_value(s_row_idx[k], k, y, 1);
    y = (uint16_t)(y + ROW_STEP);
  }
  if (s_row_cnt == 0)
    st7789_draw_text(RIGHT_X + 4, RIGHT_Y + 24, "(no data)", C_TEXT2, C_BG, 1);

  s_drawn_sel = s_sel;
}

static void data_update(void)
{
  uint16_t y = (uint16_t)(RIGHT_Y + 22);
  for (uint8_t k = 0; k < s_row_cnt; k++)
  {
    draw_value(s_row_idx[k], k, y, 0);
    y = (uint16_t)(y + ROW_STEP);
  }
}

/* ================= 对外 ================= */
void ui_init(void)
{
  st7789_fill(C_BG);
  draw_title();
  draw_list();
  data_full();
}

void ui_poll(void)
{
  static uint32_t last = 0;
  uint8_t d = joystick_dir_event();
  uint8_t sel_changed = 0;

  if (d == JOY_UP)
  {
    s_sel = (s_sel == 0) ? (uint8_t)(gw_slave_count() - 1) : (uint8_t)(s_sel - 1);
    sel_changed = 1;
  }
  else if (d == JOY_DOWN)
  {
    s_sel = (uint8_t)((s_sel + 1) % gw_slave_count());
    sel_changed = 1;
  }

  if (sel_changed)
  {
    draw_list();
    data_full();
  }

  if ((uint32_t)(HAL_GetTick() - last) >= 150)
  {
    last = HAL_GetTick();
    list_update();
    data_update();
    draw_cycle();
  }
}
