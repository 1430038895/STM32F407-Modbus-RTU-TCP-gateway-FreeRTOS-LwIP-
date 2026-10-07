/**
  ******************************************************************************
  * @file    ui.c
  * @brief   自绘界面（不依赖 LVGL）：左侧 8 台从机列表（摇杆上下切换），
  *          右侧一行一个点（中文标签 + 绿色数值），右上角显示轮询耗时。
  * @note    刷新策略：切从机时整块重画；平时每 150ms 只重画"变了的值"和
  *          "在线状态变了的行"，尽量避免整屏闪烁。
  ******************************************************************************
  */

#include "main.h"
#include "st7789.h"
#include "joystick.h"
#include "mb_gateway.h"
#include "zh_labels.h"   /* 点名 -> 中文标签（Unicode 码点数组） */
#include <string.h>
#include <stdio.h>

/* ================= 配色（RGB565） ================= */
#define C_BG      0x10A3   /**< 背景 #101418 */
#define C_PANEL   0x1904   /**< 面板/标题栏 #1B2026 */
#define C_BORDER  0x2966   /**< 边框/分隔线 #2A2F36 */
#define C_TEXT    0xEF7D   /**< 主文字 #EFEFEF */
#define C_TEXT2   0xAD55   /**< 次要文字（标签） #AAAAAA */
#define C_BLUE    0x453E   /**< 强调蓝 #42A5F5 */
#define C_GREEN   0x4D6A   /**< 数值绿 #4CAF50 */
#define C_RED     0xF206   /**< 离线红点 #F44336 */
#define C_BLACK   0x0000   /**< 失败过多的黑点 */
#define C_SEL     0x1A4E   /**< 选中行背景 #1E4976 */

/* ================= 布局（像素） ================= */
#define SCR_W       240    /**< 屏宽 */
#define SCR_H       320    /**< 屏高 */
#define TITLE_H     28     /**< 标题栏高度 */

#define LIST_X      0      /**< 左列表左上角 x */
#define LIST_Y      30     /**< 左列表左上角 y */
#define LIST_W      100    /**< 左列表宽度 */
#define ROW_H       34     /**< 左列表每行高度 */

#define RIGHT_X     104    /**< 右侧区域左上角 x */
#define RIGHT_Y     30     /**< 右侧区域左上角 y */
#define VALUE_X     (RIGHT_X + 70)   /**< 右侧数值起始 x */
#define ROW_STEP    20               /**< 右侧每个点占的行高 */

#define UI_MAX_SLAVES  8   /**< 从机数上限（与屏幕行数一致） */
#define UI_MAX_ROWS    16  /**< 右侧最多显示的点行数 */

/* ================= 运行期状态 ================= */
static uint8_t  s_sel        = 0;      /**< 当前选中的从机序号（slv 下标） */
static uint8_t  s_drawn_sel  = 0xFF;   /**< 上次整块重画时的选中序号（用于判断是否变了） */
static uint8_t  s_drawn_online[UI_MAX_SLAVES];  /**< 上次画每行时的在线状态 */
static uint16_t s_row_idx[UI_MAX_ROWS];         /**< 右侧每行对应的点号 */
static uint8_t  s_row_cnt    = 0;               /**< 右侧当前实际显示的行数 */
static char     s_row_val[UI_MAX_ROWS][10];     /**< 上次画每行时的值字符串（判变用） */

/* ================= 小工具 ================= */

/**
  * @brief  收集当前选中从机的所有点号（含配置项）
  * @param  out  输出：点号数组
  * @retval 收集到的行数
  */
static uint8_t collect_rows(uint16_t *out)
{
  uint8_t addr = gw_slave_addr(s_sel);   /* 当前从机的地址 */
  uint8_t n = 0;                         /* 已收集行数 */
  for (uint16_t idx = 0; idx < dp_count() && n < UI_MAX_ROWS; idx++)
  {
    if (dp_dev_addr(idx) != addr) continue;   /* 不是这台设备的点，跳过 */
    out[n++] = idx;
  }
  return n;
}

/**
  * @brief  在标题栏右侧画"最近一轮轮询耗时"
  * @param  无
  * @retval 无
  * @note   用固定宽度原地覆盖，避免闪烁。
  */
static void draw_cycle(void)
{
  char num[12];
  char b[16];
  snprintf(num, sizeof(num), "%lums", (unsigned long)gw_cycle_ms());
  snprintf(b, sizeof(b), "%-7s", num);          /* 固定宽度，原地覆盖，不闪 */
  st7789_draw_text(156, 6, b, C_GREEN, C_PANEL, 1);
}

/**
  * @brief  画标题栏
  * @param  无
  * @retval 无
  */
static void draw_title(void)
{
  st7789_fill_rect(0, 0, SCR_W, TITLE_H, C_PANEL);
  st7789_draw_text(8, 6, "Modbus Gateway", C_TEXT, C_PANEL, 1);
  st7789_fill_rect(0, TITLE_H, SCR_W, 1, C_BORDER);
  draw_cycle();
}

/* ================= 左侧列表 ================= */

/**
  * @brief  画左侧一行（从机名 + 状态点）
  * @param  i        从机序号
  * @param  selected 1=选中（高亮），0=普通
  * @retval 无
  */
static void draw_row(uint8_t i, uint8_t selected)
{
  uint16_t y  = (uint16_t)(LIST_Y + i * ROW_H);      /* 本行 y */
  uint16_t bg = selected ? C_SEL  : C_BG;            /* 背景色 */
  uint16_t fg = selected ? C_TEXT : C_TEXT2;         /* 文字色 */

  st7789_fill_rect(LIST_X, y, LIST_W, ROW_H, bg);
  if (selected)
    st7789_fill_rect(LIST_X, y, 4, ROW_H, C_BLUE);   /* 选中行左侧蓝色竖条 */
  st7789_draw_text(LIST_X + 10, (uint16_t)(y + 9), gw_slave_name(i), fg, bg, 1);

  /* 右下角状态点：绿=在线；离线显示 N 个红点(1~4)；连续失败≥5 显示黑点 */
  {
    uint16_t doty = (uint16_t)(y + ROW_H - 9);
    uint16_t dotx = (uint16_t)(LIST_X + LIST_W - 9);
    uint8_t  fail = gw_slave_fail(i);               /* 连续失败次数 */

    if (gw_slave_online(i))
      st7789_fill_rect(dotx, doty, 6, 6, C_GREEN);
    else if (fail >= 5)
      st7789_fill_rect(dotx, doty, 6, 6, C_BLACK);
    else
    {
      uint8_t n = (fail == 0) ? 1 : fail;           /* 至少 1 个红点 */
      for (uint8_t k = 0; k < n; k++)
        st7789_fill_rect((uint16_t)(dotx - k * 7), doty, 5, 5, C_RED);
    }
  }
  st7789_fill_rect(LIST_X, (uint16_t)(y + ROW_H - 1), LIST_W, 1, C_BORDER);
}

/**
  * @brief  整块重画左侧列表
  * @param  无
  * @retval 无
  * @note   同时记录每行的在线状态，供后续增量刷新判断。
  */
static void draw_list(void)
{
  for (uint8_t i = 0; i < gw_slave_count(); i++)
  {
    draw_row(i, (uint8_t)(i == s_sel));
    s_drawn_online[i] = gw_slave_online(i);
  }
}

/**
  * @brief  只重画"在线状态变了"的行
  * @param  无
  * @retval 无
  */
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

/**
  * @brief  画一个点的中文标签（找不到中文则用英文点名）
  * @param  idx  点号
  * @param  y    该行 y
  * @retval 无
  */
static void draw_label(uint16_t idx, uint16_t y)
{
  uint8_t len = 0;
  const uint16_t *codes = zh_label_lookup(dp_name(idx), &len);   /* 查中文标签 */
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

/**
  * @brief  画一个点的数值（固定宽度原地覆盖）
  * @param  idx    点号
  * @param  slot   该点在右侧的第几行（用于记忆上次的值）
  * @param  y      该行 y
  * @param  force  1=强制重画；0=只有值变了才重画
  * @retval 无
  */
static void draw_value(uint16_t idx, uint16_t slot, uint16_t y, uint8_t force)
{
  char val[16];
  char pad[12];
  dp_format_num(idx, val, 2); val[8] = 0;    /* 值格式化（保留2位，截断到8字符） */
  snprintf(pad, sizeof(pad), "%-8s", val);   /* 固定宽度，原地覆盖，不闪 */
  if (force || strcmp(pad, s_row_val[slot]) != 0)
  {
    st7789_draw_text(VALUE_X, y, pad, C_GREEN, C_BG, 1);
    strncpy(s_row_val[slot], pad, 9); s_row_val[slot][9] = 0;
  }
}

/**
  * @brief  整块重画右侧数据区（切换从机时用）
  * @param  无
  * @retval 无
  */
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

/**
  * @brief  只重画右侧"值变了"的行
  * @param  无
  * @retval 无
  */
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

/**
  * @brief  初始化并整屏画一遍（清屏 + 标题 + 列表 + 数据）
  * @param  无
  * @retval 无
  */
void ui_init(void)
{
  st7789_fill(C_BG);
  draw_title();
  draw_list();
  data_full();
}

/**
  * @brief  处理一次摇杆 + 周期刷新（在主循环里周期性调用）
  * @param  无
  * @retval 无
  * @note   摇杆上下切换从机（循环）；切换时整块重画；否则每 150ms 增量刷新。
  */
void ui_poll(void)
{
  static uint32_t last = 0;               /* 上次周期刷新的时刻 */
  uint8_t d = joystick_dir_event();       /* 摇杆方向事件（每次动作只给一次） */
  uint8_t sel_changed = 0;

  if (d == JOY_UP)
  {
    s_sel = (s_sel == 0) ? (uint8_t)(gw_slave_count() - 1) : (uint8_t)(s_sel - 1);   /* 上：循环上移 */
    sel_changed = 1;
  }
  else if (d == JOY_DOWN)
  {
    s_sel = (uint8_t)((s_sel + 1) % gw_slave_count());                              /* 下：循环下移 */
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
    list_update();     /* 只改在线状态变化的行 */
    data_update();     /* 只改变化的值 */
    draw_cycle();      /* 刷新轮询耗时 */
  }
}
