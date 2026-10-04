/**
  ******************************************************************************
  * @file    st7789.c
  * @brief   ST7789 SPI TFT 驱动�?40x320�?�?SPI，只写，阻塞版）
  * @note    依赖：CubeMX �?hspi1（SPI1�?位，Mode0，主，软件NSS�?  *          控制脚：CS=PB0  DC=PB1  RST=PB2  BLK=PB10
  ******************************************************************************
  */

#include "main.h"
#include "spi.h"
#include "cmsis_os.h"
#include "st7789.h"
#include "font8x16.h"
#include "zh16.h"

/* ================= 引脚定义（接线照这个来） ================= */

#define ST_CS_PORT    GPIOB
#define ST_CS_PIN     GPIO_PIN_0
#define ST_DC_PORT    GPIOB
#define ST_DC_PIN     GPIO_PIN_1
#define ST_RST_PORT   GPIOB
#define ST_RST_PIN    GPIO_PIN_2
#define ST_BLK_PORT   GPIOB
#define ST_BLK_PIN    GPIO_PIN_10

#define ST_CS_LOW()   HAL_GPIO_WritePin(ST_CS_PORT, ST_CS_PIN, GPIO_PIN_RESET)
#define ST_CS_HIGH()  HAL_GPIO_WritePin(ST_CS_PORT, ST_CS_PIN, GPIO_PIN_SET)
#define ST_DC_CMD()   HAL_GPIO_WritePin(ST_DC_PORT, ST_DC_PIN, GPIO_PIN_RESET)  /* �?命令 */
#define ST_DC_DATA()  HAL_GPIO_WritePin(ST_DC_PORT, ST_DC_PIN, GPIO_PIN_SET)    /* �?数据 */
#define ST_RST_LOW()  HAL_GPIO_WritePin(ST_RST_PORT, ST_RST_PIN, GPIO_PIN_RESET)
#define ST_RST_HIGH() HAL_GPIO_WritePin(ST_RST_PORT, ST_RST_PIN, GPIO_PIN_SET)
#define ST_BLK_ON()   HAL_GPIO_WritePin(ST_BLK_PORT, ST_BLK_PIN, GPIO_PIN_SET)
#define ST_BLK_OFF()  HAL_GPIO_WritePin(ST_BLK_PORT, ST_BLK_PIN, GPIO_PIN_RESET)

/* 面板偏移�?40x320 一般是 0；若显示偏移可微调这两值） */
#define ST_XOFF   0
#define ST_YOFF   0

/* 分块发送的块大小（像素）；越大越快，占�?RAM 越多 */
#define ST_CHUNK  512

static uint16_t s_w = ST7789_WIDTH;    /* 当前有效宽（随旋转变�?*/
static uint16_t s_h = ST7789_HEIGHT;   /* 当前有效高（随旋转变�?*/
static uint8_t  s_chunk[ST_CHUNK * 2]; /* 字节交换用的临时�?*/

/* ================= 底层：写命令 / 写数�?================= */

static void st_wr_cmd(uint8_t cmd)
{
  ST_DC_CMD();
  ST_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 1000);
  ST_CS_HIGH();
}

static osSemaphoreId_t   s_dmaSem = NULL;      /* DMA 完成信号量（st7789_dma_init 创建） */
static DMA_HandleTypeDef s_hdma_spi1_tx;        /* SPI1 TX 的 DMA 句柄 */

static void st_wr_data(const uint8_t *data, uint32_t len)
{
  if (len == 0) return;
  ST_DC_DATA();
  ST_CS_LOW();
  if (s_dmaSem && len >= 16)
  {
    if (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, (uint16_t)len) == HAL_OK)
    {
      (void)osSemaphoreAcquire(s_dmaSem, osWaitForever);   /* 等 DMA 发完(会让出 CPU) */
    }
    else
    {
      HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 1000);
    }
  }
  else
  {
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 1000);
  }
  ST_CS_HIGH();
}

static void st_wr_data8(uint8_t d)
{
  st_wr_data(&d, 1);
}

/** @brief 设置写入窗口 [x0..x1]x[y0..y1]，之后连续写像素 */
static void st_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t b[4];
  x0 += ST_XOFF; x1 += ST_XOFF;
  y0 += ST_YOFF; y1 += ST_YOFF;

  st_wr_cmd(0x2A);                       /* CASET：列地址 */
  b[0] = (uint8_t)(x0 >> 8); b[1] = (uint8_t)x0;
  b[2] = (uint8_t)(x1 >> 8); b[3] = (uint8_t)x1;
  st_wr_data(b, 4);

  st_wr_cmd(0x2B);                       /* RASET：行地址 */
  b[0] = (uint8_t)(y0 >> 8); b[1] = (uint8_t)y0;
  b[2] = (uint8_t)(y1 >> 8); b[3] = (uint8_t)y1;
  st_wr_data(b, 4);

  st_wr_cmd(0x2C);                       /* RAMWR：开始写显存 */
}

/** @brief 发�?n �?RGB565 像素（自动做字节交换，ST7789 要高字节在前�?*/
static void st_push_pixels(const uint16_t *pix, uint32_t n)
{
  while (n)
  {
    uint32_t m = (n > ST_CHUNK) ? ST_CHUNK : n;
    for (uint32_t i = 0; i < m; i++)
    {
      uint16_t c = pix[i];
      s_chunk[2 * i]     = (uint8_t)(c >> 8);   /* 高字节先�?*/
      s_chunk[2 * i + 1] = (uint8_t)(c & 0xFF);
    }
    st_wr_data(s_chunk, m * 2);
    pix += m;
    n   -= m;
  }
}

/* ================= 对外接口 ================= */

void st7789_backlight(bool on)
{
  if (on) ST_BLK_ON(); else ST_BLK_OFF();
}

void st7789_set_rotation(uint8_t rot)
{
  uint8_t madctl;
  switch (rot & 3)
  {
    case 0:  madctl = 0x00; s_w = 240; s_h = 320; break;   /* 竖屏 */
    case 1:  madctl = 0x60; s_w = 320; s_h = 240; break;   /* 横屏 */
    case 2:  madctl = 0xC0; s_w = 240; s_h = 320; break;   /* 竖屏翻转 */
    default: madctl = 0xA0; s_w = 320; s_h = 240; break;   /* 横屏翻转 */
  }
  st_wr_cmd(0x36);          /* MADCTL */
  st_wr_data8(madctl);
}

void st7789_init(void)
{
  /* --- 硬件复位 --- */
  ST_RST_HIGH(); HAL_Delay(10);
  ST_RST_LOW();  HAL_Delay(20);
  ST_RST_HIGH(); HAL_Delay(120);

  /* --- 上电序列（ST7789 通用，不同批次可微调�?--- */
  st_wr_cmd(0x01); HAL_Delay(150);   /* SWRESET 软件复位 */

  st_wr_cmd(0x11); HAL_Delay(120);   /* SLPOUT  退出睡�?*/

  st_wr_cmd(0x3A); st_wr_data8(0x55);   /* COLMOD = 16�?RGB565 */

  st7789_set_rotation(0);               /* 默认竖屏 */

  st_wr_cmd(0xB2);                       /* PORCTRL 门控 */
  { uint8_t p[5] = {0x0C, 0x0C, 0x00, 0x33, 0x33}; st_wr_data(p, 5); }

  st_wr_cmd(0xB7); st_wr_data8(0x35);    /* GCTRL */
  st_wr_cmd(0xBB); st_wr_data8(0x19);    /* VCOMS */
  st_wr_cmd(0xC0); st_wr_data8(0x2C);    /* LCMCTRL */
  st_wr_cmd(0xC2); st_wr_data8(0x01);    /* VDVVRHEN */
  st_wr_cmd(0xC3); st_wr_data8(0x12);    /* VRHS */
  st_wr_cmd(0xC4); st_wr_data8(0x20);    /* VDVS */
  st_wr_cmd(0xC6); st_wr_data8(0x0F);    /* FRCTRL2 */
  st_wr_cmd(0xD0);                       /* PWCTRL1 */
  { uint8_t p[2] = {0xA4, 0xA1}; st_wr_data(p, 2); }

  st_wr_cmd(0x20);                       /* INVON（IPS 屏一般要开反显�?*/

  st_wr_cmd(0xE0);                       /* PVGAMCTRL 正极性伽�?*/
  { uint8_t p[14] = {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23}; st_wr_data(p, 14); }
  st_wr_cmd(0xE1);                       /* NVGAMCTRL 负极性伽�?*/
  { uint8_t p[14] = {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23}; st_wr_data(p, 14); }

  st_wr_cmd(0x13); HAL_Delay(10);          /* NORON 普通显示模�?*/
  st_wr_cmd(0x29); HAL_Delay(100);         /* DISPON 开显示 */

  ST_BLK_ON();                           /* 开背光 */
}

void st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  uint32_t n = (uint32_t)w * h;
  st_set_window(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
  while (n)
  {
    uint32_t m = (n > ST_CHUNK) ? ST_CHUNK : n;
    for (uint32_t i = 0; i < m; i++)
    {
      s_chunk[2 * i]     = (uint8_t)(color >> 8);
      s_chunk[2 * i + 1] = (uint8_t)(color & 0xFF);
    }
    st_wr_data(s_chunk, m * 2);
    n -= m;
  }
}

void st7789_fill(uint16_t color)
{
  st7789_fill_rect(0, 0, s_w, s_h, color);
}

void st7789_draw_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pix)
{
  if (w == 0 || h == 0) return;
  st_set_window(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
  st_push_pixels(pix, (uint32_t)w * h);
}

/* ================= 文字（8x16 字模，可放大） ================= */

#define ST_TXT_SCALE_MAX 3
static uint16_t s_txt[8 * ST_TXT_SCALE_MAX * 16 * ST_TXT_SCALE_MAX];

void st7789_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
  const uint8_t *g;
  uint16_t cw, ch;
  uint8_t gy, gx, sy, sx, idx;

  if (scale < 1) scale = 1;
  if (scale > ST_TXT_SCALE_MAX) scale = ST_TXT_SCALE_MAX;
  if (c < 32 || c > 126) c = ' ';

  idx = (uint8_t)(c - 32);
  g = font8x16[idx];
  cw = (uint16_t)(8 * scale);
  ch = (uint16_t)(16 * scale);

  for (gy = 0; gy < 16; gy++)
  {
    for (gx = 0; gx < 8; gx++)
    {
      uint16_t col = (g[gy] & (0x80 >> gx)) ? fg : bg;
      for (sy = 0; sy < scale; sy++)
        for (sx = 0; sx < scale; sx++)
          s_txt[(uint32_t)(gy * scale + sy) * cw + (gx * scale + sx)] = col;
    }
  }
  st7789_draw_rgb565(x, y, cw, ch, s_txt);
}

void st7789_draw_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
  if (scale < 1) scale = 1;
  if (scale > ST_TXT_SCALE_MAX) scale = ST_TXT_SCALE_MAX;
  while (*s)
  {
    st7789_draw_char(x, y, *s, fg, bg, scale);
    x = (uint16_t)(x + 8 * scale);
    s++;
  }
}

/* ================= 中文（16x16 字模） ================= */

static const uint8_t *zh_find(uint16_t code)
{
  uint8_t n = (uint8_t)(sizeof(zh_code) / sizeof(zh_code[0]));
  for (uint8_t i = 0; i < n; i++)
    if (zh_code[i] == code) return zh_font[i];
  return 0;
}

static void zh_draw_glyph(uint16_t x, uint16_t y, const uint8_t *g, uint16_t fg, uint16_t bg)
{
  static uint16_t buf[16 * 16];
  for (uint8_t row = 0; row < 16; row++)
  {
    uint16_t hi = g[row * 2];
    uint16_t lo = g[row * 2 + 1];
    for (uint8_t i = 0; i < 8; i++)
      buf[row * 16 + i] = (hi & (0x80 >> i)) ? fg : bg;
    for (uint8_t i = 0; i < 8; i++)
      buf[row * 16 + 8 + i] = (lo & (0x80 >> i)) ? fg : bg;
  }
  st7789_draw_rgb565(x, y, 16, 16, buf);
}

void st7789_draw_codes(uint16_t x, uint16_t y, const uint16_t *codes, uint8_t n, uint16_t fg, uint16_t bg)
{
  for (uint8_t k = 0; k < n; k++)
  {
    uint16_t c = codes[k];
    if (c < 128)
    {
      st7789_draw_char(x, y, (char)c, fg, bg, 1);
      x = (uint16_t)(x + 8);
    }
    else
    {
      const uint8_t *g = zh_find(c);
      if (g) zh_draw_glyph(x, y, g, fg, bg);
      x = (uint16_t)(x + 16);
    }
  }
}

/* ================= 屏幕 SPI DMA ================= */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1) (void)osSemaphoreRelease(s_dmaSem);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1) (void)osSemaphoreRelease(s_dmaSem);
}
void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&s_hdma_spi1_tx);
}

void st7789_dma_init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  s_hdma_spi1_tx.Instance                 = DMA2_Stream3;
  s_hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_3;
  s_hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  s_hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  s_hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
  s_hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.Mode                = DMA_NORMAL;
  s_hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
  s_hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&s_hdma_spi1_tx);

  __HAL_LINKDMA(&hspi1, hdmatx, s_hdma_spi1_tx);

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

  s_dmaSem = osSemaphoreNew(1, 0, NULL);
}
