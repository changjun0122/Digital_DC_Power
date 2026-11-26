#include "oled.h"
#include "i2c.h"
#include "font.h"

#define OLED_CTRL_CMD   0x00U
#define OLED_CTRL_DATA  0x40U
#define OLED_TIMEOUT    20U

static void OLED_WriteCmd(uint8_t cmd)
{
  uint8_t buf[2];

  buf[0] = OLED_CTRL_CMD;
  buf[1] = cmd;
  (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2U, OLED_TIMEOUT);
}

static void OLED_SetPos(uint8_t x, uint8_t page)
{
  uint8_t buf[4];

  buf[0] = OLED_CTRL_CMD;
  buf[1] = (uint8_t)(0xB0U | (page & 0x07U));
  buf[2] = (uint8_t)(0x00U | (x & 0x0FU));
  buf[3] = (uint8_t)(0x10U | ((x >> 4) & 0x0FU));
  (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 4U, OLED_TIMEOUT);
}

static void OLED_FillPage(uint8_t page, uint8_t value)
{
  uint8_t buf[33];
  uint8_t n;

  buf[0] = OLED_CTRL_DATA;
  for (n = 0U; n < 32U; n++)
  {
    buf[n + 1U] = value;
  }

  OLED_SetPos(0U, page);
  for (n = 0U; n < 4U; n++)
  {
    (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, sizeof(buf), OLED_TIMEOUT);
  }
}

void OLED_Init(void)
{
  static const uint8_t initSeq[] =
  {
    0xAE,              /* display off            */
    0xD5, 0x80,        /* clock div              */
    0xA8, 0x3F,        /* multiplex = 64         */
    0xD3, 0x00,        /* display offset 0       */
    0x40,              /* start line 0           */
    0x8D, 0x14,        /* charge pump enable     */
    0x20, 0x02,        /* page addressing mode   */
    0xA1,              /* segment remap          */
    0xC8,              /* COM scan reverse       */
    0xDA, 0x12,        /* COM pins               */
    0x81, 0xCF,        /* contrast               */
    0xD9, 0xF1,        /* pre-charge             */
    0xDB, 0x40,        /* VCOMH                  */
    0xA4,              /* output follows RAM     */
    0xA6,              /* normal (not inverted)  */
    0x2E,              /* scroll off             */
    0xAF               /* display on             */
  };
  uint32_t i;

  HAL_Delay(100U);
  for (i = 0U; i < sizeof(initSeq); i++)
  {
    OLED_WriteCmd(initSeq[i]);
  }
  OLED_Clear();
}

void OLED_Clear(void)
{
  uint8_t page;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    OLED_FillPage(page, 0x00U);
  }
}

void OLED_ClearPage(uint8_t page)
{
  if (page < OLED_PAGES)
  {
    OLED_FillPage(page, 0x00U);
  }
}

void OLED_ShowChar(uint8_t x, uint8_t page, char ch, uint8_t invert)
{
  uint8_t buf[OLED_CHAR_W + 1U];
  uint8_t c = (uint8_t)ch;
  uint8_t i;

  if (((uint16_t)x + OLED_CHAR_W) > OLED_WIDTH)
  {
    return;
  }
  if (page >= OLED_PAGES)
  {
    return;
  }

  if ((c >= 'a') && (c <= 'z'))
  {
    c -= 32U;
  }
  if ((c < FONT_FIRST) || (c > FONT_LAST))
  {
    c = ' ';
  }

  buf[0] = OLED_CTRL_DATA;
  for (i = 0U; i < FONT_W; i++)
  {
    buf[i + 1U] = Font5x7[c - FONT_FIRST][i];
  }
  buf[FONT_W + 1U] = 0x00U;

  if (invert != 0U)
  {
    for (i = 1U; i <= OLED_CHAR_W; i++)
    {
      buf[i] = (uint8_t)~buf[i];
    }
  }

  OLED_SetPos(x, page);
  (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, sizeof(buf), OLED_TIMEOUT);
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str, uint8_t invert)
{
  while ((*str != '\0') && (((uint16_t)x + OLED_CHAR_W) <= OLED_WIDTH))
  {
    OLED_ShowChar(x, page, *str, invert);
    x += OLED_CHAR_W;
    str++;
  }
}

void OLED_ShowCN(uint8_t x, uint8_t page, uint8_t idx, uint8_t invert)
{
  uint8_t buf[17];
  uint8_t i;

  if ((idx >= CN_NUM) || (page >= (OLED_PAGES - 1U)) ||
      (((uint16_t)x + 16U) > OLED_WIDTH))
  {
    return;
  }

  buf[0] = OLED_CTRL_DATA;
  for (i = 0U; i < 16U; i++)
  {
    buf[i + 1U] = zh16x16[idx][CN16_OFFSET + i];
    if (invert != 0U)
    {
      buf[i + 1U] = (uint8_t)~buf[i + 1U];
    }
  }
  OLED_SetPos(x, page);
  (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, sizeof(buf), OLED_TIMEOUT);

  for (i = 0U; i < 16U; i++)
  {
    buf[i + 1U] = zh16x16[idx][CN16_OFFSET + 16U + i];
    if (invert != 0U)
    {
      buf[i + 1U] = (uint8_t)~buf[i + 1U];
    }
  }
  OLED_SetPos(x, (uint8_t)(page + 1U));
  (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, sizeof(buf), OLED_TIMEOUT);
}

void OLED_ClearBlock(uint8_t x, uint8_t page, uint8_t width)
{
  uint8_t buf[32U + 1U];
  uint8_t p;
  uint8_t i;

  if ((page >= (OLED_PAGES - 1U)) || (width == 0U) || (width > 32U) ||
      (((uint16_t)x + width) > OLED_WIDTH))
  {
    return;
  }

  buf[0] = OLED_CTRL_DATA;
  for (i = 0U; i < width; i++)
  {
    buf[i + 1U] = 0x00U;
  }

  for (p = 0U; p < 2U; p++)
  {
    OLED_SetPos(x, (uint8_t)(page + p));
    (void)HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf,
                                  (uint16_t)(width + 1U), OLED_TIMEOUT);
  }
}