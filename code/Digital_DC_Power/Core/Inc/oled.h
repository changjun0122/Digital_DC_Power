#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_ADDR    0x78U
#define OLED_WIDTH   128U
#define OLED_PAGES   8U
#define OLED_CHAR_W  6U
#define OLED_CHAR_H  8U

/* 汉字索引，顺序必须与 font_cn.h 中 zh16x16 一致 */
enum
{
  CN_SHU = 0,   /* 输 */
  CN_CHU,       /* 出 */
  CN_KAI,       /* 开 */
  CN_GUAN,      /* 关 */
  CN_DIAN,      /* 电 */
  CN_YA,        /* 压 */
  CN_XIAN,      /* 限 */
  CN_LIU,       /* 流 */
  CN_BAO,       /* 保 */
  CN_HU,        /* 护 */
  CN_GONG,      /* 功 */
  CN_LV,        /* 率 */
  CN_NUM
};

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearPage(uint8_t page);
void OLED_ShowChar(uint8_t x, uint8_t page, char ch, uint8_t invert);
void OLED_ShowString(uint8_t x, uint8_t page, const char *str, uint8_t invert);
void OLED_ShowCN(uint8_t x, uint8_t page, uint8_t idx, uint8_t invert);
void OLED_ClearBlock(uint8_t x, uint8_t page, uint8_t width);

#endif /* __OLED_H */