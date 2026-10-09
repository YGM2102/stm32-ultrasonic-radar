#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/* 0.96" SSD1306 OLED 驱动（标准库 · 软件 I2C 版）
 * 引脚：PB6 = SCL，PB7 = SDA（与原理图网络 ⑥⑦ 对应）
 * I2C 从机地址 0x78（不亮可试 0x7A）
 * 字体：8x16，共 4 行 x 16 列
 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);      /* 行1~4，列1~16 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Num, uint8_t Length); /* 固定位数，高位补0 */

#endif
