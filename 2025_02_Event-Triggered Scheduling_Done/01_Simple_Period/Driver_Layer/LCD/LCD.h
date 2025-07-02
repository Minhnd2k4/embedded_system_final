/*
 * LCD.h
 *
 *  Created on: Jul 1, 2025
 *      Author: dangm
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Địa chỉ I2C của LCD (có thể là 0x27 hoặc 0x3F)
#define LCD_ADDR       (0x27 << 1)  // shift left vì HAL expects 8-bit addr
#define LCD_BACKLIGHT  0x08
#define LCD_ENABLE     0x04
#define LCD_COMMAND    0
#define LCD_DATA       1

// Khai báo các hàm chính
void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Clear(void);
void LCD_PrintFloat(uint8_t row, uint8_t col, const char *label, float value, const char *unit);

#endif /* LCD_LCD_H_ */
