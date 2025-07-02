/*
 * LCD.c
 *
 *  Created on: Jul 1, 2025
 *      Author: dangm
 */
#include "lcd.h"
#include "string.h"

static I2C_HandleTypeDef *_lcd_i2c;

static void LCD_Send_4Bits(uint8_t data)
{
    HAL_I2C_Write(_lcd_i2c, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
}

static void LCD_Send(uint8_t data, uint8_t mode)
{
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = high_nibble | LCD_BACKLIGHT | mode | LCD_ENABLE;
    data_arr[1] = high_nibble | LCD_BACKLIGHT | mode;
    data_arr[2] = low_nibble  | LCD_BACKLIGHT | mode | LCD_ENABLE;
    data_arr[3] = low_nibble  | LCD_BACKLIGHT | mode;

    HAL_I2C_Master_Transmit(_lcd_i2c, LCD_ADDR, data_arr, 4, HAL_MAX_DELAY);
    HAL_Delay(1);
}

void LCD_Send_Cmd(uint8_t cmd)
{
    LCD_Send(cmd, LCD_COMMAND);
}

void LCD_Send_Data(uint8_t data)
{
    LCD_Send(data, LCD_DATA);
}

void LCD_Send_String(char *str)
{
    while (*str)
    {
        LCD_Send_Data(*str++);
    }
}

void LCD_Set_Cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_Send_Cmd(addr);
}

void LCD_Clear(void)
{
    LCD_Send_Cmd(0x01);  // Clear display
    HAL_Delay(2);
}

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    _lcd_i2c = hi2c;

    HAL_Delay(50);

    // Gửi chế độ khởi tạo
    LCD_Send_Cmd(0x33);
    LCD_Send_Cmd(0x32);  // 4-bit mode
    LCD_Send_Cmd(0x28);  // 2 lines, 5x8 dots
    LCD_Send_Cmd(0x0C);  // Display ON, Cursor OFF
    LCD_Send_Cmd(0x06);  // Entry mode
    LCD_Clear();
}

void LCD_PrintFloat(uint8_t row, uint8_t col, const char *label, float value, const char *unit)
{
    char buffer[17];  // LCD 16x2
    char value_str[10];

    // Format giá trị float
    snprintf(value_str, sizeof(value_str), "%.1f", value);

    // Ghép chuỗi: label + value + unit
    snprintf(buffer, sizeof(buffer), "%s%s %s", label, value_str, unit);

    // Set con trỏ và hiển thị
    LCD_Set_Cursor(row, col);
    LCD_Send_String(buffer);
}
