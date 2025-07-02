/*
 * my_uart.h
 *
 *  Created on: Jul 1, 2025
 *      Author: dangm
 */
#include "stm32f1xx_hal.h"  // Thay đổi tùy dòng chip


#ifndef UART_MY_UART_H_
#define UART_MY_UART_H_

// Gửi chuỗi null-terminated
void UART_SendString(UART_HandleTypeDef *huart, const char *str);

// Gửi số nguyên (int32_t)
void UART_SendInt(UART_HandleTypeDef *huart, int32_t num);

// Gửi số thực (float), tùy chọn số chữ số sau dấu phẩy
void UART_SendFloat(UART_HandleTypeDef *huart, float num, uint8_t decimal);

void UART_StartReceive_IT(void);
void UART_ProcessReceivedData(char *data);

#endif /* UART_MY_UART_H_ */
