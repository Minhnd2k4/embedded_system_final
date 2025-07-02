/*
 * my_uart.c
 *
 *  Created on: Jul 1, 2025
 *      Author: dangm
 */

#include "my_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 64

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_data;
static uint16_t uart_rx_index = 0;

extern TIM_HandleTypeDef htim1;
// Khai báo bên ngoài của huart2
extern UART_HandleTypeDef huart2;

// Gửi chuỗi
void UART_SendString(UART_HandleTypeDef *huart, const char *str) {
    HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// Gửi số nguyên
void UART_SendInt(UART_HandleTypeDef *huart, int32_t num) {
    char buffer[16];
    sprintf(buffer, "%ld", (long)num);
    UART_SendString(huart, buffer);
}

// Gửi số thực
void UART_SendFloat(UART_HandleTypeDef *huart, float num, uint8_t decimal) {
    char format[10];
    char buffer[32];

    // Tạo chuỗi format kiểu: "%.2f", "%.3f", v.v.
    sprintf(format, "%%.%df", decimal);
    sprintf(buffer, format, num);
    UART_SendString(huart, buffer);
}

void UART_StartReceive_IT(void) {
    uart_rx_index = 0;
    memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
}

// Hàm callback ngắt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            uart_rx_buffer[uart_rx_index++] = uart_rx_data;

            if (uart_rx_data == '.') {
                uart_rx_buffer[uart_rx_index] = '\0';
                UART_ProcessReceivedData((char *)uart_rx_buffer);
                uart_rx_index = 0;
                memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
            }
        } else {
            uart_rx_index = 0; // Reset nếu tràn buffer
        }

        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1); // Tiếp tục nhận
    }
}

// Hàm xử lý dữ liệu nhận được (có thể override)
void UART_ProcessReceivedData(char *data) {
	 uint32_t new_period;

	    // Kiểm tra cú pháp chuỗi có đúng "Period: <số>."
	    if (sscanf(data, "Period: %lu.", &new_period) == 1) {
	        if (new_period > 0 && new_period <= 0xFFFF) {  // Giới hạn hợp lệ
	            htim1.Init.Period = new_period;

	            if (HAL_TIM_Base_Init(&htim1) == HAL_OK) {
	                UART_SendString(&huart2, "Period updated to: ");
	                UART_SendInt(&huart2, new_period);

	            } else {
	                UART_SendString(&huart2, "Failed to re-init TIM1.\n");
	            }
	        } else {
	            UART_SendString(&huart2, "Invalid Period range.\n");
	        }
	    } else {
	        UART_SendString(&huart2, "Invalid command format.\n");
	    }
}
