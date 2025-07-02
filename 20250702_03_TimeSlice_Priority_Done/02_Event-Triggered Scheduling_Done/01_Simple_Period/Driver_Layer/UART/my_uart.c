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
#include "cmsis_os.h"
#define UART_RX_BUFFER_SIZE 64

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_data;
static uint16_t uart_rx_index = 0;

extern uint32_t delayTaskDHT;
extern uint32_t delayTaskFre1;
extern uint32_t delayTaskFre2;
extern uint32_t delayTaskLCD;
extern uint32_t delayTaskUART;

extern osThreadId Task_Sensor_DhtHandle;
extern osThreadId Task_Fre_1Handle;
extern osThreadId Task_Fre_2Handle;
extern osThreadId Task_LcdHandle;
extern osThreadId TaskUartHandle;

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

// Định nghĩa loại sự kiện cho queue
typedef enum {
    EVENT_NONE = 0,
    EVENT_UART,
    EVENT_LCD,
    EVENT_TEMP,
    EVENT_FREQ
} EventType;

#define EVENT_QUEUE_SIZE 10
typedef struct {
    EventType buffer[EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
} EventQueue;

// Khai báo queue toàn cục (hoặc extern nếu dùng chung nhiều file)
EventQueue uart_event_queue;

// Hàm đẩy sự kiện vào queue
void event_queue_push(EventQueue *q, EventType event) {
    uint8_t next = (q->tail + 1) % EVENT_QUEUE_SIZE;
    if (next != q->head) { // tránh tràn queue
        q->buffer[q->tail] = event;
        q->tail = next;
    }
}

// Hàm lấy sự kiện ra khỏi queue
EventType event_queue_pop(EventQueue *q) {
    if (q->head == q->tail) return EVENT_NONE; // queue rỗng
    EventType event = q->buffer[q->head];
    q->head = (q->head + 1) % EVENT_QUEUE_SIZE;
    return event;
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

// Hàm xử lý dữ liệu nhận được và đẩy vào queue
void UART_ProcessReceivedData(char *data) {
    // Dữ liệu dạng: "task: uart." hoặc "task: lcd." ...
//    if (strncmp(data, "task: uart.", 11) == 0) {
//        event_queue_push(&uart_event_queue, EVENT_UART);
//        UART_SendString(&huart2, "Queued: UART event\r\n");
//    } else if (strncmp(data, "task: lcd.", 10) == 0) {
//        event_queue_push(&uart_event_queue, EVENT_LCD);
//        UART_SendString(&huart2, "Queued: LCD event\r\n");
//    } else if (strncmp(data, "task: temp.", 11) == 0) {
//        event_queue_push(&uart_event_queue, EVENT_TEMP);
//        UART_SendString(&huart2, "Queued: TEMP event\r\n");
//    } else if (strncmp(data, "task: fre.", 10) == 0) {
//        event_queue_push(&uart_event_queue, EVENT_FREQ);
//        UART_SendString(&huart2, "Queued: FREQ event\r\n");
	 uint32_t value;
	 char buffer[64];
	    // Lệnh thay đổi delay: "Change_DHT: <giá trị>."
	    if (sscanf(data, "Change_DHT: %lu.", &value) == 1) {
	        delayTaskDHT = value;
	        UART_SendString(&huart2, "Changed DHT delay!\r\n");
    } else if (sscanf(data, "Change_FRE1: %lu.", &value) == 1) {
        delayTaskFre1 = value;
        UART_SendString(&huart2, "Changed FRE1 delay!\r\n");
    } else if (sscanf(data, "Change_FRE2: %lu.", &value) == 1) {
        delayTaskFre2 = value;
        UART_SendString(&huart2, "Changed FRE2 delay!\r\n");
    } else if (sscanf(data, "Change_LCD: %lu.", &value) == 1) {
        delayTaskLCD = value;
        UART_SendString(&huart2, "Changed LCD delay!\r\n");
    } else if (sscanf(data, "Change_UART: %lu.", &value) == 1) {
        delayTaskUART = value;
        UART_SendString(&huart2, "Changed UART delay!\r\n");
    }
    else if (strncmp(data, "GetPriority.", 12) == 0) {
            // Lấy mức ưu tiên các task
            sprintf(buffer, "DHT: %d\r\n", osThreadGetPriority(Task_Sensor_DhtHandle));
            UART_SendString(&huart2, buffer);
            sprintf(buffer, "FRE1: %d\r\n", osThreadGetPriority(Task_Fre_1Handle));
            UART_SendString(&huart2, buffer);
            sprintf(buffer, "FRE2: %d\r\n", osThreadGetPriority(Task_Fre_2Handle));
            UART_SendString(&huart2, buffer);
            sprintf(buffer, "LCD: %d\r\n", osThreadGetPriority(Task_LcdHandle));
            UART_SendString(&huart2, buffer);
            sprintf(buffer, "UART: %d\r\n", osThreadGetPriority(TaskUartHandle));
            UART_SendString(&huart2, buffer);
        }

	    else {
        UART_SendString(&huart2, "Invalid command format.\r\n");
    }
}

// Hàm xử lý từng sự kiện lấy ra từ queue (gọi trong while(1) ở main)
//void HandleEvent(EventType event) {
//    switch (event) {
//        case EVENT_UART:
//            // Xử lý task UART ở đây
//            UART_SendString(&huart2, "Handle UART task\r\n");
//            break;
//        case EVENT_LCD:
//            // Xử lý task LCD ở đây
//            UART_SendString(&huart2, "Handle LCD task\r\n");
//            break;
//        case EVENT_TEMP:
//            // Xử lý task nhiệt độ ở đây
//            UART_SendString(&huart2, "Handle TEMP task\r\n");
//            break;
//        case EVENT_FREQ:
//            // Xử lý task tần số ở đây
//            UART_SendString(&huart2, "Handle FREQ task\r\n");
//            break;
//        default:
//            break;
//    }
//}
