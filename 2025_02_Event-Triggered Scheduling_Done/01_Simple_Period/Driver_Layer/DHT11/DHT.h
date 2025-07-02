#ifndef __DHT_H__
#define __DHT_H__

#include "stm32f1xx_hal.h"  // thay đổi theo dòng chip bạn dùng nếu khác
#include "stdbool.h"
// ------ Configurable parameters ------
#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_8

// ------ Public variables ------
extern float Temperature;
extern float Humidity;

// ------ Public functions ------
void DHT11_Init(TIM_HandleTypeDef *htim);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read_Byte(void);
bool DHT11_Read(float *temperature, float *humidity);

#endif
