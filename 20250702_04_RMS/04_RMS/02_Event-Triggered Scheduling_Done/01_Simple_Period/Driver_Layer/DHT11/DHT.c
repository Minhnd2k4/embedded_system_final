#include "dht.h"
#include "cmsis_os.h"

float Temperature = 0;
float Humidity = 0;

static TIM_HandleTypeDef *dht_timer = NULL;

void DHT11_Init(TIM_HandleTypeDef *htim) {
    dht_timer = htim;
    HAL_TIM_Base_Start(dht_timer);
}

static void delay_us(uint16_t time) {
    __HAL_TIM_SET_COUNTER(dht_timer, 0);
    while (__HAL_TIM_GET_COUNTER(dht_timer) < time);
}

static void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
//    HAL_Delay(18);
    osDelay(18);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(20);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void) {
    uint8_t response = 0;
    delay_us(40);
    if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) response = 1;
    }
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    return response;
}

uint8_t DHT11_Read_Byte(void) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
        delay_us(40);
        if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            data &= ~(1 << (7 - i));
        else {
            data |= (1 << (7 - i));
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
        }
    }
    return data;
}

bool DHT11_Read(float *temperature, float *humidity)
{
    uint8_t rh1, rh2, temp1, temp2, sum;

    DHT11_Start();
    if (DHT11_Check_Response()) {
        rh1 = DHT11_Read_Byte();
        rh2 = DHT11_Read_Byte();
        temp1 = DHT11_Read_Byte();
        temp2 = DHT11_Read_Byte();
        sum = DHT11_Read_Byte();

        if (sum == (rh1 + rh2 + temp1 + temp2)) {
            *temperature = (float)temp1;
            *humidity = (float)rh1;
            return true;
        }
    }

    *temperature = -100.0f;
    *humidity = -1.0f;
    return false;
}

