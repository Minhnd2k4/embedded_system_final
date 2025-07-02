// frequency.c
#include "frequency.h"
#include <stdint.h>

extern UART_HandleTypeDef huart2;  // Nếu bạn cần dùng UART bên trong

static TIM_HandleTypeDef *htim1 = NULL;
static TIM_HandleTypeDef *htim2 = NULL;

static uint32_t IC1_Val1 = 0, IC1_Val2 = 0;
static uint8_t  Is1_First = 0;
static volatile uint16_t IC1_Overflow = 0;
static uint32_t Freq1 = 0;

static uint32_t IC2_Val1 = 0, IC2_Val2 = 0;
static uint8_t  Is2_First = 0;
static volatile uint16_t IC2_Overflow = 0;
static uint32_t Freq2 = 0;

void Frequency1_Init(TIM_HandleTypeDef *htim)
{
    htim1 = htim;
    HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_1);
}

void Frequency2_Init(TIM_HandleTypeDef *htim)
{
    htim2 = htim;
    HAL_TIM_IC_Start_IT(htim2, TIM_CHANNEL_1);
}

uint32_t Frequency1_Read(void)
{
    return Freq1;
}

uint32_t Frequency2_Read(void)
{
    return Freq2;
}

void Frequency_UpdateOverflow(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim1->Instance)
        IC1_Overflow++;
    if (htim->Instance == htim2->Instance)
        IC2_Overflow++;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim1->Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (!Is1_First)
        {
            IC1_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            IC1_Overflow = 0;
            Is1_First = 1;
        }
        else
        {
            IC1_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t diff = (IC1_Val2 >= IC1_Val1)
                            ? (IC1_Val2 - IC1_Val1 + IC1_Overflow * 0x10000)
                            : (0x10000 + IC1_Val2 - IC1_Val1 + (IC1_Overflow - 1) * 0x10000);
            if(diff == 2000) {
            	diff = diff/2;
            }

            Freq1 = (1000000 / diff);
            Is1_First = 0;
        }
    }

    if (htim->Instance == htim2->Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (!Is2_First)
        {
            IC2_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            IC2_Overflow = 0;
            Is2_First = 1;
        }
        else
        {
            IC2_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t diff = (IC2_Val2 >= IC2_Val1)
                            ? (IC2_Val2 - IC2_Val1 + IC2_Overflow * 0x10000)
                            : (0x10000 + IC2_Val2 - IC2_Val1 + (IC2_Overflow - 1) * 0x10000);
            	if(diff == 2000) {
                       	diff = diff/2;
                       }

            Freq2 = (1000000 / diff);
            Is2_First = 0;
        }
    }
}
