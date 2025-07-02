/*
 * Frequency.h
 *
 *  Created on: Jul 1, 2025
 *      Author: dangm
 */

#ifndef __FRE_H__
#define __FRE_H__

#include "stm32f1xx_hal.h"

void Frequency1_Init(TIM_HandleTypeDef *htim);
void Frequency2_Init(TIM_HandleTypeDef *htim);

void Frequency_UpdateOverflow(TIM_HandleTypeDef *htim);

uint32_t Frequency1_Read(void);
uint32_t Frequency2_Read(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif

