/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "DHT11/DHT.h"
#include "UART/my_uart.h"
#include "LCD/LCD.h"
#include "Frequency/Frequency.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "DHT11/DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Task_Sensor_DhtHandle;
osThreadId Task_Fre_1Handle;
osThreadId Task_Fre_2Handle;
osThreadId Task_LcdHandle;
osThreadId TaskUartHandle;
/* USER CODE BEGIN PV */
//osMutexId dataMutexHandle;
//SensorData_t sharedData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask_Sensor_Dht(void const * argument);
void StartTaskFre_1(void const * argument);
void StartTaskFre_2(void const * argument);
void StartTask05(void const * argument);
void StartTaskUart(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    float temperature;
    float humidity;
    uint32_t fre1;
    uint32_t fre2;
} SensorData_t;
//
//osMessageQId sensorQueueHandle;
//
//osMessageQId queue_DHT_to_Fre1Handle;
//osMessageQId queue_Fre1_to_Fre2Handle;
//osMessageQId queue_Fre2_to_LCDHandle;
//osMessageQId queue_LCD_to_UARTHandle;
osMutexId dataMutexHandle;
SensorData_t sharedData;

uint32_t delayTaskDHT = 2000;
uint32_t delayTaskFre1 = 1000;
uint32_t delayTaskFre2 = 1000;
uint32_t delayTaskLCD = 1000;
uint32_t delayTaskUART = 2000;

//osThreadId taskDHTHandle;
//osThreadId taskFre1Handle;
//osThreadId taskFre2Handle;
//osThreadId taskLCDHandle;
//osThreadId taskUARTHandle;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Frequency1_Init(&htim3); // HAL_TIM_IC_Start_IT
  Frequency2_Init(&htim4);
  UART_StartReceive_IT();
  DHT11_Init(&htim4);
  LCD_Init(&hi2c2);

  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE BEGIN RTOS_QUEUES */
  // Khởi tạo queue với kích thước phù hợp
//  osMessageQDef(sensorQueue, 4, SensorData_t);
//  sensorQueueHandle = osMessageCreate(osMessageQ(sensorQueue), NULL);


//  osMessageQDef(queue_DHT_to_Fre1, 4, SensorData_t);
//  queue_DHT_to_Fre1Handle = osMessageCreate(osMessageQ(queue_DHT_to_Fre1), NULL);
//
//  osMessageQDef(queue_Fre1_to_Fre2, 4, SensorData_t);
//  queue_Fre1_to_Fre2Handle = osMessageCreate(osMessageQ(queue_Fre1_to_Fre2), NULL);
//
//  osMessageQDef(queue_Fre2_to_LCD, 4, SensorData_t);
//  queue_Fre2_to_LCDHandle = osMessageCreate(osMessageQ(queue_Fre2_to_LCD), NULL);
//
//  osMessageQDef(queue_LCD_to_UART, 4, SensorData_t);
//  queue_LCD_to_UARTHandle = osMessageCreate(osMessageQ(queue_LCD_to_UART), NULL);

  /* USER CODE END RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_Sensor_Dht */
  osThreadDef(Task_Sensor_Dht, StartTask_Sensor_Dht, osPriorityHigh, 0, 256);
  Task_Sensor_DhtHandle = osThreadCreate(osThread(Task_Sensor_Dht), NULL);

  /* definition and creation of Task_Fre_1 */
  osThreadDef(Task_Fre_1, StartTaskFre_1, osPriorityAboveNormal, 0, 128);
  Task_Fre_1Handle = osThreadCreate(osThread(Task_Fre_1), NULL);

  /* definition and creation of Task_Fre_2 */
  osThreadDef(Task_Fre_2, StartTaskFre_2, osPriorityAboveNormal, 0, 128);
  Task_Fre_2Handle = osThreadCreate(osThread(Task_Fre_2), NULL);

  /* definition and creation of Task_Lcd */
  osThreadDef(Task_Lcd, StartTask05, osPriorityNormal, 0, 128);
  Task_LcdHandle = osThreadCreate(osThread(Task_Lcd), NULL);

  /* definition and creation of TaskUart */
  osThreadDef(TaskUart, StartTaskUart, osPriorityIdle, 0, 128);
  TaskUartHandle = osThreadCreate(osThread(TaskUart), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 300;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask_Sensor_Dht */
/**
* @brief Function implementing the Task_Sensor_Dht thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Sensor_Dht */
//void StartTask_Sensor_Dht(void const * argument)
//{
//  /* USER CODE BEGIN StartTask_Sensor_Dht */
//	UART_SendString(&huart2, "StartTask_Sensor_Dht");
//	float temperature = 0, humidity = 0;
//	uint8_t DHT_OK = 1;
//	SensorData_t data;
//  /* Infinite loop */
//  for(;;)
//  {
//	if(DHT11_Read(&temperature, &humidity) == DHT_OK) {
//        data.temperature = temperature;
//        data.humidity = humidity;
//        UART_SendInt(&huart2, data.humidity);
//        UART_SendInt(&huart2, data.temperature);
//        UART_SendString(&huart2, "PreOngoingTask_Sensor_Dht");
//		osMessagePut(sensorQueueHandle, (uint32_t)&data, osWaitForever);
//		UART_SendString(&huart2, "OngoingTask_Sensor_Dht");
//	}
//	else {
//		UART_SendString(&huart2, "Read data failed");
//	}
//    osDelay(2000);
//    UART_SendString(&huart2, "EndTask_Sensor_Dht");
//  }
//  /* USER CODE END StartTask_Sensor_Dht */
//}
//
///* USER CODE BEGIN Header_StartTaskFre_1 */
///**
//* @brief Function implementing the Task_Fre_1 thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTaskFre_1 */
//void StartTaskFre_1(void const * argument)
//{
//  /* USER CODE BEGIN StartTaskFre_1 */
//	UART_SendString(&huart2, "StartTask_Fre");
//    SensorData_t data;
//    osEvent evt;
//    for(;;)
//    {
//        evt = osMessageGet(sensorQueueHandle, osWaitForever);
//        if(evt.status == osEventMessage) {
//            data = *(SensorData_t*)evt.value.p;
//            data.fre1 = Frequency1_Read(); // Đọc tần số fre1
//            osMessagePut(sensorQueueHandle, (uint32_t)&data, osWaitForever);
//            UART_SendString(&huart2, "OngoingTask_Sensor_Dht");
//        }
//        osDelay(1000);
//    }
//  /* USER CODE END StartTaskFre_1 */
//}
//
///* USER CODE BEGIN Header_StartTaskFre_2 */
///**
//* @brief Function implementing the Task_Fre_2 thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTaskFre_2 */
//void StartTaskFre_2(void const * argument)
//{
//  /* USER CODE BEGIN StartTaskFre_2 */
//	UART_SendString(&huart2, "StartTask_Fre2");
//    SensorData_t data;
//    osEvent evt;
//    for(;;)
//    {
//        evt = osMessageGet(sensorQueueHandle, osWaitForever);
//        if(evt.status == osEventMessage) {
//            data = *(SensorData_t*)evt.value.p;
//            data.fre2 = Frequency2_Read(); // Đọc tần số fre2
//            osMessagePut(sensorQueueHandle, (uint32_t)&data, osWaitForever);
//        }
//        osDelay(1000);
//        UART_SendString(&huart2, "EndTask_Fre");
//    }
//  /* USER CODE END StartTaskFre_2 */
//}
//
///* USER CODE BEGIN Header_StartTask05 */
///**
//* @brief Function implementing the Task_Lcd thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTask05 */
//void StartTask05(void const * argument)
//{
//  /* USER CODE BEGIN StartTask05 */
//	UART_SendString(&huart2, "StartTask_LCD");
//    SensorData_t data;
//    osEvent evt;
//    for(;;)
//    {
//        evt = osMessageGet(sensorQueueHandle, osWaitForever);
//        if(evt.status == osEventMessage) {
//            data = *(SensorData_t*)evt.value.p;
//            LCD_PrintFloat(0, 0, "Temp", data.temperature, "C");
//            LCD_PrintFloat(1, 0, "Hum", data.humidity, "%");
//            osMessagePut(sensorQueueHandle, (uint32_t)&data, osWaitForever);
//        }
//        osDelay(1000);
//    }
//  /* USER CODE END StartTask05 */
//}
//
///* USER CODE BEGIN Header_StartTaskUart */
///**
//* @brief Function implementing the TaskUart thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTaskUart */
//void StartTaskUart(void const * argument)
//{
//  /* USER CODE BEGIN StartTaskUart */
//	UART_SendString(&huart2, "StartTask_UART");
//	 SensorData_t data;
//	    osEvent evt;
//	    for(;;)
//	    {
//	        evt = osMessageGet(sensorQueueHandle, osWaitForever);
//	        if(evt.status == osEventMessage) {
//	            data = *(SensorData_t*)evt.value.p;
//	            UART_SendInt(&huart2, data.temperature);
//	            UART_SendInt(&huart2, data.humidity);
//	            UART_SendInt(&huart2, data.fre1);
//	            UART_SendInt(&huart2, data.fre2);
//	            // Không gửi lại queue nữa, kết thúc chu trình
//	        }
//	        osDelay(2000);
//	    }
//  /* USER CODE END StartTaskUart */
//}

void StartTask_Sensor_Dht(void const * argument)
{
    float temperature = 0, humidity = 0;
    for(;;)
    {
        if(DHT11_Read(&temperature, &humidity) == 1) {
            osMutexWait(dataMutexHandle, osWaitForever);
            sharedData.temperature = temperature;
            sharedData.humidity = humidity;
            osMutexRelease(dataMutexHandle);
        }
        osDelay(delayTaskDHT);
    }
}
void StartTaskFre_1(void const * argument)
{
    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);
        sharedData.fre1 = Frequency1_Read();
        osMutexRelease(dataMutexHandle);
        osDelay(delayTaskFre1);
    }
}
void StartTaskFre_2(void const * argument)
{
    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);
        sharedData.fre2 = Frequency2_Read();
        osMutexRelease(dataMutexHandle);
        osDelay(delayTaskFre2);
    }
}
void StartTask05(void const * argument)
{
    SensorData_t temp;
    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);
        temp = sharedData;
        osMutexRelease(dataMutexHandle);

        LCD_PrintFloat(0, 0, "Temp", temp.temperature, "C");
        LCD_PrintFloat(1, 0, "Hum", temp.humidity, "%");
        osDelay(delayTaskLCD);
    }
}
void StartTaskUart(void const * argument)
{
    SensorData_t temp;
    for(;;)
    {
        osMutexWait(dataMutexHandle, osWaitForever);
        temp = sharedData;
        osMutexRelease(dataMutexHandle);
        UART_SendString(&huart2, " \r\n Temp: ");
        UART_SendInt(&huart2, temp.temperature);
        UART_SendString(&huart2, " | Hum: ");
        UART_SendInt(&huart2, temp.humidity);
        UART_SendString(&huart2, " | Fre1: ");
        UART_SendInt(&huart2, temp.fre1);
        UART_SendString(&huart2, " | Fre2: ");
        UART_SendInt(&huart2, temp.fre2);
        osDelay(delayTaskUART);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
