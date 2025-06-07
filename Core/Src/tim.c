/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1;
DMA_HandleTypeDef hdma_tim3_ch2;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim3_ch4;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 DMA Init */
    /* TIM2_CH1 Init */
    hdma_tim2_ch1.Instance = DMA1_Stream0;
    hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim2_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);

    /* TIM2_CH2 Init */
    hdma_tim2_ch2.Instance = DMA1_Stream1;
    hdma_tim2_ch2.Init.Request = DMA_REQUEST_TIM2_CH2;
    hdma_tim2_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim2_ch2.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch2.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2);

    /* TIM2_CH3 Init */
    hdma_tim2_ch3.Instance = DMA1_Stream2;
    hdma_tim2_ch3.Init.Request = DMA_REQUEST_TIM2_CH3;
    hdma_tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim2_ch3.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch3.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim2_ch3);

    /* TIM2_CH4 Init */
    hdma_tim2_ch4.Instance = DMA1_Stream3;
    hdma_tim2_ch4.Init.Request = DMA_REQUEST_TIM2_CH4;
    hdma_tim2_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch4.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch4.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim2_ch4.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch4.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim2_ch4.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch4.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch4);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 DMA Init */
    /* TIM3_CH1 Init */
    hdma_tim3_ch1.Instance = DMA1_Stream4;
    hdma_tim3_ch1.Init.Request = DMA_REQUEST_TIM3_CH1;
    hdma_tim3_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim3_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim3_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim3_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim3_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim3_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim3_ch1);

    /* TIM3_CH2 Init */
    hdma_tim3_ch2.Instance = DMA1_Stream5;
    hdma_tim3_ch2.Init.Request = DMA_REQUEST_TIM3_CH2;
    hdma_tim3_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim3_ch2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim3_ch2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim3_ch2.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim3_ch2.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim3_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim3_ch2);

    /* TIM3_CH3 Init */
    hdma_tim3_ch3.Instance = DMA1_Stream6;
    hdma_tim3_ch3.Init.Request = DMA_REQUEST_TIM3_CH3;
    hdma_tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim3_ch3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim3_ch3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim3_ch3.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim3_ch3.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim3_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim3_ch3);

    /* TIM3_CH4 Init */
    hdma_tim3_ch4.Instance = DMA1_Stream7;
    hdma_tim3_ch4.Init.Request = DMA_REQUEST_TIM3_CH4;
    hdma_tim3_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch4.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch4.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim3_ch4.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim3_ch4.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_tim3_ch4.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim3_ch4.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim3_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim3_ch4);

  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
