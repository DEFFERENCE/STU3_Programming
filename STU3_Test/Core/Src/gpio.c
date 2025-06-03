/* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file    gpio.c
//  * @brief   This file provides code for the configuration
//  *          of all used GPIO pins.
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
//
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
//
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_MD20A_24V_GPIO_Port, DIR_MD20A_24V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_MD20A_18V_GPIO_Port, DIR_MD20A_18V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PS2_Attention_GPIO_Port, PS2_Attention_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DIR_MD20A_24V_Pin */
  GPIO_InitStruct.Pin = DIR_MD20A_24V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_MD20A_24V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_Switch_pen_2_Pin Limit_Switch_Prismatic1_Pin Limit_Switch_Prismatic2_Pin Limit_Switch_pen_1_Pin */
  GPIO_InitStruct.Pin = Limit_Switch_pen_2_Pin|Limit_Switch_Prismatic1_Pin|Limit_Switch_Prismatic2_Pin|Limit_Switch_pen_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_MD20A_18V_Pin PS2_Attention_Pin */
  GPIO_InitStruct.Pin = DIR_MD20A_18V_Pin|PS2_Attention_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergency_Pin */
  GPIO_InitStruct.Pin = Emergency_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Emergency_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Proximity_sensor_Pin */
  GPIO_InitStruct.Pin = Proximity_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Proximity_sensor_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
//
/* USER CODE END 2 */
