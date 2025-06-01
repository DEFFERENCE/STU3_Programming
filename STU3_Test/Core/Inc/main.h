/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Current_Sensor_Pin GPIO_PIN_0
#define Current_Sensor_GPIO_Port GPIOA
#define Current_SensorA1_Pin GPIO_PIN_1
#define Current_SensorA1_GPIO_Port GPIOA
#define Encoder_1_Pin GPIO_PIN_6
#define Encoder_1_GPIO_Port GPIOA
#define Encoder_1A7_Pin GPIO_PIN_7
#define Encoder_1A7_GPIO_Port GPIOA
#define DIR_MD20A_24V_Pin GPIO_PIN_1
#define DIR_MD20A_24V_GPIO_Port GPIOB
#define PWM_MD20A_24V_Pin GPIO_PIN_2
#define PWM_MD20A_24V_GPIO_Port GPIOB
#define Limit_Switch_pen_2_Pin GPIO_PIN_10
#define Limit_Switch_pen_2_GPIO_Port GPIOB
#define Limit_Switch_pen_2_EXTI_IRQn EXTI15_10_IRQn
#define Proximity_Sensor_Pin GPIO_PIN_13
#define Proximity_Sensor_GPIO_Port GPIOB
#define Proximity_Sensor_EXTI_IRQn EXTI15_10_IRQn
#define Limit_Switch_Prismatic1_Pin GPIO_PIN_14
#define Limit_Switch_Prismatic1_GPIO_Port GPIOB
#define Limit_Switch_Prismatic1_EXTI_IRQn EXTI15_10_IRQn
#define Limit_Switch_Prismatic2_Pin GPIO_PIN_15
#define Limit_Switch_Prismatic2_GPIO_Port GPIOB
#define Limit_Switch_Prismatic2_EXTI_IRQn EXTI15_10_IRQn
#define DIR_MD20A_18V_Pin GPIO_PIN_6
#define DIR_MD20A_18V_GPIO_Port GPIOC
#define PWM_MD20A_18V_Pin GPIO_PIN_8
#define PWM_MD20A_18V_GPIO_Port GPIOC
#define PWM_Servo_Pin GPIO_PIN_9
#define PWM_Servo_GPIO_Port GPIOC
#define Encoder_2_Pin GPIO_PIN_11
#define Encoder_2_GPIO_Port GPIOA
#define Encoder_2A12_Pin GPIO_PIN_12
#define Encoder_2A12_GPIO_Port GPIOA
#define PS2_Joy_stick_Attention_Pin GPIO_PIN_10
#define PS2_Joy_stick_Attention_GPIO_Port GPIOC
#define Emergency_Pin GPIO_PIN_11
#define Emergency_GPIO_Port GPIOC
#define Emergency_EXTI_IRQn EXTI15_10_IRQn
#define Proximity_sensor_Pin GPIO_PIN_12
#define Proximity_sensor_GPIO_Port GPIOC
#define Proximity_sensor_EXTI_IRQn EXTI15_10_IRQn
#define Limit_Switch_pen_1_Pin GPIO_PIN_9
#define Limit_Switch_pen_1_GPIO_Port GPIOB
#define Limit_Switch_pen_1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
