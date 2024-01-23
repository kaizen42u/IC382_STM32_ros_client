/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CORE_LED1_Pin GPIO_PIN_5
#define CORE_LED1_GPIO_Port GPIOE
#define USER_SWITCH_1_Pin GPIO_PIN_0
#define USER_SWITCH_1_GPIO_Port GPIOF
#define USER_SWITCH_1_EXTI_IRQn EXTI0_IRQn
#define USER_SWITCH_2_Pin GPIO_PIN_1
#define USER_SWITCH_2_GPIO_Port GPIOF
#define USER_SWITCH_2_EXTI_IRQn EXTI1_IRQn
#define USER_SWITCH_3_Pin GPIO_PIN_2
#define USER_SWITCH_3_GPIO_Port GPIOF
#define USER_SWITCH_3_EXTI_IRQn EXTI2_IRQn
#define USER_SWITCH_4_Pin GPIO_PIN_3
#define USER_SWITCH_4_GPIO_Port GPIOF
#define USER_SWITCH_4_EXTI_IRQn EXTI3_IRQn
#define M1_IN2_Pin GPIO_PIN_6
#define M1_IN2_GPIO_Port GPIOF
#define M1_IN1_Pin GPIO_PIN_7
#define M1_IN1_GPIO_Port GPIOF
#define M2_IN2_Pin GPIO_PIN_8
#define M2_IN2_GPIO_Port GPIOF
#define M2_IN1_Pin GPIO_PIN_9
#define M2_IN1_GPIO_Port GPIOF
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define EXTENSION_LED3_Pin GPIO_PIN_0
#define EXTENSION_LED3_GPIO_Port GPIOD
#define EXTENSION_LED4_Pin GPIO_PIN_1
#define EXTENSION_LED4_GPIO_Port GPIOD
#define CORE_LED0_Pin GPIO_PIN_5
#define CORE_LED0_GPIO_Port GPIOB
#define EXTENSION_LED1_Pin GPIO_PIN_8
#define EXTENSION_LED1_GPIO_Port GPIOB
#define EXTENSION_LED2_Pin GPIO_PIN_9
#define EXTENSION_LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
