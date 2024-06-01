/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define HEARBEAT_LED_Pin GPIO_PIN_13
#define HEARBEAT_LED_GPIO_Port GPIOC
#define IR0_Pin GPIO_PIN_0
#define IR0_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_1
#define IR1_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_2
#define IR2_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_3
#define IR3_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_4
#define IR4_GPIO_Port GPIOA
#define IR5_Pin GPIO_PIN_5
#define IR5_GPIO_Port GPIOA
#define IR6_Pin GPIO_PIN_6
#define IR6_GPIO_Port GPIOA
#define IR7_Pin GPIO_PIN_7
#define IR7_GPIO_Port GPIOA
#define AUX1_ESP_Pin GPIO_PIN_1
#define AUX1_ESP_GPIO_Port GPIOB
#define BUTTOM_MODE_Pin GPIO_PIN_4
#define BUTTOM_MODE_GPIO_Port GPIOB
#define BUTTOM_MODE_EXTI_IRQn EXTI4_IRQn
#define LED_MODE_Pin GPIO_PIN_5
#define LED_MODE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
