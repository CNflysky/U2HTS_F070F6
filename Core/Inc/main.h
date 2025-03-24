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
#include "stm32f0xx_hal.h"

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
#define TP_RST_Pin GPIO_PIN_0
#define TP_RST_GPIO_Port GPIOA
#define TP_INT_Pin GPIO_PIN_1
#define TP_INT_GPIO_Port GPIOA
#define TP_INT_EXTI_IRQn EXTI0_1_IRQn
#define USR_LED_Pin GPIO_PIN_2
#define USR_LED_GPIO_Port GPIOA
#define USR_KEY_Pin GPIO_PIN_3
#define USR_KEY_GPIO_Port GPIOA
#define USR_KEY_EXTI_IRQn EXTI2_3_IRQn
#define TP_SCL_NSS_Pin GPIO_PIN_4
#define TP_SCL_NSS_GPIO_Port GPIOA
#define TP_SDA_Pin GPIO_PIN_1
#define TP_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
