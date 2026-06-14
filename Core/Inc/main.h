/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define SM_Signal_Pin GPIO_PIN_13
#define SM_Signal_GPIO_Port GPIOC
#define LEFT_DIR_Pin GPIO_PIN_4
#define LEFT_DIR_GPIO_Port GPIOA
#define RIGHT_DIR_Pin GPIO_PIN_6
#define RIGHT_DIR_GPIO_Port GPIOA
#define IR4_DO_Pin GPIO_PIN_0
#define IR4_DO_GPIO_Port GPIOB
#define IR3_DO_Pin GPIO_PIN_1
#define IR3_DO_GPIO_Port GPIOB
#define IR2_DO_Pin GPIO_PIN_2
#define IR2_DO_GPIO_Port GPIOB
#define IR1_DO_Pin GPIO_PIN_10
#define IR1_DO_GPIO_Port GPIOB
#define XSHUT_5_Pin GPIO_PIN_3
#define XSHUT_5_GPIO_Port GPIOB
#define XSHUT_4_Pin GPIO_PIN_4
#define XSHUT_4_GPIO_Port GPIOB
#define XSHUT_3_Pin GPIO_PIN_5
#define XSHUT_3_GPIO_Port GPIOB
#define XSHUT_2_Pin GPIO_PIN_6
#define XSHUT_2_GPIO_Port GPIOB
#define XSHUT_1_Pin GPIO_PIN_7
#define XSHUT_1_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */

   /* GPIO Pin definitions for buttons */
#define MODE_PIN_PB12           GPIO_PIN_12
#define MODE_PIN_GPIO_Port_PB12 GPIOB
#define CONFIRM_PIN_PB13        GPIO_PIN_13
#define CONFIRM_PIN_GPIO_Port_PB13 GPIOB

/* GPIO Pin definitions for LEDs */
#define MODE1_LED_PIN       GPIO_PIN_15
#define MODE1_LED_GPIO_Port GPIOB
#define MODE2_LED_PIN       GPIO_PIN_14
#define MODE2_LED_GPIO_Port GPIOB
#define MODE3_LED_PIN       GPIO_PIN_8
#define MODE3_LED_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
