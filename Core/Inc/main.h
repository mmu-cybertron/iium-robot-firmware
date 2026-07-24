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
#define FRONT_IR_Pin GPIO_PIN_2
#define FRONT_IR_GPIO_Port GPIOA
#define RIGHT_IR_Pin GPIO_PIN_3
#define RIGHT_IR_GPIO_Port GPIOA
#define IR_BR_Pin GPIO_PIN_5
#define IR_BR_GPIO_Port GPIOA
#define IR_BL_Pin GPIO_PIN_6
#define IR_BL_GPIO_Port GPIOA
#define LEFT_IR_Pin GPIO_PIN_7
#define LEFT_IR_GPIO_Port GPIOA
#define IR_FR_Pin GPIO_PIN_0
#define IR_FR_GPIO_Port GPIOB
#define IR_FL_Pin GPIO_PIN_1
#define IR_FL_GPIO_Port GPIOB
#define Mode_Button_Pin GPIO_PIN_12
#define Mode_Button_GPIO_Port GPIOB
#define Confirm_Button_Pin GPIO_PIN_13
#define Confirm_Button_GPIO_Port GPIOB
#define LED_D6_Pin GPIO_PIN_14
#define LED_D6_GPIO_Port GPIOB
#define LED_D7_Pin GPIO_PIN_15
#define LED_D7_GPIO_Port GPIOB
#define LED_D8_Pin GPIO_PIN_8
#define LED_D8_GPIO_Port GPIOA
#define PWM_RIGHT_Pin GPIO_PIN_10
#define PWM_RIGHT_GPIO_Port GPIOA
#define XSHUT_1_Pin GPIO_PIN_3
#define XSHUT_1_GPIO_Port GPIOB
#define XSHUT_2_Pin GPIO_PIN_4
#define XSHUT_2_GPIO_Port GPIOB
#define XSHUT_3_Pin GPIO_PIN_5
#define XSHUT_3_GPIO_Port GPIOB
#define XSHUT_4_Pin GPIO_PIN_6
#define XSHUT_4_GPIO_Port GPIOB
#define PWM_LEFT_Pin GPIO_PIN_9
#define PWM_LEFT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* 
 * Fallbacks for pins removed from the .ioc file.
 * We map LED_D6 to LED_D7 so the code still compiles safely,
 * and we restore SM_Signal to its original PC13 just in case it was accidentally removed.
 */
#ifndef LED_D6_Pin
#define LED_D6_Pin LED_D7_Pin
#define LED_D6_GPIO_Port LED_D7_GPIO_Port
#endif

#ifndef SM_Signal_Pin
#define SM_Signal_Pin GPIO_PIN_13
#define SM_Signal_GPIO_Port GPIOC
#endif

#ifndef IR_BL_Pin
#define IR_BL_Pin GPIO_PIN_6
#define IR_BL_GPIO_Port GPIOA
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
