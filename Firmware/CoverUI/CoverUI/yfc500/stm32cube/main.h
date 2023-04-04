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
#define TIM_BLINK_SLOW TIM16
#define TIM_BLINK_FAST TIM17
#define UART_LL USART2
#define UART_U4 USART1
#define LED_LIFTED_Pin GPIO_PIN_0
#define LED_LIFTED_GPIO_Port GPIOC
#define LED_WIRE_Pin GPIO_PIN_1
#define LED_WIRE_GPIO_Port GPIOC
#define LED_BAT_Pin GPIO_PIN_2
#define LED_BAT_GPIO_Port GPIOC
#define LED_CHARGE_Pin GPIO_PIN_3
#define LED_CHARGE_GPIO_Port GPIOC
#define LED_S1_Pin GPIO_PIN_0
#define LED_S1_GPIO_Port GPIOA
#define LED_S2_Pin GPIO_PIN_1
#define LED_S2_GPIO_Port GPIOA
#define UART_LL_TX_Pin GPIO_PIN_2
#define UART_LL_TX_GPIO_Port GPIOA
#define UART_LL_RX_Pin GPIO_PIN_3
#define UART_LL_RX_GPIO_Port GPIOA
#define LED_2HR_Pin GPIO_PIN_4
#define LED_2HR_GPIO_Port GPIOA
#define LED_4HR_Pin GPIO_PIN_5
#define LED_4HR_GPIO_Port GPIOA
#define LED_6HR_Pin GPIO_PIN_6
#define LED_6HR_GPIO_Port GPIOA
#define LED_LOCK_Pin GPIO_PIN_7
#define LED_LOCK_GPIO_Port GPIOA
#define LED_LOCKC4_Pin GPIO_PIN_4
#define LED_LOCKC4_GPIO_Port GPIOC
#define LED_PCB_Pin GPIO_PIN_0
#define LED_PCB_GPIO_Port GPIOB
#define LED_MON_Pin GPIO_PIN_15
#define LED_MON_GPIO_Port GPIOA
#define LED_TUE_Pin GPIO_PIN_10
#define LED_TUE_GPIO_Port GPIOC
#define LED_WED_Pin GPIO_PIN_11
#define LED_WED_GPIO_Port GPIOC
#define LED_THU_Pin GPIO_PIN_12
#define LED_THU_GPIO_Port GPIOC
#define LED_FRI_Pin GPIO_PIN_2
#define LED_FRI_GPIO_Port GPIOD
#define LED_SAT_Pin GPIO_PIN_3
#define LED_SAT_GPIO_Port GPIOB
#define LED_SUN_Pin GPIO_PIN_4
#define LED_SUN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
