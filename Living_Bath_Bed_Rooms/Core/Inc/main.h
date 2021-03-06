/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

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
#define DIO0_Pin GPIO_PIN_1
#define DIO0_GPIO_Port GPIOF
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_1
#define FAN_GPIO_Port GPIOA
#define FLOOD_ALARM_Pin GPIO_PIN_3
#define FLOOD_ALARM_GPIO_Port GPIOA
#define HUMIDITY_Pin GPIO_PIN_4
#define HUMIDITY_GPIO_Port GPIOA
#define MOTION_LIVING_Pin GPIO_PIN_5
#define MOTION_LIVING_GPIO_Port GPIOA
#define MOTION_BATH_Pin GPIO_PIN_6
#define MOTION_BATH_GPIO_Port GPIOA
#define MOTION_BED_Pin GPIO_PIN_7
#define MOTION_BED_GPIO_Port GPIOA
#define LIGHT_LIVING_Pin GPIO_PIN_0
#define LIGHT_LIVING_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_8
#define RST_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_11
#define CS_GPIO_Port GPIOA
#define BUZZER_OFF_Pin GPIO_PIN_12
#define BUZZER_OFF_GPIO_Port GPIOA
#define LIGHT_BED_Pin GPIO_PIN_6
#define LIGHT_BED_GPIO_Port GPIOB
#define LIGHT_BATH_Pin GPIO_PIN_7
#define LIGHT_BATH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
