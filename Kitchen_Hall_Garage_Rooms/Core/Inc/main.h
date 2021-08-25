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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SUN_UP_Pin GPIO_PIN_0
#define SUN_UP_GPIO_Port GPIOC
#define SUN_DOWN_Pin GPIO_PIN_1
#define SUN_DOWN_GPIO_Port GPIOC
#define SUN_RIGHT_Pin GPIO_PIN_2
#define SUN_RIGHT_GPIO_Port GPIOC
#define SUN_LEFT_Pin GPIO_PIN_3
#define SUN_LEFT_GPIO_Port GPIOC
#define CS_BMP_Pin GPIO_PIN_5
#define CS_BMP_GPIO_Port GPIOC
#define CS_LORA_Pin GPIO_PIN_0
#define CS_LORA_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_1
#define RST_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_2
#define DIO0_GPIO_Port GPIOB
#define MOTION_KIT_Pin GPIO_PIN_7
#define MOTION_KIT_GPIO_Port GPIOE
#define MOTION_HALL_Pin GPIO_PIN_8
#define MOTION_HALL_GPIO_Port GPIOE
#define MOTION_GAR_Pin GPIO_PIN_9
#define MOTION_GAR_GPIO_Port GPIOE
#define LIGHT_KIT_Pin GPIO_PIN_10
#define LIGHT_KIT_GPIO_Port GPIOE
#define LIGHT_HALL_Pin GPIO_PIN_11
#define LIGHT_HALL_GPIO_Port GPIOE
#define LIGHT_GAR_Pin GPIO_PIN_12
#define LIGHT_GAR_GPIO_Port GPIOE
#define CMS_Pin GPIO_PIN_13
#define CMS_GPIO_Port GPIOE
#define CM_BUZZER_Pin GPIO_PIN_14
#define CM_BUZZER_GPIO_Port GPIOE
#define DOOR_Pin GPIO_PIN_15
#define DOOR_GPIO_Port GPIOE
#define CS_PRESS_Pin GPIO_PIN_10
#define CS_PRESS_GPIO_Port GPIOB
#define PWM_TOP_Pin GPIO_PIN_13
#define PWM_TOP_GPIO_Port GPIOB
#define PWM_BOTTOM_Pin GPIO_PIN_14
#define PWM_BOTTOM_GPIO_Port GPIOB
#define SCLK_RTC_Pin GPIO_PIN_6
#define SCLK_RTC_GPIO_Port GPIOC
#define SDA_RTC_Pin GPIO_PIN_7
#define SDA_RTC_GPIO_Port GPIOC
#define RST_RTC_Pin GPIO_PIN_8
#define RST_RTC_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
