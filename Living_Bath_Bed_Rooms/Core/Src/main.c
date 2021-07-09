/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <stdio.h>
#include <semphr.h>
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEMP_THRESHOLD 27
#define HUMID_THRESHOLD 80

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

xTaskHandle light_liv_handle;
xTaskHandle light_bed_handle;
xTaskHandle light_bath_handle;
xTaskHandle light_fl_handle;
xTaskHandle light_alarm_handle;
xTaskHandle radio_module_handle;

uint8_t temperature_measure=0;
uint8_t temperature=0;

uint8_t humidity_measure=0;
uint8_t humidity=0;


uint8_t isLivingroomEmpty=0;
uint8_t isBathroomEmpty=0;
uint8_t isBedroomEmpty=0;
uint8_t isFlood=0;
uint8_t isAlarmOn=0;
uint8_t isAlarmOff=0;

uint8_t time=0;
uint8_t hours=0;
uint8_t minutes=0;
uint8_t seconds=0;



static SemaphoreHandle_t mutex;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void light_livingroom   (void *pvParameters);
void light_bathroom   	(void *pvParameters);
void light_bedroom   	(void *pvParameters);
void ceiling_fan		(void *pvParameters);
void flood_protection 	(void *pvParameters);
void alarm_clock		(void *pvParameters);
void radio_module		(void *pvParameters);

void TX_radio();
void RX_radio();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file , char *ptr , int len){
  HAL_UART_Transmit(&huart1 , ptr , len , 50);
  return len;
}



void light_livingroom   (void *pvParameters){
	while(1){

		isLivingroomEmpty = HAL_GPIO_ReadPin(MOTION_LIVING_GPIO_Port, MOTION_LIVING_Pin);

		if( !isLivingroomEmpty )
			HAL_GPIO_WritePin(LIGHT_LIVING_GPIO_Port, LIGHT_LIVING_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_LIVING_GPIO_Port, LIGHT_LIVING_Pin, GPIO_PIN_RESET);

	}
	vTaskDelete(NULL);
}
int a=0;
void light_bathroom   	(void *pvParameters){
	while(1){

		isBathroomEmpty = HAL_GPIO_ReadPin(MOTION_BATH_GPIO_Port, MOTION_BATH_Pin);

		if( !isBathroomEmpty )
			HAL_GPIO_WritePin(LIGHT_BATH_GPIO_Port, LIGHT_BATH_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_BATH_GPIO_Port, LIGHT_BATH_Pin, GPIO_PIN_RESET);

	}
	vTaskDelete(NULL);
}

void light_bedroom   	(void *pvParameters){
	while(1){

		isBedroomEmpty = HAL_GPIO_ReadPin(MOTION_BED_GPIO_Port, MOTION_BED_Pin);

		if( !isBedroomEmpty )
			HAL_GPIO_WritePin(LIGHT_BED_GPIO_Port, LIGHT_BED_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_BED_GPIO_Port, LIGHT_BED_Pin, GPIO_PIN_RESET);

	}
	vTaskDelete(NULL);
}



void ceiling_fan 	 	(void *pvParameters){
	while(1){

		temperature = ( temperature_measure * 50) / 255;

		if( temperature > TEMP_THRESHOLD )
			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}
int b=0;
void flood_protection 	(void *pvParameters){
	while(1){

		  humidity_measure = HAL_ADC_GetValue(&hadc2);
		  humidity = (humidity_measure * 70)/127  + 20;

		  HAL_ADC_Start(&hadc2);

		  //printf(" Humidity : %d , measure : %d \r\n", humidity , humidity_measure);
		  printf(" Stan : %d \r\n", isBathroomEmpty );

		  //printf(" Stack : %d \r\n", uxTaskGetStackHighWaterMark(light_fl_handle) );
		  //vTaskResume(light_bath_handle);
		  //vTaskResume(light_bed_handle);
			vTaskDelay( 5 / portTICK_PERIOD_MS);

		  if( humidity >= 70)
			HAL_GPIO_WritePin(FLOOD_ALARM_GPIO_Port, FLOOD_ALARM_Pin, GPIO_PIN_SET);
		  else
			HAL_GPIO_WritePin(FLOOD_ALARM_GPIO_Port, FLOOD_ALARM_Pin, GPIO_PIN_RESET);

	}
	vTaskDelete(NULL);
}

void alarm_clock		(void *pvParameters){
	while(1){
		if( hours == 10 )
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

		if( HAL_GPIO_ReadPin(BUZZER_OFF_GPIO_Port, BUZZER_OFF_Pin) == GPIO_PIN_RESET )
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}

void radio_module		(void *pvParameters) {


	HAL_GPIO_WritePin(TxEN_GPIO_Port, TxEN_Pin, GPIO_PIN_SET);
	TX_radio();

	HAL_GPIO_WritePin(TxEN_GPIO_Port, TxEN_Pin, GPIO_PIN_RESET);
	RX_radio();

}

void TX_radio(){
	HAL_GPIO_WritePin(TxEN_GPIO_Port, TxEN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_NRF_GPIO_Port, CS_NRF_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &isLivingroomEmpty	, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &isBathroomEmpty		, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &isBedroomEmpty		, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &isFlood				, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &isAlarmOn			, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &isAlarmOff			, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &humidity				, 1 , 1000);


	HAL_GPIO_WritePin(CS_NRF_GPIO_Port, CS_NRF_Pin, GPIO_PIN_SET);

}

void RX_radio(){
	HAL_GPIO_WritePin(TxEN_GPIO_Port, TxEN_Pin, GPIO_PIN_RESET);


	HAL_GPIO_WritePin(CS_NRF_GPIO_Port, CS_NRF_Pin, GPIO_PIN_RESET);

	HAL_SPI_Receive( &hspi1, &temperature_measure , 1 , 1000);
	HAL_SPI_Receive( &hspi1, &hours , 1 , 1000);
	HAL_SPI_Receive( &hspi1, &minutes , 1 , 1000);
	HAL_SPI_Receive( &hspi1, &seconds , 1 , 1000);

	HAL_GPIO_WritePin(CS_NRF_GPIO_Port, CS_NRF_Pin, GPIO_PIN_SET);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc2);


  xTaskCreate( light_livingroom	, "LIGHT_LIVINGROOM_TASK"	, 100, NULL, 1, light_liv_handle );
  xTaskCreate( light_bathroom	, "LIGHT_BATHROOM_TASK"		, 100, NULL, 1, light_bath_handle );
  xTaskCreate( light_bedroom	, "LIGHT_BEDROOM_TASK"		, 100, NULL, 1, light_bed_handle );

  xTaskCreate( ceiling_fan		, "CEILING_FAN_TASK" 		, 100, NULL, 1, NULL );
  xTaskCreate( flood_protection , "FLOOD_PROTECTION_TASK"	, 200, NULL, 2, light_fl_handle );
  xTaskCreate( alarm_clock		, "ALARM_CLOCK_TASK"		, 100, NULL, 1, light_alarm_handle );

  mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(mutex, portMAX_DELAY);
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1)
  {
	vTaskDelay( 10 / portTICK_PERIOD_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
