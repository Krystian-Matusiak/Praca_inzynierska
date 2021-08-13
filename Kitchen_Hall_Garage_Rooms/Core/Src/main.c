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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "LoRa.h"
#include "RTC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// -------------------------------------
// Task handlers

xTaskHandle light_kit_handle;
xTaskHandle light_hall_handle;
xTaskHandle light_gar_handle;
xTaskHandle carbon_sensor_handle;
xTaskHandle temp_humid_handle;
xTaskHandle clock_handle;
xTaskHandle door_handle;
xTaskHandle pressure_handle;
xTaskHandle tx_handle;
xTaskHandle rx_handle;



// -------------------------------------
// Boolean variables

uint8_t isKitchenEmpty=0;
uint8_t isHallEmpty=0;
uint8_t isGarageEmpty=0;
uint8_t isCar=0;
uint8_t isCMSensor=0;
uint8_t isDoorClosed=0;


// -------------------------------------
// Variables related with temperature and pressure

uint8_t temperature_measure=0;
uint8_t temperature=0;

uint16_t press_measure=0;
uint16_t press=0;


// -------------------------------------
// Variables related with data and time

uint8_t hours=0;
uint8_t minutes=0;
uint8_t seconds=0;

uint8_t day=0;
uint8_t month=0;
uint8_t year=0;
uint8_t day_week=0;



// -------------------------------------
// Transmit and receive buffor

char tx_data[256];
char rx_data[256];


// Continuous updatable time buffor
uint8_t buffor[8] = {0,1,2,3,4,5,6,7};
// Current time send
uint8_t real_time[8] = { 1 , 21 , 7 , 3 , 13 , 20 , 0 ,2 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void light_kitchen	(void *pvParameters);
void light_hall		(void *pvParameters);
void light_garage	(void *pvParameters);
void carbon_sensor	(void *pvParameters);
void temp_humid 	(void *pvParameters);
void clock			(void *pvParameters);
void door 			(void *pvParameters);
void pressure		(void *pvParameters);
void sun_tracker	(void *pvParameters);

void TX_radio		(void *pvParameters);
void RX_radio		(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void light_kitchen	(void *pvParameters){
	while(1){

		isKitchenEmpty = HAL_GPIO_ReadPin(MOTION_KIT_GPIO_Port, MOTION_KIT_Pin);

		if( !isKitchenEmpty )
			HAL_GPIO_WritePin(LIGHT_KIT_GPIO_Port, LIGHT_KIT_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_KIT_GPIO_Port, LIGHT_KIT_Pin, GPIO_PIN_RESET);

		vTaskDelay( 100 / portTICK_PERIOD_MS);

	}
	vTaskDelete(NULL);
}

void light_hall		(void *pvParameters){
	while(1){

		isHallEmpty = HAL_GPIO_ReadPin(MOTION_HALL_GPIO_Port, MOTION_HALL_Pin);

		if( !isHallEmpty )
			HAL_GPIO_WritePin(LIGHT_HALL_GPIO_Port, LIGHT_HALL_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_HALL_GPIO_Port, LIGHT_HALL_Pin, GPIO_PIN_RESET);

		vTaskDelay( 100 / portTICK_PERIOD_MS);

	}
	vTaskDelete(NULL);
}

void light_garage	(void *pvParameters){
	while(1){

		isGarageEmpty = HAL_GPIO_ReadPin(MOTION_GAR_GPIO_Port, MOTION_GAR_Pin);

		if( !isGarageEmpty )
			HAL_GPIO_WritePin(LIGHT_GAR_GPIO_Port, LIGHT_GAR_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(LIGHT_GAR_GPIO_Port, LIGHT_GAR_Pin, GPIO_PIN_RESET);

		vTaskDelay( 100 / portTICK_PERIOD_MS);

	}
	vTaskDelete(NULL);
}

void carbon_sensor	(void *pvParameters){
	while(1){

		isCMSensor = HAL_GPIO_ReadPin(CMS_GPIO_Port, CMS_Pin);

		if( !isGarageEmpty )
			HAL_GPIO_WritePin(CM_BUZZER_GPIO_Port, CM_BUZZER_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(CM_BUZZER_GPIO_Port, CM_BUZZER_Pin, GPIO_PIN_RESET);

		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void temp_humid 	(void *pvParameters){
	while(1){



	}
	vTaskDelete(NULL);
}

void clock			(void *pvParameters){
	while(1){
		Read_Time(buffor);

		year = buffor[1];
		month = buffor[2];
		day = buffor[3];

		hours = buffor[4];
		minutes = buffor[5];
		seconds = buffor[6];

		vTaskDelay( 500 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void door 			(void *pvParameters){
	while(1){
		isDoorClosed = HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin);
		vTaskDelay( 300 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void pressure		(void *pvParameters){
	while(1){



	}
	vTaskDelete(NULL);
}

void sun_tracker	(void *pvParameters){
	while(1){



	}
	vTaskDelete(NULL);
}



void TX_radio(void *pvParameters){
	while(1){
		set_OPMODE(OPMODE_TX);
		Transmit(tx_data, strlen((char*)tx_data));
		vTaskDelay( 100 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void RX_radio(void *pvParameters){
	while(1){
		set_OPMODE(OPMODE_RX);
		Receive(rx_data);
		vTaskDelay( 400 / portTICK_PERIOD_MS);
	}
vTaskDelete(NULL);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  xTaskCreate( light_kitchen	, "LIGHT_KITCHEN_TASK"		, 100, NULL, 1, light_kit_handle );
  xTaskCreate( light_hall		, "LIGHT_HALL_TASK"			, 100, NULL, 1, light_hall_handle );
  xTaskCreate( light_garage		, "LIGHT_GARAGE_TASK"		, 100, NULL, 1, light_gar_handle );

  xTaskCreate( carbon_sensor	, "CARBON_SENSOR_TASK" 		, 100, NULL, 1, carbon_sensor_handle );
  xTaskCreate( temp_humid 		, "FLOOD_PROTECTION_TASK"	, 200, NULL, 2, temp_humid_handle );
  xTaskCreate( pressure			, "PRESSURE_TASK"			, 100, NULL, 1, pressure_handle );

  xTaskCreate( clock			, "CLOCK"					, 100, NULL, 2, clock_handle );
  xTaskCreate( door				, "DOOR"					, 100, NULL, 1, door_handle );

  xTaskCreate( TX_radio			, "RADIO_TRANSMIT"			, 100, NULL, 2, tx_handle );
  xTaskCreate( RX_radio			, "RADIO_RECEIVE"			, 100, NULL, 1, rx_handle );


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  RTC_Init();
  LoRa_init(868);

  while (1)
  {

	//vTaskDelay( 10 / portTICK_PERIOD_MS);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
