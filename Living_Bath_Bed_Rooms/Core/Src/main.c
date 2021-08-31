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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <stdio.h>
#include <string.h>
#include <semphr.h>
#include "FreeRTOS.h"
#include "task.h"
#include "LoRa.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEMP_THRESHOLD 27
#define HUMID_THRESHOLD 80

#define STM32F303K8_DEVICE 0x03

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// -------------------------------------
// Task handlers

xTaskHandle light_liv_handle;
xTaskHandle light_bed_handle;
xTaskHandle light_bath_handle;
xTaskHandle light_fl_handle;
xTaskHandle light_alarm_handle;
xTaskHandle radio_module_handle;
xTaskHandle fan_handle;
xTaskHandle tx_handle;
xTaskHandle rx_handle;


// -------------------------------------
// Boolean variables

uint8_t isLivingroomEmpty=0;
uint8_t isBathroomEmpty=0;
uint8_t isBedroomEmpty=0;
uint8_t isFlood=0;
uint8_t isAlarmOn=0;
uint8_t isAlarmOff=0;


// -------------------------------------
// Variables related with temperature and humidity

uint8_t temperature=30;

uint8_t humidity_measure=0;
uint8_t humidity=0;

// -------------------------------------
// Variables related with data and time

uint8_t hours=10;
uint8_t minutes=0;
uint8_t seconds=0;

uint8_t day=0;
uint8_t month=0;
uint8_t year=0;
uint8_t day_week=0;


// -------------------------------------
// Transmit and receive buffor

// -------------------------------------
// tx_frame : | device | humidity | livingroom | bathroom | bedroom | alarm switch-off | alarm active |  fan  |
//				  8bit      8bit	   	1bit		1bit		1bit		  1bit				1bit	 1bit

uint8_t tx_frame[3] = { 0x03 , 0x00 , 0b01000000 };
uint8_t rx_frame[11]= {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 };

char tx_test[256] = "TEST FreeRTOS 303!      ";

enum devices {begin, RPi , STM32_F4 , STM32_F3 , end};
enum devices curr_dev=begin;

// Continuous updatable time buffor
uint8_t buffor[8] = {0,1,2,3,4,5,6,7};
// Current time send
uint8_t real_time[8] = { 1 , 21 , 7 , 3 , 13 , 20 , 0 ,2};


//static SemaphoreHandle_t mutex;



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

void TX_radio		(void *pvParameters);
void RX_radio		(void *pvParameters);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void light_livingroom   (void *pvParameters){
	while(1){

		isLivingroomEmpty = HAL_GPIO_ReadPin(MOTION_LIVING_GPIO_Port, MOTION_LIVING_Pin);

		if( !isLivingroomEmpty ){
			HAL_GPIO_WritePin(LIGHT_LIVING_GPIO_Port, LIGHT_LIVING_Pin, GPIO_PIN_SET);
			tx_frame[2] = tx_frame[2] | (1<<5);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_LIVING_GPIO_Port, LIGHT_LIVING_Pin, GPIO_PIN_RESET);
			tx_frame[2] = tx_frame[2] & ~(1<<5);
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
int a=0;
void light_bathroom   	(void *pvParameters){
	while(1){

		isBathroomEmpty = HAL_GPIO_ReadPin(MOTION_BATH_GPIO_Port, MOTION_BATH_Pin);

		if( !isBathroomEmpty ){
			HAL_GPIO_WritePin(LIGHT_BATH_GPIO_Port, LIGHT_BATH_Pin, GPIO_PIN_SET);
			tx_frame[2] = tx_frame[2] | (1<<4);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_BATH_GPIO_Port, LIGHT_BATH_Pin, GPIO_PIN_RESET);
			tx_frame[2] = tx_frame[2] & ~(1<<4);
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void light_bedroom   	(void *pvParameters){
	while(1){

		isBedroomEmpty = HAL_GPIO_ReadPin(MOTION_BED_GPIO_Port, MOTION_BED_Pin);

		if( !isBedroomEmpty ){
			HAL_GPIO_WritePin(LIGHT_BED_GPIO_Port, LIGHT_BED_Pin, GPIO_PIN_SET);
			tx_frame[2] = tx_frame[2] | (1<<3);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_BED_GPIO_Port, LIGHT_BED_Pin, GPIO_PIN_RESET);
			tx_frame[2] = tx_frame[2] & ~(1<<3);
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}



void ceiling_fan 	 	(void *pvParameters){
	while(1){

		if( temperature > TEMP_THRESHOLD ){
			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
			tx_frame[2] = tx_frame[2] | 1;
		}
		else{
			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
			tx_frame[2] = tx_frame[2] & ~1;
		}

		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void flood_protection 	(void *pvParameters){
	while(1){
		  HAL_ADC_Start(&hadc2);
		  vTaskDelay( 10 / portTICK_PERIOD_MS);

		  //humidity_measure = HAL_ADC_GetValue(&hadc2);
		  humidity = (HAL_ADC_GetValue(&hadc2) * 70)/113 + 20;

		  if(humidity > 90)
			  humidity = 90;

		  if( humidity >= 50)
			HAL_GPIO_WritePin(FLOOD_ALARM_GPIO_Port, FLOOD_ALARM_Pin, GPIO_PIN_SET);
		  else
			HAL_GPIO_WritePin(FLOOD_ALARM_GPIO_Port, FLOOD_ALARM_Pin, GPIO_PIN_RESET);

	}
	vTaskDelete(NULL);
}

void alarm_clock		(void *pvParameters){
	uint8_t alarm_off=0;
	while(1){
		if( !alarm_off && hours == 10 && HAL_GPIO_ReadPin(BUZZER_OFF_GPIO_Port, BUZZER_OFF_Pin) != GPIO_PIN_RESET){
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			tx_frame[2] = tx_frame[2] | (1<<1);
		}

		if( HAL_GPIO_ReadPin(BUZZER_OFF_GPIO_Port, BUZZER_OFF_Pin) == GPIO_PIN_RESET ){
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			tx_frame[2] = tx_frame[2] | (1<<2);
			tx_frame[2] = tx_frame[2] & ~(1<<1);
			alarm_off = 1;
		}
		else
			tx_frame[2] = tx_frame[2] & ~(1<<2);

		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

int cnt=0;
void TX_radio(void *pvParameters){
	vTaskDelay(30 / portTICK_PERIOD_MS);
	while(1){
		tx_frame[1] = humidity;
		vTaskDelay( 30 / portTICK_PERIOD_MS);
		Transmit(tx_frame, sizeof(tx_frame));

		HAL_GPIO_WritePin(LIGHT_BATH_GPIO_Port, LIGHT_BATH_Pin, GPIO_PIN_SET);

		vTaskDelay( 20 / portTICK_PERIOD_MS);
		vTaskResume(rx_handle);
		vTaskSuspend( NULL );
	}
	vTaskDelete(NULL);
}

void RX_radio(void *pvParameters){
	unsigned char help[11];
	uint32_t time=0;
	while(1){
		vTaskSuspend(tx_handle);

		set_OPMODE(OPMODE_RX_SINGLE);
		time = HAL_GetTick();
		while( (HAL_GetTick()-time < 110) &&
				HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_RESET ){
//				printf("czas --------------------> %d \r\n",HAL_GetTick());
		}
		if( HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_SET ){
			Receive(rx_frame);
//			rx_frame[0]=0x01;
//			rx_frame[1]=STM32F303K8_DEVICE;

			if( rx_frame[0] == 0x02 ){
				printf("dev=%d \t %d:%d:%d \r\n",rx_frame[0] , rx_frame[4], rx_frame[5] , rx_frame[6]);
			}
			if( rx_frame[0] == 0x01 && rx_frame[1] == STM32F303K8_DEVICE){
				vTaskResume(tx_handle);
				vTaskSuspend( NULL );

				Write_Reg(REG_HOP_PERIOD,0x00);
				Write_Reg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
				Write_Reg(REG_IRQ_FLAGS, 0xFF);
				Write_Reg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
				Write_Reg(REG_FIFO_RX_BASE_AD, 0x00);
				Write_Reg(REG_FIFO_ADDR_PTR, 0x00);
			}
			printf("RPI data = %d - %d - %d \r\n" , rx_frame[0], rx_frame[1], rx_frame[2]);
		}
		else{
			printf("No carrier found. \r\n");
		}
		rx_frame[0]=0x00;
		rx_frame[1]=0x00;

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
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc2);
  LoRa_init(868);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  tx_frame[0] = STM32F303K8_DEVICE;

  xTaskCreate( flood_protection , "FLOOD_PROTECTION_TASK"	, 180, NULL, 1, &light_fl_handle );
  xTaskCreate( TX_radio			, "RADIO_TRANSMIT_TASK"		, 150, NULL, 1, &tx_handle );
  xTaskCreate( RX_radio			, "RADIO_RECEIVE_TASK"		, 250, NULL, 1, &rx_handle );

  xTaskCreate( alarm_clock		, "ALARM_CLOCK_TASK"		, 90, NULL, 1, &light_alarm_handle );
  xTaskCreate( ceiling_fan		, "CEILING_FAN_TASK" 		, 70, NULL, 1, &fan_handle );

  xTaskCreate( light_livingroom	, "LIGHT_LIVINGROOM_TASK"	, 70, NULL, 1, &light_liv_handle );
  xTaskCreate( light_bathroom	, "LIGHT_BATHROOM_TASK"		, 70, NULL, 1, &light_bath_handle );
  xTaskCreate( light_bedroom	, "LIGHT_BEDROOM_TASK"		, 70, NULL, 1, &light_bed_handle );



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
	vTaskDelay( 2 / portTICK_PERIOD_MS);

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
