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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "LoRa.h"
#include "bmp280.h"
#include "RTC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STM32F411_DEVICE 0x02

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
xTaskHandle clock_handle;
xTaskHandle door_handle;
xTaskHandle car_handle;
xTaskHandle pressure_handle;
xTaskHandle tx_handle;
xTaskHandle rx_handle;


// -------------------------------------
// PID and sun tracker variables

typedef struct Servo{
	uint8_t Kp;
	uint8_t Ki;
	uint8_t Kd;

	int16_t P;
	int16_t I;
	int16_t D;

	int16_t error;
	int16_t prev_error;
}Servo;

Servo Top;
Servo Bottom;

void servo_init(){
	Top.Kp = 1;
	Top.Ki = 1;
	Top.Kd = 1;
	Top.error = 0;
	Top.prev_error = 0;


	Bottom.Kp = 1;
	Bottom.Ki = 1;
	Bottom.Kd = 1;
	Bottom.error = 0;
	Bottom.prev_error = 0;
}

int32_t positionTOP = 0;
int32_t positionBOTTOM = 0;

// -------------------------------------
// Boolean variables

uint8_t isKitchenEmpty=0;
uint8_t isHallEmpty=0;
uint8_t isGarageEmpty=0;
uint8_t isCar=0;
uint8_t isCMSensor=0;
uint8_t isDoorClosed=0;


// -------------------------------------
// Variables related with temperature, pressure and ph_resis

int32_t temp ;
uint32_t press;

volatile uint8_t ph_resis[4];

// -------------------------------------
// Variables related with data and time

uint8_t hours=0;
uint8_t minutes=0;
uint8_t seconds=1;

uint8_t day=0;
uint8_t month=0;
uint8_t year=0;
uint8_t day_week=0;

// -------------------------------------
// Transmit and receive buffor

// -------------------------------------
// tx_frame : | device | pressure | temp | clock | kitchen | hall | garage | CMS | door |
//				  8bit      16bit	8bit   5x8bit   1bit	 1bit	  1bit	 1bit   1bit

uint8_t tx_frame[11] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t rx_frame[11] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

char tx_test[256] = "TEST FreeRTOS 411!      ";

enum devices {begin, RPi , STM32_F4 , STM32_F3 , dev4 , dev5 , dev6 ,end};
enum devices curr_dev=begin;

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
void clock			(void *pvParameters);
void door 			(void *pvParameters);
void car 			(void *pvParameters);
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

		if( !isKitchenEmpty ){
			HAL_GPIO_WritePin(LIGHT_KIT_GPIO_Port, LIGHT_KIT_Pin, GPIO_PIN_SET);
			tx_frame[10] = tx_frame[10] | (1<<4);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_KIT_GPIO_Port, LIGHT_KIT_Pin, GPIO_PIN_RESET);
			tx_frame[10] = tx_frame[10] & ~(1<<4);
		}
		vTaskDelay( 20 / portTICK_PERIOD_MS);

	}
	vTaskDelete(NULL);
}

void light_hall		(void *pvParameters){
	while(1){

		isHallEmpty = HAL_GPIO_ReadPin(MOTION_HALL_GPIO_Port, MOTION_HALL_Pin);

		if( !isHallEmpty ){
			HAL_GPIO_WritePin(LIGHT_HALL_GPIO_Port, LIGHT_HALL_Pin, GPIO_PIN_SET);
			tx_frame[10] = tx_frame[10] | (1<<3);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_HALL_GPIO_Port, LIGHT_HALL_Pin, GPIO_PIN_RESET);
			tx_frame[10] = tx_frame[10] & ~(1<<3);
		}
		vTaskDelay( 20 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void light_garage	(void *pvParameters){
	while(1){

		isGarageEmpty = HAL_GPIO_ReadPin(MOTION_GAR_GPIO_Port, MOTION_GAR_Pin);

		if( !isGarageEmpty ){
			HAL_GPIO_WritePin(LIGHT_GAR_GPIO_Port, LIGHT_GAR_Pin, GPIO_PIN_SET);
			tx_frame[10] = tx_frame[10] | (1<<2);
		}
		else{
			HAL_GPIO_WritePin(LIGHT_GAR_GPIO_Port, LIGHT_GAR_Pin, GPIO_PIN_RESET);
			tx_frame[10] = tx_frame[10] & ~(1<<2);
		}
		vTaskDelay( 20 / portTICK_PERIOD_MS);

	}
	vTaskDelete(NULL);
}

void carbon_sensor	(void *pvParameters){
	while(1){

		isCMSensor = HAL_GPIO_ReadPin(CMS_GPIO_Port, CMS_Pin);

		if( !isCMSensor ){
			HAL_GPIO_WritePin(CM_BUZZER_GPIO_Port, CM_BUZZER_Pin, GPIO_PIN_SET);
			tx_frame[10] = tx_frame[10] | (1<<1);
		}
		else{
			HAL_GPIO_WritePin(CM_BUZZER_GPIO_Port, CM_BUZZER_Pin, GPIO_PIN_RESET);
			tx_frame[10] = tx_frame[10] & ~(1<<1);
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void clock			(void *pvParameters){
	while(1){
//		Read_Time(buffor);
//
//		year = buffor[1];
//		month = buffor[2];
//		day = buffor[3];
//
//		hours = buffor[4];
//		minutes = buffor[5];
//		seconds = buffor[6];

		year = 21;
		month = 8;
		day = 24;

		hours = 9;
		minutes = 1;

		tx_frame[4] = hours;
		tx_frame[5] = minutes;
		tx_frame[6] = seconds;
		tx_frame[7] = day;
		tx_frame[8] = month;
		tx_frame[9] = year;
		seconds ++;
		if(seconds == 60){
			seconds=0;
			minutes++;
		}
		vTaskDelay( 500 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void door 			(void *pvParameters){
	while(1){
		isDoorClosed = HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin);
		if( !isCar ){
			tx_frame[10] = tx_frame[10] | (1<<5);
		}
		else{
			tx_frame[10] = tx_frame[10] & ~(1<<5);
		}
		vTaskDelay( 50 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void car 			(void *pvParameters){
	while(1){
		isCar = HAL_GPIO_ReadPin(CAR_GPIO_Port, CAR_Pin);
		vTaskDelay( 300 / portTICK_PERIOD_MS);
		if( isDoorClosed )
			tx_frame[10] = tx_frame[10] | 1;
		else
			tx_frame[10] = tx_frame[10] & ~1;
		vTaskDelay( 50 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void pressure		(void *pvParameters){
	while(1){
		get_temp_press( &temp, &press);
		tx_frame[1] = press;
		tx_frame[3] = temp;
		printf("temp=%d \t cis=%d \r\n" , (int)temp , (int)press);
		vTaskDelay( 80 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void sun_tracker	(void *pvParameters){
	while(1){

		//-----------------------------------------------------------------
		// TOP

		Top.error = 0 - (ph_resis[0] - ph_resis[1]);

		Top.P = Top.Kp * Top.error;
		Top.I = Top.Ki * (Top.error - Top.prev_error);
		Top.D = Top.Kd * Top.error;

		Top.prev_error = Top.error;

		positionTOP += Top.P + Top.I;

		__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1 , positionTOP );
		vTaskDelay( 20 / portTICK_PERIOD_MS);

		//-----------------------------------------------------------------
		//BOTTOM

		Bottom.error = 0 - (ph_resis[2] - ph_resis[3]);

		Bottom.P = Bottom.Kp * Bottom.error;
		Bottom.I = Bottom.Ki * (Bottom.error - Bottom.prev_error);
		Bottom.D = Bottom.Kd * Bottom.error;

		Bottom.prev_error = Bottom.error;

		positionBOTTOM += Bottom.P + Bottom.I;

		__HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1 , positionBOTTOM );
		vTaskDelay( 20 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


int cnt=0;
void TX_radio(void *pvParameters){
	while(1){

		set_OPMODE(OPMODE_TX);
		vTaskDelay( 10 / portTICK_PERIOD_MS);
		Transmit(tx_frame, sizeof(tx_frame));

		printf("Wyslano \r\n");

		curr_dev++;
		if(curr_dev == end){
			curr_dev = begin;
			curr_dev++;
		}

		vTaskResume(rx_handle);
		vTaskSuspend( NULL );
		vTaskDelay( 10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void RX_radio(void *pvParameters){
	while(1){
		vTaskSuspend(tx_handle);

		set_OPMODE(OPMODE_RX);
		vTaskDelay( 1 / portTICK_PERIOD_MS);

		if( HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_SET ){
			printf("Carrier found. \r\n");
			Receive(rx_frame);
			if( rx_frame[0] == 0x03 ){
				printf("dev=%d \t humid=%d \t data=%d \r\n",rx_frame[0] , rx_frame[1], rx_frame[2] );
			}
			if( rx_frame[0] == 0x01 && rx_frame[1] == STM32F411_DEVICE ){
				printf("Transmit start \r\n" );
				vTaskResume(tx_handle);
				vTaskSuspend( NULL );
				vTaskResume(tx_handle);
			}
		}
		else{
			printf("No carrier found. \r\n");
		}


//		printf("Current device -> %d \r\n",curr_dev);
//		printf("My device -> %d \r\n",STM32F411_DEVICE);

//		curr_dev++;
//		if(curr_dev == end){
//			curr_dev = begin;
//			curr_dev++;
//		}
//		if( curr_dev == STM32F411_DEVICE ){
//			vTaskResume(tx_handle);
//			vTaskSuspend( NULL );
//			vTaskResume(tx_handle);
//		}

		vTaskDelay( 200 / portTICK_PERIOD_MS);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  HAL_ADC_Start_DMA(&hadc1, ph_resis, 4);

  // Setting up BMP280
  BMP280_setup();
  setConstants();

  // Setting up Real Time Clock
  RTC_Init();

  servo_init();

  // Setting up LoRa
  LoRa_init(868);
  tx_frame[0] = 0x02;

//  setConstants();
//  BMP280_setup();

  xTaskCreate( light_kitchen	, "LIGHT_KITCHEN_TASK"		, 100, NULL, 1, &light_kit_handle );
  xTaskCreate( light_hall		, "LIGHT_HALL_TASK"			, 100, NULL, 1, &light_hall_handle );
  xTaskCreate( light_garage		, "LIGHT_GARAGE_TASK"		, 100, NULL, 1, &light_gar_handle );
//
  xTaskCreate( carbon_sensor	, "CARBON_SENSOR_TASK" 		, 120, NULL, 1, &carbon_sensor_handle );
  xTaskCreate( pressure			, "PRESSURE_TASK"			, 200, NULL, 1, &pressure_handle );
//
  xTaskCreate( clock			, "CLOCK_TASK"				, 200, NULL, 1, &clock_handle );
  xTaskCreate( door				, "DOOR_TASK"				, 120, NULL, 1, &door_handle );
  xTaskCreate( car				, "CAR_TASK"				, 120, NULL, 1, &car_handle );

  xTaskCreate( TX_radio			, "RADIO_TRANSMIT_TASK"		, 200, NULL, 1, &tx_handle );
  xTaskCreate( RX_radio			, "RADIO_RECEIVE_TASK"		, 200, NULL, 1, &rx_handle );

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
