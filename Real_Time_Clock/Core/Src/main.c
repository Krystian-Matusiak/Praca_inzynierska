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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REG_SEC				0x80
#define REG_MIN				0x82
#define REG_HOUR			0x84
#define REG_DATE			0x86
#define REG_MONTH			0x88
#define REG_DAY				0x8a
#define REG_YEAR			0x8c
#define REG_CONTROL			0x8e
#define REG_CHARGER			0x90

#define HEX_to_BCD(value)	((value) % 10 + (value) / 10 * 16)
#define BCD_to_Hex(value)	((value) % 16 + (value) / 16 * 10)

#define SCLK	GPIO_PIN_6
#define SDA	GPIO_PIN_7
#define RST	GPIO_PIN_8



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int _write(int file , char *ptr , int len){
  HAL_UART_Transmit(&huart2 , ptr , len , 50);
  return len;
}

///////////////////////////////////////////////////////////////////////////////////////////
static void Mode_Write(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = SDA;
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

}

static void Mode_Read(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = SDA;
	GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void Send_Command(uint8_t command) {
	for ( uint8_t i = 0; i < 8; i ++){

		HAL_GPIO_WritePin(GPIOC, SDA, (command & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_SET);

		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_RESET);
		HAL_Delay(1);

		command >>= 1;
	}
}


static void Write_Byte(uint8_t address, uint8_t value){

	HAL_GPIO_WritePin(GPIOC, RST,  GPIO_PIN_SET);
	Send_Command(address);

	for ( uint8_t i = 0; i < 8; i ++) {
		HAL_GPIO_WritePin(GPIOC, SDA, (value & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_SET);

		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_RESET);
		HAL_Delay(1);

		value >>= 1;
	}

	HAL_GPIO_WritePin(GPIOC, RST,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, SDA,  GPIO_PIN_RESET);
}


static uint8_t Read_Byte(uint8_t address)
{
	uint8_t Temp = 0;

	HAL_GPIO_WritePin(GPIOC, RST,  GPIO_PIN_SET);
	address = address | 0x01;

	Send_Command(address);
	Mode_Read();
	for ( uint8_t i = 0; i < 8; i ++){
		Temp >>= 1;
		if(HAL_GPIO_ReadPin(GPIOC, SDA))
			Temp |= 0x80;

		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_RESET);
		HAL_Delay(1);
	}
	Mode_Write();

	HAL_GPIO_WritePin(GPIOC, RST,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, SDA,  GPIO_PIN_RESET);
	return Temp;
}

void Write_Time(uint8_t *buffor)
{
	Write_Byte(REG_CONTROL,0x00);
	HAL_Delay(1);

	Write_Byte(REG_SEC,0x80);
	Write_Byte(REG_YEAR,HEX_to_BCD(buffor[1]));
	Write_Byte(REG_MONTH,HEX_to_BCD(buffor[2]));
	Write_Byte(REG_DATE,HEX_to_BCD(buffor[3]));
	Write_Byte(REG_HOUR,HEX_to_BCD(buffor[4]));
	Write_Byte(REG_MIN,HEX_to_BCD(buffor[5]));
	Write_Byte(REG_SEC,HEX_to_BCD(buffor[6]));
	Write_Byte(REG_DAY,HEX_to_BCD(buffor[7]));

	Write_Byte(REG_CONTROL,0x80);
	HAL_Delay(1);
}


void Read_Time(uint8_t *buffor){
   	uint8_t Temp;

	Temp = Read_Byte(REG_YEAR);
	buffor[1] = BCD_to_Hex(Temp);
	Temp = Read_Byte(REG_MONTH);
	buffor[2] = BCD_to_Hex(Temp);
	Temp = Read_Byte(REG_DATE);
	buffor[3] = BCD_to_Hex(Temp);
	Temp = Read_Byte(REG_HOUR);
	buffor[4] = BCD_to_Hex(Temp);
	Temp = Read_Byte(REG_MIN);
	buffor[5] = BCD_to_Hex(Temp);
	Temp = Read_Byte(  (REG_SEC)  ) & 0x7F ;
	buffor[6] = BCD_to_Hex(Temp);
	Temp = Read_Byte(REG_DAY);
	buffor[7] = BCD_to_Hex(Temp);
}

void RTC_Init(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = SCLK | SDA | RST;
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	Write_Byte(REG_CHARGER,0x00);

	HAL_GPIO_WritePin(GPIOC, RST,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, SCLK,  GPIO_PIN_RESET);

	DWT->CTRL |= 1 ;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  RTC_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint8_t buffor[8] = {0,1,2,3,4,5,6,7};
  uint8_t real_time[8] = { 1 , 21 , 7 , 3 , 13 , 20 , 0 ,2};

  Write_Time(real_time);



  while (1)
  {
	  Read_Time(buffor);

	  printf(" %d:%d:%d %d-%d-20%d \n\r"  , buffor[4] , buffor[5] , buffor[6] , buffor[3] , buffor[2] , buffor[1] );

	  HAL_Delay(500);

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
