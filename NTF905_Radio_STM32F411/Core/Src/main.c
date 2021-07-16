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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ever (;;)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t W_CONFIG	 = 0x00;
uint8_t R_CONFIG 	 = 0x10;

uint8_t W_TX_PAYLOAD = 0x20;
uint8_t R_TX_PAYLOAD = 0x21;

uint8_t W_TX_ADDRESS = 0x22;
uint8_t R_TX_ADDRESS = 0x23;

uint8_t R_RX_PAYLOAD = 0x24;
uint8_t CHANNEL_CONFIG = 0x80;

uint8_t CH_NO = 0x6C;
uint8_t HFREQ_PLL = 0x00;

uint8_t PA_PWR_n_10dbm = 0x00;
uint8_t PA_PWR_n_2dbm = 0x01;
uint8_t PA_PWR_6dbm = 0x02;
uint8_t PA_PWR_10dbm = 0x03;

uint8_t rx_data=0;
uint8_t tx_data=0x2F;

uint8_t stm32f3_address[4] = { 0xE7, 0xE7,0xE7,0xE7 };

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


void Read_RX_PAYLOAD(){
	uint8_t read = R_RX_PAYLOAD;
	//read = 0x24;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &rx_data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	printf("Received -> %d \r\n" , rx_data );

	PWR_ON();
}


uint8_t is_Data_Ready(){
	if( HAL_GPIO_ReadPin(DR_GPIO_Port, DR_Pin)== GPIO_PIN_SET )
		return 1;
	else
		return 0;
}

uint8_t is_Address_Match(){
	if( HAL_GPIO_ReadPin(Address_match_GPIO_Port, Address_match_Pin)== GPIO_PIN_SET )
		return 1;
	else
		return 0;
}

uint8_t is_Carrier_Detect(){
	if( HAL_GPIO_ReadPin(Carrier_detect_GPIO_Port, Carrier_detect_Pin)== GPIO_PIN_SET )
		return 1;
	else
		return 0;
}

void set_Standby_mode(){
	HAL_GPIO_WritePin( CE_GPIO_Port, CE_Pin , GPIO_PIN_RESET);
}

void set_TxRx_mode(){
	HAL_GPIO_WritePin( CE_GPIO_Port, CE_Pin , GPIO_PIN_SET);
}

void set_Tx_mode(){
	HAL_GPIO_WritePin( Tx_GPIO_Port, Tx_Pin, GPIO_PIN_SET);
}

void set_Rx_mode(){
	HAL_GPIO_WritePin( Tx_GPIO_Port, Tx_Pin, GPIO_PIN_RESET);
}

void PWR_ON(){
	HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_SET);
}

void PWR_OFF(){
	HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_RESET);
}

void set_Channel(){
	PWR_OFF();

	uint8_t channel_add=0x00;
	uint8_t setCHANNEL = ( CHANNEL_CONFIG | (PA_PWR_6dbm<<2) | HFREQ_PLL );
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &setCHANNEL , 1 , 1000);
	HAL_SPI_Transmit( &hspi4, &CH_NO, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void read_reg(){
	uint8_t read=0x81;
	//read = 0x24;
	uint8_t data=0;
	uint32_t DATA=0;

	PWR_OFF();
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	printf("Register -> %d \r\n" , data);

	PWR_ON();
}

void read_CONFIG(){
	uint8_t add[4] = {0,0,0,0};
	uint32_t ADD=0;
	uint8_t read = R_CONFIG ;

	PWR_OFF();

	read = R_CONFIG | 0x05;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &add[0], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	read = R_CONFIG | 0x06;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &add[1], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);


	read = R_CONFIG | 0x07;
	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &add[2], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);


	read = R_CONFIG | 0x08;
	HAL_SPI_Transmit( &hspi4, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi4, &add[3], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	ADD = (add[0]<<24) | (add[1]<<16) | (add[2]<<8) | add[3] ;
	printf("add1-> %d \r\n" , add[0] );
	printf("add2-> %d \r\n" , add[1] );
	printf("add3-> %d \r\n" , add[2] );
	printf("add4-> %d \r\n" , add[3] );

	printf("ADDRESS -> %d \r\n" , ADD );

	PWR_ON();

}

void TX_radio(){

	set_Tx_mode();
	PWR_ON();
	set_Standby_mode();

	uint8_t add[4] = { 0xE7,0xE7,0xE7,0xE7};
	printf("add : %d %d %d %d \r\n" , add[0] ,add[1] ,add[2] ,add[3] );

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &W_TX_ADDRESS, 1 , 1000);
	HAL_SPI_Transmit( &hspi4, &add, 4 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi4, &W_TX_PAYLOAD, 1 , 1000);
	HAL_SPI_Transmit( &hspi4, &tx_data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	read_reg();
	set_TxRx_mode();

	//while( !is_Data_Ready() );
	//printf("DR is ready \r\n");

	HAL_Delay(10);
}

void RX_radio(){
	set_Standby_mode();
	set_Rx_mode();
	PWR_ON();

	while( HAL_GPIO_ReadPin(Carrier_detect_GPIO_Port, Carrier_detect_Pin) == GPIO_PIN_SET ){

		if( is_Address_Match() ){
			if( is_Data_Ready() ){
				if( HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) == GPIO_PIN_RESET )
					Read_RX_PAYLOAD();
			}
		}
	}

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
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  HAL_SPI_Init(&hspi4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  set_Channel();
  set_Rx_mode();
  set_Standby_mode();

  while (1)
  {
//	set_TxRx_mode();
//	if( HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) == GPIO_PIN_SET )
//	  TX_radio();


	  if( HAL_GPIO_ReadPin(Carrier_detect_GPIO_Port, Carrier_detect_Pin) == GPIO_PIN_SET )
		  printf("Carrier Detect \r\n");
	  if( HAL_GPIO_ReadPin(Address_match_GPIO_Port, Address_match_Pin) == GPIO_PIN_SET )
		  printf("Address Match \r\n");
	  if( HAL_GPIO_ReadPin(DR_GPIO_Port, DR_Pin) == GPIO_PIN_SET )
		  printf("Data Ready \r\n");

	  read_reg();
	  //read_CONFIG();
//	  for ever {
//
//	  }

	  HAL_Delay(1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
