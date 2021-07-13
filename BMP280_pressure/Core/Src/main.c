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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t BMP280_REG_TEMP_XLSB   =0xFC; /* bits: 7-4 */
uint8_t BMP280_REG_TEMP_LSB    =0xFB;
uint8_t BMP280_REG_TEMP_MSB    =0xFA;
uint8_t BMP280_REG_PRESS_XLSB  =0xF9 ;/* bits: 7-4 */
uint8_t BMP280_REG_PRESS_LSB   =0xF8;
uint8_t BMP280_REG_PRESS_MSB   =0xF7;
uint8_t BMP280_REG_CONFIG      =0xF5 ;/* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
uint8_t BMP280_REG_CTRL        =0xF4 ;/* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
uint8_t BMP280_REG_STATUS      =0xF3 ;/* bits: 3 measuring; 0 im_update */
uint8_t BMP280_REG_CTRL_HUM    =0xF2 ;/* bits: 2-0 osrs_h; */
uint8_t BMP280_REG_RESET       =0xE0;
uint8_t BMP280_REG_ID          =0xD0;
uint8_t BMP280_REG_CALIB       =0x88;
uint8_t BMP280_REG_HUM_CALIB   =0x88;

#define BMP280_RESET_VALUE     0xB6

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
  HAL_UART_Transmit(&huart1 , ptr , len , 50);
  return len;
}


    uint16_t dig_T1=27504;
    int16_t  dig_T2=26435;
    int16_t  dig_T3=-1000;
    uint16_t dig_P1=36477;
    int16_t  dig_P2=-10685;
    int16_t  dig_P3=3024;
    int16_t  dig_P4=2855;
    int16_t  dig_P5=140;
    int16_t  dig_P6=-7;
    int16_t  dig_P7=15500;
    int16_t  dig_P8=-14600;
    int16_t  dig_P9=6000;

    uint16_t addr;


static int read_data( uint8_t addr, uint8_t * value , uint8_t len) {
	uint8_t rx_buff[2];

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	HAL_SPI_TransmitReceive(&hspi1, addr, value, len, 100);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	return 1;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_SPI_Init(&hspi1);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint8_t press1=0;
  uint8_t press2=0;
  uint8_t press3=0;
  uint32_t press=0;

  uint8_t P[3];
  uint32_t PP=0;

  uint8_t T[3];
  uint32_t TT=0;

  uint8_t ID=19;
  uint8_t ID2=0xD0;

  int i=0;
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);
  int cos=0;


  uint8_t s1=0xE4;
  uint8_t s2=0x03;

  uint8_t s11=0xF4;
  uint8_t czytaj=0;

  int var1;
  int var2;
  int Var1;
  int Var2;

  int t_fine=0;
  int temp=0;
  int cis=0;
  int Cis=0;

  while (1)
  {


	  // MODE
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &s11, 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, &czytaj, 1, HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);



	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &s1, 1, 200);
	  HAL_SPI_Transmit(&hspi1, &s2, 1, 200);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);





	  // ID
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  //HAL_SPI_TransmitReceive(&hspi1, &ID2, &ID, 1, HAL_MAX_DELAY);
	  //HAL_SPI_TransmitReceive(&hspi1, &ID2, &ID, 1, HAL_MAX_DELAY);
	  HAL_SPI_Transmit(&hspi1, &ID2, 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, &ID, 1, HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);





	  // PRESS 1-3
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_MSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &press1, 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);



	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_LSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &press2, 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);


	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_XLSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &press3, 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);




	  // Drugie
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_MSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[0], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_LSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[1], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_XLSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[2], 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);


	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_MSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[0], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_LSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[1], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_XLSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[2], 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

////////////////////////////////////////////////////////////////////////////////


	  press3 = press3 & 0x07;

	  press = (press1 << 12) | (press2 << 4) | (press3);


	  press = press / 4095;
	  PP = ( P[0] << 12 | P[1] << 4 | (P[2]>>4) );
	  TT = ( T[0] << 12 | T[1] << 4 | (T[2]>>4) );

	  HAL_Delay(200);


	  // TEMP
	  var1 = (((double)TT)/16384.0 - ((double)dig_T1)/1024.0)*((double)dig_T2);
	  var2 = ((((double)TT)/131072.0 - ((double)dig_T1)/8192.0)*(((double)TT)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	  t_fine = var1+var2;
	  temp = t_fine/5120.0;

	  // PRESS
	  Var1 = ((double)t_fine/2.0)-64000.0;
	  Var2=Var1*Var1*((double)dig_P6)/32768.0;
	  Var2=Var2+Var1*((double)dig_P5)*2.0;
	  Var2=Var2/4.0 + (((double)dig_P4)*65536.0);
	  Var1 = (      ( (double)dig_P3   )  *  Var1*Var1/524288.0+((double)dig_P2)*Var1)/524288.0;
	  Var1 = (1.0+Var1/32768.0)*((double)dig_P1);
	  cis=1048576.0 - (double)PP;
	  printf(" Test: %d \r\n", cis);
	  cis = (cis-(Var2/4096.0))*6250.0/Var1;
	  Var1=((double)dig_P9)*cis*cis/2147483648.0;
	  Var2=cis*((double)dig_P8)/32768.0;
	  cis = cis + (Var1+Var2+((double)dig_P7))/16.0;
	  Cis = cis/100.0;


	  printf("%d \t ID : %d \r\n", i++ , ID );
	  if( i==1000 ) i=0;

	  printf(" Press1 : %d \r\n", press1 );
	  printf(" Press2 : %d \r\n", press2 );
	  printf(" Press3 : %d \r\n", press3 );

	  printf(" T1: %d \r\n", T[0] );
	  printf(" T2: %d \r\n", T[1] );
	  printf(" T3: %d \r\n", T[2] );

//	  printf(" CISNIENIE : %d hPa \r\n", PP );
//	  printf(" TEMPERATURA: %d hPa \r\n", TT );
	  printf(" temp: %d *C \r\n", temp );
	  printf(" cis: %d hPa \r\n", Cis );


	  printf(" czytaj mode : %d \r\n", czytaj);
	  printf("\r\n" );


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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
