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
uint8_t reg_T1l=0x88;
uint8_t reg_T2l=0x8A;
uint8_t reg_T3l=0x8C;
uint8_t reg_P1l=0x8E;
uint8_t reg_P2l=0x90;
uint8_t reg_P3l=0x92;
uint8_t reg_P4l=0x94;
uint8_t reg_P5l=0x96;
uint8_t reg_P6l=0x98;
uint8_t reg_P7l=0x9A;
uint8_t reg_P8l=0x9C;
uint8_t reg_P9l=0x9E;

uint8_t reg_T1m=0x89;
uint8_t reg_T2m=0x8B;
uint8_t reg_T3m=0x8D;
uint8_t reg_P1m=0x8F;
uint8_t reg_P2m=0x91;
uint8_t reg_P3m=0x93;
uint8_t reg_P4m=0x95;
uint8_t reg_P5m=0x97;
uint8_t reg_P6m=0x99;
uint8_t reg_P7m=0x9B;
uint8_t reg_P8m=0x9D;
uint8_t reg_P9m=0x9F;

uint8_t dig_T12[2];
int8_t  dig_T22[2];
int8_t  dig_T32[2];
uint8_t dig_P12[2];
int8_t  dig_P22[2];
int8_t  dig_P32[2];
int8_t  dig_P42[2];
int8_t  dig_P52[2];
int8_t  dig_P62[2];
int8_t  dig_P72[2];
int8_t  dig_P82[2];
int8_t  dig_P92[2];

uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

uint16_t addr;


static int read_data( uint8_t addr, uint8_t * value , uint8_t len) {
	uint8_t rx_buff[2];

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	HAL_SPI_TransmitReceive(&hspi1, addr, value, len, 100);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	return 1;
}

void setConstants(){
	  // DIG_Tx/Px

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_T1l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_T12, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_T2l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_T22, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_T3l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_T32, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P1l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P12, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P2l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P22, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P3l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P32, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P4l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P42, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P5l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P52, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P6l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P62, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P7l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P72, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P8l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P82, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, &reg_P9l, 1, 100);
	  HAL_SPI_Receive(&hspi1, &dig_P92, 2, 100);
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  dig_T1 = dig_T12[0] | (dig_T12[1]<<8);
	  dig_T2 = dig_T22[0] | (dig_T22[1]<<8);
	  dig_T3 = dig_T32[0] | (dig_T32[1]<<8);
	  dig_P1 = dig_P12[0] | (dig_P12[1]<<8);
	  dig_P2 = dig_P22[0] | (dig_P22[1]<<8);
	  dig_P3 = dig_P32[0] | (dig_P32[1]<<8);
	  dig_P4 = dig_P42[0] | (dig_P42[1]<<8);
	  dig_P5 = dig_P52[0] | (dig_P52[1]<<8);
	  dig_P6 = dig_P62[0] | (dig_P62[1]<<8);
	  dig_P7 = dig_P72[0] | (dig_P72[1]<<8);
	  dig_P8 = dig_P82[0] | (dig_P82[1]<<8);
	  dig_P9 = dig_P92[0] | (dig_P92[1]<<8);
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

  uint8_t P[3];
  uint32_t PP=0;

  uint8_t T[3];
  uint32_t TT=0;

  uint8_t ID=19;
  uint8_t ID2=0xD0;

  int i=0;
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

  uint8_t write_F4=0x74;
  uint8_t overwrite_F4=0xFF;

  uint8_t read_F4=0xF4;
  uint8_t czytaj=0;

  int var1;
  int var2;
  int Var1;
  int Var2;

  int32_t t_fine=0;
  int32_t temp=0;
  int32_t cis=0;



  setConstants();

  while (1)
  {
	  // MODE
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &read_F4, 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, &czytaj, 1, HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);


	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &write_F4, 1, 200);
	  HAL_SPI_Transmit(&hspi1, &overwrite_F4, 1, 200);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);



	  // ID
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &ID2, 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, &ID, 1, HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);





	  // Ciśnienie
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_MSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[0], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_LSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[1], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_PRESS_XLSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &P[2], 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

	  // Temperatura
	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, RESET);

	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_MSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[0], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_LSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[1], 1, 100);
	  HAL_SPI_Transmit(&hspi1, &BMP280_REG_TEMP_XLSB, 1, 100);
	  HAL_SPI_Receive(&hspi1, &T[2], 1, 100);

	  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, SET);

////////////////////////////////////////////////////////////////////////////////

	  PP = ( P[0] << 12 | P[1] << 4 | (P[2]>>4) );
	  TT = ( T[0] << 12 | T[1] << 4 | (T[2]>>4) );

	  HAL_Delay(400);


//	   TEMP
//	  var1 = (((double)PP)/16384.0 - ((double)dig_T1)/1024.0)*((double)dig_T2);
//	  printf(" Test: %d \r\n", var1);
//	  var2 = ((((double)PP)/131072.0 - ((double)dig_T1)/8192.0)*(((double)PP)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
//	  t_fine = var1+var2;
//	  temp = t_fine/5120.0;

	  var1 = (((TT>>3)-((int32_t)dig_T1<<1)) *((int32_t)dig_T2))>>11;
	  var2 = (((((TT>>4) - ((int32_t)dig_T1))*((TT>>4)-((int32_t)dig_T1))) >> 12)*(int32_t)dig_T3)>>14;
	  t_fine = var1+var2;
	  temp = (t_fine*5 + 128) >> 8;
	  temp = temp / 100;


	  // PRESS
//	  Var1 = ((double)t_fine/2.0)-64000.0;
//	  Var2=Var1*Var1*((double)dig_P6)/32768.0;
//	  Var2=Var2+Var1*((double)dig_P5)*2.0;
//	  Var2=Var2/4.0 + (((double)dig_P4)*65536.0);
//	  Var1 = (      ( (double)dig_P3   )  *  Var1*Var1/524288.0+((double)dig_P2)*Var1)/524288.0;
//	  Var1 = (1.0+Var1/32768.0)*((double)dig_P1);
//	  cis=1048576.0 - (double)PP;
//	  cis = (cis-(Var2/4096.0))*6250.0/Var1;
//	  Var1=((double)dig_P9)*cis*cis/2147483648.0;
//	  Var2=cis*((double)dig_P8)/32768.0;
//	  cis = cis + (Var1+Var2+((double)dig_P7))/16.0;
//	  cis = cis/100.0;


	  Var1 = ((int32_t)t_fine>>1) - (int32_t)64000;
	  Var2 = (((Var1>>2) * (Var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	  Var2 = Var2 + ((Var1*((int32_t)dig_P5))<<1);
	  Var2 = (Var2>>2)+(((int32_t)dig_P4)<<16);
	  Var1 = (((dig_P3 * (((Var1>>2) * (Var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * Var1)>>1))>>18;
	  Var1 =((((32768+Var1))*((int32_t)dig_P1))>>15);
	  printf(" TEST: %d \r\n", Var1);
	  if (Var1 == 0){
		  return 0; // avoid exception caused by division by zero
	  }
	  cis = (((uint32_t)(      ((int32_t)1048576)-PP)-(Var2>>12)))*3125;
	  if (cis < 0x80000000) {
		  cis = (cis << 1) / ((uint32_t)Var1);
	  }
	  else{
		  cis = (cis / (uint32_t)Var1) * 2;
	  }
//	  cis = 1048576.0 - (double)PP;
//	  cis = (cis - (Var2/4096.0)) * 6250.0 / Var1;
	  Var1 = (((int32_t)dig_P9) * ((int32_t)(((cis>>3) * (cis>>3))>>13)))>>12;
	  Var2 = (((int32_t)(cis>>2)) * ((int32_t)dig_P8))>>13;
	  cis = (uint32_t)((int32_t)cis + ((Var1 + Var2 + dig_P7) >> 4));
	  cis = cis/100;






	  printf("%d  ID : %d \r\n", i++ , ID );
	  if( i==1000 ) i=0;

	  printf(" P: ( %d , %d , %d ) \r\n", P[0] , P[1] , P[2] );
	  printf(" T: ( %d , %d , %d ) \r\n", T[0] , T[1] , T[2] );

	  printf(" CISNIENIE : %d hPa \r\n", PP );
	  printf(" TEMPERATURA: %d *C \r\n", TT );
//	  printf(" Register read: %d \r\n", dig_T1);

	  printf(" temp: %d *C \r\n", temp );
	  printf(" cis: %d hPa \r\n", cis );

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
