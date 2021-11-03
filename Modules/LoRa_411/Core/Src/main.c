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

// Registers

#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01

#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F

#define REG_RX_NB_BYTES             0x13
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define PAYLOAD_LENGTH              0x40


// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07


// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66


// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01


// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

// LOW NOISE AMPLIFIER
#define REG_LNA			0x0C
#define LNA_MAX_GAIN	0x23
#define LNA_OFF_GAIN	0x00
#define LNA_LOW_GAIN	0x20

#define RegDioMapping1	0x40 // common
#define RegDioMapping2	0x41 // common

#define RegPaConfig		0x09 // common
#define RegPaRamp		0x0A // common
#define RegPaDac		0x5A // common



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char rx_data[256];
char tx_data[256];
unsigned char hello[32];// = "HELLO! ";

unsigned char receivedbytes;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void 	SPI_CS_LOW		();
void 	SPI_CS_HIGH		();

unsigned char Read_Reg		(unsigned char addr);
void 	Write_Reg		(unsigned char addr , unsigned char value);
void 	Write_Buf		(unsigned char addr, unsigned char *value, unsigned char len);

void 	set_OPMODE		(uint8_t mode);
void 	set_LoRa_mode	();
void 	set_Freq		(uint64_t freq);
void 	set_config		();
void 	set_Power		(int8_t pw);

uint8_t Receive		(char * payload);
void 	Transmit	(unsigned char *frame, unsigned char datalen);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file , char *ptr , int len){
  HAL_UART_Transmit(&huart2 , ptr , len , 50);
  return len;
}

void SPI_CS_LOW(){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void SPI_CS_HIGH(){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

unsigned char Read_Reg(unsigned char addr){
	uint8_t reg = addr & 0x7F;
	uint8_t data;

	SPI_CS_LOW();
	HAL_SPI_Transmit(&hspi1, &reg, 1 , HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
	SPI_CS_HIGH();

	return data;
}

void Write_Reg(unsigned char addr , unsigned char value){
	uint8_t reg = addr | 0x80;
	uint8_t Value = value;

	SPI_CS_LOW();
	HAL_SPI_Transmit(&hspi1, &reg,	1 , HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &Value,1 , HAL_MAX_DELAY);
	SPI_CS_HIGH();
}

void set_OPMODE(uint8_t mode){
	Write_Reg(REG_OPMODE, Read_Reg(REG_OPMODE) & ~OPMODE_MASK | mode );
}

void set_LoRa_mode(){
	set_OPMODE(OPMODE_SLEEP);
	HAL_Delay(15);

	uint8_t data = OPMODE_LORA;
	Write_Reg(REG_OPMODE, data);
}

void set_Freq(uint64_t freq){
	set_OPMODE(OPMODE_SLEEP);
	HAL_Delay(15);

	uint64_t frf = (( uint64_t)freq <<14 );  // freq *  2^19 / 32
	Write_Reg(REG_FRF_MSB, (uint8_t)(frf>>16) );
	Write_Reg(REG_FRF_MID, (uint8_t)(frf>>8) );
	Write_Reg(REG_FRF_LSB, (uint8_t)(frf>>0) );
}

void set_config(){
	set_OPMODE(OPMODE_SLEEP);
	HAL_Delay(15);

	set_LoRa_mode();

	unsigned char version = Read_Reg(REG_VERSION);

	printf("%d version \n\r", version);

	Write_Reg(REG_SYNC_WORD, 0x34);


	Write_Reg(REG_MODEM_CONFIG3,0x04);
	Write_Reg(REG_MODEM_CONFIG,0x72);
	Write_Reg(REG_MODEM_CONFIG2, (0x07<<4) | 0x04);

	Write_Reg(REG_SYMB_TIMEOUT_LSB,0x08);

	Write_Reg(REG_MAX_PAYLOAD_LENGTH, 0x80);
	Write_Reg(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
	Write_Reg(REG_HOP_PERIOD, 0xFF);
	Write_Reg(REG_FIFO_ADDR_PTR, Read_Reg(REG_FIFO_TX_BASE_AD));

	Write_Reg(REG_LNA, LNA_MAX_GAIN);
}

uint8_t Receive(char *payload){
	Write_Reg(REG_IRQ_FLAGS, 0x40);

	int IRQ = Read_Reg(REG_IRQ_FLAGS);

	if(( IRQ & 0x20) == 0x20){
		printf("CRC error \n\r");
		Write_Reg(REG_IRQ_FLAGS, 0x20);
		return 0;
	}
	else {
		uint8_t currentAddr = Read_Reg(REG_FIFO_RX_CURRENT_ADDR);
		uint8_t receivedCount = Read_Reg(REG_RX_NB_BYTES);
		receivedbytes = receivedCount;

		Write_Reg(REG_FIFO_ADDR_PTR, currentAddr);

		for(int i = 0; i < receivedCount; i++)
			payload[i] = (char)Read_Reg(REG_FIFO);
	}

	return 1;
}

void Write_Buf(unsigned char addr, unsigned char *value, unsigned char len) {

	uint8_t reg = addr | 0x80;
	uint8_t buf[256];
	for (int i = 0; i < len; i++) {
		buf[i] = value[i];
	}

	SPI_CS_LOW();
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, buf, len, HAL_MAX_DELAY);
	SPI_CS_HIGH();
}

void Transmit(unsigned char *frame, unsigned char datalen) {

	Write_Reg(REG_HOP_PERIOD,0x00);
	Write_Reg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
	Write_Reg(REG_IRQ_FLAGS, 0xFF);
	Write_Reg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

	Write_Reg(REG_FIFO_TX_BASE_AD, 0x00);
	Write_Reg(REG_FIFO_ADDR_PTR, 0x00);
	Write_Reg(REG_PAYLOAD_LENGTH, datalen);

	Write_Buf(REG_FIFO, frame, datalen);
	set_OPMODE(OPMODE_TX);
}

void set_Power(int8_t pw) {
	if(pw >= 17) {
		pw = 15;
	} else if(pw < 2) {
		pw = 2;
	}
	Write_Reg(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
	Write_Reg(RegPaDac, Read_Reg(RegPaDac)|0x4);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("Sending \n\n\n\r");
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  set_Freq(868);
  set_config();
  set_LoRa_mode();

  set_OPMODE(OPMODE_STANDBY);
  Write_Reg(RegPaRamp, (Read_Reg(RegPaRamp)& 0xF0) | 0x80 );
  set_Power(23);




  while (1)
  {
	strncpy((char *)hello, "HELO", sizeof(hello));
	Transmit(hello, strlen((char*)hello));
	//HAL_Delay(100);

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
