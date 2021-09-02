/*
 * bmp280.c
 *
 *  Created on: Aug 23, 2021
 *      Author: kus
 */

#include "bmp280.h"
#include "spi.h"


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

int read_data( uint8_t *addr, int8_t * value , uint8_t len) {

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi3, addr, 1, 100);
	HAL_SPI_Receive(&hspi3, value, len, 100);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

	HAL_Delay(3);
	return 1;
}

int uread_data( uint8_t *addr, uint8_t * value , uint8_t len) {

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi3, addr, 1, 100);
	HAL_SPI_Receive(&hspi3, value, len, 100);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

	HAL_Delay(3);
	return 1;
}

void setConstants(){
	  uread_data(&reg_T1l, dig_T12,2);
	  read_data(&reg_T2l, dig_T22,2);
	  read_data(&reg_T3l, dig_T32,2);
	  uread_data(&reg_P1l, dig_P12,2);
	  read_data(&reg_P2l, dig_P22,2);
	  read_data(&reg_P3l, dig_P32,2);
	  read_data(&reg_P4l, dig_P42,2);
	  read_data(&reg_P5l, dig_P52,2);
	  read_data(&reg_P6l, dig_P62,2);
	  read_data(&reg_P7l, dig_P72,2);
	  read_data(&reg_P8l, dig_P82,2);
	  read_data(&reg_P9l, dig_P92,2);

	  dig_T1 = (uint16_t) ((uint16_t)dig_T12[0] | ( (uint16_t) (((uint16_t)dig_T12[1])<<8)));
	  dig_T2 = (int16_t)  ((int16_t)dig_T22[0]  | ( (int16_t)  (((int16_t)dig_T22[1])<<8)));
	  dig_T3 = (int16_t)  ((int16_t)dig_T32[0]  | ( (int16_t)  (((int16_t)dig_T32[1])<<8)));
	  dig_P1 = (uint16_t) ((uint16_t)dig_P12[0] | ( (uint16_t) (((uint16_t)dig_P12[1])<<8)));
	  dig_P2 = (int16_t)  ((int16_t)dig_P22[0]  | ( (int16_t)  (((int16_t)dig_P22[1])<<8)));
	  dig_P3 = (int16_t)  ((int16_t)dig_P32[0]  | ( (int16_t)  (((int16_t)dig_P32[1])<<8)));
	  dig_P4 = (int16_t)  ((int16_t)dig_P42[0]  | ( (int16_t)  (((int16_t)dig_P42[1])<<8)));
	  dig_P5 = (int16_t)  ((int16_t)dig_P52[0]  | ( (int16_t)  (((int16_t)dig_P52[1])<<8)));
	  dig_P6 = (int16_t)  ((int16_t)dig_P62[0]  | ( (int16_t)  (((int16_t)dig_P62[1])<<8)));
	  dig_P7 = (int16_t)  ((int16_t)dig_P72[0]  | ( (int16_t)  (((int16_t)dig_P72[1])<<8)));
	  dig_P8 = (int16_t)  ((int16_t)dig_P82[0]  | ( (int16_t)  (((int16_t)dig_P82[1])<<8)));
	  dig_P9 = (int16_t)  ((int16_t)dig_P92[0]  | ( (int16_t)  (((int16_t)dig_P92[1])<<8)));
}


void BMP280_setup(){

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

	uint8_t ctrl_meas_addr = 0x74;
	uint8_t osrs_p = OSRS_P; // skipped / x1 / x2 / x4 / x8 / x16
	uint8_t osrs_t = OSRS_T; // skipped / x1 / x2 / x4 / x8 / x16
	uint8_t mode = MODE;    // sleep (00) / forced (01 / 10) / normal (11)

	uint8_t t_sb_filtr_addr = 0x75;
	uint8_t spi3w_en = SPI3W_EN;
	uint8_t filter= FILTER ;  // off / 2 / 4 / 8 / 16
	uint8_t t_sb= T_SB; // 0.5 / 62.5 / 125 / 250 / 500 / 1000 / 2000 / 4000

//	uint8_t rst_addr= 0xE0;
//	uint8_t rst = 0x00;

	uint8_t read_F4=0xF4;
	uint8_t czytaj=0;

	uint8_t ctrl_meas =  mode | osrs_p | osrs_t;
	uint8_t tsb_filtr_msk = t_sb | filter | spi3w_en;

	// MODE
	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi3, &read_F4, 1, 100);
	HAL_SPI_Receive(&hspi3, &czytaj, 1, 200);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi3, &ctrl_meas_addr, 1, 200);
	HAL_SPI_Transmit(&hspi3, &ctrl_meas, 1, 200);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
	HAL_Delay(5);


	// Filter and time standby
	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi3, &t_sb_filtr_addr, 1, 200);
	HAL_SPI_Transmit(&hspi3, &tsb_filtr_msk, 1, 200);

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

}

void get_temp_press(int32_t * temp , uint32_t * press){
	  uint8_t P[3];
	  int32_t PP=0;

	  uint8_t T[3];
	  int32_t TT=0;

	  uint8_t A[6];
	  int32_t At=0;
	  int32_t Ap=0;

	  int32_t var1;
	  int32_t var2;
	  int32_t Var1;
	  int32_t Var2;

	  int32_t t_fine=0;

	  uint8_t reg_press = BMP280_REG_PRESS_MSB;
	  uint8_t reg_temp = BMP280_REG_TEMP_MSB;

	  // Temperatura
	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3, &reg_temp, 1, 200);
	  HAL_SPI_Receive(&hspi3, &T[0], 3, 200);
	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

	  // Ciśnienie
	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3, &reg_press, 1, 200);
	  HAL_SPI_Receive(&hspi3, &P[0], 3, 200);
	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);



	  // All
//	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi3, &reg_press, 1, 200);
//	  HAL_SPI_Receive(&hspi3, &A[0], 6, 200);
//	  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
//	  HAL_Delay(200);




	  PP = ((uint32_t)( (uint16_t) ((uint16_t)P[0] << 8) | (uint16_t)P[1]) << 4) | ((uint16_t)P[2] >>4    ) ;
	  TT = ((uint32_t)( (uint16_t) ((uint16_t)T[0] << 8) | (uint16_t)T[1]) << 4) | (uint16_t)(T[2] >>4    ) ;
//	  At = ((uint32_t)( (uint16_t) ((uint16_t)A[3] << 8) | (uint16_t)A[4]) << 4) | (uint16_t)(A[5] >>4    ) ;
//	  Ap = ((uint32_t)( (uint16_t) ((uint16_t)A[0] << 8) | (uint16_t)A[1]) << 4) | (uint16_t)(A[2] >>4    ) ;

	  HAL_Delay(300);


//	  dig_T1 = 28704;
//	  dig_T2 = 26435;
//	  dig_T3 = -1000;

	  dig_T1 = 28800;
	  dig_T2 = 26435;
	  dig_T3 = -1000;

//   TEMP

	  var1 = ((((TT>>3)-((int32_t)dig_T1<<1))) *((int32_t)dig_T2))>>11;
	  var2 = (((((TT>>4) - ((int32_t)dig_T1))*((TT>>4)-((int32_t)dig_T1))) >> 12)*((int32_t)dig_T3))>>14;
	  t_fine = var1+var2;
	  *temp = (t_fine*5 + 128) >> 8;
	  *temp = *temp / 100;

//	  PRESS

	  Var1 = (((int32_t)t_fine)/2) - (int32_t)64000;
	  Var2 = (((Var1/4) * (Var1/4)) / 2048 ) * ((int32_t)dig_P6);
	  Var2 = Var2 + ((Var1*((int32_t)dig_P5))<<1);
	  Var2 = (Var2>>2)+(((int32_t)dig_P4)<<16);
	  Var1 = (((dig_P3 * (((Var1>>2) * (Var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * Var1)>>1))>>18;
	  Var1 = ((((32768+Var1))*((int32_t)dig_P1))>>15);
//	  if (Var1 == 0){
//		  return ; // avoid exception caused by division by zero
//	  }
	  *press = (((uint32_t)(  ((int32_t)1048576)-PP )-(Var2>>12)))*3125;
	  if (*press < 0x80000000) {
		  *press = (*press << 1) / ((uint32_t)Var1);
	  }
	  else{
		  *press = (*press / (uint32_t)Var1) * 2;
	  }
	  Var1 = (((int32_t)dig_P9) * ((int32_t)(((*press>>3) * (*press>>3))>>13)))>>12;
	  Var2 = (((int32_t)(*press)/4) * ((int32_t)dig_P8))/8192;
	  *press = (uint32_t)((int32_t)*press + ((Var1 + Var2 + dig_P7) >> 4));
	  *press = *press/100;

//	  printf(" P: ( %d , %d , %d ) \r\n", P[0] , P[1] , P[2] );
//	  printf(" T: ( %d , %d , %d ) \r\n", T[0] , T[1] , T[2] );
//	  printf(" CISNIENIE : %d hPa \r\n", (int)PP );
//	  printf(" TEMPERATURA: %d *C \r\n", (int)TT );

//	  printf("digital T1: %d \r\n", dig_T1 );
//	  printf("digital T2: %d \r\n", dig_T2 );
//	  printf("digital T3: %d \r\n", dig_T3 );
//	  printf("digital P1: %d \r\n", dig_P1 );
//	  printf("digital P2: %d \r\n", dig_P2 );
//	  printf("digital P3: %d \r\n", dig_P3 );
//	  printf("digital P4: %d \r\n", dig_P4 );
//	  printf("digital P5: %d \r\n", dig_P5 );
//	  printf("digital P6: %d \r\n", dig_P6 );
//	  printf("digital P7: %d \r\n", dig_P7 );
//	  printf("digital P8: %d \r\n", dig_P8 );
//	  printf("digital P9: %d \r\n", dig_P9 );

}

void reset(){
	uint8_t reg_reset=0xE0 & 0x7F;
	uint8_t value_reset=0xB6;

	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg_reset, 1, 200);
	HAL_SPI_Receive(&hspi3, &value_reset, 1, 200);
	HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);
}





