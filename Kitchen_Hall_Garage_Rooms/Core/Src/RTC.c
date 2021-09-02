/*
 * RTC.c
 *
 *  Created on: Aug 13, 2021
 *      Author: kus
 */

#include "RTC.h"

void Mode_Write(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = SDA_RTC_Pin;
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void Mode_Read(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = SDA_RTC_Pin;
	GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Send_Command(uint8_t command) {
	for ( uint8_t i = 0; i < 8; i ++){

		HAL_GPIO_WritePin(SDA_RTC_GPIO_Port, SDA_RTC_Pin, (command & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_SET);

		HAL_Delay(1);
		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_RESET);
		HAL_Delay(1);

		command >>= 1;
	}
}


void Write_Byte(uint8_t address, uint8_t value){

	HAL_GPIO_WritePin(RST_RTC_GPIO_Port, RST_RTC_Pin,  GPIO_PIN_SET);
	Send_Command(address);

	for ( uint8_t i = 0; i < 8; i ++) {
		HAL_GPIO_WritePin(SDA_RTC_GPIO_Port, SDA_RTC_Pin, (value & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_SET);

		HAL_Delay(1);
		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_RESET);
		HAL_Delay(1);

		value >>= 1;
	}

	HAL_GPIO_WritePin(RST_RTC_GPIO_Port, RST_RTC_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SDA_RTC_GPIO_Port, SDA_RTC_Pin,  GPIO_PIN_RESET);
}


uint8_t Read_Byte(uint8_t address)
{
	uint8_t Temp = 0;

	HAL_GPIO_WritePin(RST_RTC_GPIO_Port, RST_RTC_Pin,  GPIO_PIN_SET);
	address = address | 0x01;

	Send_Command(address);
	Mode_Read();
	for ( uint8_t i = 0; i < 8; i ++){
		Temp >>= 1;
		if(HAL_GPIO_ReadPin(SDA_RTC_GPIO_Port, SDA_RTC_Pin))
			Temp |= 0x80;

		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_RESET);
		HAL_Delay(1);
	}
	Mode_Write();

	HAL_GPIO_WritePin(RST_RTC_GPIO_Port, RST_RTC_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SDA_RTC_GPIO_Port, SDA_RTC_Pin,  GPIO_PIN_RESET);
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

void Write_Zero_Seconds(){
	Write_Byte(REG_CONTROL,0x00);
	HAL_Delay(1);

	Write_Byte(REG_SEC,HEX_to_BCD(0));

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

	GPIO_InitStructure.Pin = SCLK_RTC_Pin | SDA_RTC_Pin | RST_RTC_Pin;
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	Write_Byte(REG_CHARGER,0x00);

	HAL_GPIO_WritePin(RST_RTC_GPIO_Port, RST_RTC_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SCLK_RTC_GPIO_Port, SCLK_RTC_Pin,  GPIO_PIN_RESET);

	DWT->CTRL |= 1 ;
}
