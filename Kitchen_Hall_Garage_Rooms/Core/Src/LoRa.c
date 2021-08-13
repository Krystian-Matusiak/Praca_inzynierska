/*
 * LoRa.c
 *
 *  Created on: Aug 13, 2021
 *      Author: kus
 */

#include "LoRa.h"

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

void LoRa_init(uint64_t freq){
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	set_Freq(freq);
	set_config();
	set_LoRa_mode();

	set_OPMODE(OPMODE_STANDBY);
	Write_Reg(RegPaRamp, (Read_Reg(RegPaRamp)& 0xF0) | 0x80 );
	set_Power(23);
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

