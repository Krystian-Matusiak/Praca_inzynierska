/*
 * nrf905.c
 *
 *  Created on: Jul 22, 2021
 *      Author: kus
 */

#include "nrf905.h"



void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void Read_RX_PAYLOAD(uint8_t * rx_data){
	uint8_t read = R_RX_PAYLOAD;

	PWR_ON();
	set_Rx_mode();

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, rx_data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	printf("Received ----> %d \r\n" , rx_data );

	PWR_OFF();
}

uint8_t is_Data_Ready(){
	if( HAL_GPIO_ReadPin(Data_ready_GPIO_Port, Data_ready_Pin)== GPIO_PIN_SET )
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

void set_RX_Payload_width(uint8_t width){
	PWR_OFF();
	uint8_t add = 0x03 ;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &add , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &width, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void set_TX_Payload_width(uint8_t width){
	PWR_OFF();
	uint8_t add = 0x04 ;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &add , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &width, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
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

void Clock_enable(){
	PWR_OFF();
	uint8_t add = 0x09 ;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &add , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, CLK_ENABLE, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void Clock_disable(){
	PWR_OFF();
	uint8_t add = 0x09 ;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &add , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, CLK_DISABLE, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void set_CONFIG(uint8_t nr , uint8_t reg){
	PWR_OFF();
	uint8_t add = W_CONFIG | nr ;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &add , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &reg, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void set_Channel(){
	PWR_OFF();

	uint8_t channel_add=0x00;
	uint8_t setCHANNEL = ( CHANNEL_CONFIG | (PA_PWR_10dbm<<2) | HFREQ_PLL );
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &setCHANNEL , 1 , 1000);
	HAL_SPI_Transmit( &hspi1, CH_NO, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	PWR_ON();
}

void read_reg(uint8_t reg){
	uint8_t read=reg;
	//read = 0x24;
	uint8_t data=0;

	PWR_OFF();
	//set_Rx_mode();
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, &data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	printf("Read -> %d \r\n" , read );
	printf("Register -> %d \r\n" , data );

	PWR_ON();
}

void read_CONFIG(){
	uint8_t add[4] = {0,0,0,0};
	uint32_t ADD=0;
	uint8_t read = R_CONFIG ;

	PWR_OFF();

	read = R_CONFIG | 0x05;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, &add[0], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	read = R_CONFIG | 0x06;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, &add[1], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	read = R_CONFIG | 0x07;
	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, &add[2], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	read = R_CONFIG | 0x08;
	HAL_SPI_Transmit( &hspi1, &read, 1 , 1000);
	HAL_SPI_Receive( &hspi1, &add[3], 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	ADD = (add[0]<<24) | (add[1]<<16) | (add[2]<<8) | add[3] ;
	printf("add1-> %d \r\n" , add[0] );
	printf("add2-> %d \r\n" , add[1] );
	printf("add3-> %d \r\n" , add[2] );
	printf("add4-> %d \r\n" , add[3] );

	printf("ADDRESS -> %d \r\n" , ADD );

	PWR_ON();

}

void TX_radio(uint8_t * tx_data){
	PWR_ON();
	set_Standby_mode();
	set_Tx_mode();

	uint8_t add[4] = { 0xE7,0xE7,0xE7,0xE7};

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, W_TX_ADDRESS, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &add, 4 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit( &hspi1, W_TX_PAYLOAD, 1 , 1000);
	HAL_SPI_Transmit( &hspi1, &tx_data, 1 , 1000);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	read_reg(R_TX_ADDRESS);
	read_reg(R_TX_PAYLOAD);
	HAL_Delay(2);
	//Clock_disable();
	set_Tx_mode();
	set_TxRx_mode();
	delay_us(10);
	set_Standby_mode();
	delay_us(650);

	while( !is_Data_Ready() );

	//Clock_enable();
	HAL_Delay(10);
}

void RX_radio(uint8_t * rx_data){
	set_Standby_mode();
	PWR_ON();
	set_Rx_mode();
	set_TxRx_mode();
	delay_us(650);

	if ( HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) != GPIO_PIN_SET ){
		printf("No transmit/receive mode detected. \r\n");
		return ;
	}

	//while( !is_Carrier_Detect() );
	while( is_Carrier_Detect() && HAL_GPIO_ReadPin(CE_GPIO_Port, CE_Pin) == GPIO_PIN_SET  ){
		printf("wykryto!!!!! \r\n ");
		delay_us(20);
		if( is_Address_Match() ){
			printf("adres git!!!! \r\n ");
			HAL_Delay(2);
			if( is_Data_Ready() ){
				Read_RX_PAYLOAD(rx_data);
			}
		}
	}

}
