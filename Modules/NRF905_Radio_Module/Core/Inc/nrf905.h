/*
 * nrf905.h
 *
 *  Created on: Jul 22, 2021
 *      Author: kus
 */

#ifndef INC_NRF905_H_
#define INC_NRF905_H_

#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/*! Variables */

/*
uint8_t W_CONFIG	 = 0x00;
uint8_t R_CONFIG 	 = 0x10;

uint8_t W_TX_PAYLOAD = 0x20;
uint8_t R_TX_PAYLOAD = 0x21;

uint8_t W_TX_ADDRESS = 0x22;
uint8_t R_TX_ADDRESS = 0x23;

uint8_t R_RX_PAYLOAD = 0x24;
uint8_t CHANNEL_CONFIG = 0x80;

uint8_t CH_NO = 0x76;
uint8_t HFREQ_PLL = 0x00;

uint8_t PA_PWR_n_10dbm = 0x00;
uint8_t PA_PWR_n_2dbm = 0x01;
uint8_t PA_PWR_6dbm = 0x02;
uint8_t PA_PWR_10dbm = 0x03;

uint8_t CLK_ENABLE = 0xE7;
uint8_t CLK_DISABLE = 0xE3;

uint8_t rx_data=0;
uint8_t tx_data=11;
 */

#define W_CONFIG	  0x00
#define R_CONFIG 	  0x10

#define W_TX_PAYLOAD  0x20
#define R_TX_PAYLOAD  0x21

#define W_TX_ADDRESS  0x22
#define R_TX_ADDRESS  0x23

#define R_RX_PAYLOAD  0x24
#define CHANNEL_CONFIG  0x80

#define CH_NO  0x76
#define HFREQ_PLL 0x00

#define PA_PWR_n_10dbm  0x00
#define PA_PWR_n_2dbm  0x01
#define PA_PWR_6dbm  0x02
#define PA_PWR_10dbm  0x03

#define CLK_ENABLE  0xE7
#define CLK_DISABLE  0xE3



/*! Function prototypes */

void delay_us (uint16_t us);

void Read_RX_PAYLOAD(uint8_t * rx_data);

uint8_t is_Data_ready();
uint8_t is_Address_Match();
uint8_t is_Carrier_Detect();


void set_TX_Payload_width(uint8_t width);
void set_RX_Payload_width(uint8_t width);

void set_Standby_mode();
void set_TxRx_mode();
void set_Tx_mode();
void set_Rx_mode();

void PWR_ON();
void PWR_OFF();


void Clock_enable();
void Clock_disable();

void set_CONFIG(uint8_t nr , uint8_t reg);
void read_CONFIG();

void set_Channel();
void read_reg(uint8_t reg);

void TX_radio(uint8_t * tx_data);
void RX_radio(uint8_t * rx_data);

#endif /* INC_NRF905_H_ */


































