/*
 * LoRa.h
 *
 *  Created on: Aug 13, 2021
 *      Author: kus
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_


#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

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


// Variables

char rx_data[256];
char tx_data[256];

unsigned char hello[32];// = "HELLO! ";


// Function prototypes


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

void LoRa_init(uint64_t freq);

uint8_t Receive		(char * payload);
void 	Transmit	(unsigned char *frame, unsigned char datalen);



#endif /* INC_LORA_H_ */
