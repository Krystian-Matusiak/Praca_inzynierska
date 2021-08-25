/*
 * RTC.h
 *
 *  Created on: Aug 13, 2021
 *      Author: kus
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "gpio.h"

#define REG_SEC				0x80
#define REG_MIN				0x82
#define REG_HOUR			0x84
#define REG_DATE			0x86
#define REG_MONTH			0x88
#define REG_DAY				0x8a
#define REG_YEAR			0x8c
#define REG_CONTROL			0x8e
#define REG_CHARGER			0x90

#define HEX_to_BCD(value)	((value) % 10 + (value) / 10 * 16)
#define BCD_to_Hex(value)	((value) % 16 + (value) / 16 * 10)

//#define SCLK	GPIO_PIN_6
//#define SDA	GPIO_PIN_7
//#define RST_RTC	GPIO_PIN_8



void Mode_Write(void);
void Mode_Read(void);

void Send_Command(uint8_t command);

void Write_Byte(uint8_t address, uint8_t value);
uint8_t Read_Byte(uint8_t address);

void Write_Time(uint8_t *buffor);
void Read_Time(uint8_t *buffor);

void RTC_Init(void);

#endif /* INC_RTC_H_ */
