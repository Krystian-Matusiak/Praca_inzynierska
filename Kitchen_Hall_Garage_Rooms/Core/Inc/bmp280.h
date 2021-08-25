/*
 * bmp280.h
 *
 *  Created on: Aug 23, 2021
 *      Author: kus
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_


#include <stdio.h>
#include "spi.h"


#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_TEMP_MSB    0xFA


#define OSRS_P 	 (2<<2); // skipped / x1 / x2 / x4 / x8 / x16
#define OSRS_T 	 (1<<5); // skipped / x1 / x2 / x4 / x8 / x16
#define MODE  	 0x01;   // sleep (00) / forced (01 / 10) / normal (11)
#define FILTER 	 (5<<2); // off / 2 / 4 / 8 / 16
#define T_SB 	 (0<<5);   // 0.5 / 62.5 / 125 / 250 / 500 / 1000 / 2000 / 4000
#define SPI3W_EN 0x00





int read_data( uint8_t *addr, int8_t * value , uint8_t len);
int uread_data( uint8_t *addr, uint8_t * value , uint8_t len);

void setConstants();
void BMP280_setup();
void get_temp_press(int32_t *temp , uint32_t *press);


#endif /* INC_BMP280_H_ */






