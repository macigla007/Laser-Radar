/*
 * TF02.h
 *
 *  Created on: 9 paź 2020
 *      Author: Glanu
 */

#ifndef INC_TF02_H_
#define INC_TF02_H_

#include "main.h"
#include "stdio.h"
#include <stdint.h>
#include "stdbool.h"

volatile uint8_t buff[9];
volatile uint16_t speed;
volatile uint16_t distance;
volatile uint16_t lastDistance;
volatile uint16_t strength;
volatile uint8_t buffIndex;
volatile uint8_t temp;
volatile uint8_t receivedFrame;
uint32_t checksum;

volatile uint32_t actualTimeStamp;
volatile uint32_t lastTimeStamp;
volatile uint32_t diffTimeStamp;
volatile uint8_t flagTs;
volatile uint8_t flagDs;

bool measure(UART_HandleTypeDef UART1);
uint16_t getDistance();
uint16_t getStrength();

#endif /* INC_TF02_H_ */
