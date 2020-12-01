/*
 * TF02.c
 *
 *  Created on: 9 paź 2020
 *      Author: Glanu
 */
#include "TF02.h"

bool measure(UART_HandleTypeDef UART1)
{
	int i = 0;
	distance = 0;
	strength = 0;


	if(UART1.Instance->SR & UART1.RxState)
	{
		checksum = 0;

		if(UART1.Instance->DR == 0x59)
		{
			buff[0] == 0x59;

			if(UART1.Instance->DR == 0x59)
			{
				buff[1] == 0x59;

				for(i=2; i<9; i++)
				{
					buff[i] = UART1.Instance->DR;
				}
				checksum = buff[0]+buff[1]+buff[2]+buff[3]+buff[4]+buff[5]+buff[6]+buff[7];
				if(buff[8]==(checksum&0xff))
				{
					distance = buff[2] + buff[3] * 256;
					strength = buff[4] + buff[5] * 256;
					return true;
				}
			}
		}
	}
	return false;
}

uint16_t getDistance()
{
	return distance;
}

uint16_t getStrength()
{
	return strength;
}


