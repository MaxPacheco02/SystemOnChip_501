/*
 * myprintf.c
 *
 *  Created on: Apr 11, 2023
 *      Author: rahu7p
 */

#include "myprintf.h"
#include "main.h"

int _write(int file, char *ptr, int len){
	int DataIdx;
	for(DataIdx=0; DataIdx<len; DataIdx++){
		while( ( USART3->SR & USART_SR_TXE ) == 0 ){}
		USART3->DR = *ptr++;
	}
	return len;
}
