/*
 * counter.h
 *
 * Created: 2019-01-13 18:48:50
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef COUNTER_H_
#define COUNTER_H_

#include <avr/io.h>
#include <stdbool.h>



typedef struct COUNTER {
	uint16_t length;
	volatile uint16_t count;
	bool pause;
} COUNTER_t;


void counter_Init(COUNTER_t *counter, uint16_t length);
void counter_Set(COUNTER_t *counter, uint16_t length);
void counter_Reset(COUNTER_t *counter);
void counter_Update(COUNTER_t *counter, uint16_t period);
bool counter_Check(COUNTER_t *counter);
void counter_Pause(COUNTER_t *counter);

#endif /* COUNTER_H_ */