/*
 * counter.c
 *
 * Created: 2019-01-13 18:49:02
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "counter.h"


void counter_Init(COUNTER_t *counter, uint16_t length)
{
	counter->count = length;
	counter->length = length;
	counter->pause = false;
}

void counter_Set(COUNTER_t *counter, uint16_t length)
{
	counter->length = length;
}

void counter_Reset(COUNTER_t *counter)
{
	counter->count = counter->length;
	counter->pause = false;
}

void counter_Update(COUNTER_t *counter, uint16_t period)
{
	if (!counter->pause) {
		if (counter->count > period) counter->count -= period;
		else counter->count = 0;
	}
}

bool counter_Check(COUNTER_t *counter)
{
	if (counter->count == 0) return true;
	else return false;
}

void counter_Pause(COUNTER_t *counter)
{
	counter->pause = true;
}
