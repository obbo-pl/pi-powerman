/*
 * powerout.c
 *
 * Created: 2018-12-25 20:52:43
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "powerout.h"
#include "bitlib.h"


void powerout_SetOutputPin(PORT_t *port, uint8_t out, uint8_t state);

void powerout_Setup()
{
	powerout_SetOutputPin(&POWEROUT_OUTPUT_1_port, POWEROUT_OUTPUT_1_pin, POWEROUT_STATE_Off);
	powerout_SetOutputPin(&POWEROUT_OUTPUT_2_port, POWEROUT_OUTPUT_2_pin, POWEROUT_STATE_Off);
	powerout_SetOutputPin(&POWEROUT_OUTPUT_3_port, POWEROUT_OUTPUT_3_pin, POWEROUT_STATE_Off);
	powerout_SetOutputPin(&POWEROUT_OUTPUT_4_port, POWEROUT_OUTPUT_4_pin, POWEROUT_STATE_Off);
	setbit(POWEROUT_OUTPUT_1_port.DIR, POWEROUT_OUTPUT_1_pin);
	setbit(POWEROUT_OUTPUT_2_port.DIR, POWEROUT_OUTPUT_2_pin);
	setbit(POWEROUT_OUTPUT_3_port.DIR, POWEROUT_OUTPUT_3_pin);
	setbit(POWEROUT_OUTPUT_4_port.DIR, POWEROUT_OUTPUT_4_pin);
}

void powerout_SetOutputPin(PORT_t *port, uint8_t out, uint8_t state)
{
	if (state == POWEROUT_STATE_On) {
		clrbit(port->OUT, out);
	} else {
		setbit(port->OUT, out);
	}
}

void powerout_SetOutput(PWRO_t out, uint8_t state)
{
	switch(out) {
		case POWEROUT_OUTPUT_1:
			powerout_SetOutputPin(&POWEROUT_OUTPUT_1_port, POWEROUT_OUTPUT_1_pin, state);
			break;
		case POWEROUT_OUTPUT_2:
			powerout_SetOutputPin(&POWEROUT_OUTPUT_2_port, POWEROUT_OUTPUT_2_pin, state);
			break;
		case POWEROUT_OUTPUT_3:
			powerout_SetOutputPin(&POWEROUT_OUTPUT_3_port, POWEROUT_OUTPUT_3_pin, state);
			break;
		case POWEROUT_OUTPUT_4:
			powerout_SetOutputPin(&POWEROUT_OUTPUT_4_port, POWEROUT_OUTPUT_4_pin, state);
			break;			
	}
}

uint8_t powerout_CheckOutput()
{
	uint8_t result = 0b00000000;
	if (!testbit(POWEROUT_OUTPUT_1_port.OUT, POWEROUT_OUTPUT_1_pin)) setbit(result, POWEROUT_OUTPUT_1);
	if (!testbit(POWEROUT_OUTPUT_2_port.OUT, POWEROUT_OUTPUT_2_pin)) setbit(result, POWEROUT_OUTPUT_2);
	if (!testbit(POWEROUT_OUTPUT_3_port.OUT, POWEROUT_OUTPUT_3_pin)) setbit(result, POWEROUT_OUTPUT_3);
	if (!testbit(POWEROUT_OUTPUT_4_port.OUT, POWEROUT_OUTPUT_4_pin)) setbit(result, POWEROUT_OUTPUT_4);
	return result;	
}
