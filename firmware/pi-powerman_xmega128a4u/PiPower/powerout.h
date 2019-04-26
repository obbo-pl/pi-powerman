/*
 * powerout.h
 *
 * Created: 2018-12-25 20:52:20
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#ifndef POWEROUT_H_
#define POWEROUT_H_

#include <stdint.h>
#include <avr/io.h>

#define POWEROUT_OUTPUT_COUNT		4

#define POWEROUT_OUTPUT_1_port		PORTB
#define POWEROUT_OUTPUT_1_pin		3
#define POWEROUT_OUTPUT_2_port		PORTB
#define POWEROUT_OUTPUT_2_pin		2
#define POWEROUT_OUTPUT_3_port		PORTB
#define POWEROUT_OUTPUT_3_pin		1
#define POWEROUT_OUTPUT_4_port		PORTB
#define POWEROUT_OUTPUT_4_pin		0


typedef enum powerout {
	POWEROUT_OUTPUT_1,
	POWEROUT_OUTPUT_2,
	POWEROUT_OUTPUT_3,
	POWEROUT_OUTPUT_4
} PWRO_t;

enum {
	POWEROUT_STATE_Off,
	POWEROUT_STATE_On
};


void powerout_Setup();
void powerout_SetOutput(PWRO_t out, uint8_t state);
uint8_t powerout_CheckOutput();

#endif /* POWEROUT_H_ */