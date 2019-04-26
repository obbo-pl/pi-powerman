/*
 * pidaemon.h
 *
 * Created: 2019-01-12 13:25:40
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef PIDAEMON_H_
#define PIDAEMON_H_

#include <avr/io.h>
#include <stdbool.h>
#include "bitlib.h"


#define PIDAEMON_PI_INT_REQUEST_PIN		2
#define PIDAEMON_PI_INT_REQUEST_PORT	PORTC

#define PIDAEMON_LEVEL_LOW				0x0537
#define PIDAEMON_LEVEL_HI_IMPEDANCE		0x06ab
#define PIDAEMON_LEVEL_HIGH				0x0b08

enum {
	PIDAEMON_STATE_LOW,   // 0x03c4, Daemon set port to low
	PIDAEMON_STATE_OFF,   // 0x06ab, RasPi port in high impedance state, daemon not working
	PIDAEMON_STATE_HIGH   // 0x0f66, Daemon set port to high
};


static inline void pidaemon_Init();
static inline void pidaemon_Init()
{
	// set pin of int_request to high and set as output
	setbit(PIDAEMON_PI_INT_REQUEST_PORT.OUT, PIDAEMON_PI_INT_REQUEST_PIN);
	setbit(PIDAEMON_PI_INT_REQUEST_PORT.DIR, PIDAEMON_PI_INT_REQUEST_PIN);
}


void pidaemon_SetRequest();
void pidaemon_ClearRequest();
uint8_t pidaemon_CheckDaemonState(uint16_t v);


#endif /* PIDAEMON_H_ */