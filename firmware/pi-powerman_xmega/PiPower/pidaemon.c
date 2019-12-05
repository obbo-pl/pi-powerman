/*
 * pidaemon.c
 *
 * Created: 2019-01-12 13:25:53
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "pidaemon.h"


#define FILTER_SIZE		8
uint16_t filter[FILTER_SIZE] = {
	PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE,
	PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE, PIDAEMON_LEVEL_HI_IMPEDANCE
};
uint8_t filter_counter = 0;


/* Set request for daemon, low level */
void pidaemon_SetRequest()
{
	clrbit(CONFIG_PIDAEMON_INT_REQUEST_PORT.OUT, CONFIG_PIDAEMON_INT_REQUEST_PIN);
}

/* Clear request for daemon, high level */
void pidaemon_ClearRequest()
{
	setbit(CONFIG_PIDAEMON_INT_REQUEST_PORT.OUT, CONFIG_PIDAEMON_INT_REQUEST_PIN);
}

uint8_t pidaemon_CheckDaemonState(uint16_t v)
{
#ifdef	CONFIG_PIDAEMON_LOGIC_LEVEL_CHECK
	if (v == 1) return PIDAEMON_STATE_HIGH;
	else return PIDAEMON_STATE_LOW;
#else
	filter[filter_counter] = v;
	filter_counter++;
	if (filter_counter >= FILTER_SIZE) filter_counter = 0;
	uint16_t v_aver = 0;		// good enough for 16 measurement 
	for (uint8_t i = 0; i < FILTER_SIZE; i++) {
		v_aver += filter[i];
	}
	v_aver = v_aver >> 3;
	if (v_aver > PIDAEMON_LEVEL_HIGH) return PIDAEMON_STATE_HIGH;
	else if (v_aver < PIDAEMON_LEVEL_LOW) return PIDAEMON_STATE_LOW;
	return PIDAEMON_STATE_OFF;
#endif
}
