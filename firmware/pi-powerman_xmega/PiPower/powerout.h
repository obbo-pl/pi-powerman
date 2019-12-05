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
#include <stdbool.h>
#include "PiConfig.h"


typedef struct OB_POWEROUT {
	bool presence;
	void (*set_out)(uint8_t);
	uint8_t (*read_out)(void);
} OB_POWEROUT_t;

enum {
	POWEROUT_STATE_OFF,
	POWEROUT_STATE_ON
};

void powerout_Init(OB_POWEROUT_t *out, void (*set_out)(uint8_t), uint8_t (*read_out)(void));
void powerout_SetOn(OB_POWEROUT_t *out);
void powerout_SetOff(OB_POWEROUT_t *out);
void powerout_SetPresent(OB_POWEROUT_t *out);
bool powerout_CheckOutput(OB_POWEROUT_t *out);

#endif /* POWEROUT_H_ */
