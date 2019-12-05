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



void powerout_Init(OB_POWEROUT_t *out, void (*set_out)(uint8_t), uint8_t (*read_out)(void))
{
	out->presence = false;
	out->set_out = set_out;
	out->read_out = read_out;
}

void powerout_SetOn(OB_POWEROUT_t *out)
{
	out->set_out(POWEROUT_STATE_ON);
}

void powerout_SetOff(OB_POWEROUT_t *out)
{
	out->set_out(POWEROUT_STATE_OFF);
}

void powerout_SetPresent(OB_POWEROUT_t *out)
{
	out->presence = true;
}

bool powerout_CheckOutput(OB_POWEROUT_t *out)
{
	bool result = false;
	if (out->read_out()) result = true;
	return result;	
}
