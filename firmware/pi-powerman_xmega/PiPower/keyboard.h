/*
 * keyboard.h
 *
 * Created: 2016-06-23 22:44:03
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <stdio.h>
#include <avr/io.h>
#include <stdbool.h>
#include "bitlib.h"


typedef struct OB_KEY {
	bool presence;
	uint8_t keystatus;
	uint8_t keyhistory;
	uint8_t keycounter;
	uint8_t keytimer;
	uint8_t (*read_key)(void);
} OB_KEY_t;


// set KEY_LOGIC_INVERT = 0 if pressed button sets bit to 1
// set KEY_LOGIC_INVERT = 1 if pressed button sets bit to 0
#define KEY_LOGIC_INVERT			0
#define KEY_FILTER					0b00000011

#define KEY_TIMER_LONG				10
#define KEY_TIMER_SHORT				1
#define KEY_REPEAT_FAST				3
#define KEY_MULTIPY_VALUE			12

enum {
	KEY_STATE_PRESSED,
	KEY_STATE_UNPRESSED
};

enum {
	KEYBOARD_KeyStatusPressed,
	KEYBOARD_KeyStatusMultiply,
	KEYBOARD_KeyEnabled,
	KEYBOARD_KeyRepeatEnable,
	KEYBOARD_KeyRepeatFastEnable,
	KEYBOARD_KeyMultiplyEnable  
};


void key_Init(OB_KEY_t *key, uint8_t (*read_key)(void));
void key_SetPresent(OB_KEY_t *key);
void key_SetupKey(OB_KEY_t *key, uint8_t setup);
void key_ClearKey(OB_KEY_t *key);
uint8_t key_CheckKey(OB_KEY_t *key);
// use this function in timer event or invoke in regular time period
void key_UpdateKeyStatus(OB_KEY_t *key);     



#endif /* KEYBOARD_H_ */