/*
 * keyboard.c
 *
 * Created: 2016-06-23 22:43:26
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "keyboard.h"


// Prototypes of functions 
void key_SetKeyHistory(OB_KEY_t *key, uint8_t keystatus);
uint8_t key_ScanFilter(OB_KEY_t *key);
void key_Status(OB_KEY_t *key);



void key_Init(OB_KEY_t *key, uint8_t (*read_key)(void))
{
	key->presence = false;
	key->read_key = read_key;
}

void key_SetPresent(OB_KEY_t *key)
{
	key->presence = true;
}

void key_SetupKey(OB_KEY_t *key, uint8_t setup)
{
	uint8_t setupmask = 0xff;
	clrbit(setupmask, KEYBOARD_KeyStatusPressed);
	clrbit(setupmask, KEYBOARD_KeyStatusMultiply);
	key->keystatus |= setup & setupmask;
}

void key_ClearKey(OB_KEY_t *key)
{
	clrbit(key->keystatus, KEYBOARD_KeyStatusPressed);
	clrbit(key->keystatus, KEYBOARD_KeyStatusMultiply);
}

uint8_t key_CheckKey(OB_KEY_t *key)
{
	return key->keystatus;
}

void key_Status(OB_KEY_t *key) 
{
	uint8_t k;
	
	if (key->presence) {
		if (testbit(key->keystatus, KEYBOARD_KeyEnabled)) {
			k = key_ScanFilter(key);
		} else {
			k = KEY_STATE_UNPRESSED;
		}
		if (k == KEY_STATE_UNPRESSED) {
			key_ClearKey(key);
			key->keytimer = 0;
			key->keycounter = 0;
		} else if (k == KEY_STATE_PRESSED) {
			if (key->keytimer == 0) {
				setbit(key->keystatus, KEYBOARD_KeyStatusPressed);
				if (key->keycounter < 0xff) {
					key->keycounter++;
				}
				key->keytimer = KEY_TIMER_LONG;
				if ((key->keycounter > KEY_REPEAT_FAST) && (testbit(key->keystatus, KEYBOARD_KeyRepeatFastEnable))) {
					key->keytimer = KEY_TIMER_SHORT;
				}
				if ((key->keycounter > KEY_MULTIPY_VALUE) && (testbit(key->keystatus, KEYBOARD_KeyMultiplyEnable))){
					setbit(key->keystatus, KEYBOARD_KeyStatusMultiply);
				}
			}
			if ((key->keytimer > 0) && (testbit(key->keystatus, KEYBOARD_KeyRepeatEnable))) {
				key->keytimer--;
			}
		} else {
		}
	}
}

uint8_t key_ScanFilter(OB_KEY_t *key) 
{
	#if KEYBOARD_LOGIC_INVERT == 0
		if ((key->keyhistory & KEY_FILTER) == KEY_FILTER) {
			return KEY_STATE_PRESSED;
		} else if ((key->keyhistory & KEY_FILTER) == 0) {
			return KEY_STATE_UNPRESSED;
		} else return 0;
	#else
		if ((key->keyhistory & KEY_FILTER) == KEY_FILTER) {
			return KEY_STATE_UNPRESSED;
		} else if ((key->keyhistory & KEY_FILTER) == 0) {
			return KEY_STATE_PRESSED;
		} else return 0;
	#endif
}

void key_SetKeyHistory(OB_KEY_t *key, uint8_t keystatus)
{
	key->keyhistory <<= 1;
	if (keystatus == KEY_STATE_PRESSED) {  
		key->keyhistory |= 0x01;
	} else {
		key->keyhistory &= ~0x01;
	}
}

void key_UpdateKeyStatus(OB_KEY_t *key) 
{
	uint8_t keyboard_keypress = key->read_key();
	key_SetKeyHistory(key, keyboard_keypress);
	key_Status(key);
}




