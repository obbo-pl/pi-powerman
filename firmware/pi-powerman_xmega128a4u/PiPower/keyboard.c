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
void keyboard_SetKeyHistory(uint8_t keystatus, uint8_t *keyhistory);
static uint8_t keyboard_ScanFilter(const uint8_t keyhistory);
void keyboard_Status(uint8_t *const keyhistory, uint8_t *const keystatus, uint8_t *const keytimer,
					 uint8_t *const speedcounter, uint8_t const keycount);


#define KEYBOARD_KEY_DOWN				0x01
#define KEYBOARD_KEY_UP					0x80

uint8_t keyboard_keypress;
uint8_t keyboard_lastkey;
uint8_t keyboard_keystatus[KEYBOARD_KEY_COUNT];
uint8_t keyboard_keyhistory[KEYBOARD_KEY_COUNT];
uint8_t keyboard_keycounter[KEYBOARD_KEY_COUNT];
uint8_t keyboard_keytimer[KEYBOARD_KEY_COUNT];



void keyboard_Status(uint8_t *const keyhistory, uint8_t *const keystatus, uint8_t *const keytimer,
					 uint8_t *const speedcounter, uint8_t const keycount) 
{
	uint8_t k;
	
	for (uint8_t i = 0; i < keycount; i++) {
		if (testbit(keystatus[i], KEYBOARD_KeyEnabled)) {
			k = keyboard_ScanFilter(keyhistory[i]);
		} else {
			k = KEYBOARD_KEY_UP;
		}
		if (k == KEYBOARD_KEY_UP) {
			keyboard_ClearKey(i);
			keytimer[i] = 0;
			speedcounter[i] = 0;
		} else if (k == KEYBOARD_KEY_DOWN) {
			if (keytimer[i] == 0) {
				setbit(keystatus[i], KEYBOARD_KeyStatusPressed);
				if (speedcounter[i] < 0xff) {
					speedcounter[i]++;
				}
				keytimer[i] = KEYBOARD_TIMER_LONG;
				if ((speedcounter[i] > KEYBOARD_REPEAT_FAST) && (testbit(keystatus[i], KEYBOARD_KeyRepeatFastEnable))) {
					keytimer[i] = KEYBOARD_TIMER_SHORT;
				}
				if ((speedcounter[i] > KEYBOARD_MULTIPY_VALUE) && (testbit(keystatus[i], KEYBOARD_KeyMultiplyEnable))){
					setbit(keystatus[i], KEYBOARD_KeyStatusMultiply);
				}
			}
			if ((keytimer[i] > 0) && (testbit(keystatus[i], KEYBOARD_KeyRepeatEnable))) {
				keytimer[i]--;
			}
		} else {
		}
	}
}

static uint8_t keyboard_ScanFilter(const uint8_t keyhistory) 
{
	#if KEYBOARD_LOGIC_INVERT == 0
		if ((keyhistory & KEYBOARD_FILTER) == KEYBOARD_FILTER) {
			return KEYBOARD_KEY_DOWN;
		} else if ((keyhistory & KEYBOARD_FILTER) == 0) {
			return KEYBOARD_KEY_UP;
		} else return 0;
	#else
		if ((keyhistory & KEYBOARD_FILTER) == KEYBOARD_FILTER) {
			return KEYBOARD_KEY_UP;
		} else if ((keyhistory & KEYBOARD_FILTER) == 0) {
			return KEYBOARD_KEY_DOWN;
		} else return 0;
	#endif
}

void keyboard_SetKeyHistory(uint8_t keystatus, uint8_t *keyhistory)
{
	for (int i = 0; i < KEYBOARD_KEY_COUNT; i++) { 
		keyhistory[i] <<= 1;
		if (testbit(keystatus, i)) {  
			keyhistory[i] |= 0x01;
		} else {
			keyhistory[i] &= ~0x01;
		}
	}
}

void keyboard_UpdateKeyboardStatus(uint8_t (*keyscan)()) 
{
	keyboard_keypress = keyscan();
	keyboard_SetKeyHistory(keyboard_keypress, keyboard_keyhistory);
	keyboard_Status(keyboard_keyhistory, keyboard_keystatus, keyboard_keytimer, keyboard_keycounter, KEYBOARD_KEY_COUNT);
}

void keyboard_ClearKey(KEY_t key)
{
	clrbit(keyboard_keystatus[key], KEYBOARD_KeyStatusPressed);
	clrbit(keyboard_keystatus[key], KEYBOARD_KeyStatusMultiply);
}

uint8_t keyboard_ReadKey(KEY_t key)
{
	return keyboard_keystatus[key];
}

void keyboard_SetupKey(KEY_t key, uint8_t setup)
{
	uint8_t setupmask = 0xff;
	clrbit(setupmask, KEYBOARD_KeyStatusPressed);
	clrbit(setupmask, KEYBOARD_KeyStatusMultiply);
	keyboard_keystatus[key] |= setup & setupmask;
}

