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

#include "bitlib.h"
#include <stdio.h>
#include <avr/io.h>


// hardware setup
#define KEYBOARD_KEY_COUNT				4
#define KEYBOARD_KEY_PWR_pin			3
#define KEYBOARD_KEY_PWR_port			PORTD
#define KEYBOARD_KEY_SW1_pin			4
#define KEYBOARD_KEY_SW1_port			PORTC
#define KEYBOARD_KEY_SW2_pin			6
#define KEYBOARD_KEY_SW2_port			PORTC
#define KEYBOARD_KEY_SW3_pin			0
#define KEYBOARD_KEY_SW3_port			PORTD

typedef enum kyboard_key_name {
	KEYBOARD_KEY_PWR,
	KEYBOARD_KEY_SW1,
	KEYBOARD_KEY_SW2,
	KEYBOARD_KEY_SW3
} KEY_t;

// set KEYBOARD_LOGIC_INVERT = 0 if pressed button sets bit to 1
// set KEYBOARD_LOGIC_INVERT = 1 if pressed button sets bit to 0
#define KEYBOARD_LOGIC_INVERT			0
#define KEYBOARD_FILTER					0b00000011

#define KEYBOARD_TIMER_LONG				10
#define KEYBOARD_TIMER_SHORT			1
#define KEYBOARD_REPEAT_FAST			3
#define KEYBOARD_MULTIPY_VALUE			12
#define KEYBOARD_KEY_MULTIPLYER			4


enum {
	KEYBOARD_KeyStatusPressed,
	KEYBOARD_KeyStatusMultiply,
	KEYBOARD_KeyEnabled,
	KEYBOARD_KeyRepeatEnable,
	KEYBOARD_KeyRepeatFastEnable,
	KEYBOARD_KeyMultiplyEnable  
};

static inline void keyboard_Init();
static inline void keyboard_Init()
{
	// set pull-up resistor on button input 
	*(&KEYBOARD_KEY_PWR_port.PIN0CTRL + KEYBOARD_KEY_PWR_pin) = PORT_OPC_PULLUP_gc;
	*(&KEYBOARD_KEY_SW1_port.PIN0CTRL + KEYBOARD_KEY_SW1_pin) = PORT_OPC_PULLUP_gc;
	*(&KEYBOARD_KEY_SW2_port.PIN0CTRL + KEYBOARD_KEY_SW2_pin) = PORT_OPC_PULLUP_gc;
	*(&KEYBOARD_KEY_SW3_port.PIN0CTRL + KEYBOARD_KEY_SW3_pin) = PORT_OPC_PULLUP_gc;
}


void keyboard_UpdateKeyboardStatus(uint8_t (*keyscan)());     // use this function in timer event or invoke in regular time period
                                                              // keyscan() should read every key pin
void keyboard_SetupKey(KEY_t key, uint8_t setup);
uint8_t keyboard_ReadKey(KEY_t key);
void keyboard_ClearKey(KEY_t key);


#endif /* KEYBOARD_H_ */