/*
 * led.h
 *
 * Created: 2018-12-25 22:39:18
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include <avr/io.h>
#include <stdbool.h>
#include "bitlib.h"


#define LED_COUNT				5
#define LED_PWR_RED_pin			1
#define LED_PWR_RED_port		PORTD
#define LED_PWR_GREEN_pin		2
#define LED_PWR_GREEN_port		PORTD
#define LED_SW1_pin				3
#define LED_SW1_port			PORTC
#define LED_SW2_pin				5
#define LED_SW2_port			PORTC
#define LED_SW3_pin				7
#define LED_SW3_port			PORTC


typedef enum led_name {
	LED_PWR_GREEN,
	LED_PWR_RED,
	LED_SW1,
	LED_SW2,
	LED_SW3
} LED_t;
	

// configure hardware, set pin as output
static inline void led_Init();
static inline void led_Init()
{
	setbit(LED_PWR_GREEN_port.DIR, LED_PWR_GREEN_pin);
	setbit(LED_PWR_RED_port.DIR, LED_PWR_RED_pin);
	setbit(LED_SW1_port.DIR, LED_SW1_pin);
	setbit(LED_SW2_port.DIR, LED_SW2_pin);
	setbit(LED_SW3_port.DIR, LED_SW3_pin);
}

// led_on, led_off - pointer to process corresponding port to turn on or off
void led_UpdateStatus(void (*led_on)(LED_t), void (*led_off)(LED_t));
void led_SetLedOn(LED_t led, bool state);
void led_SetLedBlink(LED_t led, bool state);
void led_SetLed(LED_t led, bool state, bool blink);
void led_SetLedBlinkTimers(LED_t led, uint8_t period, uint8_t duty);
void led_SetLedBlinkInverted(LED_t led, bool state);

#endif /* LED_H_ */

