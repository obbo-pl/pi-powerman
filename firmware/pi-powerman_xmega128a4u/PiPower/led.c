/*
 * led.c
 *
 * Created: 2018-12-25 22:39:03
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "led.h"

enum led_state {
	LED_STATE_On,
	LED_STATE_Blink,
	LED_STATE_Invert
};

uint8_t led_ledcounter[LED_COUNT];
uint8_t led_ledstate[LED_COUNT];
uint8_t led_ledperiod[LED_COUNT];
uint8_t led_ledduty[LED_COUNT];


void led_UpdateStatus(void (*led_on)(LED_t), void (*led_off)(LED_t))
{
	for (uint8_t i = 0; i < LED_COUNT; i++) {
		if (testbit(led_ledstate[i], LED_STATE_On)) {
			if (testbit(led_ledstate[i], LED_STATE_Blink)) {
				led_ledcounter[i] += 1;
				if (led_ledcounter[i] >= led_ledperiod[i]) led_ledcounter[i] = 0x00;
				if (testbit(led_ledstate[i], LED_STATE_Invert)) {
					if (led_ledduty[i] > led_ledcounter[i]) {
						led_on(i);
					} else {
						led_off(i);
					}
				} else {
					if (led_ledduty[i] < led_ledcounter[i]) {
						led_on(i);
					} else {
						led_off(i);
					}
				}
			} else {
				led_on(i);
			}
		} else {
			led_off(i);
		}
	}
}

// Sets LED light on or off
void led_SetLedOn(LED_t led, bool state)
{
	led_ledcounter[led] = 0x00;
	if (state) {
		setbit(led_ledstate[led], LED_STATE_On);
	} else {
		clrbit(led_ledstate[led], LED_STATE_On);
	}
}

// Sets LED light blinking or solid
void led_SetLedBlink(LED_t led, bool state)
{
	led_ledcounter[led] = 0x00;
	if (state) {
		setbit(led_ledstate[led], LED_STATE_Blink);
	} else {
		clrbit(led_ledstate[led], LED_STATE_Blink);
	}
}

void led_SetLedBlinkTimers(LED_t led, uint8_t period, uint8_t duty) 
{
	led_ledperiod[led] = period;
	led_ledduty[led] = duty;
}

void led_SetLedBlinkInverted(LED_t led, bool state)
{
	led_ledcounter[led] = 0x00;
	if (state) {
		setbit(led_ledstate[led], LED_STATE_Invert);
	} else {
		clrbit(led_ledstate[led], LED_STATE_Invert);
	}
}

void led_SetLed(LED_t led, bool state, bool blink)
{
	led_SetLedOn(led, state);
	led_SetLedBlink(led, blink);
}
