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
#include "bitlib.h"


enum led_setup {
	LED_STATE_On,
	LED_STATE_Blink,
	LED_STATE_Invert
};


void led_Init(OB_LED_t *led, void (*set_led_off)(void), void (*set_led_on)(void))
{
	led->presence = false;
	led->counter = 0;
	led->period = 100;
	led->duty = 50;
	led->led_off = set_led_off;
	led->led_on = set_led_on;
	led->setup = 0x00;
}

void led_SetPresent(OB_LED_t *led)
{
	led->presence = true;
}

void led_UpdateStatus(OB_LED_t *led)
{
	if ((testbit(led->setup, LED_STATE_On)) && (led->presence)) {
		if (testbit(led->setup, LED_STATE_Blink)) {
			led->counter += 1;
			if (led->counter >= led->period) led->counter = 0;
			if (testbit(led->setup, LED_STATE_Invert)) {
				if (led->duty > led->counter) {
					led->led_on();
				} else {
					led->led_off();
				}
			} else {
				if (led->duty < led->counter) {
					led->led_on();
				} else {
					led->led_off();
				}
			}
		} else {
			led->led_on();
		}
	} else {
		led->led_off();
	}
}

// Sets LED light on or off
void led_SetLedOn(OB_LED_t *led, bool state)
{
	led->counter = 0;
	if (state) {
		setbit(led->setup, LED_STATE_On);
	} else {
		clrbit(led->setup, LED_STATE_On);
	}
}

// Sets LED light blinking or solid
void led_SetLedBlink(OB_LED_t *led, bool state)
{
	led->counter = 0;
	if (state) {
		setbit(led->setup, LED_STATE_Blink);
	} else {
		clrbit(led->setup, LED_STATE_Blink);
	}
}

void led_SetLedTimers(OB_LED_t *led, uint8_t period, uint8_t duty) 
{
	led->counter = 0;
	led->period = period;
	led->duty = duty;
}

// Sets LED blinking behavior
void led_SetLedBlinkInverted(OB_LED_t *led, bool state)
{
	led->counter = 0;
	if (state) {
		setbit(led->setup, LED_STATE_Invert);
	} else {
		clrbit(led->setup, LED_STATE_Invert);
	}
}

void led_SetLed(OB_LED_t *led, bool state, bool blink)
{
	led_SetLedOn(led, state);
	led_SetLedBlink(led, blink);
}
