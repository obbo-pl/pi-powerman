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
#include <stdbool.h>


typedef struct OB_LED {
	bool presence;
	uint8_t counter;
	uint8_t setup;
	uint8_t period;
	uint8_t duty;
	void (*led_on)(void);
	void (*led_off)(void);
} OB_LED_t;
	

void led_Init(OB_LED_t *led, void (*set_led_off)(void), void (*set_led_on)(void));
void led_SetPresent(OB_LED_t *led);
void led_UpdateStatus(OB_LED_t *led);
void led_SetLedOn(OB_LED_t *led, bool state);
void led_SetLedBlink(OB_LED_t *led, bool state);
void led_SetLed(OB_LED_t *led, bool state, bool blink);
void led_SetLedTimers(OB_LED_t *led, uint8_t period, uint8_t duty);
void led_SetLedBlinkInverted(OB_LED_t *led, bool state);

#endif /* LED_H_ */

