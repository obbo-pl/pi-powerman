/*
 * ups.h
 *
 * Created: 2019-01-13 23:11:41
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

// UPS module base on LTC4011
// 8 * Ni-MH 1900mA eneloop AA

#ifndef UPS_H_
#define UPS_H_

#include <stdio.h>
#include <avr/io.h>
#include <stdbool.h>
#include "bitlib.h"


// hardware setup
#define UPS_STATUS_FAULT_pin			7
#define UPS_STATUS_FAULT_port			PORTD
#define UPS_STATUS_CHRG_pin				6
#define UPS_STATUS_CHRG_port			PORTD
#define UPS_STATUS_TOC_pin				5
#define UPS_STATUS_TOC_port				PORTD
#define UPS_STATUS_READY_pin			4
#define UPS_STATUS_READY_port			PORTD

#define UPS_BATTERY_DISCHARGE_PORT		PORTE
#define UPS_BATTERY_DISCHARGE_PIN		3

#define UPS_BATTARY_TEMPERATURE_PORT	PORTA
#define UPS_BATTERY_TEMPERATURE_PIN		7

typedef struct ups_module {
	uint8_t state;
	bool charge_disabled;
	bool discharge;
	bool onbatery;
} UPS_t;

enum {
	UPS_STATUS_FAULT,
	UPS_STATUS_CHRG,
	UPS_STATUS_TOC,
	UPS_STATUS_READY,
	UPS_STATUS_CHRG_DISABLE,
	UPS_STATUS_DISCHARGE,
	UPS_STATUS_ON_BAT
};

enum {
	UPS_SHUT_IMMEDIATELY,
	UPS_SHUT_DELAYED,
	UPS_SHUT_ON_DISCHARGE		
};

// configure hardware, set pin as output
static inline void ups_Init(UPS_t *ups);
static inline void ups_Init(UPS_t *ups)
{
	setbit(UPS_BATTERY_DISCHARGE_PORT.DIR, UPS_BATTERY_DISCHARGE_PIN);
	ups->discharge = false;
	ups->charge_disabled = false;
	ups->onbatery = false;
}


uint8_t ups_CheckStatus(UPS_t *ups);
void ups_DischargeBattery(UPS_t *ups, bool state);  // <----------TODO: test
uint16_t ups_NTCTermistorToKelvin(float volt);
void ups_DisableCharge(UPS_t *ups, bool state);     // <---------TODO: test
void ups_OnBattery(UPS_t *ups, bool state);


#endif /* UPS_H_ */