/*
 * ups.c
 *
 * Created: 2019-01-13 23:11:29
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#include "ups.h"
#include <math.h>
#include <avr/eeprom.h>


uint8_t EEMEM offset_calibration = ((uint8_t)(25));
uint8_t EEMEM gain_calibration = 0;


uint8_t ups_CheckStatus(UPS_t *ups)
{
	uint8_t result = 0b00000000;
#ifdef CONFIG_UPS_INSTALLED
	if (testbit(UPS_STATUS_FAULT_port.IN, UPS_STATUS_FAULT_pin)) setbit(result, UPS_STATUS_FAULT);
	if (testbit(UPS_STATUS_CHRG_port.IN, UPS_STATUS_CHRG_pin)) setbit(result, UPS_STATUS_CHRG);
	if (testbit(UPS_STATUS_TOC_port.IN, UPS_STATUS_TOC_pin)) setbit(result, UPS_STATUS_TOC);
	if (testbit(UPS_STATUS_READY_port.IN, UPS_STATUS_READY_pin)) setbit(result, UPS_STATUS_READY);
	if (!ups->onbatery) setbit(result, UPS_STATUS_ON_BAT);
	if (!ups->charge_disabled) setbit(result, UPS_STATUS_CHRG_DISABLE);
	if (!ups->discharge) setbit(result, UPS_STATUS_DISCHARGE);
#endif
	return result;
}

void ups_DischargeBattery(UPS_t *ups, bool state)
{
#ifdef CONFIG_UPS_INSTALLED
	if (state) {
		setbit(CONFIG_BATTERY_DISCHARGE_PORT.OUT, CONFIG_BATTERY_DISCHARGE_PIN);
	} else {
		clrbit(CONFIG_BATTERY_DISCHARGE_PORT.OUT, CONFIG_BATTERY_DISCHARGE_PIN);
	}
	ups->discharge = state;
#endif
}

uint16_t ups_NTCTermistorToKelvin(float volt)
{
	#define r1		10000L	// R56
	#define r2		33000L	// R58
	#define beta	3435L	// NTC B=3435
	#define r0		10000L	// NTC 10k
	#define t0		298.15	// 298.15K = 25C
	// temperature LSB = 0.01K
	
	float rz = volt * r1 / (3.3 - volt);
	float rx = r2 * rz / (r2 - rz);
	float t = t0 * beta / (beta + (t0 * log(rx / r0)));
	return (uint16_t)((100 + (int8_t)(eeprom_read_byte(&gain_calibration)) / 100.0) * (t + (int8_t)(eeprom_read_byte(&offset_calibration)) / 10));
}

void ups_DisableCharge(UPS_t *ups, bool state)
{
#ifdef CONFIG_UPS_INSTALLED
	if (state) {
		clrbit(CONFIG_BATTARY_TEMPERATURE_PORT.OUT, CONFIG_BATTERY_TEMPERATURE_PIN);
		setbit(CONFIG_BATTARY_TEMPERATURE_PORT.DIR, CONFIG_BATTERY_TEMPERATURE_PIN);
	} else {
		clrbit(CONFIG_BATTARY_TEMPERATURE_PORT.DIR, CONFIG_BATTERY_TEMPERATURE_PIN);
	}
	ups->charge_disabled = state;
#endif
}

void ups_OnBattery(UPS_t *ups, bool state)
{
	ups->onbatery = state;
}

