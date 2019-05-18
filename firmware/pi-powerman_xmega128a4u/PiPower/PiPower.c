/*
 * PiPower.c
 *
 * Created: 2018-12-25 18:20:32
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "PiPower.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/fuse.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pidaemon.h"
#include "adc.h"

// Hardware resources: 
// RTC  - timer Int every 50ms, control: keyboard, LED, delays
// TWIE - master, to read INA219
// TWIC - slave, to communication with Raspberry Pi
// ADC  - measure voltage and temperature


FUSES = {
	.FUSEBYTE1 = 0x00,  /* Watchdog Configuration */
	.FUSEBYTE2 = 0xBD,  /* Reset Configuration */
	.FUSEBYTE4 = 0xF7,  /* Start-up Configuration */
	.FUSEBYTE5 = 0xE9,  /* EESAVE and BOD Level */
};

#define DATA_FRAME_FORMAT		"1"
#define UPS_PRESENCE			"1"
#define BUTTONS_COUNT			"4"
#define OUTPUTS_COUNT			"4"
#define VERSION_MAJOR			"0"
#define VERISON_MINOR			"8"
const char DEVICE_INFO_0[OUTPUT_BUFFER_SIZE]	PROGMEM = DATA_FRAME_FORMAT UPS_PRESENCE BUTTONS_COUNT OUTPUTS_COUNT;
const char DEVICE_INFO_1[OUTPUT_BUFFER_SIZE]	PROGMEM = "HW: Pi-Powerman." VERSION_MAJOR "." VERISON_MINOR " "; 
const char DEVICE_INFO_2[OUTPUT_BUFFER_SIZE]	PROGMEM = "(Build: " __DATE__ " " __TIME__ ")";
const char DEVICE_INFO_3[OUTPUT_BUFFER_SIZE]	PROGMEM = "UPS: LTC4011 8*NiMh 1900mAh";

// Default setup values
uint16_t EEMEM adc_vin_cal = 10403;
uint16_t EEMEM adc_vbat_cal = 9945;
uint8_t EEMEM main_OnBootOutput = (1 << POWERPORT_HDD);
uint16_t EEMEM main_OnBootDelay_s = 15;
uint16_t EEMEM main_daemon_timeout_s = 150;
uint8_t EEMEM main_error_recoverable = ((1 << MAIN_ERROR_TIMEOUT_DAEMON) | (1 << MAIN_ERROR_DAEMON));
uint8_t EEMEM ups_behavior = UPS_SHUT_DELAYED;
uint16_t EEMEM ups_delay_s = 180;
uint16_t EEMEM ups_cut_level_mv = 8000;

// Prototypes of functions 
void main_ReadSetup(void);
void twi_SlaveProcessData(void);
void main_ADCInitConversion(uint8_t mux);
void main_ClearButtonState();
void main_NewButtonState(KEY_t button, uint8_t state);
void main_LedConfig();
void main_SetLedOn(LED_t led);
void main_SetLedOff(LED_t led);
uint8_t main_ScanKeyStatus();
void main_KeyboardConfig();
void main_RC32MClockInit(uint8_t timeoutms);
void main_RTCInit();
void main_ClearButtonRequest();
void main_CopyToTWIBuffer(uint8_t *source, register8_t *dest);
void main_CopyFromTWIBuffer(register8_t *source, uint8_t *dest);
void main_CopyCharToTWIBuffer(const char *source, register8_t *dest);
void main_SaveSetup(void);
void main_SendSetup(register8_t *dest);
void main_PowerReduction(void);
void main_MapButtonsState(void);
void CCPWrite( volatile uint8_t *address, uint8_t value );


int main(void)
{
	main_MapStatusToBuffer();
	main_MapSetupToBuffer();
	// Initialize system clock
	main_RC32MClockInit(MAIN_TIMEOUT_MAINCLOCK_MS);
	main_RTCInit();
	// Read setup from EEMEM
	main_ReadSetup();
	// Initialize power output
	powerout_Setup();
	// Initialize keyboard
	keyboard_Init();
	main_KeyboardConfig();
	// Initialize LED's
	led_Init();
	main_LedConfig();
	// Initialize wired to RasPi (INT_request, RasPi_state) 
	pidaemon_Init();
	// Initialize ADC
	adc_Setup();
	// Initialize TWI slave
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, twi_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, TWI_SLAVE_ADDRESS, TWI_SLAVE_INTLVL_HI_gc);
	// Initialize TWI master
	TWI_MasterInit(&twiMaster, &TWIE, TWI_MASTER_INTLVL_MED_gc, TWI_BAUDSETTING);
	// Initialize UPS
	ups_Init(&ups);
    // Power reduction
	main_PowerReduction();
	// Initialize IRQ
    PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
    sei();
	_delay_ms(10);
    // Initialize INA219
	bool result = ina219_Init(&ina219, &twiMaster, 0x40);
	_delay_ms(10);
	result &= ina219_SetCalibration_12bit(&ina219, INA219_CONFIG_BVOLTAGERANGE_32V, INA219_CONFIG_GAIN_8_320MV, 0.1, 0.1);
	if (!result) setbit(*(pipo_status.errors), MAIN_ERROR_INA219);
	// Set button state to all off
	main_ClearButtonState();
	// Start ADC conversion
	main_ADCInitConversion(adc_CurrentInput());
	// Initialize delays and LowPass filters
	delays_Init(&timer_rtc_ms, MAIN_TIMER_RTC_TO_1S);
	delays_Init(&daemon_timeout_s, *(pipo_setup.daemon_timeout_s));
	delays_Pause(&daemon_timeout_s);
	delays_Init(&after_power_loss_s, *(pipo_setup.ups_shut_delay_s));
	delays_Pause(&after_power_loss_s);
	delays_Init(&adc_conversion_ms, MAIN_TIMER_ADCCONVERSION_MS);
	delays_Pause(&adc_conversion_ms);
	delays_Init(&adc_maesurement_ready, 0x0fff);
	uint8_t last_error = 0;
	lpfilter_Set(&lpf_voltage_in, 10);
	lpfilter_Set(&lpf_voltage_battery, 8);
	lpfilter_Set(&lpf_temperature, 8);
	uint8_t request;
	uint8_t ina219_request = 0;
	// start the main loop, the loop should take less then 50ms (MAIN_TIMER_KEBOARD_MS)
    while(1)
    {	
		// Scan the keyboard
		if(testbit(*(pipo_status.request), MAIN_REQUEST_SCAN_KEYBOARD)) {
			clrbit(*(pipo_status.request), MAIN_REQUEST_SCAN_KEYBOARD);
			keyboard_UpdateKeyboardStatus(main_ScanKeyStatus);
		}
		// On key press action
		// Power Button, switch to On
		if (testbit(keyboard_ReadKey(KEYBOARD_KEY_PWR), KEYBOARD_KeyStatusPressed)) {
			if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_OFF) {
				led_SetLed(LED_PWR_GREEN, true, true);
				if (testbit(*(pipo_setup.on_boot_output), POWERPORT_USB1)) powerout_SetOutput(POWERPORT_USB1, POWEROUT_STATE_On);
				if (testbit(*(pipo_setup.on_boot_output), POWERPORT_USB2)) powerout_SetOutput(POWERPORT_USB2, POWEROUT_STATE_On);
				if (testbit(*(pipo_setup.on_boot_output), POWERPORT_HDD)) powerout_SetOutput(POWERPORT_HDD, POWEROUT_STATE_On);
				delays_Init(&delay_on_boot_s, *(pipo_setup.on_boot_delay_s));
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_ON_01);
				keyboard_ClearKey(KEYBOARD_KEY_PWR);
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_ON_01) {
			if (delays_Check(&delay_on_boot_s)) {
				main_ClearButtonRequest();
				powerout_SetOutput(POWERPORT_RASPI, POWEROUT_STATE_On);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_ON_02);
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_ON_02) {
			if (*pipo_status.daemon_state == PIDAEMON_STATE_HIGH) {
				led_SetLed(LED_PWR_GREEN, true, false);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_ON);
				delays_Reset(&daemon_timeout_s);
				keyboard_ClearKey(KEYBOARD_KEY_PWR);
				keyboard_ClearKey(KEYBOARD_KEY_SW1);
				keyboard_ClearKey(KEYBOARD_KEY_SW2);
				keyboard_ClearKey(KEYBOARD_KEY_SW3);
			}
		}
		// Power Button, switch to Off
		request = button_request[KEYBOARD_KEY_PWR];
		if (testbit(keyboard_ReadKey(KEYBOARD_KEY_PWR), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
			if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_ON) {
				led_SetLed(LED_PWR_GREEN, true, true);
				delays_Pause(&daemon_timeout_s);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_01);
				// On BUTTON_STATE_TURN_OFF_01 RasPi should initiate shutdown procedure
				keyboard_ClearKey(KEYBOARD_KEY_PWR);
				button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_IDLE;
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_01) {
			if (*pipo_status.daemon_state == PIDAEMON_STATE_OFF) {
				delays_Init(&delay_after_shutdown_ms, DELAY_AFTER_SHUTDOWN_MS);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_02);
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_02) {
			if (delays_Check(&delay_after_shutdown_ms)) {
				powerout_SetOutput(POWERPORT_RASPI, POWEROUT_STATE_Off);
				powerout_SetOutput(POWERPORT_USB2, POWEROUT_STATE_Off);
				powerout_SetOutput(POWERPORT_USB1, POWEROUT_STATE_Off);
				delays_Init(&delay_after_raspi_poweroff_ms, DELAY_AFTER_RASPI_POWEROFF_MS);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_03);
			}
		}	
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_03) {
			if (delays_Check(&delay_after_raspi_poweroff_ms)) {
				powerout_SetOutput(POWERPORT_HDD, POWEROUT_STATE_Off);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_OFF);
				led_SetLed(LED_PWR_GREEN, false, false);
				led_SetLed(LED_SW1, false, false);
				led_SetLed(LED_SW2, false, false);
				led_SetLed(LED_SW3, false, false);
				main_ClearButtonState();
				main_ClearButtonRequest();
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_ON) {
			// Button 1
			if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] != BUTTON_STATE_DISABLE) {
				request = button_request[KEYBOARD_KEY_SW1];
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW1), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_ON)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] == BUTTON_STATE_OFF) {
						led_SetLed(LED_SW1, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW1, BUTTON_STATE_TURN_ON_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW1);
						button_request[KEYBOARD_KEY_SW1] = BUTTON_STATE_IDLE;
					}
				}
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW1), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] == BUTTON_STATE_ON) {
						led_SetLed(LED_SW1, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW1, BUTTON_STATE_TURN_OFF_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW1);
						button_request[KEYBOARD_KEY_SW1] = BUTTON_STATE_IDLE;
					}
				}
			} else {
				button_request[KEYBOARD_KEY_SW1] = BUTTON_STATE_IDLE;
			}
			// Button 2
			if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] != BUTTON_STATE_DISABLE) {
				request = button_request[KEYBOARD_KEY_SW2];
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW2), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_ON)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] == BUTTON_STATE_OFF) {
						led_SetLed(LED_SW2, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW2, BUTTON_STATE_TURN_ON_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW2);
						button_request[KEYBOARD_KEY_SW2] = BUTTON_STATE_IDLE;
					}
				}
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW2), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] == BUTTON_STATE_ON) {
						led_SetLed(LED_SW2, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW2, BUTTON_STATE_TURN_OFF_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW2);
						button_request[KEYBOARD_KEY_SW2] = BUTTON_STATE_IDLE;
					}
				}
			} else {
				button_request[KEYBOARD_KEY_SW2] = BUTTON_STATE_IDLE;
			}
			// Button 3
			if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] != BUTTON_STATE_DISABLE) {
				request = button_request[KEYBOARD_KEY_SW3];
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW3), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_ON)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] == BUTTON_STATE_OFF) {
						led_SetLed(LED_SW3, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW3, BUTTON_STATE_TURN_ON_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW3);
						button_request[KEYBOARD_KEY_SW3] = BUTTON_STATE_IDLE;
					}
				}
				if (testbit(keyboard_ReadKey(KEYBOARD_KEY_SW3), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
					if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] == BUTTON_STATE_ON) {
						led_SetLed(LED_SW3, true, true);
						main_NewButtonState(KEYBOARD_KEY_SW3, BUTTON_STATE_TURN_OFF_01);
						keyboard_ClearKey(KEYBOARD_KEY_SW3);
						button_request[KEYBOARD_KEY_SW3] = BUTTON_STATE_IDLE;
					}
				}
			} else {
				button_request[KEYBOARD_KEY_SW3] = BUTTON_STATE_IDLE;
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] == BUTTON_STATE_DISABLE) led_SetLed(LED_SW1, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] == BUTTON_STATE_OFF) led_SetLed(LED_SW1, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW1] == BUTTON_STATE_ON) led_SetLed(LED_SW1, true, false);
		//
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] == BUTTON_STATE_DISABLE) led_SetLed(LED_SW1, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] == BUTTON_STATE_OFF) led_SetLed(LED_SW2, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW2] == BUTTON_STATE_ON) led_SetLed(LED_SW2, true, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] == BUTTON_STATE_DISABLE) led_SetLed(LED_SW1, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] == BUTTON_STATE_OFF) led_SetLed(LED_SW3, false, false);
		if (pipo_status.buttons->state[KEYBOARD_KEY_SW3] == BUTTON_STATE_ON) led_SetLed(LED_SW3, true, false);
		// Update LED
		if(testbit(*(pipo_status.request), MAIN_REQUEST_UPDATE_LED)) {
			clrbit(*(pipo_status.request), MAIN_REQUEST_UPDATE_LED);
			led_UpdateStatus(main_SetLedOn, main_SetLedOff);
		}
		// Set MUX and start ADC conversion
		if (testbit(*(pipo_status.request), MAIN_REQUEST_ADC_READY)) {
			uint16_t v = adc_ReadInput();
			float volt;
			switch(adc_CurrentInput()) {
				case ADC_MUX_PiState:
					*pipo_status.daemon_state = pidaemon_CheckDaemonState(v);
					delays_Update(&adc_maesurement_ready, 1);
					break;
				case ADC_MUX_V_In:
				    *pipo_status.adc_V_In = (uint16_t)((*(pipo_setup.calibration_vin) / 10000.0) * lpfilter_Filter(&lpf_voltage_in, v) * 2500 / 4096);
					break;
				case ADC_MUX_V_Bat:
					*pipo_status.adc_V_Bat = (uint16_t)((*(pipo_setup.calibration_vbat) / 10000.0) * lpfilter_Filter(&lpf_voltage_battery, v) * 2500 / 4096);
					break;
				case ADC_MUX_BAT_Temp:
					volt = lpfilter_Filter(&lpf_temperature, v) * 2.5 / 4096;
					*pipo_status.adc_BAT_Temp = ups_NTCTermistorToKelvin(volt);
					// incorrect temperature readings can cause an overheating alarm
					// when the charging is disabled, the temperature measurement is not available
					if (ups.charge_disabled) {
						delays_Reset(&adc_maesurement_ready);
						*pipo_status.adc_BAT_Temp = 0;
					}
					break;
			}
			main_ADCInitConversion(adc_NextInput());
		}
		// Power supply outputs, check the change request
		if (output_request[0]) {
			for (int i = 0; i < POWEROUT_OUTPUT_COUNT; i++) {
				// skip Raspberry Pi supply output
				if (i == POWERPORT_RASPI) continue;
				if (testbit(output_request[0], i)) {
					if (testbit(output_request[1], i)) {
						powerout_SetOutput(i, POWEROUT_STATE_On);
					} else {
						powerout_SetOutput(i, POWEROUT_STATE_Off);
					}
					clrbit(output_request[0], i);
				}
			}
		}		
		// Power supply outputs, check the current state
		*pipo_status.output_state = powerout_CheckOutput();
		// Check UPS state
		*pipo_status.ups_state = ups_CheckStatus(&ups);
		// Read INA219, one request per loop
		switch(ina219_request) {
			uint16_t ina219_result;
			
			result = true;
			case INA219_REQUEST_BUS_VOLTAGE:
				result = ina219_GetRawBusVoltage(&ina219, &ina219_result);
				if (result && ina219.ready) pipo_status.ina219->bus_voltage = ina219_result;
				break;
			case INA219_REQUEST_SHUNT_VOLTAGE:
				result = ina219_GetRawShuntVoltage(&ina219, (int16_t *)(&ina219_result));
				if (result && ina219.ready) pipo_status.ina219->shunt_voltage = ina219_result;
				break;
			case INA219_REQUEST_CURRENT:
				result = ina219_GetRawCurrent(&ina219, (int16_t *)(&ina219_result));
				if (result && ina219.ready) pipo_status.ina219->current = ina219_result;
				break;
		}
		if (!result) setbit(*(pipo_status.errors), MAIN_ERROR_INA219);
		ina219_request++;
		if (ina219_request > INA219_REQUEST_COUNT) ina219_request = 0; 
		// Action on the main power loss
		// adc_V_In LSB = 0.01V, adc_V_Bat LSB = 0.01V
		if (delays_Check(&adc_maesurement_ready)) {
			uint16_t cut_level = trunc(*(pipo_setup.ups_cut_level_mv) / 10);
			if (*pipo_status.adc_V_In < 700) {
				if (after_power_loss_s.pause) delays_Reset(&after_power_loss_s);
				if ((*(pipo_setup.ups_behavior) == UPS_SHUT_IMMEDIATELY) || ((*(pipo_setup.ups_behavior) == UPS_SHUT_DELAYED) && (delays_Check(&after_power_loss_s))) ||
							(*pipo_status.adc_V_Bat < cut_level)) {
					button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_OFF;
					ups_OnBattery(&ups, true);
				}
			} else {
				if (!(after_power_loss_s.pause)) {
					delays_Reset(&after_power_loss_s);
					delays_Pause(&after_power_loss_s);
				}
				ups_OnBattery(&ups, false);
			}
		}
		// Check battery overheat, temperature LSB = 0.01K
		if (delays_Check(&adc_maesurement_ready)) {
			if (*pipo_status.adc_BAT_Temp > 33500) {
				setbit(*(pipo_status.errors), MAIN_ERROR_BATTERY_OVERHEAT);
				button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_OFF;
			} else {
				if (testbit(*(pipo_setup.error_recoverable), MAIN_ERROR_BATTERY_OVERHEAT)) clrbit(*(pipo_status.errors), MAIN_ERROR_BATTERY_OVERHEAT); 
			}
		}
		// Check for errors
		if  (delays_Check(&daemon_timeout_s)) {
			setbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_DAEMON);
		} else {
			if (testbit(*(pipo_setup.error_recoverable), MAIN_ERROR_TIMEOUT_DAEMON)) clrbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_DAEMON);
		}
		if (*(pipo_status.errors) != last_error) {
			pidaemon_SetRequest();
			last_error = *(pipo_status.errors);
			if (last_error) {
				led_SetLed(LED_PWR_RED, true, true);
			} else {
				led_SetLed(LED_PWR_RED, false, false);
			}
		}
		// Save new setup
		if (testbit(*(pipo_status.request), MAIN_REQUEST_SAVE_SETUP)) {
			clrbit(*(pipo_status.request), MAIN_REQUEST_SAVE_SETUP);
			main_SaveSetup();
			main_ReadSetup();
		}
	}
}

// Timers
ISR(RTC_OVF_vect) 
{
	setbit(*(pipo_status.request), MAIN_REQUEST_UPDATE_LED);
	setbit(*(pipo_status.request), MAIN_REQUEST_SCAN_KEYBOARD);
	delays_Update(&delay_after_raspi_poweroff_ms, MAIN_TIMER_KEBOARD_MS);
	delays_Update(&delay_after_shutdown_ms, MAIN_TIMER_KEBOARD_MS);
	delays_Update(&adc_conversion_ms, MAIN_TIMER_KEBOARD_MS);
	delays_Update(&timer_rtc_ms, MAIN_TIMER_KEBOARD_MS);
	if (delays_Check(&adc_conversion_ms)) setbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_ADC);
	if (delays_Check(&timer_rtc_ms)) {
		delays_Reset(&timer_rtc_ms);
		delays_Update(&daemon_timeout_s, 1);
		delays_Update(&after_power_loss_s, 1);
		delays_Update(&delay_on_boot_s, 1);
	}
}

// ADC conversion complete
ISR(ADCA_CH0_vect) 
{
	setbit(*(pipo_status.request), MAIN_REQUEST_ADC_READY);
}

// TWIC Slave Interrupt vector.
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

// TWIC Master Interrupt vector. 
ISR(TWIE_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

// TWI Slave execute command from master
void twi_SlaveProcessData(void)
{	
	// Raspberry Pi timeout for I2C operation ~ 600us
	// "twi_SlaveProcessData" is a part of TWIC_TWIS_vect, execution should take less then 600us
	// Read commands
	if (twiSlave.bytesReceived == 0) {
		switch(twiSlave.receivedData[0]) {
			case DAEMON_REQUEST_READ_STATUS:
				// ~20us
				delays_Reset(&daemon_timeout_s);
				pidaemon_ClearRequest();
				main_MapButtonsState();
				main_CopyToTWIBuffer(pipo_status_buffer, twiSlave.sendData);
				break;
			case DAEMON_REQUEST_READ_INFO0:
				// ~20us
				main_CopyCharToTWIBuffer(DEVICE_INFO_0, twiSlave.sendData);
				break;
			case DAEMON_REQUEST_READ_INFO1:
				// ~20us
				main_CopyCharToTWIBuffer(DEVICE_INFO_1, twiSlave.sendData);
				break;
			case DAEMON_REQUEST_READ_INFO2:
				// ~20us
				main_CopyCharToTWIBuffer(DEVICE_INFO_2, twiSlave.sendData);
				break;
			case DAEMON_REQUEST_READ_INFO3:
				// ~20us
				main_CopyCharToTWIBuffer(DEVICE_INFO_3, twiSlave.sendData);
				break;
			case DAEMON_REQUEST_READ_SETUP:
				// ~20us
				main_SendSetup(twiSlave.sendData);
				break;
		}
	}
	// Write commands, 2 bytes for values
	if (twiSlave.bytesReceived == 2) {
		uint8_t state;
		switch(twiSlave.receivedData[0]) {
			case DAEMON_REQUEST_BUTTON:
				// DAEMON_REQUEST_BUTTON <button> <new_state>
				// Changing the status of the PowerButton are locked
				if (twiSlave.receivedData[1] != KEYBOARD_KEY_PWR) main_NewButtonState(twiSlave.receivedData[1], twiSlave.receivedData[2]);
				break;
			case DAEMON_REQUEST_PRESS:
				// DAEMON_REQUEST_PRESS <button> [<new_state {ON, OFF} or default toggle {PRESS}>]
				state = twiSlave.receivedData[2];
				if ((state == BUTTON_STATE_ON) || (state == BUTTON_STATE_OFF)) {
					button_request[twiSlave.receivedData[1]] = state;			
				} else {
					button_request[twiSlave.receivedData[1]] = BUTTON_STATE_PRESS;
				}
				break;
			case DAEMON_REQUEST_OUTPUT:
				// DAEMON_REQUEST_OUTPUT <list_of_PS_output_to_change> <new_state>
				output_request[1] = twiSlave.receivedData[2];
				output_request[0] = twiSlave.receivedData[1];
				break; 
			case DAEMON_REQUEST_KEEPALIVE:
				// DAEMON_REQUEST_KEEPALIVE
				delays_Reset(&daemon_timeout_s);
				break;
			case DAEMON_REQUEST_ERROR:
				// DAEMON_REQUEST_ERROR
				setbit(*(pipo_status.errors), MAIN_ERROR_DAEMON);
				break;
			case DAEMON_REQUEST_SYSTEM_ERROR:
				// DAEMON_REQUEST_SYSTEM_ERROR
				setbit(*(pipo_status.errors), MAIN_ERROR_SYSTEM);
				break;
			case DAEMON_REQUEST_CLEAR_ERROR:
				// DAEMON_REQUEST_CLEAR_ERROR <error>
				clrbit(*(pipo_status.errors), twiSlave.receivedData[1]);
				break;
			case DAEMON_REQUEST_DISABLE_CHARGE:
				// DAEMON_REQUEST_DISABLE_CHARGE
				ups_DisableCharge(&ups, true);
				break;
			case DAEMON_REQUEST_ENABLE_CHARGE:
				// DAEMON_REQUEST_ENABLE_CHARGE
				ups_DisableCharge(&ups, false);
				break;
			case DAEMON_REQUEST_DISCHARGE_START:
				// DAEMON_REQUEST_CHARGE_START
				ups_DischargeBattery(&ups, true);
				break;
			case DAEMON_REQUEST_DISCHARGE_STOP:
				// DAEMON_REQUEST_CHARGE_STOP
				ups_DischargeBattery(&ups, false);
				break;
		}
	}
	// Write buffer
	if (twiSlave.bytesReceived == (OUTPUT_BUFFER_SIZE - 1)) {
		switch(twiSlave.receivedData[0]) {
			case DAEMON_REQUEST_WRITE_SETUP:
				// DAEMON_REQUEST_CONFIG <list_of_values>
				main_CopyFromTWIBuffer(twiSlave.receivedData, twi_read_buffer);
				setbit(*(pipo_status.request), MAIN_REQUEST_SAVE_SETUP);
				break;
		}
	}
}

void main_MapButtonsState(void)
{
	*(pipo_status.buttons_state) = 0;
	for (int i = 0; i < MAX_KEYBOARD_KEY_HALF; i++) {
		*(pipo_status.buttons_state) |= pipo_status.buttons->state[2 * i] << (8 * i + 4);
		*(pipo_status.buttons_state) |= pipo_status.buttons->state[2 * i + 1] << (8 * i);
	}
}

void main_CopyToTWIBuffer(uint8_t *source, register8_t *dest)
{
	for (int i = 0; i < OUTPUT_BUFFER_SIZE; i++) {
		*dest++ = (register8_t)(*source++); 
	}
}

void main_CopyFromTWIBuffer(register8_t *source, uint8_t *dest)
{
	// skip first byte (command)
	*source++;
	for (int i = 1; i < (OUTPUT_BUFFER_SIZE); i++) {
		*dest++ = (uint8_t)(*source++);
	}
}

void main_CopyCharToTWIBuffer(const char *source, register8_t *dest)
{
	for (int i = 0; i < OUTPUT_BUFFER_SIZE; i++) {
		*dest++ = (register8_t)pgm_read_byte(source++);
	}
}

void main_ReadSetup(void)
{
	*(pipo_setup.calibration_vin) = eeprom_read_word(&adc_vin_cal);
	*(pipo_setup.calibration_vbat) = eeprom_read_word(&adc_vbat_cal);
	*(pipo_setup.on_boot_output) = eeprom_read_byte(&main_OnBootOutput);
	*(pipo_setup.on_boot_delay_s) = eeprom_read_word(&main_OnBootDelay_s);
	*(pipo_setup.daemon_timeout_s) = eeprom_read_word(&main_daemon_timeout_s);
	*(pipo_setup.error_recoverable) = eeprom_read_byte(&main_error_recoverable);
	*(pipo_setup.ups_behavior) = eeprom_read_byte(&ups_behavior);
	*(pipo_setup.ups_shut_delay_s) = eeprom_read_word(&ups_delay_s);
	*(pipo_setup.ups_cut_level_mv) = eeprom_read_word(&ups_cut_level_mv);
}

void main_SaveSetup(void)
{
	eeprom_write_word(&adc_vin_cal, twi_read_buffer[1] * 0x100 + twi_read_buffer[0]);
	eeprom_write_word(&adc_vbat_cal, twi_read_buffer[3] * 0x100 + twi_read_buffer[2]);
	eeprom_write_byte(&main_OnBootOutput, twi_read_buffer[4]);
	eeprom_write_word(&main_OnBootDelay_s, twi_read_buffer[6] * 0x100 + twi_read_buffer[5]);
	eeprom_write_word(&main_daemon_timeout_s, twi_read_buffer[8] * 0x100 + twi_read_buffer[7]);
	eeprom_write_byte(&main_error_recoverable, twi_read_buffer[9]);
	eeprom_write_byte(&ups_behavior, twi_read_buffer[10]);
	eeprom_write_word(&ups_delay_s, twi_read_buffer[12] * 0x100 + twi_read_buffer[11]);
	eeprom_write_word(&ups_cut_level_mv, twi_read_buffer[14] * 0x100 + twi_read_buffer[13]);
}

void main_SendSetup(register8_t *dest)
{
	 main_CopyToTWIBuffer(pipo_setup_buffer, dest);
}

void main_ClearButtonState()
{
	for (int i = 0; i < KEYBOARD_KEY_COUNT; i++) {
		pipo_status.buttons->state[i] = BUTTON_STATE_OFF;
	}
}

void main_ClearButtonRequest()
{
	for (int i = 0; i < KEYBOARD_KEY_COUNT; i++) {
		button_request[i] = BUTTON_STATE_IDLE;
	}
}

void main_NewButtonState(KEY_t button, uint8_t state)
{
	pipo_status.buttons->state[button] = state;
	pidaemon_SetRequest();
}

void main_PowerReduction(void) 
{
	// Stop unused peripherals
	PR.PRPA = PR_AC_bm;
	PR.PRPB = PR_DAC_bm;
	PR.PRPC = PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm;
	PR.PRPD = PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_USB_bm | PR_HIRES_bm;
	PR.PRPE = PR_USART0_bm | PR_HIRES_bm;
}

void main_ADCInitConversion(uint8_t mux)
{
	adc_SetMux(mux);
	_delay_us(50);
	clrbit(*(pipo_status.request), MAIN_REQUEST_ADC_READY);
	adc_StartConversion();
	delays_Reset(&adc_conversion_ms);
}

void main_LedConfig()
{
	led_SetLedBlinkTimers(LED_PWR_GREEN, 20, 10);
	led_SetLedBlinkTimers(LED_PWR_RED, 10, 5);
	led_SetLedBlinkTimers(LED_SW1, 20, 10);
	led_SetLedBlinkTimers(LED_SW2, 20, 10);
	led_SetLedBlinkTimers(LED_SW3, 20, 10);
	led_SetLedBlinkInverted(LED_PWR_GREEN, true);
}

void main_SetLedOn(LED_t led)
{
	switch (led) {
		case LED_PWR_RED:
			setbit(LED_PWR_RED_port.OUT, LED_PWR_RED_pin);
			break;
		case LED_PWR_GREEN:
			setbit(LED_PWR_GREEN_port.OUT, LED_PWR_GREEN_pin);
			break;
		case LED_SW1:
			setbit(LED_SW1_port.OUT, LED_SW1_pin);
			break;
		case LED_SW2:
			setbit(LED_SW2_port.OUT, LED_SW2_pin);
			break;
		case LED_SW3:
			setbit(LED_SW3_port.OUT, LED_SW3_pin);
			break;
	}
}

void main_SetLedOff(LED_t led)
{
	switch (led) {
		case LED_PWR_RED:
			clrbit(LED_PWR_RED_port.OUT, LED_PWR_RED_pin);
			break;
		case LED_PWR_GREEN:
			clrbit(LED_PWR_GREEN_port.OUT, LED_PWR_GREEN_pin);
			break;
		case LED_SW1:
			clrbit(LED_SW1_port.OUT, LED_SW1_pin);
			break;
		case LED_SW2:
			clrbit(LED_SW2_port.OUT, LED_SW2_pin);
			break;
		case LED_SW3:
			clrbit(LED_SW3_port.OUT, LED_SW3_pin);
			break;
	}
}

uint8_t main_ScanKeyStatus()
{
	uint8_t result = 0b00000000;
	if (!testbit(KEYBOARD_KEY_PWR_port.IN, KEYBOARD_KEY_PWR_pin)) setbit(result, KEYBOARD_KEY_PWR);
	if (!testbit(KEYBOARD_KEY_SW1_port.IN, KEYBOARD_KEY_SW1_pin)) setbit(result, KEYBOARD_KEY_SW1);
	if (!testbit(KEYBOARD_KEY_SW2_port.IN, KEYBOARD_KEY_SW2_pin)) setbit(result, KEYBOARD_KEY_SW2);
	if (!testbit(KEYBOARD_KEY_SW3_port.IN, KEYBOARD_KEY_SW3_pin)) setbit(result, KEYBOARD_KEY_SW3);
	return result;
}

void main_KeyboardConfig()
{
	keyboard_SetupKey(KEYBOARD_KEY_PWR, bitmask(KEYBOARD_KeyEnabled) | bitmask(KEYBOARD_KeyMultiplyEnable));
	keyboard_SetupKey(KEYBOARD_KEY_SW1, bitmask(KEYBOARD_KeyEnabled));
	keyboard_SetupKey(KEYBOARD_KEY_SW2, bitmask(KEYBOARD_KeyEnabled));
	keyboard_SetupKey(KEYBOARD_KEY_SW3, bitmask(KEYBOARD_KeyEnabled));
}

void main_RC32MClockInit(uint8_t timeoutms) 
{
	OSC.CTRL = OSC_RC32MEN_bm;
	while (timeoutms) {
		_delay_ms(1);
		if (OSC.STATUS & OSC_RC32MRDY_bm) {
			CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
			break;
		}
		timeoutms--;
		if (!timeoutms) {
			setbit(*(pipo_status.errors), MAIN_ERROR_MAIN_CLOCK);
		}
	}
}

void main_RTCInit() 
{
	// The order of commands is important
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
	while (RTC.STATUS & RTC_SYNCBUSY_bm) {};
	RTC.PER = (uint16_t)MAIN_TIMER_KEBOARD_MS;
	RTC.INTCTRL = RTC_OVFINTLVL_HI_gc;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
}

// From Application Note AVR1003
void CCPWrite( volatile uint8_t* address, uint8_t value ) 
{
	uint8_t volatile saved_sreg = SREG;
	cli();

	#ifdef __ICCAVR__
	asm("movw r30, r16");
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm("ldi  r16,  0xD8 \n"
	"out  0x34, r16  \n"
	#if (__MEMORY_MODEL__ == 1)
	"st     Z,  r17  \n");
	#elif (__MEMORY_MODEL__ == 2)
	"st     Z,  r18  \n");
	#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	"st     Z,  r19  \n");
	#endif /* __MEMORY_MODEL__ */

	#elif defined __GNUC__
	volatile uint8_t * tmpAddr = address;
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
	);

	#endif
	SREG = saved_sreg;
}
  