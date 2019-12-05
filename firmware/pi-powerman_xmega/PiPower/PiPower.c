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
#include "macro.h"



// Hardware resources: 
// RTC  - timer Int every 50ms, control: keyboard, LED, delays
// TWIE - master, to read INA219
// TWIC - slave, to communication with Raspberry Pi
// ADC  - measure Raspberry status, voltage and temperature


FUSES = {
	.FUSEBYTE1 = 0x00,  /* Watchdog Configuration */
	.FUSEBYTE2 = 0xBD,  /* Reset Configuration */
	.FUSEBYTE4 = 0xF7,  /* Start-up Configuration */
	.FUSEBYTE5 = 0xE9,  /* EESAVE and BOD Level */
};

#define DATA_FRAME_FORMAT		0x01
#ifdef CONFIG_UPS_INSTALLED
	#define UPS_PRESENCE			0x01
#else
	#define UPS_PRESENCE			0x00
#endif
#ifdef CONFIG_KEY_PWR_port
	#define KEY_B0		1
#else
	#define KEY_B0		0
#endif
#ifdef CONFIG_KEY_SW1_port	
	#define KEY_B1		1
#else
	#define KEY_B1		0
#endif
#ifdef CONFIG_KEY_SW2_port
	#define KEY_B2		1
#else
	#define KEY_B2		0
#endif
#ifdef CONFIG_KEY_SW3_port
	#define KEY_B3		1
#else
	#define KEY_B3		0
#endif
#ifdef CONFIG_KEY_SW4_port
	#define KEY_B4		1
#else
	#define KEY_B4		0
#endif
#ifdef CONFIG_KEY_SW5_port
	#define KEY_B5		1
#else
	#define KEY_B5		0
#endif
#ifdef CONFIG_KEY_SW6_port
	#define KEY_B6		1
#else
	#define KEY_B6		0
#endif
#ifdef CONFIG_KEY_SW7_port
	#define KEY_B7		1
#else
	#define KEY_B7		0
#endif
#define BUTTON_PRESENCE		gluebyte(KEY_B7, KEY_B6, KEY_B5, KEY_B4, KEY_B3, KEY_B2, KEY_B1, KEY_B0)
#ifdef CONFIG_LED_PWR_port
	#define LED_B0		1
#else
	#define LED_B0		0
#endif
#ifdef CONFIG_LED_SW1_port
	#define LED_B1		1
#else
	#define LED_B1		0
#endif
#ifdef CONFIG_LED_SW2_port
	#define LED_B2		1
#else
	#define LED_B2		0
#endif
#ifdef CONFIG_LED_SW3_port
	#define LED_B3		1
#else
	#define LED_B3		0
#endif
#ifdef CONFIG_LED_SW4_port
	#define LED_B4		1
#else
	#define LED_B4		0
#endif
#ifdef CONFIG_LED_SW5_port
	#define LED_B5		1
#else
	#define LED_B5		0
#endif
#ifdef CONFIG_LED_SW6_port
	#define LED_B6		1
#else
	#define LED_B6		0
#endif
#ifdef CONFIG_LED_SW7_port
	#define LED_B7		1
#else
	#define LED_B7		0
#endif
#define LED_PRESENCE		gluebyte(LED_B7, LED_B6, LED_B5, LED_B4, LED_B3, LED_B2, LED_B1, LED_B0)
#ifdef CONFIG_POWEROUT_Pi_port
	#define OUT_B0		1
#else
	#define OUT_B0		0
#endif
#ifdef CONFIG_POWEROUT_1_port
	#define OUT_B1		1
#else
	#define OUT_B1		0
#endif
#ifdef CONFIG_POWEROUT_2_port
	#define OUT_B2		1
#else
	#define OUT_B2		0
#endif
#ifdef CONFIG_POWEROUT_3_port
	#define OUT_B3		1
#else
	#define OUT_B3		0
#endif
#ifdef CONFIG_POWEROUT_4_port
	#define OUT_B4		1
#else
	#define OUT_B4		0
#endif
#ifdef CONFIG_POWEROUT_5_port
	#define OUT_B5		1
#else
	#define OUT_B5		0
#endif
#ifdef CONFIG_POWEROUT_6_port
	#define OUT_B6		1
#else
	#define OUT_B6		0
#endif
#ifdef CONFIG_POWEROUT_7_port
	#define OUT_B7		1
#else
	#define OUT_B7		0
#endif
#define OUTPUT_PRESENCE		gluebyte(OUT_B7, OUT_B6, OUT_B5, OUT_B4, OUT_B3, OUT_B2, OUT_B1, OUT_B0)
#ifdef CONFIG_LED_STATUS_port
	#define LED_STATUS_PRESENCE		0x01
#else 
	#define LED_STATUS_PRESENCE		0x00
#endif
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	#define ADC_MEASURE_PRESENCE	0x07
#else
	#define ADC_MEASURE_PRESENCE	0x00
#endif
#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
	#define I2C_1MEASURE_PRESENCE	0x01
	#define I2C_2MEASURE_PRESENCE	0x01
	#define I2C_3MEASURE_PRESENCE	0x01
#else
	#define I2C_1MEASURE_PRESENCE	0x00
	#define I2C_2MEASURE_PRESENCE	0x00
	#define I2C_3MEASURE_PRESENCE	0x00
#endif
#define VERSION_MAJOR			"0"
#define VERISON_MINOR			"8"
const char DEVICE_INFO_0[OUTPUT_BUFFER_SIZE]	PROGMEM = {DATA_FRAME_FORMAT, UPS_PRESENCE, BUTTON_PRESENCE, LED_PRESENCE,
														OUTPUT_PRESENCE, LED_STATUS_PRESENCE, ADC_MEASURE_PRESENCE, I2C_1MEASURE_PRESENCE,
														I2C_2MEASURE_PRESENCE, I2C_3MEASURE_PRESENCE};
const char DEVICE_INFO_1[OUTPUT_BUFFER_SIZE]	PROGMEM = "HW: Pi-Powerman." VERSION_MAJOR "." VERISON_MINOR " "; 
const char DEVICE_INFO_2[OUTPUT_BUFFER_SIZE]	PROGMEM = "(Build: " __DATE__ " " __TIME__ ")";
#ifdef CONFIG_UPS_INSTALLED
	const char DEVICE_INFO_3[OUTPUT_BUFFER_SIZE]	PROGMEM = "UPS: " CONFIG_UPS_TYPE;
#else
	const char DEVICE_INFO_3[OUTPUT_BUFFER_SIZE]	PROGMEM = "no UPS";
#endif

OB_LED_t led_pwr;
OB_LED_t led_status;
OB_LED_t led_sw1;
OB_LED_t led_sw2;
OB_LED_t led_sw3;
OB_LED_t led_sw4;
OB_LED_t led_sw5;
OB_LED_t led_sw6;
OB_LED_t led_sw7;
OB_LED_t *led_sw_all[LED_MAX_COUNT];

OB_KEY_t key_pwr;
OB_KEY_t key_sw1;
OB_KEY_t key_sw2;
OB_KEY_t key_sw3;
OB_KEY_t key_sw4;
OB_KEY_t key_sw5;
OB_KEY_t key_sw6;
OB_KEY_t key_sw7;
OB_KEY_t *key_sw_all[KEY_MAX_COUNT];

OB_POWEROUT_t powerout_outpi;
OB_POWEROUT_t powerout_out1;
OB_POWEROUT_t powerout_out2;
OB_POWEROUT_t powerout_out3;
OB_POWEROUT_t powerout_out4;
OB_POWEROUT_t powerout_out5;
OB_POWEROUT_t powerout_out6;
OB_POWEROUT_t powerout_out7;
OB_POWEROUT_t *powerout_out_all[POWEROUT_MAX_COUNT];

// Default setup values
uint16_t EEMEM adc_vin_cal = 10403;
uint16_t EEMEM adc_vbat_cal = 9945;
uint8_t EEMEM main_on_boot_output = (1 << POWEROUT_OUTPUT_3);
uint16_t EEMEM main_on_boot_delay_s = 3;
uint16_t EEMEM main_on_down_delay_s = 5;
uint8_t EEMEM main_on_pioff_output = (1 << POWEROUT_OUTPUT_3);
uint16_t EEMEM main_on_pioff_delay_s = 3;
uint16_t EEMEM main_daemon_timeout_s = 150;
uint8_t EEMEM main_error_recoverable = ((1 << MAIN_ERROR_TIMEOUT_DAEMON) | (1 << MAIN_ERROR_DAEMON));
uint8_t EEMEM ups_behavior = UPS_SHUT_DELAYED;
uint16_t EEMEM ups_delay_s = 180;
uint16_t EEMEM ups_cut_level_mv = 8000;
uint16_t EEMEM ups_min_charge_time_s = 30;
uint16_t EEMEM battery_overheat_01k = 33500;
uint16_t EEMEM main_power_loss_mv = 7000;
uint16_t EEMEM dummy = 0x1111;

COUNTER_t timer_rtc_ms;
COUNTER_t delay_on_boot_s;
COUNTER_t delay_on_down_s;
COUNTER_t delay_on_pioff_s;
COUNTER_t daemon_timeout_s;
COUNTER_t after_power_loss_s;
COUNTER_t adc_conversion_timeout_ms;
COUNTER_t adc_maesurement_ready;

LPFu16_t lpf_voltage_in;
LPFu16_t lpf_voltage_battery;
LPFu16_t lpf_temperature;

// Prototypes of functions 
void main_MapStatusToBuffer();
void main_MapSetupToBuffer();
void main_ReadSetup(void);
void twi_SlaveProcessData(void);
void main_ADCInitConversion(uint8_t mux);
void main_ClearButtonState();
void main_NewButtonState(uint8_t button, uint8_t state);
void main_LedInit(void);
void main_LedPwrOn(void);
void main_LedPwrOff(void);
void main_LedStatusOn(void);
void main_LedStatusOff(void);
void main_LedSw1On(void);
void main_LedSw1Off(void);
void main_LedSw2On(void);
void main_LedSw2Off(void);
void main_LedSw3On(void);
void main_LedSw3Off(void);
void main_LedSw4On(void);
void main_LedSw4Off(void);
void main_LedSw5On(void);
void main_LedSw5Off(void);
void main_LedSw6On(void);
void main_LedSw6Off(void);
void main_LedSw7On(void);
void main_LedSw7Off(void);
void main_KeyboardInit(void);
uint8_t main_ReadKeyPwr(void);
uint8_t main_ReadKeySw1(void);
uint8_t main_ReadKeySw2(void);
uint8_t main_ReadKeySw3(void);
uint8_t main_ReadKeySw4(void);
uint8_t main_ReadKeySw5(void);
uint8_t main_ReadKeySw6(void);
uint8_t main_ReadKeySw7(void);
void main_PoweroutInit(void);
void main_SetPoweroutOutPi(uint8_t state);
uint8_t main_ReadPoweroutOutPi(void);
void main_SetPoweroutOut1(uint8_t state);
uint8_t main_ReadPoweroutOut1(void);
void main_SetPoweroutOut2(uint8_t state);
uint8_t main_ReadPoweroutOut2(void);
void main_SetPoweroutOut3(uint8_t state);
uint8_t main_ReadPoweroutOut3(void);
void main_SetPoweroutOut4(uint8_t state);
uint8_t main_ReadPoweroutOut4(void);
void main_SetPoweroutOut5(uint8_t state);
uint8_t main_ReadPoweroutOut5(void);
void main_SetPoweroutOut6(uint8_t state);
uint8_t main_ReadPoweroutOut6(void);
void main_SetPoweroutOut7(uint8_t state);
uint8_t main_ReadPoweroutOut7(void);
uint8_t main_CheckOutput(OB_POWEROUT_t **all);
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
	main_PoweroutInit();
	// Initialize keyboard
	main_KeyboardInit();
	// Initialize LED's
	main_LedInit();
	// Initialize wired to RasPi (INT_request, RasPi_state) 
	pidaemon_Init();
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED	
	// Initialize ADC
	adc_Setup();
#endif
	// Initialize TWI slave
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, twi_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, TWI_SLAVE_ADDRESS, TWI_SLAVE_INTLVL_HI_gc);
#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
	// Initialize TWI master
	TWI_MasterInit(&twiMaster, &TWIE, TWI_MASTER_INTLVL_MED_gc, TWI_BAUDSETTING);
#endif
#ifdef CONFIG_UPS_INSTALLED
	// Initialize UPS
	ups_Init(&ups);
#endif
    // Power reduction
	main_PowerReduction();
	// Initialize IRQ
    PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
    sei();
	_delay_ms(10);
#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
    // Initialize INA219
	bool result = ina219_Init(&ina219, &twiMaster, 0x40);
	_delay_ms(10);
	result &= ina219_SetCalibration_12bit(&ina219, INA219_CONFIG_BVOLTAGERANGE_32V, INA219_CONFIG_GAIN_8_320MV, 0.1, 0.1);
	if (!result) setbit(*(pipo_status.errors), MAIN_ERROR_INA219);
#endif
	// Set button state to all off
	main_ClearButtonState();
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	// Start ADC conversion
	main_ADCInitConversion(adc_CurrentInput());
#endif
	// Initialize delays and LowPass filters
	counter_Init(&timer_rtc_ms, MAIN_TIMER_RTC_TO_1S);
	counter_Init(&daemon_timeout_s, *(pipo_setup.daemon_timeout_s));
	counter_Pause(&daemon_timeout_s);
#ifdef CONFIG_UPS_INSTALLED
	counter_Init(&after_power_loss_s, *(pipo_setup.ups_shut_delay_s));
	counter_Pause(&after_power_loss_s);
#endif
	counter_Init(&adc_conversion_timeout_ms, MAIN_TIMER_ADCCONVERSION_MS);
	counter_Pause(&adc_conversion_timeout_ms);
	counter_Init(&adc_maesurement_ready, ADC_SKIP_FIRST_MEASURE);
	uint8_t last_error = 0;
	lpfilter_Set(&lpf_voltage_in, 10);
	lpfilter_Set(&lpf_voltage_battery, 8);
	lpfilter_Set(&lpf_temperature, 8);
	uint8_t request;
#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
	uint8_t ina219_request = 0;
#endif
	// start the main loop, the loop should take less then 50ms (MAIN_TIMER_KEBOARD_MS)
    while(1)
    {	
		// Scan the keyboard
		if(testbit(main_request, MAIN_REQUEST_SCAN_KEYBOARD)) {
			clrbit(main_request, MAIN_REQUEST_SCAN_KEYBOARD);
			for (int i = KEYBOARD_KEY_PWR; i < KEY_MAX_COUNT; i++) {
				key_UpdateKeyStatus(key_sw_all[i]);
			}
		}
		// On key press action
		// Power Button, switch to On
		if (testbit(key_CheckKey(&key_pwr), KEYBOARD_KeyStatusPressed)) {
			if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_OFF) {
				led_SetLed(&led_pwr, true, true);
				for (int i = POWEROUT_OUTPUT_1; i < POWEROUT_MAX_COUNT; i++) {
					if (testbit(*(pipo_setup.on_boot_output), i)) powerout_SetOn(powerout_out_all[i]);					
				}
				counter_Init(&delay_on_boot_s, *(pipo_setup.on_boot_delay_s));
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_ON_01);
				key_ClearKey(&key_pwr);
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_ON_01) {
			if (counter_Check(&delay_on_boot_s)) {
				main_ClearButtonRequest();
				powerout_SetOn(&powerout_outpi);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_ON_02);
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_ON_02) {
			if (*pipo_status.daemon == PIDAEMON_STATE_HIGH) {
				led_SetLed(&led_pwr, true, false);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_ON);
				counter_Reset(&daemon_timeout_s);
				for (int i = KEYBOARD_KEY_PWR; i < KEY_MAX_COUNT; i++) {
					if (key_sw_all[i]->presence) key_ClearKey(key_sw_all[i]);
				}				
			}
		}
		// Power Button, switch to Off
		request = button_request[KEYBOARD_KEY_PWR];
		if (testbit(key_CheckKey(&key_pwr), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
			if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_ON) {
				led_SetLed(&led_pwr, true, true);
				counter_Pause(&daemon_timeout_s);
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_01);
				// On BUTTON_STATE_TURN_OFF_01 RasPi should initiate shutdown procedure
				key_ClearKey(&key_pwr);
				button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_IDLE;
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_01) {
			if (*pipo_status.daemon != PIDAEMON_STATE_HIGH) {
				counter_Init(&delay_on_down_s, *(pipo_setup.on_down_delay_s));
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_02);
			}
			counter_Pause(&daemon_timeout_s);
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_02) {
			if (counter_Check(&delay_on_down_s)) {
				for (int i = POWEROUT_OUTPUT_Pi; i < POWEROUT_MAX_COUNT; i++) {
					if (!(testbit(*(pipo_setup.on_pioff_output), i))) powerout_SetOff(powerout_out_all[i]);
				}
				counter_Init(&delay_on_pioff_s, *(pipo_setup.on_pioff_delay_s));
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_TURN_OFF_03);
			}
		}	
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_TURN_OFF_03) {
			if (counter_Check(&delay_on_pioff_s)) {
				for (int i = POWEROUT_OUTPUT_Pi; i < POWEROUT_MAX_COUNT; i++) {
					if (testbit(*(pipo_setup.on_pioff_output), i)) powerout_SetOff(powerout_out_all[i]);
				}
				main_NewButtonState(KEYBOARD_KEY_PWR, BUTTON_STATE_OFF);
				for (int i = 0; i < LED_MAX_COUNT; i++) {
					led_SetLed(led_sw_all[i], false, false);
				}
				main_ClearButtonState();
				main_ClearButtonRequest();
			}
		}
		if (pipo_status.buttons->state[KEYBOARD_KEY_PWR] == BUTTON_STATE_ON) {
			// Scan buttons
			for (int i = KEYBOARD_KEY_SW1; i < KEY_MAX_COUNT; i++) {
				if (key_sw_all[i]->presence) {
					request = button_request[i];
					if (testbit(key_CheckKey(key_sw_all[i]), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_ON)) {
						if (pipo_status.buttons->state[i] == BUTTON_STATE_OFF) {
							if (led_sw_all[i]->presence) led_SetLed(led_sw_all[i], true, true);
							main_NewButtonState(i, BUTTON_STATE_TURN_ON_01);
							key_ClearKey(key_sw_all[i]);
							button_request[i] = BUTTON_STATE_IDLE;
						}
					}
					if (testbit(key_CheckKey(key_sw_all[i]), KEYBOARD_KeyStatusPressed) || (request == BUTTON_STATE_PRESS) || (request == BUTTON_STATE_OFF)) {
						if (pipo_status.buttons->state[i] == BUTTON_STATE_ON) {
							if (led_sw_all[i]->presence) led_SetLed(led_sw_all[i], true, true);
							main_NewButtonState(i, BUTTON_STATE_TURN_OFF_01);
							key_ClearKey(key_sw_all[i]);
							button_request[i] = BUTTON_STATE_IDLE;
						}
					}
				} else {
					button_request[i] = BUTTON_STATE_IDLE;
				}
			}
		}
		// Set LEDs
		for (int i = KEYBOARD_KEY_SW1; i < KEY_MAX_COUNT; i++) {
			if (led_sw_all[i]->presence) {
				if (pipo_status.buttons->state[i] == BUTTON_STATE_DISABLE) led_SetLed(led_sw_all[i], false, false);
				if (pipo_status.buttons->state[i] == BUTTON_STATE_OFF) led_SetLed(led_sw_all[i], false, false);
				if (pipo_status.buttons->state[i] == BUTTON_STATE_ON) led_SetLed(led_sw_all[i], true, false);
			}
		}
		// Update LED
		if(testbit(main_request, MAIN_REQUEST_UPDATE_LED)) {
			clrbit(main_request, MAIN_REQUEST_UPDATE_LED);
#ifdef CONFIG_LED_STATUS_port
			led_UpdateStatus(&led_status);
#endif
			for (int i = 0; i < LED_MAX_COUNT; i++) {
				led_UpdateStatus(led_sw_all[i]);
			}
		}
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
		// Set MUX and start ADC conversion
		if (testbit(main_request, MAIN_REQUEST_ADC_READY)) {
			uint16_t v = adc_ReadInput();
	#ifdef CONFIG_UPS_INSTALLED
			float volt;
	#endif
			switch(adc_CurrentInput()) {
				case CONFIG_ADC_MUX_PiState:
	#ifndef	CONFIG_PIDAEMON_LOGIC_LEVEL_CHECK
					*(pipo_status.daemon) = pidaemon_CheckDaemonState(v);
	#endif
					counter_Update(&adc_maesurement_ready, 1);
					break;
				case CONFIG_ADC_MUX_V_In:
				    *(pipo_status.adc_V_In) = (uint16_t)((*(pipo_setup.calibration_vin) / 10000.0) * lpfilter_Filter(&lpf_voltage_in, v) * 2500 / 4096);
					break;
	#ifdef CONFIG_UPS_INSTALLED
				case CONFIG_ADC_MUX_V_Bat:
					*(pipo_status.adc_V_Bat) = (uint16_t)((*(pipo_setup.calibration_vbat) / 10000.0) * lpfilter_Filter(&lpf_voltage_battery, v) * 2500 / 4096);
					break;
				case CONFIG_ADC_MUX_BAT_Temp:
					volt = lpfilter_Filter(&lpf_temperature, v) * 2.5 / 4096;
					*(pipo_status.adc_BAT_Temp) = ups_NTCTermistorToKelvin(volt);
					// incorrect temperature readings can cause an overheating alarm
					// when the charging is disabled, the temperature measurement is not available
					if (ups.charge_disabled) {
						counter_Reset(&adc_maesurement_ready);
						*(pipo_status.adc_BAT_Temp) = 0;
					}
					break;
	#endif
			}
			main_ADCInitConversion(adc_NextInput());
		}
#endif
#ifdef	CONFIG_PIDAEMON_LOGIC_LEVEL_CHECK
		*(pipo_status.daemon) = pidaemon_CheckDaemonState((CONFIG_PIDAEMON_STATE_PORT.IN >> CONFIG_PIDAEMON_STATE_PIN) & 0x01);
#endif
		// Power supply outputs, check the change request
		if (output_request[0]) {
			for (int i = 0; i < POWEROUT_MAX_COUNT; i++) {
				// skip Raspberry Pi supply output
				if (i == POWEROUT_OUTPUT_Pi) continue;
				if (testbit(output_request[0], i)) {
					if (powerout_out_all[i]->presence) {
						if (testbit(output_request[1], i)) {
							powerout_SetOn(powerout_out_all[i]);
						} else {
							powerout_SetOff(powerout_out_all[i]);
						}
					}
					clrbit(output_request[0], i);
				}
			}
		}		
		// Power supply outputs, check the current state
		*pipo_status.output = main_CheckOutput(powerout_out_all);			
#ifdef CONFIG_UPS_INSTALLED 
		// Check UPS state
		*pipo_status.ups = ups_CheckStatus(&ups);
#endif
#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
		// Read INA219, one request per loop
		switch(ina219_request) {
			uint16_t ina219_result;
			
			result = true;
			case INA219_REQUEST_VOLTAGE:
				result = ina219_GetRawBusVoltage(&ina219, &ina219_result);
				if (result && ina219.ready) pipo_status.ina219->voltage = ina219_result;
				break;
			case INA219_REQUEST_CURRENT:
				result = ina219_GetRawCurrent(&ina219, (int16_t *)(&ina219_result));
				if (result && ina219.ready) pipo_status.ina219->current = ina219_result;
				break;
		}
		if (!result) setbit(*(pipo_status.errors), MAIN_ERROR_INA219);
		ina219_request++;
		if (ina219_request > INA219_REQUEST_COUNT) ina219_request = 0; 
#endif
#ifdef CONFIG_UPS_INSTALLED
		// Action on the main power loss
		// adc_V_In LSB = 0.01V, adc_V_Bat LSB = 0.01V
		if (counter_Check(&adc_maesurement_ready)) {
			uint16_t cut_level = trunc(*(pipo_setup.ups_cut_level_mv) / 10);
			if (*pipo_status.adc_V_In < *(pipo_setup.power_loss_level_mv)) {
				if (after_power_loss_s.pause) counter_Reset(&after_power_loss_s);
				if ((*(pipo_setup.ups_behavior) == UPS_SHUT_IMMEDIATELY) || ((*(pipo_setup.ups_behavior) == UPS_SHUT_DELAYED) && 
												   (counter_Check(&after_power_loss_s))) || (*pipo_status.adc_V_Bat < cut_level)) {
					button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_OFF;
					ups_OnBattery(&ups, true);
				}
			} else {
				if (!(after_power_loss_s.pause)) {
					counter_Reset(&after_power_loss_s);
					counter_Pause(&after_power_loss_s);
				}
				ups_OnBattery(&ups, false);
			}
		}
#endif
#ifdef CONFIG_UPS_INSTALLED
		// Check battery overheat, temperature LSB = 0.01K
		if (counter_Check(&adc_maesurement_ready)) {
			if (*pipo_status.adc_BAT_Temp > *(pipo_setup.battery_overheat)) {
				setbit(*(pipo_status.errors), MAIN_ERROR_BATTERY_OVERHEAT);
				button_request[KEYBOARD_KEY_PWR] = BUTTON_STATE_OFF;
			} else {
				if (testbit(*(pipo_setup.error_recoverable), MAIN_ERROR_BATTERY_OVERHEAT)) clrbit(*(pipo_status.errors), MAIN_ERROR_BATTERY_OVERHEAT); 
			}
		}
#endif
		// Check for errors
		if  (counter_Check(&daemon_timeout_s)) {
			setbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_DAEMON);
		} else {
			if (testbit(*(pipo_setup.error_recoverable), MAIN_ERROR_TIMEOUT_DAEMON)) clrbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_DAEMON);
		}
		if (*(pipo_status.errors) != last_error) {
			pidaemon_SetRequest();
			last_error = *(pipo_status.errors);
			if (last_error) {
				led_SetLed(&led_status, true, true);
			} else {
				led_SetLed(&led_status, false, false);
			}
		}
		// Save new setup
		if (testbit(main_request, MAIN_REQUEST_SAVE_SETUP)) {
			clrbit(main_request, MAIN_REQUEST_SAVE_SETUP);
			main_SaveSetup();
			main_ReadSetup();
		}
	}
}

// Timers
ISR(RTC_OVF_vect) 
{
	setbit(main_request, MAIN_REQUEST_UPDATE_LED);
	setbit(main_request, MAIN_REQUEST_SCAN_KEYBOARD);
	counter_Update(&adc_conversion_timeout_ms, MAIN_TIMER_KEBOARD_MS);
	counter_Update(&timer_rtc_ms, MAIN_TIMER_KEBOARD_MS);
	if (counter_Check(&adc_conversion_timeout_ms)) setbit(*(pipo_status.errors), MAIN_ERROR_TIMEOUT_ADC);
	if (counter_Check(&timer_rtc_ms)) {
		counter_Reset(&timer_rtc_ms);
		counter_Update(&daemon_timeout_s, 1);
#ifdef CONFIG_UPS_INSTALLED
		counter_Update(&after_power_loss_s, 1);
#endif
		counter_Update(&delay_on_boot_s, 1);
		counter_Update(&delay_on_down_s, 1);
		counter_Update(&delay_on_pioff_s, 1);
	}
}

// ADC conversion complete
ISR(ADCA_CH0_vect) 
{
	setbit(main_request, MAIN_REQUEST_ADC_READY);
}

// TWIC Slave Interrupt vector.
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

#ifdef CONFIG_I2C_MEASUREMENT_INSTALLED
// TWIC Master Interrupt vector. 
ISR(TWIE_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}
#endif

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
				counter_Reset(&daemon_timeout_s);
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
				counter_Reset(&daemon_timeout_s);
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
#ifdef CONFIG_UPS_INSTALLED
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
#endif
		}
	}
	// Write buffer
	if (twiSlave.bytesReceived == (OUTPUT_BUFFER_SIZE - 1)) {
		switch(twiSlave.receivedData[0]) {
			case DAEMON_REQUEST_WRITE_SETUP:
				// DAEMON_REQUEST_CONFIG <list_of_values>
				main_CopyFromTWIBuffer(twiSlave.receivedData, twi_read_buffer);
				setbit(main_request, MAIN_REQUEST_SAVE_SETUP);
				break;
		}
	}
}

void main_MapButtonsState(void)
{
	*(pipo_status.buttons_state) = 0;
	for (int i = 0; i < KEY_MAX_COUNT_HALF; i++) {
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
	*(pipo_setup.on_boot_output) = eeprom_read_byte(&main_on_boot_output);
	*(pipo_setup.on_boot_delay_s) = eeprom_read_word(&main_on_boot_delay_s);
	*(pipo_setup.on_down_delay_s) = eeprom_read_word(&main_on_down_delay_s);
	*(pipo_setup.on_pioff_output) = eeprom_read_byte(&main_on_pioff_output);
	*(pipo_setup.on_pioff_delay_s) = eeprom_read_word(&main_on_pioff_delay_s);
	*(pipo_setup.daemon_timeout_s) = eeprom_read_word(&main_daemon_timeout_s);
	*(pipo_setup.error_recoverable) = eeprom_read_byte(&main_error_recoverable);
	*(pipo_setup.ups_behavior) = eeprom_read_byte(&ups_behavior);
	*(pipo_setup.ups_shut_delay_s) = eeprom_read_word(&ups_delay_s);
	*(pipo_setup.ups_cut_level_mv) = eeprom_read_word(&ups_cut_level_mv);
	*(pipo_setup.ups_min_charge_time_s) = eeprom_read_word(&ups_min_charge_time_s);
	*(pipo_setup.battery_overheat) = eeprom_read_word(&battery_overheat_01k);
	*(pipo_setup.power_loss_level_mv) = eeprom_read_word(&main_power_loss_mv);
}

void main_SaveSetup(void)
{
	eeprom_write_word(&adc_vin_cal, (twi_read_buffer[1] << 8) | twi_read_buffer[0]);
	eeprom_write_word(&adc_vbat_cal, (twi_read_buffer[3] << 8) | twi_read_buffer[2]);
	eeprom_write_byte(&main_on_boot_output, twi_read_buffer[4]);
	eeprom_write_word(&main_on_boot_delay_s, (twi_read_buffer[6] << 8) | twi_read_buffer[5]);
	eeprom_write_word(&main_on_down_delay_s, (twi_read_buffer[8] << 8) | twi_read_buffer[7]);
	eeprom_write_byte(&main_on_pioff_output, twi_read_buffer[9]);
	eeprom_write_word(&main_on_pioff_delay_s, (twi_read_buffer[11] << 8) | twi_read_buffer[10]);
	eeprom_write_word(&main_daemon_timeout_s, (twi_read_buffer[13] << 8) | twi_read_buffer[12]);
	eeprom_write_byte(&main_error_recoverable, twi_read_buffer[14]);
	eeprom_write_byte(&ups_behavior, twi_read_buffer[15]);
	eeprom_write_word(&ups_delay_s, (twi_read_buffer[17] << 8) | twi_read_buffer[16]);
	eeprom_write_word(&ups_cut_level_mv, (twi_read_buffer[19] << 8) | twi_read_buffer[18]);
	eeprom_write_word(&ups_min_charge_time_s, (twi_read_buffer[21] << 8) | twi_read_buffer[20]);
	eeprom_write_word(&battery_overheat_01k, (twi_read_buffer[23] << 8) | twi_read_buffer[22]);
	eeprom_write_word(&main_power_loss_mv, (twi_read_buffer[25] << 8) | twi_read_buffer[24]);
}

void main_SendSetup(register8_t *dest)
{
	 main_CopyToTWIBuffer(pipo_setup_buffer, dest);
}

void main_ClearButtonState()
{
	for (int i = 0; i < KEY_MAX_COUNT; i++) {
		pipo_status.buttons->state[i] = BUTTON_STATE_OFF;
	}
}

void main_ClearButtonRequest()
{
	for (int i = 0; i < KEY_MAX_COUNT; i++) {
		button_request[i] = BUTTON_STATE_IDLE;
	}
}

void main_NewButtonState(uint8_t button, uint8_t state)
{
	pipo_status.buttons->state[button] = state;
	pidaemon_SetRequest();
}

void main_PowerReduction(void) 
{
	// Stop unused peripherals
	PR.PRPA = PR_AC_bm;
#ifdef PR_PRPB
	PR.PRPB = PR_DAC_bm;
#endif
#ifdef PR_PRPC
	PR.PRPC = PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm;
	#ifdef PR_USART1_bm
		PR.PRPC |= PR_USART1_bm;
	#endif
#endif
#ifdef PR_PRPD
	PR.PRPD = PR_USART0_bm | PR_SPI_bm | PR_HIRES_bm;
	#ifdef PR_USART1_bm
		PR.PRPD |= PR_USART1_bm;
	#endif
	#ifdef PR_USB_bm
		PR.PRPD |= PR_USB_bm;
	#endif
#endif
#ifdef PR_PRPE
	PR.PRPE = PR_USART0_bm | PR_HIRES_bm;
#endif
}

void main_ADCInitConversion(uint8_t mux)
{
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	adc_SetMux(mux);
	_delay_us(50);
	clrbit(main_request, MAIN_REQUEST_ADC_READY);
	adc_StartConversion();
	counter_Reset(&adc_conversion_timeout_ms);
#endif
}

void main_LedInit(void)
{
	uint8_t counter = 0;
	led_Init(&led_status, main_LedStatusOff, main_LedStatusOn);
	led_SetLedTimers(&led_status, 10, 5);
#ifdef CONFIG_LED_STATUS_port
	setbit(CONFIG_LED_STATUS_port.DIR, CONFIG_LED_STATUS_pin);
	led_SetPresent(&led_status);
#endif
	led_Init(&led_pwr, main_LedPwrOff, main_LedPwrOn);	
	led_sw_all[counter++] = &led_pwr;
#ifdef CONFIG_LED_PWR_port
	led_SetLedTimers(&led_pwr, 20, 10);
	led_SetLedBlinkInverted(&led_pwr, true);
	setbit(CONFIG_LED_PWR_port.DIR, CONFIG_LED_PWR_pin);
	led_SetPresent(&led_pwr);
#endif
	led_Init(&led_sw1, main_LedSw1Off, main_LedSw1On);
	led_sw_all[counter++] = &led_sw1;
#ifdef CONFIG_LED_SW1_port
	led_SetLedTimers(&led_sw1, 20, 10);
	setbit(CONFIG_LED_SW1_port.DIR, CONFIG_LED_SW1_pin);
	led_SetPresent(&led_sw1);
#endif
	led_Init(&led_sw2, main_LedSw2Off, main_LedSw2On);
	led_sw_all[counter++] = &led_sw2;
#ifdef CONFIG_LED_SW2_port
	led_SetLedTimers(&led_sw2, 20, 10);
	setbit(CONFIG_LED_SW2_port.DIR, CONFIG_LED_SW2_pin);
	led_SetPresent(&led_sw2);
#endif
	led_Init(&led_sw3, main_LedSw3Off, main_LedSw3On);
	led_sw_all[counter++] = &led_sw3;
#ifdef CONFIG_LED_SW3_port
	led_SetLedTimers(&led_sw3, 20, 10);
	setbit(CONFIG_LED_SW3_port.DIR, CONFIG_LED_SW3_pin);
	led_SetPresent(&led_sw3);
#endif
	led_Init(&led_sw4, main_LedSw4Off, main_LedSw4On);
	led_sw_all[counter++] = &led_sw4;
#ifdef CONFIG_LED_SW4_port
	led_SetLedTimers(&led_sw4, 20, 10);
	setbit(CONFIG_LED_SW4_port.DIR, CONFIG_LED_SW4_pin);
	led_SetPresent(&led_sw4);
#endif
	led_Init(&led_sw5, main_LedSw5Off, main_LedSw5On);
	led_sw_all[counter++] = &led_sw5;
#ifdef CONFIG_LED_SW5_port
	led_SetLedTimers(&led_sw5, 20, 10);
	setbit(CONFIG_LED_SW5_port.DIR, CONFIG_LED_SW5_pin);
	led_SetPresent(&led_sw5);
#endif
	led_Init(&led_sw6, main_LedSw6Off, main_LedSw6On);
	led_sw_all[counter++] = &led_sw6;
#ifdef CONFIG_LED_SW6_port
	led_SetLedTimers(&led_sw6, 20, 10);
	setbit(CONFIG_LED_SW6_port.DIR, CONFIG_LED_SW6_pin);
	led_SetPresent(&led_sw6);
#endif
	led_Init(&led_sw7, main_LedSw7Off, main_LedSw7On);
	led_sw_all[counter++] = &led_sw7;
#ifdef CONFIG_LED_SW7_port
	led_SetLedTimers(&led_sw7, 20, 10);
	setbit(CONFIG_LED_SW7_port.DIR, CONFIG_LED_SW7_pin);
	led_SetPresent(&led_sw7);
#endif
}

void main_LedPwrOn(void)
{
#ifdef CONFIG_LED_PWR_port
	setbit(CONFIG_LED_PWR_port.OUT, CONFIG_LED_PWR_pin);
#endif
}

void main_LedPwrOff(void)
{
#ifdef CONFIG_LED_PWR_port
	clrbit(CONFIG_LED_PWR_port.OUT, CONFIG_LED_PWR_pin);
#endif
}

void main_LedStatusOn(void)
{
#ifdef CONFIG_LED_STATUS_port
	setbit(CONFIG_LED_STATUS_port.OUT, CONFIG_LED_STATUS_pin);
#endif
}

void main_LedStatusOff(void)
{
#ifdef CONFIG_LED_STATUS_port
	clrbit(CONFIG_LED_STATUS_port.OUT, CONFIG_LED_STATUS_pin);
#endif
}

void main_LedSw1On(void)
{
#ifdef CONFIG_LED_SW1_port
	setbit(CONFIG_LED_SW1_port.OUT, CONFIG_LED_SW1_pin);
#endif
}

void main_LedSw1Off(void)
{
#ifdef CONFIG_LED_SW1_port
	clrbit(CONFIG_LED_SW1_port.OUT, CONFIG_LED_SW1_pin);
#endif
}

void main_LedSw2On(void)
{
#ifdef CONFIG_LED_SW2_port
	setbit(CONFIG_LED_SW2_port.OUT, CONFIG_LED_SW2_pin);
#endif
}

void main_LedSw2Off(void)
{
#ifdef CONFIG_LED_SW2_port
	clrbit(CONFIG_LED_SW2_port.OUT, CONFIG_LED_SW2_pin);
#endif
}

void main_LedSw3On(void)
{
#ifdef CONFIG_LED_SW3_port
	setbit(CONFIG_LED_SW3_port.OUT, CONFIG_LED_SW3_pin);
#endif
}

void main_LedSw3Off(void)
{
#ifdef CONFIG_LED_SW3_port
	clrbit(CONFIG_LED_SW3_port.OUT, CONFIG_LED_SW3_pin);
#endif
}

void main_LedSw4On(void)
{
#ifdef CONFIG_LED_SW4_port
	setbit(CONFIG_LED_SW4_port.OUT, CONFIG_LED_SW4_pin);
#endif
}

void main_LedSw4Off(void)
{
#ifdef CONFIG_LED_SW4_port
	clrbit(CONFIG_LED_SW4_port.OUT, CONFIG_LED_SW4_pin);
#endif
}

void main_LedSw5On(void)
{
#ifdef CONFIG_LED_SW5_port
	setbit(CONFIG_LED_SW5_port.OUT, CONFIG_LED_SW5_pin);
#endif
}

void main_LedSw5Off(void)
{
#ifdef CONFIG_LED_SW5_port
	clrbit(CONFIG_LED_SW5_port.OUT, CONFIG_LED_SW5_pin);
#endif
}

void main_LedSw6On(void)
{
#ifdef CONFIG_LED_SW6_port
	setbit(CONFIG_LED_SW6_port.OUT, CONFIG_LED_SW6_pin);
#endif
}

void main_LedSw6Off(void)
{
#ifdef CONFIG_LED_SW6_port
	clrbit(CONFIG_LED_SW6_port.OUT, CONFIG_LED_SW6_pin);
#endif
}

void main_LedSw7On(void)
{
#ifdef CONFIG_LED_SW7_port
	setbit(CONFIG_LED_SW7_port.OUT, CONFIG_LED_SW7_pin);
#endif
}

void main_LedSw7Off(void)
{
#ifdef CONFIG_LED_SW7_port
	clrbit(CONFIG_LED_SW7_port.OUT, CONFIG_LED_SW7_pin);
#endif
}

void main_KeyboardInit(void)
{
	uint8_t counter = 0;
		
	key_Init(&key_pwr, main_ReadKeyPwr);
	key_sw_all[counter++] = &key_pwr;
#ifdef CONFIG_KEY_PWR_port
	// set pull-up resistor on button input
	*(&CONFIG_KEY_PWR_port.PIN0CTRL + CONFIG_KEY_PWR_pin) = PORT_OPC_PULLUP_gc;
	key_SetupKey(&key_pwr, bitmask(KEYBOARD_KeyEnabled) | bitmask(KEYBOARD_KeyMultiplyEnable));
	key_SetPresent(&key_pwr);
#endif
	key_Init(&key_sw1, main_ReadKeySw1);
	key_sw_all[counter++] = &key_sw1;
#ifdef CONFIG_KEY_SW1_port
	key_SetupKey(&key_sw1, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW1_port.PIN0CTRL + CONFIG_KEY_SW1_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw1);
#endif
	key_Init(&key_sw2, main_ReadKeySw2);
	key_sw_all[counter++] = &key_sw2;
#ifdef CONFIG_KEY_SW2_port
	key_SetupKey(&key_sw2, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW2_port.PIN0CTRL + CONFIG_KEY_SW2_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw2);
#endif
	key_Init(&key_sw3, main_ReadKeySw3);
	key_sw_all[counter++] = &key_sw3;
#ifdef CONFIG_KEY_SW3_port
	key_SetupKey(&key_sw3, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW3_port.PIN0CTRL + CONFIG_KEY_SW3_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw3);
#endif
	key_Init(&key_sw4, main_ReadKeySw4);
	key_sw_all[counter++] = &key_sw4;
#ifdef CONFIG_KEY_SW4_port
	key_SetupKey(&key_sw4, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW4_port.PIN0CTRL + CONFIG_KEY_SW4_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw4);
#endif
	key_Init(&key_sw5, main_ReadKeySw5);
	key_sw_all[counter++] = &key_sw5;
#ifdef CONFIG_KEY_SW5_port
	key_SetupKey(&key_sw5, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW5_port.PIN0CTRL + CONFIG_KEY_SW5_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw5);
#endif
	key_Init(&key_sw6, main_ReadKeySw6);
	key_sw_all[counter++] = &key_sw6;
#ifdef CONFIG_KEY_SW6_port
	key_SetupKey(&key_sw6, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW6_port.PIN0CTRL + CONFIG_KEY_SW6_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw6);
#endif
	key_Init(&key_sw7, main_ReadKeySw7);
	key_sw_all[counter++] = &key_sw7;
#ifdef CONFIG_KEY_SW7_port
	key_SetupKey(&key_sw7, bitmask(KEYBOARD_KeyEnabled));
	*(&CONFIG_KEY_SW7_port.PIN0CTRL + CONFIG_KEY_SW7_pin) = PORT_OPC_PULLUP_gc;
	key_SetPresent(&key_sw7);
#endif
}

uint8_t main_ReadKeyPwr(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_PWR_port
	if (!testbit(CONFIG_KEY_PWR_port.IN, CONFIG_KEY_PWR_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw1(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW1_port
	if (!testbit(CONFIG_KEY_SW1_port.IN, CONFIG_KEY_SW1_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw2(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW2_port
	if (!testbit(CONFIG_KEY_SW2_port.IN, CONFIG_KEY_SW2_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw3(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW3_port
	if (!testbit(CONFIG_KEY_SW3_port.IN, CONFIG_KEY_SW3_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw4(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW4_port
	if (!testbit(CONFIG_KEY_SW4_port.IN, CONFIG_KEY_SW4_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw5(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW5_port
	if (!testbit(CONFIG_KEY_SW5_port.IN, CONFIG_KEY_SW5_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw6(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW6_port
	if (!testbit(CONFIG_KEY_SW6_port.IN, CONFIG_KEY_SW6_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

uint8_t main_ReadKeySw7(void)
{
	uint8_t result = KEY_STATE_UNPRESSED;
#ifdef CONFIG_KEY_SW7_port
	if (!testbit(CONFIG_KEY_SW7_port.IN, CONFIG_KEY_SW7_pin)) result = KEY_STATE_PRESSED;
#endif
	return result;
}

void main_PoweroutInit(void)
{
	uint8_t counter = 0;
	
	powerout_Init(&powerout_outpi, main_SetPoweroutOutPi, main_ReadPoweroutOutPi);
	powerout_out_all[counter++] = &powerout_outpi;
#ifdef CONFIG_POWEROUT_Pi_port
	powerout_SetOff(&powerout_outpi);
	setbit(CONFIG_POWEROUT_Pi_port.DIR, CONFIG_POWEROUT_Pi_pin);
	powerout_SetPresent(&powerout_outpi);
#endif
	powerout_Init(&powerout_out1, main_SetPoweroutOut1, main_ReadPoweroutOut1);
	powerout_out_all[counter++] = &powerout_out1;
#ifdef CONFIG_POWEROUT_1_port
	powerout_SetOff(&powerout_out1);
	setbit(CONFIG_POWEROUT_1_port.DIR, CONFIG_POWEROUT_1_pin);
	powerout_SetPresent(&powerout_out1);
#endif
	powerout_Init(&powerout_out2, main_SetPoweroutOut2, main_ReadPoweroutOut2);
	powerout_out_all[counter++] = &powerout_out2;
#ifdef CONFIG_POWEROUT_2_port
	powerout_SetOff(&powerout_out2);
	setbit(CONFIG_POWEROUT_2_port.DIR, CONFIG_POWEROUT_2_pin);
	powerout_SetPresent(&powerout_out2);
#endif
	powerout_Init(&powerout_out3, main_SetPoweroutOut3, main_ReadPoweroutOut3);
	powerout_out_all[counter++] = &powerout_out3;
#ifdef CONFIG_POWEROUT_3_port
	powerout_SetOff(&powerout_out3);
	setbit(CONFIG_POWEROUT_3_port.DIR, CONFIG_POWEROUT_3_pin);
	powerout_SetPresent(&powerout_out3);
#endif
	powerout_Init(&powerout_out4, main_SetPoweroutOut4, main_ReadPoweroutOut4);
	powerout_out_all[counter++] = &powerout_out4;
#ifdef CONFIG_POWEROUT_4_port
	powerout_SetOff(&powerout_out4);
	setbit(CONFIG_POWEROUT_4_port.DIR, CONFIG_POWEROUT_4_pin);
	powerout_SetPresent(&powerout_out4);
#endif
	powerout_Init(&powerout_out5, main_SetPoweroutOut5, main_ReadPoweroutOut5);
	powerout_out_all[counter++] = &powerout_out5;
#ifdef CONFIG_POWEROUT_5_port
	powerout_SetOff(&powerout_out5);
	setbit(CONFIG_POWEROUT_5_port.DIR, CONFIG_POWEROUT_5_pin);
	powerout_SetPresent(&powerout_out5);
#endif
	powerout_Init(&powerout_out6, main_SetPoweroutOut6, main_ReadPoweroutOut6);
	powerout_out_all[counter++] = &powerout_out6;
#ifdef CONFIG_POWEROUT_6_port
	powerout_SetOff(&powerout_out6);
	setbit(CONFIG_POWEROUT_6_port.DIR, CONFIG_POWEROUT_6_pin);
	powerout_SetPresent(&powerout_out6);
#endif
	powerout_Init(&powerout_out7, main_SetPoweroutOut7, main_ReadPoweroutOut7);
	powerout_out_all[counter++] = &powerout_out7;
#ifdef CONFIG_POWEROUT_7_port
	powerout_SetOff(&powerout_out7);
	setbit(CONFIG_POWEROUT_7_port.DIR, CONFIG_POWEROUT_7_pin);
	powerout_SetPresent(&powerout_out7);
#endif
}

void main_SetPoweroutOutPi(uint8_t state)
{
#ifdef CONFIG_POWEROUT_Pi_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_Pi_port.OUT, CONFIG_POWEROUT_Pi_pin);
	else setbit(CONFIG_POWEROUT_Pi_port.OUT, CONFIG_POWEROUT_Pi_pin);
#endif
}

uint8_t main_ReadPoweroutOutPi(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_Pi_port
	if (!testbit(CONFIG_POWEROUT_Pi_port.OUT, CONFIG_POWEROUT_Pi_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut1(uint8_t state)
{
#ifdef CONFIG_POWEROUT_1_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_1_port.OUT, CONFIG_POWEROUT_1_pin);
	else setbit(CONFIG_POWEROUT_1_port.OUT, CONFIG_POWEROUT_1_pin);
#endif
}

uint8_t main_ReadPoweroutOut1(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_1_port
	if (!testbit(CONFIG_POWEROUT_1_port.OUT, CONFIG_POWEROUT_1_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut2(uint8_t state)
{
#ifdef CONFIG_POWEROUT_2_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_2_port.OUT, CONFIG_POWEROUT_2_pin);
	else setbit(CONFIG_POWEROUT_2_port.OUT, CONFIG_POWEROUT_2_pin);
#endif
}

uint8_t main_ReadPoweroutOut2(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_2_port
	if (!testbit(CONFIG_POWEROUT_2_port.OUT, CONFIG_POWEROUT_2_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut3(uint8_t state)
{
#ifdef CONFIG_POWEROUT_3_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_3_port.OUT, CONFIG_POWEROUT_3_pin);
	else setbit(CONFIG_POWEROUT_3_port.OUT, CONFIG_POWEROUT_3_pin);
#endif
}

uint8_t main_ReadPoweroutOut3(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_3_port
	if (!testbit(CONFIG_POWEROUT_3_port.OUT, CONFIG_POWEROUT_3_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut4(uint8_t state)
{
#ifdef CONFIG_POWEROUT_4_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_4_port.OUT, CONFIG_POWEROUT_4_pin);
	else setbit(CONFIG_POWEROUT_4_port.OUT, CONFIG_POWEROUT_4_pin);
#endif
}

uint8_t main_ReadPoweroutOut4(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_4_port
	if (!testbit(CONFIG_POWEROUT_4_port.OUT, CONFIG_POWEROUT_4_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut5(uint8_t state)
{
#ifdef CONFIG_POWEROUT_5_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_5_port.OUT, CONFIG_POWEROUT_5_pin);
	else setbit(CONFIG_POWEROUT_5_port.OUT, CONFIG_POWEROUT_5_pin);
#endif
}

uint8_t main_ReadPoweroutOut5(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_5_port
	if (!testbit(CONFIG_POWEROUT_5_port.OUT, CONFIG_POWEROUT_5_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut6(uint8_t state)
{
#ifdef CONFIG_POWEROUT_6_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_6_port.OUT, CONFIG_POWEROUT_6_pin);
	else setbit(CONFIG_POWEROUT_6_port.OUT, CONFIG_POWEROUT_6_pin);
#endif
}

uint8_t main_ReadPoweroutOut6(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_6_port
	if (!testbit(CONFIG_POWEROUT_6_port.OUT, CONFIG_POWEROUT_6_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

void main_SetPoweroutOut7(uint8_t state)
{
#ifdef CONFIG_POWEROUT_7_port
	if (state == POWEROUT_STATE_ON) clrbit(CONFIG_POWEROUT_7_port.OUT, CONFIG_POWEROUT_7_pin);
	else setbit(CONFIG_POWEROUT_7_port.OUT, CONFIG_POWEROUT_7_pin);
#endif
}

uint8_t main_ReadPoweroutOut7(void)
{
	uint8_t result = POWEROUT_STATE_OFF;
#ifdef CONFIG_POWEROUT_7_port
	if (!testbit(CONFIG_POWEROUT_7_port.OUT, CONFIG_POWEROUT_7_pin)) result = POWEROUT_STATE_ON;
#endif
	return result;
}

uint8_t main_CheckOutput(OB_POWEROUT_t **all)
{
	uint8_t result = 0b00000000;
	
	for (int i = POWEROUT_OUTPUT_Pi; i < POWEROUT_MAX_COUNT; i++) {
		if (all[i]->presence) {
			if (powerout_CheckOutput(all[i])) setbit(result, i);
		}
	}
	return result;
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

void main_MapStatusToBuffer()
{
	pipo_status.buttons = (PIPO_Btn_t *)(pipo_buttons_buffer);
	
	pipo_status.buttons_state = (uint32_t *)(pipo_status_buffer);
	pipo_status.daemon = (uint8_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF);
	pipo_status.adc_V_In = (uint16_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 1);
	pipo_status.adc_V_Bat = (uint16_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 3);
	pipo_status.adc_BAT_Temp = (uint16_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 5);
	pipo_status.output = (uint8_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 7);
	pipo_status.ups = (uint8_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 8);
	pipo_status.errors = (uint8_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 9);
	pipo_status.ina219 = (PIPO_INA_t *)(pipo_status_buffer + KEY_MAX_COUNT_HALF + 10);
}

void main_MapSetupToBuffer()
{
	pipo_setup.calibration_vin = (uint16_t *)(pipo_setup_buffer);
	pipo_setup.calibration_vbat = (uint16_t *)(pipo_setup_buffer + 2);
	pipo_setup.on_boot_output = (uint8_t *)(pipo_setup_buffer + 4);
	pipo_setup.on_boot_delay_s = (uint16_t *)(pipo_setup_buffer + 5);
	pipo_setup.on_down_delay_s = (uint16_t *)(pipo_setup_buffer + 7);
	pipo_setup.on_pioff_output = (uint8_t *)(pipo_setup_buffer + 9);
	pipo_setup.on_pioff_delay_s = (uint16_t *)(pipo_setup_buffer + 10);
	pipo_setup.daemon_timeout_s = (uint16_t *)(pipo_setup_buffer + 12);
	pipo_setup.error_recoverable = (uint8_t *)(pipo_setup_buffer + 14);
	pipo_setup.ups_behavior = (uint8_t *)(pipo_setup_buffer + 15);
	pipo_setup.ups_shut_delay_s = (uint16_t *)(pipo_setup_buffer + 16);
	pipo_setup.ups_cut_level_mv = (uint16_t *)(pipo_setup_buffer + 18);
	pipo_setup.ups_min_charge_time_s = (uint16_t *)(pipo_setup_buffer + 20);
	pipo_setup.battery_overheat = (uint16_t *)(pipo_setup_buffer + 22);
	pipo_setup.power_loss_level_mv = (uint16_t *)(pipo_setup_buffer + 24);
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
  