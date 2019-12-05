/*
 * PiPower.h
 *
 * Created: 2018-12-25 18:22:57
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#ifndef PIPOWER_H_
#define PIPOWER_H_

#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "PiConfig.h"
#include "keyboard.h"
#include "bitlib.h"
#include "led.h"
#include "twi_slave_driver.h"
#include "twi_master_driver.h"
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	#include "adc.h"
#endif
#include "ina219.h"
#include "counter.h"
#include "powerout.h"
#include "ups.h"
#include "lowpass_filter.h"



#define OUTPUT_BUFFER_SIZE				32			// I2C can send 32 bytes in one session, one byte is reserved for the transmission status  

#define MAIN_TIMER_KEBOARD_MS			50			// RTC_OVF_vect interval
#define MAIN_TIMER_RTC_TO_1S			1000
#define MAIN_TIMER_ADCCONVERSION_MS		1000
#define MAIN_TIMEOUT_MAINCLOCK_MS		200

// TWI
#define TWI_SLAVE_ADDRESS				0x55
#define TWI_CLK							100000
#define TWI_BAUDSETTING					TWI_BAUD(F_CPU, TWI_CLK)

#define ADC_SKIP_FIRST_MEASURE			(0x0fff)
#define LED_MAX_COUNT					(8)
#define POWEROUT_MAX_COUNT				(8)

enum {
	POWEROUT_OUTPUT_Pi,
	POWEROUT_OUTPUT_1,
	POWEROUT_OUTPUT_2,
	POWEROUT_OUTPUT_3,
	POWEROUT_OUTPUT_4,
	POWEROUT_OUTPUT_5,
	POWEROUT_OUTPUT_6,
	POWEROUT_OUTPUT_7
};

#define KEY_MAX_COUNT					(8)
#define KEY_MAX_COUNT_HALF				(4)
enum {
	KEYBOARD_KEY_PWR,
	KEYBOARD_KEY_SW1,
	KEYBOARD_KEY_SW2,
	KEYBOARD_KEY_SW3,
	KEYBOARD_KEY_SW4,
	KEYBOARD_KEY_SW5,
	KEYBOARD_KEY_SW6,
	KEYBOARD_KEY_SW7
};

enum {
	MAIN_ERROR_MAIN_CLOCK,
	MAIN_ERROR_RTC_CLOCK,
	MAIN_ERROR_INA219,
	MAIN_ERROR_TIMEOUT_ADC,
	MAIN_ERROR_DAEMON,
	MAIN_ERROR_TIMEOUT_DAEMON,
	MAIN_ERROR_SYSTEM,
	MAIN_ERROR_BATTERY_OVERHEAT
};

enum {
	MAIN_REQUEST_UPDATE_LED,
	MAIN_REQUEST_SCAN_KEYBOARD,
	MAIN_REQUEST_ADC_READY,
	MAIN_REQUEST_SAVE_SETUP
};

enum {
	DAEMON_REQUEST_READ_INFO0		= 0x41,
	DAEMON_REQUEST_READ_INFO1		= 0x42,
	DAEMON_REQUEST_READ_INFO2		= 0x43,
	DAEMON_REQUEST_READ_INFO3		= 0x44,
	DAEMON_REQUEST_READ_STATUS		= 0x45,
	DAEMON_REQUEST_READ_SETUP		= 0x46,
	DAEMON_REQUEST_WRITE_SETUP		= 0x47,
	DAEMON_REQUEST_BUTTON			= 0x48,
	DAEMON_REQUEST_PRESS			= 0x49,
	DAEMON_REQUEST_OUTPUT			= 0x4a,
	DAEMON_REQUEST_KEEPALIVE		= 0x4b,
	DAEMON_REQUEST_ERROR			= 0x4c,
	DAEMON_REQUEST_CLEAR_ERROR		= 0x4d,
	DAEMON_REQUEST_SYSTEM_ERROR		= 0x4e,
	DAEMON_REQUEST_DISABLE_CHARGE	= 0x4f,
	DAEMON_REQUEST_ENABLE_CHARGE	= 0x50,
	DAEMON_REQUEST_DISCHARGE_START	= 0x51,
	DAEMON_REQUEST_DISCHARGE_STOP	= 0x52
};

#define INA219_REQUEST_COUNT		2
enum {
	INA219_REQUEST_VOLTAGE,
	INA219_REQUEST_CURRENT
};

enum {
	BUTTON_STATE_IDLE,
	BUTTON_STATE_ON,
	BUTTON_STATE_TURN_OFF_01,
	BUTTON_STATE_TURN_OFF_02,
	BUTTON_STATE_TURN_OFF_03,
	BUTTON_STATE_TURN_OFF_04,
	BUTTON_STATE_TURN_OFF_05,
	BUTTON_STATE_OFF,
	BUTTON_STATE_TURN_ON_01,
	BUTTON_STATE_TURN_ON_02,
	BUTTON_STATE_TURN_ON_03,
	BUTTON_STATE_TURN_ON_04,
	BUTTON_STATE_TURN_ON_05,
	BUTTON_STATE_PRESS,
	BUTTON_STATE_DISABLE
};

typedef struct PiPowerman_Buttons {
	volatile uint8_t state[KEY_MAX_COUNT];
} PIPO_Btn_t;

typedef struct ina219_measurement {
	uint16_t voltage;
	uint16_t current;
} PIPO_INA_t;

typedef struct PiPowerman_Status {
	PIPO_Btn_t *buttons;
	volatile uint32_t *buttons_state;
	uint8_t *daemon;
	uint16_t *adc_V_In;
	uint16_t *adc_V_Bat;
	uint16_t *adc_BAT_Temp;
	uint8_t *output;
	uint8_t *ups;
	uint8_t *errors;
	PIPO_INA_t *ina219;
} PIPO_State_t;

typedef struct PiPowerman_Setup {
	uint16_t *calibration_vin;
	uint16_t *calibration_vbat;
	uint8_t *on_boot_output;
	uint16_t *on_boot_delay_s;
	uint16_t *on_down_delay_s;
	uint8_t *on_pioff_output;
	uint16_t *on_pioff_delay_s;
	uint16_t *daemon_timeout_s;
	uint8_t *error_recoverable;
	uint8_t *ups_behavior;
	uint16_t *ups_shut_delay_s;
	uint16_t *ups_cut_level_mv;
	uint16_t *ups_min_charge_time_s;
	uint16_t *battery_overheat;
	uint16_t *power_loss_level_mv;
} PIPO_Setup_t; 
	
volatile uint8_t main_request;
PIPO_State_t pipo_status;
PIPO_Setup_t pipo_setup;	
TWI_Slave_t twiSlave;		// TWI slave module
TWI_Master_t twiMaster;		// TWI master module
INA219_t ina219;			// INA219 chip
#ifdef CONFIG_UPS_INSTALLED
	UPS_t ups;
#endif

volatile uint8_t button_request[KEY_MAX_COUNT];
volatile uint8_t output_request[2];

const char DEVICE_INFO_0[OUTPUT_BUFFER_SIZE];
const char DEVICE_INFO_1[OUTPUT_BUFFER_SIZE]; 
const char DEVICE_INFO_2[OUTPUT_BUFFER_SIZE];
const char DEVICE_INFO_3[OUTPUT_BUFFER_SIZE];

uint8_t pipo_status_buffer[OUTPUT_BUFFER_SIZE];
uint8_t pipo_buttons_buffer[KEY_MAX_COUNT];
uint8_t pipo_setup_buffer[OUTPUT_BUFFER_SIZE];
uint8_t twi_read_buffer[OUTPUT_BUFFER_SIZE];


#endif /* PIPOWER_H_ */