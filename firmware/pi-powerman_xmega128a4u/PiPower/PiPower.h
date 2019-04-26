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
#include "keyboard.h"
#include "bitlib.h"
#include "led.h"
#include "twi_slave_driver.h"
#include "twi_master_driver.h"
#include "delays.h"
#include "powerout.h"
#include "ups.h"
#include "ina219.h"
#include "lowpass_filter.h"


#define OUTPUT_BUFFER_SIZE				31			// I2C can send 32 bytes in one session, one byte is reserved for the transmission status  

#define MAIN_TIMER_KEBOARD_MS			50			// RTC_OVF_vect interval
#define MAIN_TIMER_RTC_TO_1S			1000
#define MAIN_TIMER_ADCCONVERSION_MS		1000
#define MAIN_TIMEOUT_MAINCLOCK_MS		200

// TWI
#define TWI_SLAVE_ADDRESS				0x55
#define CPU_SPEED						32000000
#define TWI_BAUDATE						100000
#define TWI_BAUDSETTING					TWI_BAUD(CPU_SPEED, TWI_BAUDATE)

// Delay (uint16_t)
#define DELAY_AFTER_SHUTDOWN_MS			((uint16_t)(20000))
#define DELAY_AFTER_RASPI_POWEROFF_MS	((uint16_t)(4000))

#define MAX_KEYBOARD_KEY				8

enum {
	MAIN_ERROR_MAIN_CLOCK,
	MAIN_ERROR_RTC_CLOCK,
	MAIN_ERROR_INA219,
	MAIN_ERROR_ADC_TIMEOUT,
	MAIN_ERROR_PIPO_DAEMON,
	MAIN_ERROR_DAEMON_TIMEOUT,
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
	POWERPORT_RASPI		= POWEROUT_OUTPUT_1,
	POWERPORT_USB1		= POWEROUT_OUTPUT_2,
	POWERPORT_USB2		= POWEROUT_OUTPUT_3,
	POWERPORT_HDD		= POWEROUT_OUTPUT_4
};

enum {
	DAEMON_REQUEST_BUTTON			= 0x47,
	DAEMON_REQUEST_PRESS			= 0x48,
	DAEMON_REQUEST_OUTPUT			= 0x49,
	DAEMON_REQUEST_KEEPALIVE		= 0x51,
	DAEMON_REQUEST_WRITE_SETUP		= 0x52,
	DAEMON_REQUEST_ERROR			= 0x53,
	DAEMON_REQUEST_CLEAR_ERROR		= 0x54,
	DAEMON_REQUEST_SYSTEM_ERROR		= 0x55,
	DAEMON_REQUEST_WAITING			= 0x58,
	DAEMON_REQUEST_READ_STATUS		= 0x59,
	DAEMON_REQUEST_READ_INFO1		= 0x61,
	DAEMON_REQUEST_READ_INFO2		= 0x62,
	DAEMON_REQUEST_DISABLE_CHARGE	= 0x63,
	DAEMON_REQUEST_ENABLE_CHARGE	= 0x64,
	DAEMON_REQUEST_DISCHARGE_START	= 0x65,
	DAEMON_REQUEST_DISCHARGE_STOP	= 0x66,
	DAEMON_REQUEST_READ_SETUP		= 0x67
};

#define INA219_REQUEST_COUNT		4
enum {
	INA219_REQUEST_BUS_VOLTAGE,
	INA219_REQUEST_SHUNT_VOLTAGE,
	INA219_REQUEST_CURRENT,
	INA219_REQUEST_POWER
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
	volatile uint8_t state[KEYBOARD_KEY_COUNT];
} PIPO_Btn_t;

typedef struct ina219_measurement {
	uint16_t bus_voltage;
	uint16_t shunt_voltage;
	uint16_t current;
	uint16_t power;
} PIPO_INA_t;

typedef struct PiPowerman_Status {
	PIPO_Btn_t *buttons;
	uint8_t *daemon_state;
	uint16_t *adc_V_In;
	uint16_t *adc_V_Bat;
	uint16_t *adc_BAT_Temp;
	uint8_t *output_state;
	uint8_t *ups_state;
	PIPO_INA_t *ina219;
	uint8_t *errors;
	uint8_t *request;
} PIPO_State_t;

typedef struct PiPowerman_Setup {
	uint16_t *calibration_vin;
	uint16_t *calibration_vbat;
	uint8_t *on_boot_output;
	uint16_t *on_boot_delay_s;
	uint16_t *daemon_timeout_s;
	uint8_t *error_recoverable;
	uint8_t *ups_behavior;
	uint16_t *ups_shut_delay_s;
	uint16_t *ups_cut_level_mv;
} PIPO_Setup_t; 
	
PIPO_State_t pipo_status;
PIPO_Setup_t pipo_setup;	
TWI_Slave_t twiSlave;		// TWI slave module
TWI_Master_t twiMaster;		// TWI master module
INA219_t ina219;			// INA219 chip
UPS_t ups;

volatile uint8_t button_request[KEYBOARD_KEY_COUNT];
volatile uint8_t output_request[2];

DELAY_t timer_rtc_ms;
DELAY_t delay_on_boot_s;
DELAY_t delay_after_raspi_poweroff_ms;
DELAY_t delay_after_shutdown_ms;
DELAY_t daemon_timeout_s;
DELAY_t after_power_loss_s;
DELAY_t adc_conversion_ms;
DELAY_t adc_maesurement_ready;


LPFu16_t lpf_voltage_in;
LPFu16_t lpf_voltage_battery;
LPFu16_t lpf_temperature;

const char DEVICE_INFO_1[OUTPUT_BUFFER_SIZE]; 
const char DEVICE_INFO_2[OUTPUT_BUFFER_SIZE];

uint8_t pipo_status_buffer[OUTPUT_BUFFER_SIZE];
uint8_t pipo_setup_buffer[OUTPUT_BUFFER_SIZE];
uint8_t twi_read_buffer[OUTPUT_BUFFER_SIZE];

inline static void main_MapStatusToBuffer();
inline static void main_MapStatusToBuffer()
{
	pipo_status.buttons = (PIPO_Btn_t *)(pipo_status_buffer);
	pipo_status.daemon_state = (uint8_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY);
	pipo_status.adc_V_In = (uint16_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 1);
	pipo_status.adc_V_Bat = (uint16_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 3);
	pipo_status.adc_BAT_Temp = (uint16_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 5);
	pipo_status.output_state = (uint8_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 7);
	pipo_status.ups_state = (uint8_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 8);
	pipo_status.ina219 = (PIPO_INA_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 9);
	pipo_status.errors = (uint8_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 17);
	pipo_status.request = (uint8_t *)(pipo_status_buffer + MAX_KEYBOARD_KEY + 18);
}

inline static void main_MapSetupToBuffer();
inline static void main_MapSetupToBuffer()
{
	pipo_setup.calibration_vin = (uint16_t *)(pipo_setup_buffer);
	pipo_setup.calibration_vbat = (uint16_t *)(pipo_setup_buffer + 2);
	pipo_setup.on_boot_output = (uint8_t *)(pipo_setup_buffer + 4);
	pipo_setup.on_boot_delay_s = (uint16_t *)(pipo_setup_buffer + 5);
	pipo_setup.daemon_timeout_s = (uint16_t *)(pipo_setup_buffer + 7);
	pipo_setup.error_recoverable = (uint8_t *)(pipo_setup_buffer + 9);
	pipo_setup.ups_behavior = (uint8_t *)(pipo_setup_buffer + 10);
	pipo_setup.ups_shut_delay_s = (uint16_t *)(pipo_setup_buffer + 11);
	pipo_setup.ups_cut_level_mv = (uint16_t *)(pipo_setup_buffer + 13);
}


#endif /* PIPOWER_H_ */