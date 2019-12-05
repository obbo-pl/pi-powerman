/*
 * PiConfig_custom.h
 *
 * Created: 2019-10-25 21:12:57
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef PICONFIG_H_
#define PICONFIG_H_



/*
	UPS module definition
*/
#define CONFIG_UPS_INSTALLED 
#define CONFIG_UPS_TYPE						"LTC4011 8*NiMh 1900mAh"
#define CONFIG_BATTERY_DISCHARGE_PORT		PORTA
#define CONFIG_BATTERY_DISCHARGE_PIN		3
#define CONFIG_BATTARY_TEMPERATURE_PORT		PORTA
#define CONFIG_BATTERY_TEMPERATURE_PIN		7

/*
	Measurement definition
*/
#define CONFIG_I2C_MEASUREMENT_INSTALLED
#define CONFIG_I2C_PORT						TWIE

#define CONFIG_ADC_MEASUREMENT_INSTALLED
#define CONFIG_ADC_REFERENCE_PORT			ADC_REFSEL_AREFA_gc
#define CONFIG_ADC_REFERENCE_EXT_MV			(2500)
//#define CONFIG_ADC_REFERENCE_VCC_MV			(3300)
#define CONFIG_ADC_MUX_PiState				ADC_CH_MUXPOS_PIN5_gc
#define CONFIG_ADC_MUX_V_In					ADC_CH_MUXPOS_PIN3_gc
#define CONFIG_ADC_MUX_V_Bat				ADC_CH_MUXPOS_PIN4_gc
#define CONFIG_ADC_MUX_BAT_Temp				ADC_CH_MUXPOS_PIN7_gc


/*
	Power output
*/
#define CONFIG_POWEROUT_Pi_port				PORTB
#define CONFIG_POWEROUT_Pi_pin				0
#define CONFIG_POWEROUT_1_port				PORTB
#define CONFIG_POWEROUT_1_pin				1
#define CONFIG_POWEROUT_2_port				PORTB
#define CONFIG_POWEROUT_2_pin				2
#define CONFIG_POWEROUT_3_port				PORTB
#define CONFIG_POWEROUT_3_pin				3
#define CONFIG_POWEROUT_4_port				PORTB
#define CONFIG_POWEROUT_4_pin				4
#define CONFIG_POWEROUT_5_port				PORTB
#define CONFIG_POWEROUT_5_pin				5
#define CONFIG_POWEROUT_6_port				PORTB
#define CONFIG_POWEROUT_6_pin				6
#define CONFIG_POWEROUT_7_port				PORTB
#define CONFIG_POWEROUT_7_pin				7

/* 
	Key config
*/
#define CONFIG_KEY_PWR_pin					0
#define CONFIG_KEY_PWR_port					PORTD
#define CONFIG_KEY_SW1_pin					1
#define CONFIG_KEY_SW1_port					PORTD
#define CONFIG_KEY_SW2_pin					2
#define CONFIG_KEY_SW2_port					PORTD
#define CONFIG_KEY_SW3_pin					3
#define CONFIG_KEY_SW3_port					PORTD
#define CONFIG_KEY_SW4_pin					4
#define CONFIG_KEY_SW4_port					PORTD
#define CONFIG_KEY_SW5_pin					5
#define CONFIG_KEY_SW5_port					PORTD
#define CONFIG_KEY_SW6_pin					6
#define CONFIG_KEY_SW6_port					PORTD
#define CONFIG_KEY_SW7_pin					7
#define CONFIG_KEY_SW7_port					PORTD

/*
	LED
*/
#define CONFIG_LED_STATUS_pin				0
#define CONFIG_LED_STATUS_port				PORTC
#define CONFIG_LED_PWR_pin					1
#define CONFIG_LED_PWR_port					PORTC
#define CONFIG_LED_SW1_pin					2
#define CONFIG_LED_SW1_port					PORTC
#define CONFIG_LED_SW2_pin					3
#define CONFIG_LED_SW2_port					PORTC
#define CONFIG_LED_SW3_pin					4
#define CONFIG_LED_SW3_port					PORTC
#define CONFIG_LED_SW4_pin					5
#define CONFIG_LED_SW4_port					PORTC
#define CONFIG_LED_SW5_pin					6
#define CONFIG_LED_SW5_port					PORTC
#define CONFIG_LED_SW6_pin					7
#define CONFIG_LED_SW6_port					PORTC
#define CONFIG_LED_SW7_pin					0	
#define CONFIG_LED_SW7_port					PORTE

/*
	Pi daemon
*/
#define CONFIG_PIDAEMON_INT_REQUEST_PORT	PORTC
#define CONFIG_PIDAEMON_INT_REQUEST_PIN		2

#define CONFIG_PIDAEMON_LOGIC_LEVEL_CHECK
#define CONFIG_PIDAEMON_STATE_PORT			PORTC
#define CONFIG_PIDAEMON_STATE_PIN			3



#endif /* PICONFIG_H_ */
