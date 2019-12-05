/*
 * PiConfig.h
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
 * configuration file for https://github.com/obbo-pl/pi-powerman/tree/master/hardware/pi-powerman_3i1o_xmega32e5_190428
 */


/*
	UPS module definition
*/
//#define CONFIG_UPS_INSTALLED 

/*
	Measurement definition
*/
//#define CONFIG_I2C_MEASUREMENT_INSTALLED

//#define CONFIG_ADC_MEASUREMENT_INSTALLED

/*
	Power output
*/
#define CONFIG_POWEROUT_Pi_port				PORTD
#define CONFIG_POWEROUT_Pi_pin				5

/* 
	Key config
*/
#define CONFIG_KEY_PWR_pin					0
#define CONFIG_KEY_PWR_port					PORTD
#define CONFIG_KEY_SW1_pin					1
#define CONFIG_KEY_SW1_port					PORTD
#define CONFIG_KEY_SW2_pin					3
#define CONFIG_KEY_SW2_port					PORTD

/*
	LED
*/
#define CONFIG_LED_STATUS_pin				2
#define CONFIG_LED_STATUS_port				PORTD
#define CONFIG_LED_PWR_pin					4
#define CONFIG_LED_PWR_port					PORTD

/*
	Pi daemon
*/
#define CONFIG_PIDAEMON_INT_REQUEST_PORT	PORTC
#define CONFIG_PIDAEMON_INT_REQUEST_PIN		2

#define CONFIG_PIDAEMON_LOGIC_LEVEL_CHECK
#define CONFIG_PIDAEMON_STATE_PORT			PORTC
#define CONFIG_PIDAEMON_STATE_PIN			3



#endif /* PICONFIG_H_ */
