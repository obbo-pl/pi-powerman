/*
 * adc.c
 *
 * Created: 2019-01-12 20:46:53
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "adc.h"
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


uint8_t current_mux_input = 0;
INIT_INPUT_ARRAY


uint8_t adc_ReadCalibrationByte(uint8_t index);

void adc_Setup()
{
	// Set Vref on PortA pin0
	ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;
	ADCA.CALL = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL_HI_gc;
}

void adc_StartConversion(void) 
{
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
}

void adc_SetMux(uint8_t input)
{
	ADCA.CH0.MUXCTRL = input;
}

uint16_t adc_ReadInput(void) 
{
	return (uint16_t)(ADCA.CH0.RES);;
}

uint8_t adc_CurrentInput(void) 
{
	if (current_mux_input >= sizeof(adc_input)) current_mux_input = 0;
	return adc_input[current_mux_input];
}

uint8_t adc_NextInput(void)
{
	current_mux_input++;
	if (current_mux_input >= sizeof(adc_input)) current_mux_input = 0;
	return adc_input[current_mux_input];
}

uint8_t adc_ReadCalibrationByte(uint8_t index) {
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return result;
}
