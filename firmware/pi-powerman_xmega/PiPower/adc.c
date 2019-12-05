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
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	const uint8_t adc_input[ADC_MAX_INPUT_COUNT] = {CONFIG_ADC_MUX_PiState, CONFIG_ADC_MUX_V_In, CONFIG_ADC_MUX_V_Bat, CONFIG_ADC_MUX_BAT_Temp};
#endif

uint8_t adc_ReadCalibrationByte(uint8_t index);

void adc_Setup()
{
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	ADCA.REFCTRL = CONFIG_ADC_REFERENCE_PORT;
	ADCA.CALL = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	ADCA.CALH = adc_ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL_MED_gc;
#endif
}

void adc_StartConversion(void) 
{
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
#endif
}

void adc_SetMux(uint8_t input)
{
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	ADCA.CH0.MUXCTRL = input;
#endif
}

uint16_t adc_ReadInput(void) 
{
	uint16_t result = 0;
#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	result = (uint16_t)(ADCA.CH0.RES);
#endif
	return result;
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
	uint8_t result = 0;

#ifdef CONFIG_ADC_MEASUREMENT_INSTALLED
	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);
	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
#endif
	return result;
}
