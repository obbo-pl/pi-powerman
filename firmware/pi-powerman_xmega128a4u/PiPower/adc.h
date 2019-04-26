/*
 * adc.h
 *
 * Created: 2019-01-12 20:46:39
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

#define ADC_INPUT_COUNT		4
enum {
	ADC_MUX_PiState		= ADC_CH_MUXPOS_PIN5_gc,
	ADC_MUX_V_In		= ADC_CH_MUXPOS_PIN3_gc,
	ADC_MUX_V_Bat		= ADC_CH_MUXPOS_PIN4_gc,
	ADC_MUX_BAT_Temp	= ADC_CH_MUXPOS_PIN7_gc
};
#define INIT_INPUT_ARRAY	const uint8_t adc_input[ADC_INPUT_COUNT] = {ADC_MUX_PiState, ADC_MUX_V_In, ADC_MUX_V_Bat, ADC_MUX_BAT_Temp};


void adc_Setup();
void adc_SetMux(uint8_t input);
void adc_StartConversion(void);
uint16_t adc_ReadInput(void);
uint8_t adc_CurrentInput(void);
uint8_t adc_NextInput(void);


#endif /* ADC_H_ */