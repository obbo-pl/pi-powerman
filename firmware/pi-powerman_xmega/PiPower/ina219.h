/*
 * ina291.h
 *
 * Created: 2019-01-14 14:42:20
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


// Atmel XMEGA TWI driver AVR1308.zip
#include "twi_master_driver.h"
#include <stdbool.h>

#ifndef INA291_H_
#define INA291_H_

#define INA219_TWI_TIMEOUT_MS				(uint8_t)(10)
#define INA219_CONVERSION_TIMEOUT_MS		(uint8_t)(120)

#define INA219_CONFIG_RESET					0x8000  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK	0x2000  // Bus Voltage Range Mask
enum{
	INA219_CONFIG_BVOLTAGERANGE_16V			= 0x0000,  // Range 0-16 volts
	INA219_CONFIG_BVOLTAGERANGE_32V			= 0x2000   // Range 0-32 volts
};

#define INA219_CONFIG_GAIN_MASK				0x1800  // Sets PGA gain and range (Shunt Voltage Only) Mask
enum{
	INA219_CONFIG_GAIN_1_40MV				= 0x0000,  // Gain 1, 40mV Range
	INA219_CONFIG_GAIN_2_80MV				= 0x0800,  // Gain 2, 80mV Range
	INA219_CONFIG_GAIN_4_160MV				= 0x1000,  // Gain 4, 160mV Range
	INA219_CONFIG_GAIN_8_320MV				= 0x1800   // Gain 8, 320mV Range
};

#define INA219_CONFIG_BADCRES_MASK			0x0780  // BADC Bus ADC Resolution/Averaging Mask
enum {
	INA219_CONFIG_BADCRES_9BIT				= 0x0000,  // 9-bit bus res = 0..511
	INA219_CONFIG_BADCRES_10BIT				= 0x0080,  // 10-bit bus res = 0..1023
	INA219_CONFIG_BADCRES_11BIT				= 0x0100,  // 11-bit bus res = 0..2047
	INA219_CONFIG_BADCRES_12BIT				= 0x0180   // 12-bit bus res = 0..4097
};

#define INA219_CONFIG_SADCRES_MASK			0x0078  // SADC Shunt ADC Resolution/Averaging Mask
enum {
	INA219_CONFIG_SADCRES_9BIT_1S_84US		= 0x0000,  // 9bit conversion time  84us
	INA219_CONFIG_SADCRES_10BIT_1S_148US	= 0x0008,  // 10bit conversion time 148us
	INA219_CONFIG_SADCRES_11BIT_1S_276US	= 0x0010,  // 11bit conversion time 2766us
	INA219_CONFIG_SADCRES_12BIT_1S_532US	= 0x0018,  // 12bit conversion time 532us
	INA219_CONFIG_SADCRES_12BIT_2S_1060US	= 0x0048,  // 2 samples 12bit conversion time 1.06ms
	INA219_CONFIG_SADCRES_12BIT_4S_2130US	= 0x0050,  // 4 samples 12bit conversion time 2.13ms
	INA219_CONFIG_SADCRES_12BIT_8S_4260US	= 0x0058,  // 8 samples 12bit conversion time 4.26ms
	INA219_CONFIG_SADCRES_12BIT_16S_8510US	= 0x0060,  // 16 samples 12bit conversion time 8.51ms
	INA219_CONFIG_SADCRES_12BIT_32S_17MS	= 0x0068,  // 32 samples 12bit conversion time 17.02ms
	INA219_CONFIG_SADCRES_12BIT_64S_34MS	= 0x0070,  // 64 samples 12bit conversion time 34.05ms
	INA219_CONFIG_SADCRES_12BIT_128S_69MS	= 0x0078   // 128 samples 12bit conversion time 68.10ms
};

#define INA219_CONFIG_MODE_MASK				0x0007  // Operating Mode Mask
enum {
	INA219_CONFIG_MODE_POWERDOWN			= 0x0000,
	INA219_CONFIG_MODE_SVOLT_TRIGGERED		= 0x0001,
	INA219_CONFIG_MODE_BVOLT_TRIGGERED		= 0x0002,
	INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED	= 0x0003,
	INA219_CONFIG_MODE_ADCOFF				= 0x0004,
	INA219_CONFIG_MODE_SVOLT_CONTINUOUS		= 0x0005,
	INA219_CONFIG_MODE_BVOLT_CONTINUOUS		= 0x0006,
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS	= 0x0007
};

// Register Set
#define INA219_REG_CONFIG					0x00
#define INA219_REG_SHUNTVOLTAGE				0x01
#define INA219_REG_BUSVOLTAGE				0x02
#define INA219_REG_POWER					0x03
#define INA219_REG_CURRENT					0x04
#define INA219_REG_CALIBRATION				0x05

#define INA219_BUSVOLTAGE_OVF				0x0001
#define INA219_BUSVOLTAGE_CNVR				0x0002
#define INA219_BUSVOLTAGE_DATA				0xfff8


enum {
	INA219_ERRORCODE_I2C,
	INA219_ERRORCODE_NOTREADY,
	INA219_ERRORCODE_OVERFLOW
};
	

typedef struct INA219 {
	TWI_Master_t *twi;			// Pointer to what TWI interface to use
	uint8_t address;			// I2C address, full or pin specific (A0, A1)
	bool ready;
	bool overflow;
	uint16_t calibration;
	float current_ma_lsb;
	float power_w_lsb;
} INA219_t;



bool ina219_Init(INA219_t *ina, TWI_Master_t *twi, uint8_t address);
uint8_t ina219_SetCalibration_12bit(INA219_t *ina, uint16_t voltage_range, uint16_t pga_gain, float current_ma_lsb, float rshunt_ohm);
bool ina219_ConversionReady(INA219_t *ina);
bool ina219_GetRawBusVoltage(INA219_t *ina, uint16_t *val);
bool ina219_GetRawShuntVoltage(INA219_t *ina, int16_t *val);
bool ina219_GetRawCurrent(INA219_t *ina, int16_t *val);
bool ina219_GetRawPower(INA219_t *ina, uint16_t *val);
// Returns the bus voltage in volts
bool ina219_GetBusVoltage(INA219_t *ina, float *val);
// Returns the shunt voltage in volts
bool ina219_GetShuntVoltage(INA219_t *ina, float *val);
// Returns the shunt current in amps
bool ina219_GetCurrent(INA219_t *ina, float *val);
// Returns the bus power in watts
bool ina219_GetPower(INA219_t *ina, float *val);


#endif /* INA291_H_ */

