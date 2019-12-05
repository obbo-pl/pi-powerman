/*
 * ina219.c
 *
 * Created: 2019-01-14 14:43:30
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include "ina219.h"
#include "bitlib.h"

#define INA219_BASE_ADDRESS					0x40
#define INA291_ADDRESS_MASK					0x0f



// Prototypes of functions
bool ina219_WriteRegister(INA219_t *ina, uint8_t reg, uint16_t val);
bool ina219_ReadRegister(INA219_t *ina, uint8_t reg, uint16_t *val);


bool ina219_WriteRegister(INA219_t *ina, uint8_t reg, uint16_t val)
{
	uint16_t timeout = 10 * INA219_TWI_TIMEOUT_MS;
	uint8_t buffer[3];

	buffer[0] = reg;
	buffer[1] = 0xff & (val >> 8);
	buffer[2] = 0xff & val;
	bool result = TWI_MasterWriteRead(ina->twi, ina->address, buffer, 3, 0);
	while ((ina->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) result &= false;
	return result;
}

bool ina219_ReadRegister(INA219_t *ina, uint8_t reg, uint16_t *val)
{
	uint16_t timeout = 10 * INA219_TWI_TIMEOUT_MS;

	bool result = TWI_MasterWriteRead(ina->twi, ina->address, &reg, 1, 2);
	while ((ina->twi->status != TWIM_STATUS_READY) && (timeout > 0)) {
		if (timeout > 0) timeout--;
		_delay_us(100);
	}
	if (!timeout) {
		result &= false;
	} else {
		*val = ina->twi->readData[0] << 8;
		*val += ina->twi->readData[1];
	}	
	return result;
}

bool ina219_Init(INA219_t *ina, TWI_Master_t *twi, uint8_t address)
{
	ina->twi = twi;
	address = (INA219_BASE_ADDRESS | address) & (INA219_BASE_ADDRESS | INA291_ADDRESS_MASK);
	ina->address = address;
	bool result = ina219_WriteRegister(ina, INA219_REG_CONFIG, INA219_CONFIG_RESET);
	ina->ready = false;
	ina->overflow = false;
	return result;
}

bool ina219_ConversionReady(INA219_t *ina)
{
	uint16_t val;
	
	bool result = ina219_GetRawBusVoltage(ina, &val);
	result &= ina->ready;
	return result;
}

uint8_t ina219_SetCalibration_12bit(INA219_t *ina, uint16_t voltage_range, uint16_t pga_gain, float current_ma_lsb, float rshunt_ohm)
{
	uint8_t result = 0x00;

	// www.ti.com
	// INA219.pdf, INA219 Zer?-Drift, Bidirectional Current/Power Monitor With I2C Interface
	// SBOS448G –AUGUST 2008–REVISED DECEMBER 2015
	// 8.5.1 Programming the Calibration Register
	ina->calibration = trunc(0.04096 / (0.001 * current_ma_lsb * rshunt_ohm));
	ina->current_ma_lsb = current_ma_lsb;
	ina->power_w_lsb = 20 * current_ma_lsb / 1000;
	if (ina219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration)) setbit(result, INA219_ERRORCODE_I2C);
	uint16_t config = voltage_range | 
	                  pga_gain | 
					  INA219_CONFIG_BADCRES_12BIT |
					  INA219_CONFIG_SADCRES_12BIT_1S_532US | 
					  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	if (ina219_WriteRegister(ina, INA219_REG_CONFIG, config)) setbit(result, INA219_ERRORCODE_I2C);
	return result;
}

bool ina219_GetRawBusVoltage(INA219_t *ina, uint16_t *val)
{
	uint16_t v; 
	
	bool result = ina219_ReadRegister(ina, INA219_REG_BUSVOLTAGE, &v);
	if ((v & INA219_BUSVOLTAGE_OVF)) {
		ina->overflow = true;
	} else {
		ina->overflow = false;
	}
	if ((v & INA219_BUSVOLTAGE_CNVR)) {
		ina->ready = true;
	} else {
		ina->ready = false;
	}
	if (result && ina->ready) {
		*val = (uint16_t)(v & INA219_BUSVOLTAGE_DATA);	// cut CNVR and OVF
	}
	return result;
}

bool ina219_GetRawShuntVoltage(INA219_t *ina, int16_t *val)
{
	bool result = ina219_ReadRegister(ina, INA219_REG_SHUNTVOLTAGE, (uint16_t *)(val));
	return result;
}

bool ina219_GetRawCurrent(INA219_t *ina, int16_t *val)
{
	bool result = ina219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration);
	result &= ina219_ReadRegister(ina, INA219_REG_CURRENT, (uint16_t *)(val));
	return result;
}

bool ina219_GetRawPower(INA219_t *ina, uint16_t *val)
{
	bool result = ina219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration);
	result &= ina219_ReadRegister(ina, INA219_REG_POWER, val);
	return result;
}

// Returns the bus voltage in volts
bool ina219_GetBusVoltage(INA219_t *ina, float *val)
{
	uint16_t v;
	bool result = ina219_GetRawBusVoltage(ina, &v);
	v = v >> 1;
	*val = v * 0.001;
	return result;
}

// Returns the shunt voltage in volts
bool ina219_GetShuntVoltage(INA219_t *ina, float *val)
{
	int16_t v;
	bool result = ina219_GetRawShuntVoltage(ina, &v);
	*val = v * 0.00001;
	return result;
}

// Returns the shunt current in amps
bool ina219_GetCurrent(INA219_t *ina, float *val)
{
	int16_t v; 
	bool result = ina219_GetRawCurrent(ina, &v);
	*val = v * ina->current_ma_lsb;
	return result;
}

// Returns the bus power in watts
bool ina219_GetPower(INA219_t *ina, float *val)
{
	uint16_t v;
	bool result = ina219_GetRawPower(ina, &v);
	*val = v * ina->power_w_lsb;
	return result;
}
