/*
 * Barometer.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Barometer.hpp"
#include "spi.h"

Barometer::Barometer() :
		_sensor { &hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin }, _data { },
		_status { BARO_StatusTypeDef::ERROR }, _hasData { false }, _rejected { 0 } {
}

bool Barometer::isPlausible(const BMP390_Data &sample) const {
	if (!std::isfinite(sample.pressure) || !std::isfinite(sample.altitude)
			|| !std::isfinite(sample.temperature)) {
		return false;
	}
	if (sample.pressure < BAROMETER_MIN_PRESSURE_PA
			|| sample.pressure > BAROMETER_MAX_PRESSURE_PA) {
		return false; // outside the sensor's own operating range
	}
	if (_hasData) {
		// A torn 24-bit read corrupts one byte of the pressure word, which the
		// barometric formula turns into hundreds or thousands of metres in a
		// single sample. Real vertical motion cannot do that at 50 Hz.
		const double step = sample.altitude - _data.altitude;
		if (step > (double) BAROMETER_MAX_STEP_M || step < -(double) BAROMETER_MAX_STEP_M) {
			return false;
		}
	}
	return true;
}

BARO_StatusTypeDef Barometer::init(void) {
	// Drop any cached sample first -- see the note in Imu::init(). The reject
	// counter goes with it, so it counts this session rather than all time.
	_data = BMP390_Data { };
	_status = BARO_StatusTypeDef::ERROR;
	_hasData = false;
	_rejected = 0;
	return _sensor.init();
}

void Barometer::update(void) {
	BMP390_Data sample;

	_status = _sensor.readData(sample);
	if (_status != BARO_StatusTypeDef::OK) {
		return; // no new conversion, or a bus error -- keep the last good sample
	}

	if (!isPlausible(sample)) {
		_rejected++;
		_status = BARO_StatusTypeDef::ERROR;
		return; // keep the last good sample rather than pass a glitch on
	}

	_data = sample;
	_hasData = true;
}

BARO_StatusTypeDef Barometer::getData(BMP390_Data &data) {
	if (!_hasData) {
		return BARO_StatusTypeDef::ERROR;
	}
	data = _data;
	return _status;
}
