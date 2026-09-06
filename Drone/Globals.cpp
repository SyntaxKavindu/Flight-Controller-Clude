/*
 * Globals.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * The single definition point for every global object. See Globals.hpp for
 * why they all live in one translation unit.
 */

#include "Globals.hpp"
#include "i2c.h"

// Definition order is construction order within this file. None of these
// constructors touches another object, so the order is not load bearing today
// -- it is kept dependency-first anyway so it stays obvious if that changes.
//
// The M24C02 sits on I2C3. Getting this wrong is not loud: storage.init()
// simply fails, the calibrator comes up uncalibrated, and every calibration
// is silently not persisted across a reboot. Boot reports it as
// "$ERR,EEPROM INIT FAILED", and a calibration that did not persist ends
// "$CAL,ACCL,NOTSAVED" rather than SAVED.
EEPROM storage { &hi2c3 };
Telemetry telemetry;
Calibrator calibrator;
Imu imu;
Magnetometer magnetometer;
Barometer barometer;

uint8_t Globals_Init(void) {
	uint8_t status = GLOBALS_INIT_OK;

	// Storage first: the calibrator restores its stored gains from it.
	if (storage.init() != EEPROM_StatusTypeDef::OK) {
		status |= GLOBALS_INIT_STORAGE_FAIL;
		// Carry on with a null handle rather than a dead one, so the
		// calibrator reports "not saved" instead of retrying a broken bus on
		// every single calibration.
		calibrator.init(nullptr);
	} else {
		calibrator.init(&storage);
	}

	// The sensors correct their samples through the calibrator, and feed it
	// while a calibration is running.
	imu.setCalibrator(&calibrator);
	magnetometer.setCalibrator(&calibrator);

	if (imu.init() != IMU_StatusTypeDef::OK) {
		status |= GLOBALS_INIT_IMU_FAIL;
	}
	if (magnetometer.init() != MAG_StatusTypeDef::OK) {
		status |= GLOBALS_INIT_MAG_FAIL;
	}
	if (barometer.init() != BARO_StatusTypeDef::OK) {
		status |= GLOBALS_INIT_BARO_FAIL;
	}

	return status;
}
