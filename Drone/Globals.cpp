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
#include "main.h"   // SYSTEM_*/ARM_*/GPS_* pin labels, from CubeMX

// ===========================================================================
// INDICATOR LEDS -- EDIT HERE, AND ONLY HERE
//
// The pins come from main.h, the same way SPI1_CS_GPIO_Port and friends do:
// CubeMX generates <LABEL>_GPIO_Port and <LABEL>_Pin from the user labels set
// on the pinout, and the object is handed them here rather than any port being
// hardcoded inside the class. Rename a pin in CubeMX and this is the one place
// that has to follow.
//
// The third field is ACTIVE HIGH. Set it false for an LED wired to sink into
// the pin (anode to 3V3), which is the common arrangement. Getting it wrong
// does not break anything -- it inverts every pattern, so every SOLID state
// goes dark and every blink inverts. A healthy airframe with a fix, which
// should show three steady lights, shows three dark LEDs instead.
//
// The lamp test is the check: all three LEDs should be ON for the first 0.7 s
// after power-up. If they are OFF for 0.7 s and light up afterwards, this
// field is wrong on all three.
//
// !! PC13, PC14 and PC15 are BACKUP-DOMAIN pins, and they are not ordinary
// !! GPIOs. Two things to check against the datasheet and your schematic:
// !!
// !!   - Their output drive is limited compared with a normal pin. An LED sized
// !!     for a 10-20 mA direct drive is very likely out of spec here; it will
// !!     look dim rather than fail outright, which is the kind of wrong that
// !!     gets accepted. Use a high-value resistor with a low-current LED, or
// !!     drive through a transistor.
// !!   - PC14 and PC15 are OSC32_IN/OSC32_OUT. They are only free as GPIO
// !!     because the LSE crystal is not enabled. Turning the LSE on later --
// !!     for an RTC, say -- takes the ARM and GPS LEDs with it.
// ===========================================================================

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
Indicator indicator {
	{ SYSTEM_GPIO_Port, SYSTEM_Pin, true },   // PC13
	{ ARM_GPIO_Port,    ARM_Pin,    true },   // PC15
	{ GPS_GPIO_Port,    GPS_Pin,    true }    // PC14
};

uint8_t Globals_Init(void) {
	uint8_t status = GLOBALS_INIT_OK;

	// FIRST, before anything that can fail. The lamp test it starts is the only
	// signal an operator gets from a board whose telemetry never comes up, and a
	// device probe below is exactly the kind of thing that can hang -- so the
	// LEDs must already be lit by then, not waiting behind it.
	//
	// It cannot fail: a GPIO write has nothing to report, so there is no status
	// bit for it. What a dead LED looks like is covered by the lamp test.
	indicator.init();

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
