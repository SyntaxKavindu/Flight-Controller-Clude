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

// ===========================================================================
// INDICATOR LEDS -- EDIT HERE, AND ONLY HERE
//
// !! PLACEHOLDER PINS. Replace the three blocks below with your board's real
// !! CubeMX user labels, or rename the pins in CubeMX to these names, and the
// !! #warning underneath will go away by itself.
//
// The pattern matches SPI1_CS_GPIO_Port and friends: CubeMX generates
// <LABEL>_GPIO_Port and <LABEL>_Pin into main.h, and the object is handed them
// here rather than hardcoding a port inside the class.
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
// ===========================================================================
#ifndef LED_SYSTEM_Pin
#warning "Indicator LEDs are on placeholder pins -- set them in Globals.cpp"
#define LED_SYSTEM_GPIO_Port GPIOE
#define LED_SYSTEM_Pin       GPIO_PIN_0
#define LED_ARM_GPIO_Port    GPIOE
#define LED_ARM_Pin          GPIO_PIN_1
#define LED_GPS_GPIO_Port    GPIOE
#define LED_GPS_Pin          GPIO_PIN_2
#endif

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
	{ LED_SYSTEM_GPIO_Port, LED_SYSTEM_Pin, true },
	{ LED_ARM_GPIO_Port,    LED_ARM_Pin,    true },
	{ LED_GPS_GPIO_Port,    LED_GPS_Pin,    true }
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
