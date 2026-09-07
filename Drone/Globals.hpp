/*
 * Globals.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * The system's global objects, in one place.
 *
 * Each object is *declared* extern next to its own class, so a unit that needs
 * only one can include just that header:
 *
 *     EEPROM        storage       -- EEPROM.hpp
 *     Telemetry     telemetry     -- Telemetry.hpp
 *     Calibrator    calibrator    -- Calibrator.hpp
 *     Imu           imu           -- Imu.hpp
 *     Magnetometer  magnetometer  -- Magnetometer.hpp
 *     Barometer     barometer     -- Barometer.hpp
 *     Indicator     indicator     -- Indicator.hpp
 *
 * They are all *defined* in Globals.cpp. Keeping the definitions in one
 * translation unit is what makes their construction order defined: across
 * translation units the order is unspecified, so a global that touched another
 * during construction would be reading uninitialised memory. Nothing here does
 * that today, and Globals.cpp is the place to check if a constructor ever
 * starts to.
 *
 * Include this header when a unit wants the whole set (Drone, the C bridge);
 * include the individual class header when it wants one.
 */

#ifndef DRONE_GLOBALS_HPP_
#define DRONE_GLOBALS_HPP_

#include "Barometer.hpp"
#include "Calibrator.hpp"
#include "EEPROM.hpp"
#include "Imu.hpp"
#include "Indicator.hpp"
#include "Magnetometer.hpp"
#include "Telemetry.hpp"

// Which devices failed to come up, OR'd together. 0 means everything is up.
enum GlobalsInitStatus : uint8_t {
	GLOBALS_INIT_OK           = 0x00,
	GLOBALS_INIT_STORAGE_FAIL = 0x01,
	GLOBALS_INIT_IMU_FAIL     = 0x02,
	GLOBALS_INIT_MAG_FAIL     = 0x04,
	GLOBALS_INIT_BARO_FAIL    = 0x08
};

/*
 * Bring the globals up and connect them: storage first (the calibrator loads
 * from it), then the calibrator, then the sensors, which are handed the
 * calibrator so update() can correct and feed it.
 *
 * A failing device does not stop the rest -- an airframe with a dead barometer
 * should still come up far enough to say so over telemetry. The return value
 * says what did not start; the caller decides whether that is flyable.
 */
uint8_t Globals_Init(void);

#endif /* DRONE_GLOBALS_HPP_ */
