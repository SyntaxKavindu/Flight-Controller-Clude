/*
 * Magnetometer.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Magnetometer.hpp"
#include "Drone.hpp"   // drone.calibrator -- the only route to it
#include "spi.h"

// ===========================================================================
// BOARD MOUNTING AXIS REMAP -- EDIT HERE, AND ONLY HERE
//
// LIS3MDL as mounted on this airframe:
//
//     body X (Forward) =  chip -Y
//     body Y (Right)   =  chip -X
//     body Z (Down)    =  chip -Z
//
// i.e. body = M * chip with
//
//     M = [  0  -1   0 ]
//         [ -1   0   0 ]
//         [  0   0  -1 ]
//
// det(M) = +1, so this is a PROPER ROTATION and the frame stays right-handed.
// Worth checking on any change: negating a single axis, or swapping two
// without negating a third, gives det = -1. A reflected magnetic frame still
// produces a heading that looks sane at some orientations and is wrong at
// others -- there is no error flag for it.
//
// NOTE this is NOT the same mapping as the IMU's (Imu.cpp uses body X = chip
// +Y, body Y = chip +X). The two parts are mounted differently, which is
// exactly why each has its own remap. Do not unify them.
//
// A wrong mapping here does not look like a failure: heading is simply wrong,
// or wrong only in some orientations, which is very hard to spot in the air.
// If yaw ever reads plausibly but drifts against a handheld compass, this
// function is the first place to look.
// ===========================================================================
static inline Vector3f remapBoardAxes(const Vector3f &v) {
	return Vector3f(-v.y, -v.x, -v.z);
}

Vector3f Magnetometer::remapMag(const Vector3f &v) { return remapBoardAxes(v); }

Magnetometer::Magnetometer() :
		_sensor { &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin }, _data { },
		_raw_field { },
		_status { MAG_StatusTypeDef::ERROR },
		_hasData { false } {
}

MAG_StatusTypeDef Magnetometer::init(void) {
	// Drop any cached sample first -- see the note in Imu::init().
	_data = LIS3MDL_Data { };
	_status = MAG_StatusTypeDef::ERROR;
	_hasData = false;
	return _sensor.init();
}

void Magnetometer::update(void) {
	LIS3MDL_Data sample;

	_status = _sensor.readData(sample);
	if (_status != MAG_StatusTypeDef::OK) {
		return; // keep the last good sample
	}

	// LIS3MDL_Data carries loose floats; the calibrator and the remap both
	// work on a vector, so convert once here and write back once below.
	Vector3f field = remapMag(Vector3f(sample.x, sample.y, sample.z));

	// Captured BEFORE any correction, which is the whole point of it -- see
	// getRawField().
	_raw_field = field;

	{
		if (drone.calibrator.isCompassCalibrating()) {
			drone.calibrator.calibrateCompass(field);
		}
		// Hard/soft-iron correction in the sensor's own axes first, then the
		// board rotation -- the same order, and for the same reason, as the
		// accelerometer path in Imu::update(). The magnetometer is on the same
		// board as the IMU, so it takes the same rotation; leaving it out would
		// put heading in a different frame from roll and pitch.
		drone.calibrator.correctCompassData(field);
		drone.calibrator.correctBoardFrame(field);
	}

	sample.x = field.x;
	sample.y = field.y;
	sample.z = field.z;

	_data = sample;
	_hasData = true;
}

MAG_StatusTypeDef Magnetometer::getData(LIS3MDL_Data &data) {
	if (!_hasData) {
		return MAG_StatusTypeDef::ERROR;
	}
	data = _data;
	return _status;
}
