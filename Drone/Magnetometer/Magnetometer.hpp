/*
 * Magnetometer.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#ifndef MAGNETOMETER_MAGNETOMETER_HPP_
#define MAGNETOMETER_MAGNETOMETER_HPP_

#include "LIS3MDL.hpp"

class Calibrator;

/*
 * Frontend for the LIS3MDL. Same contract as Imu: update() reads once,
 * rotates into the body frame, applies the stored hard/soft-iron correction
 * and keeps the result; getData() returns that stored sample.
 *
 * A failed read keeps the previous sample and is reported through getData()'s
 * return value; before the first successful read getData() returns ERROR and
 * leaves the caller's buffer untouched.
 */
class Magnetometer {
public:
	Magnetometer();

	MAG_StatusTypeDef init(void);
	void setCalibrator(Calibrator *cal);

	void update(void);
	MAG_StatusTypeDef getData(LIS3MDL_Data &data);

	MAG_StatusTypeDef getStatus(void) const { return _status; }
	bool hasData(void) const { return _hasData; }

private:
	LIS3MDL _sensor;
	LIS3MDL_Data _data;

	Calibrator *_calibrator;
	MAG_StatusTypeDef _status;
	bool _hasData;

	// Board-mounting axis remap: chip axes -> vehicle body frame (FRD).
	// For this airframe body X = chip -Y, body Y = chip -X, body Z = chip -Z,
	// which is NOT the same as the IMU's mapping -- see the banner in
	// Magnetometer.cpp. The driver returns native/electrical axes and leaves
	// this to us -- see LIS3MDL::readData().
	static Vector3f remapMag(const Vector3f &v);
};

// The one magnetometer. Defined in Globals.cpp.
extern Magnetometer magnetometer;

#endif /* MAGNETOMETER_MAGNETOMETER_HPP_ */
