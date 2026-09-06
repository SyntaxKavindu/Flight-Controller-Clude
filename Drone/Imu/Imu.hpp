/*
 * Imu.hpp
 *
 *  Created on: Aug 23, 2026
 *      Author: KAVINDU
 */

#ifndef IMU_IMU_HPP_
#define IMU_IMU_HPP_

#include "ICM42688P.hpp"

class Calibrator;

/*
 * Frontend for the ICM42688P.
 *
 * update() does one driver read, rotates the sample into the vehicle body
 * frame, applies the stored calibration, and keeps the result. getData()
 * hands back that stored sample -- so every consumer in a control loop sees
 * exactly the same measurement for that iteration, no matter when it asks,
 * and nobody but this class talks to the driver.
 *
 * Last-known-good: a failed read leaves the previous sample in place and is
 * reported through getData()'s return value. Before the first successful read
 * getData() returns ERROR and does not touch the caller's buffer, so a
 * consumer can never mistake an all-zero struct for "hovering at rest".
 */
class Imu {
public:
	Imu();

	IMU_StatusTypeDef init(void);

	// Optional. With no calibrator attached the raw (remapped) sample is
	// stored unmodified, which is exactly what an uncalibrated airframe wants.
	void setCalibrator(Calibrator *cal);

	void update(void);
	IMU_StatusTypeDef getData(IMU_Data &data);

	// Status of the most recent update(); OK only if that read succeeded.
	IMU_StatusTypeDef getStatus(void) const { return _status; }
	// False until the first successful read.
	bool hasData(void) const { return _hasData; }

private:
	ICM42688P _sensor;
	IMU_Data _data;

	Calibrator *_calibrator;
	IMU_StatusTypeDef _status;
	bool _hasData;

	// Board-mounting axis remap: chip axes -> vehicle body frame (FRD).
	// For this airframe body X = chip +Y, body Y = chip +X, body Z = chip -Z;
	// see the banner in Imu.cpp for the derivation and why handedness matters.
	// The driver deliberately returns the chip's native/electrical axes and
	// leaves this to us -- see ICM42688P::readData(). Accel and gyro must share
	// one remap, which is enforced here by having both delegate to the same
	// function rather than by convention.
	static Vector3f remapAccel(const Vector3f &v);
	static Vector3f remapGyro(const Vector3f &v);
};

// The one IMU. Defined in Globals.cpp.
extern Imu imu;

#endif /* IMU_IMU_HPP_ */
