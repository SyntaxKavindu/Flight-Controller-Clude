/*
 * Imu.cpp
 *
 *  Created on: Aug 23, 2026
 *      Author: KAVINDU
 */

#include "Imu.hpp"
#include "Drone.hpp"   // drone.calibrator -- the only route to it
#include "spi.h"

// ===========================================================================
// BOARD MOUNTING AXIS REMAP -- EDIT HERE, AND ONLY HERE
//
// ICM-42688-P as mounted on this airframe:
//
//     body X (Forward) =  chip +Y
//     body Y (Right)   =  chip +X
//     body Z (Down)    =  chip -Z
//
// i.e. body = M * chip with
//
//     M = [ 0  1  0 ]
//         [ 1  0  0 ]
//         [ 0  0 -1 ]
//
// det(M) = +1, so this is a PROPER ROTATION and the frame stays right-handed.
// That matters more than it looks: swapping X and Y alone (det = -1) or
// negating Z alone (det = -1) is a REFLECTION. A reflected frame still gives
// a plausible-looking roll and pitch from gravity, but every gyro rotation
// integrates the wrong way round, so attitude runs backwards under motion and
// only under motion. imu_remap_preserves_handedness() pins this.
//
// Consistency check: with the drone level and at rest the ESEKF expects body
// accel (0, 0, -g). body.z = -chip.z gives chip.z = +g, i.e. the chip's +Z
// points up when the airframe is level -- which is exactly a board mounted
// component-side up.
//
// Accel and gyro MUST go through the same mapping. A mismatch between them is
// not obvious in flight: attitude drifts under rotation instead of failing
// outright, which is why the remap lives in one function both call.
//
// The LIS3MDL has its OWN remap in Magnetometer.cpp -- it is a separate part
// and is not necessarily mounted the same way. Do not copy this one over
// without checking.
// ===========================================================================
static inline Vector3f remapBoardAxes(const Vector3f &v) {
	return Vector3f(v.y, v.x, -v.z);
}

Vector3f Imu::remapAccel(const Vector3f &v) { return remapBoardAxes(v); }
Vector3f Imu::remapGyro(const Vector3f &v) { return remapBoardAxes(v); }

Imu::Imu() :
		_sensor { &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin }, _data { },
		_raw_accel { },
		_status { IMU_StatusTypeDef::ERROR },
		_hasData { false } {
}

IMU_StatusTypeDef Imu::init(void) {
	// Drop any cached sample first. This object is a global, so without it a
	// re-init would inherit the last reading from before -- getData() would
	// report a stale measurement as current, and the estimator would seed off
	// it. init() must leave a known state, not a partly-carried-over one.
	_data = IMU_Data { };
	_status = IMU_StatusTypeDef::ERROR;
	_hasData = false;
	// Report what the driver actually says rather than an unconditional OK --
	// a caller has no other way to find out the sensor never came up.
	return _sensor.init();
}

void Imu::update(void) {
	IMU_Data sample;

	_status = _sensor.readData(sample);
	if (_status != IMU_StatusTypeDef::OK) {
		// Keep the last good sample. A dropped SPI transfer is a normal
		// transient; handing the control loop a zeroed struct is not.
		return;
	}

	// Chip axes -> body frame, before anything else looks at the numbers.
	sample.accel = remapAccel(sample.accel);
	sample.gyro = remapGyro(sample.gyro);

	// Captured BEFORE any correction, which is the whole point of it -- see
	// getRawAccel().
	_raw_accel = sample.accel;

	{
		// While a procedure is running this is the only place raw samples come
		// from, so feed it here, BEFORE any correction. Both procedures want the
		// raw reading: the accelerometer fit is what produces the correction, and
		// the levelling fit applies the correction itself (see
		// Calibrator::calibrateLevel) precisely so that this call site cannot get
		// the ordering wrong.
		//
		// They are mutually exclusive -- Calibrator::beginProcedure() refuses to
		// start one while another runs -- so `else if` is not merely an
		// optimisation, it states that.
		if (drone.calibrator.isAcclCalibrating()) {
			drone.calibrator.calibrateAccelerometer(sample.accel);
		} else if (drone.calibrator.isLevelCalibrating()) {
			drone.calibrator.calibrateLevel(sample.accel);
		}

		// Sensor correction first (bias and scale, in the accelerometer's own
		// axes), then the board rotation. The two do not commute: the fit was
		// made in the axes the chip actually reads in, so rotating first would
		// apply per-axis gains to axes they were not measured on.
		drone.calibrator.correctAcclData(sample.accel);

		// The board rotation describes the BOARD, so it goes on the gyro too.
		// Correcting one and not the other leaves them disagreeing about which
		// way the airframe points, and the estimator would then integrate
		// rotations in one frame while levelling against another -- attitude
		// that drifts only under motion, which is the hardest failure to spot.
		// There is no gyro path through the Calibrator for exactly this reason;
		// see the note on Calibrator::correctBoardFrame().
		drone.calibrator.correctBoardFrame(sample.accel);
		drone.calibrator.correctBoardFrame(sample.gyro);
	}

	_data = sample;
	_hasData = true;
}

IMU_StatusTypeDef Imu::getData(IMU_Data &data) {
	if (!_hasData) {
		// Nothing valid has ever been read; leave the caller's buffer alone
		// so an ignored return value cannot turn into a bogus zero reading.
		return IMU_StatusTypeDef::ERROR;
	}
	data = _data;
	return _status;
}
