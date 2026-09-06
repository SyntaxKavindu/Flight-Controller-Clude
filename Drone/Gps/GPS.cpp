/*
 * GPS.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "GPS.hpp"

#include <cmath>

// ===========================================================================
// ANTENNA LEVER ARM
//
// The receiver reports where the ANTENNA is. The ESEKF estimates where the
// body-frame origin is. Those are not the same point, and the difference is
// not a constant the filter can absorb: it rotates with the airframe.
//
//     p_body = p_antenna - R_bn * r
//     v_body = v_antenna - R_bn * (omega x r)
//
// with r the antenna offset in body axes (FRD) and R_bn the body -> NED
// rotation. Both terms are pure geometry -- there is nothing to estimate and
// nothing to tune.
//
// Size of the problem, for a 15 cm arm: the position term is a 15 cm bias
// that swings with HEADING, so it cannot be trimmed out; the velocity term
// is |omega x r| = 0.24 m/s at 90 deg/s of yaw. To the filter that is an
// airframe genuinely moving, so it goes into the velocity state and from
// there leaks into accel bias and tilt. A hovering yaw ends up drifting in a
// small circle. There is no error flag for any of it.
//
// The gyro and the magnetometer need no equivalent: rigid-body angular rate
// is the same at every point, and Earth's field is uniform over an airframe.
// The accelerometer's lever arm is identically zero once the body origin is
// defined AT the IMU, which is why that is the definition to use.
// ===========================================================================

// WGS-84, for turning a metric NED offset into a lat/lon/alt offset. A flat
// 111320 m/deg would be off by ~0.5 % at mid-latitudes and far more further
// north; the radii of curvature cost two sqrt() per fix, which at 5-10 Hz is
// nothing.
static constexpr double WGS84_A = 6378137.0;
static constexpr double WGS84_E2 = 6.69437999014e-3;
static constexpr double DEG_TO_RAD = 0.017453292519943295;
static constexpr double RAD_TO_DEG = 57.29577951308232;

#if GPS_DATA_HAS_COURSE
// Local, and deliberately not a Vector3f member: the velocity half of the
// correction is the only place in the file that needs a cross product.
static inline Vector3f crossProduct(const Vector3f &a, const Vector3f &b) {
	return Vector3f(a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x);
}
#endif

GPS::GPS(UART_HandleTypeDef *huart) :
		_sensor { huart }, _data { }, _hasData { false }, _rejected { 0 },
		_antennaOffset { }, _R_bn { }, _gyro { }, _hasBodyState { false },
		_leverArmApplied { false } {
}

GPS_StatusTypeDef GPS::init(void) {
	return _sensor.init();
}

void GPS::setAntennaOffset(const Vector3f &offsetBody) {
	_antennaOffset = offsetBody;
}

// Parameter names deliberately kept off the estimator's own globals: this
// takes a snapshot, it does not reach back into the filter.
void GPS::setBodyState(const Mat3f &R_bn, const Vector3f &gyroBody) {
	_R_bn = R_bn;
	_gyro = gyroBody;
	_hasBodyState = true;
}

bool GPS::isPlausible(const GPS_Data &data) {
	if (!std::isfinite(data.latitude) || !std::isfinite(data.longitude)
			|| !std::isfinite(data.altitude) || !std::isfinite(data.speed)) {
		return false;
	}
	if (data.latitude < -90.0 || data.latitude > 90.0) {
		return false;
	}
	if (data.longitude < -180.0 || data.longitude > 180.0) {
		return false;
	}
	if (data.altitude < GPS_MIN_ALTITUDE_M || data.altitude > GPS_MAX_ALTITUDE_M) {
		return false;
	}
	if (data.speed < 0.0 || data.speed > GPS_MAX_SPEED_MPS) {
		return false;
	}

	// Null Island. Exactly zero on both axes is not a place any receiver is;
	// it is what a module emits when asked for a position it does not have,
	// and it passes every range check above.
	if (data.latitude == 0.0 && data.longitude == 0.0) {
		return false;
	}

	return true;
}

void GPS::applyLeverArm(GPS_Data &data) {
	_leverArmApplied = false;

	if (!_hasBodyState) {
		// No attitude has ever been handed in. Leave the fix at the antenna
		// and record that we did: reporting the antenna is a known, bounded
		// error, whereas rotating by an assumed-level attitude is an unknown
		// one that is worst precisely when the airframe is manoeuvring.
		return;
	}
	if (_antennaOffset.x == 0.0f && _antennaOffset.y == 0.0f
			&& _antennaOffset.z == 0.0f) {
		return; // antenna is at the origin -- nothing to remove
	}

	// --- position: p_body = p_antenna - R_bn * r ---------------------------
	// Mat3f spells matrix-vector product as .mul(), as it does everywhere else
	// in this tree (ESEKF, Calibrator, LevelCalibrator); there is no operator*.
	const Vector3f offsetNED = _R_bn.mul(_antennaOffset); // metres, N/E/D

	const double latRad = data.latitude * DEG_TO_RAD;
	const double sinLat = std::sin(latRad);
	const double cosLat = std::cos(latRad);
	const double denom = 1.0 - WGS84_E2 * sinLat * sinLat;
	const double radiusMeridional = WGS84_A * (1.0 - WGS84_E2)
			/ (denom * std::sqrt(denom));
	const double radiusPrimeVertical = WGS84_A / std::sqrt(denom);

	data.latitude -= (static_cast<double>(offsetNED.x) / radiusMeridional)
			* RAD_TO_DEG;
	// A metre of easting is unbounded in degrees at the pole. No airframe
	// flies there, but the guard costs one compare and stops a NaN reaching
	// the ESEKF -- which is not something a filter recovers from.
	if (std::fabs(cosLat) > 1e-6) {
		data.longitude -= (static_cast<double>(offsetNED.y)
				/ (radiusPrimeVertical * cosLat)) * RAD_TO_DEG;
	}
	// NED down is negative up, so a Down offset ADDS to altitude.
	data.altitude += offsetNED.z;

	// --- speed: v_body = v_antenna - R_bn * (omega x r) --------------------
#if GPS_DATA_HAS_COURSE
	const Vector3f velocityError = _R_bn.mul(crossProduct(_gyro, _antennaOffset));

	const double courseRad = data.course * DEG_TO_RAD;
	const double velN = data.speed * std::cos(courseRad) - velocityError.x;
	const double velE = data.speed * std::sin(courseRad) - velocityError.y;

	data.speed = std::sqrt(velN * velN + velE * velE);
	// Below a few cm/s the course of the corrected vector is noise, and
	// writing it back would spin the reported heading at random. Keep the
	// module's own value there.
	if (data.speed > 0.01) {
		double course = std::atan2(velE, velN) * RAD_TO_DEG;
		if (course < 0.0) {
			course += 360.0;
		}
		data.course = course;
	}
#else
	// GPS_Data has no course, so the speed half cannot be done: the
	// correction is a vector subtraction and a magnitude carries no
	// direction. Consequence to be aware of if you fuse speed -- a hovering
	// yaw at 90 deg/s with a 15 cm arm still reports ~0.24 m/s of ground
	// speed that is not there. The fix is to get NAV-VELNED (or VTG) out of
	// the NEO-M8N, carry a velocity vector in GPS_Data, and correct that.
#endif

	_leverArmApplied = true;
}

void GPS::update(void) {
	_sensor.update();

	GPS_Data fresh;
	if (_sensor.readData(fresh) != GPS_StatusTypeDef::OK) {
		return; // no fix yet, or the module has gone quiet -- keep the last one
	}

	if (!isPlausible(fresh)) {
		_rejected++;
		return; // keep the previous fix rather than adopt a bad one
	}

	// Screen first, correct second. A sub-metre correction cannot rescue a bad
	// fix, and range-checking an already-modified value would hide which of
	// the two was actually wrong.
	applyLeverArm(fresh);

	_data = fresh;
	_hasData = true;
}

GPS_StatusTypeDef GPS::getData(GPS_Data &data) {
	if (!_hasData) {
		return GPS_StatusTypeDef::ERROR;
	}
	data = _data;
	return GPS_StatusTypeDef::OK;
}

bool GPS::isFix(void) {
	return _hasData && _sensor.hasFix();
}