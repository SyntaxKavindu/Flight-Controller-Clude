/*
 * GPS.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#ifndef GPS_GPS_HPP_
#define GPS_GPS_HPP_

#include "NEOM8N.hpp"
// Vector3f, Matrix3f and Quaternionf together, with no HAL behind them.
#include "MathTypes.hpp"

// A GPS fix is a position on Earth, and these are the bounds of that. Anything
// outside is a parse that went wrong or a module talking rubbish.
#define GPS_MAX_ALTITUDE_M        20000.0
#define GPS_MIN_ALTITUDE_M        (-500.0)
#define GPS_MAX_SPEED_MPS         500.0

// Set to 1 only if GPS_Data carries course-over-ground in degrees (0..360,
// true north). It gates the SPEED half of the lever-arm correction, which
// needs a direction to subtract a velocity from -- a bare scalar speed has
// none. The POSITION half is always applied and does not depend on this.
#define GPS_DATA_HAS_COURSE       0

/*
 * Frontend for the NEO-M8N. Same contract as Imu, Magnetometer and Barometer:
 * update() reads once and keeps the result, getData() returns that stored fix,
 * a failed read keeps the previous one, and getData() reports ERROR without
 * touching the caller's buffer until the first good fix has arrived.
 *
 * It screens the fix before storing it, for the same reason Barometer does.
 * A position is about to become the origin the whole navigation solution is
 * referenced to, and one corrupt sample does not degrade a flight, it moves
 * the map. Three things are rejected:
 *
 *   - coordinates outside the actual range of latitude and longitude,
 *   - altitude or speed no aircraft this is going on could produce,
 *   - exactly 0.000000, 0.000000 -- "Null Island", off the coast of Ghana.
 *     No receiver is ever legitimately there, and it is what a module emits
 *     when it has been asked for a position it does not have. It passes every
 *     range check, which is exactly what makes it dangerous.
 *
 * ANTENNA LEVER ARM
 * The antenna is not at the body-frame origin, and the fix it reports is the
 * antenna's, not the airframe's. update() removes that offset before storing,
 * so every consumer of getData() sees the body origin and none of them has to
 * know where the antenna is bolted. See setAntennaOffset()/setBodyState().
 */
class GPS {
public:
	explicit GPS(UART_HandleTypeDef *huart);

	GPS_StatusTypeDef init(void);

	// Antenna position in the BODY frame (FRD: X forward, Y right, Z down),
	// measured FROM the body-frame origin TO the antenna phase centre. The
	// body-frame origin is whatever point the ESEKF estimates -- make it the
	// IMU sense point, then this is the only lever arm in the vehicle.
	//
	// Up is NEGATIVE Z. That is the sign that gets fumbled, and a flipped sign
	// does not half-fix the error, it doubles it, silently. A ruler to the
	// nearest 5 mm is already far better than GPS noise.
	//
	// Optional: with no offset set the fix is stored exactly as the module
	// reported it, which is the old behaviour.
	void setAntennaOffset(const Vector3f &offsetBody);

	// The attitude and body rate the correction needs, which this class has no
	// other way to obtain. Call it every control cycle BEFORE update(), with
	// the filter's current estimate:
	//
	//     R_bn      body -> NED rotation
	//     gyroBody  bias-corrected body angular rate, rad/s
	//
	// Skipping it is safe: the correction is then skipped too, rather than run
	// with an assumed-level attitude, which would be wrong in exactly the
	// manoeuvres where the lever arm matters most.
	void setBodyState(const Matrix3f &R_bn, const Vector3f &gyroBody);

	// Drains the module and stores the fix if it is plausible.
	void update(void);

	GPS_StatusTypeDef getData(GPS_Data &data);

	// True when there is a current, plausible fix worth navigating on.
	bool isFix(void);

	// Whether the stored fix actually had the lever arm removed. False means
	// no offset was set, or no body state was available when it arrived -- the
	// stored fix is then the ANTENNA's position, not the body origin's.
	bool isLeverArmApplied(void) const { return _leverArmApplied; }

	// Fixes thrown away by the screening above. Climbing means the module or
	// the wiring is producing positions, just not real ones.
	uint32_t getRejectedFixCount(void) const { return _rejected; }

	// The driver underneath, for diagnostics.
	const NEOM8N &getSensor(void) const { return _sensor; }

private:
	static bool isPlausible(const GPS_Data &data);

	// Applied once, inside update(), to each fresh fix before it is stored --
	// never in getData(). getData() can be called many times per cycle, and a
	// correction applied there would compound with every call.
	void applyLeverArm(GPS_Data &data);

	NEOM8N _sensor;
	GPS_Data _data;
	bool _hasData;
	uint32_t _rejected;

	Vector3f _antennaOffset;
	// Meaningless until _hasBodyState; every use of them is gated on it.
	Matrix3f _R_bn;
	Vector3f _gyro;
	bool _hasBodyState;
	bool _leverArmApplied;
};

#endif /* GPS_GPS_HPP_ */