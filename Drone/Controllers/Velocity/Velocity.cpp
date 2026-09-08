/*
 * Velocity.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Velocity.hpp"

#include <math.h>   // sqrtf, tanf, asinf, atan2f, fabsf, isfinite

namespace {

float clampf(float v, float lo, float hi) {
	if (!isfinite(v)) {
		return 0.0f;
	}
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

bool finite3(const Vector3f &v) {
	return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Scales (x, y) so its length is at most `limit`. Returns true if it had to.
// The VECTOR, not each axis: clamping north and east separately would swing a
// 45 degree acceleration demand toward whichever axis was still under its cap,
// so the aircraft would accelerate in a direction nobody asked for.
bool limitLengthNE(float &x, float &y, float limit) {
	if (limit <= 0.0f) {
		x = 0.0f;
		y = 0.0f;
		return true;
	}
	const float len = sqrtf(x * x + y * y);
	if (len <= limit || len <= 0.0f) {
		return false;
	}
	const float scale = limit / len;
	x *= scale;
	y *= scale;
	return true;
}

} // namespace

Velocity::Velocity() :
		_active { false },
		_pid_n { VELOCITY_NE_P, VELOCITY_NE_I, VELOCITY_NE_D, 0.0f,
		         VELOCITY_GRAVITY_MSS * 0.5f, VELOCITY_FILT_D_HZ },
		_pid_e { VELOCITY_NE_P, VELOCITY_NE_I, VELOCITY_NE_D, 0.0f,
		         VELOCITY_GRAVITY_MSS * 0.5f, VELOCITY_FILT_D_HZ },
		_pid_d { VELOCITY_D_P, VELOCITY_D_I, VELOCITY_D_D, 0.0f,
		         VELOCITY_ACCEL_UP, VELOCITY_FILT_D_HZ },
		_vel_target { }, _vel_error { }, _accel_target { },
		_thrust_vector { 0.0f, 0.0f, -1.0f },   // straight up
		_throttle { VELOCITY_HOVER_THROTTLE },
		_roll_target { 0.0f }, _pitch_target { 0.0f }, _tilt_target { 0.0f },
		_limits { }, _lean_max { VELOCITY_MAX_LEAN_ANGLE },
		_accel_up_max { VELOCITY_ACCEL_UP }, _accel_down_max { VELOCITY_ACCEL_DOWN },
		_hover_throttle { VELOCITY_HOVER_THROTTLE }, _vert_sat_s { 0.0f } {
	// The integrator clamp has to cover the LARGER of the two vertical limits.
	// It is symmetric, so sizing it from `up` alone would bound steady-state
	// descent trim by the climb limit -- see setVerticalAccelLimits().
	_pid_d.setIMax((VELOCITY_ACCEL_UP > VELOCITY_ACCEL_DOWN)
			? VELOCITY_ACCEL_UP : VELOCITY_ACCEL_DOWN);
}

// ---------------------------------------------------------------------------
// Engaging
// ---------------------------------------------------------------------------
void Velocity::reset(const Vector3f &vel_ned) {
	if (!finite3(vel_ned)) {
		return;
	}

	// Integrators first. One wound up while this loop was not flying -- during
	// a calibration, or under a different mode -- becomes a lean angle applied
	// the instant it takes over, before any error has even been measured.
	_pid_n.reset();
	_pid_e.reset();
	_pid_d.reset();

	// Start from what the aircraft is already doing, so the first cycle sees
	// no error. Starting from zero would command a hard stop.
	_vel_target = vel_ned;
	_vel_error = Vector3f();
	_accel_target = Vector3f();

	_thrust_vector = Vector3f(0.0f, 0.0f, -1.0f);
	_throttle = _hover_throttle;
	_roll_target = 0.0f;
	_pitch_target = 0.0f;
	_tilt_target = 0.0f;
	_limits = VelocityLimits { };
	_vert_sat_s = 0.0f;
	_active = true;
}

void Velocity::setVelocityTarget(const Vector3f &vel_ned) {
	if (finite3(vel_ned)) {
		_vel_target = vel_ned;
	}
}

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------
void Velocity::update(const Vector3f &vel_ned, float yaw_rad, float dt) {
	if (!_active) {
		return;
	}
	// A bad clock or a bad estimate must not be turned into a lean angle. The
	// last outputs stand, because they are at least something the aircraft was
	// already flying.
	if (!isfinite(dt) || dt <= 0.0f || !finite3(vel_ned) || !isfinite(yaw_rad)) {
		return;
	}

	// Anti-windup uses LAST cycle's saturation, which is the only one that has
	// happened yet: the integrator is about to be advanced by an error that
	// the actuator could not act on.
	const bool was_ne_limited = _limits.accel_ne;
	const bool was_d_limited = _limits.accel_up || _limits.accel_down
			|| _limits.throttle_upper || _limits.throttle_lower;

	_limits = VelocityLimits { };
	_vel_error = _vel_target - vel_ned;

	// --- Vertical first ------------------------------------------------------
	// NED throughout: +z is DOWN, so a climb is a negative velocity and a
	// negative acceleration. This runs before the horizontal axis because the
	// lean limit below depends on its answer.
	float ad = _pid_d.update(_vel_target.z, vel_ned.z, dt, was_d_limited);
	if (ad < -_accel_up_max) {
		ad = -_accel_up_max;
		_limits.accel_up = true;
	} else if (ad > _accel_down_max) {
		ad = _accel_down_max;
		_limits.accel_down = true;
	}

	// --- Horizontal: velocity error in, acceleration out --------------------
	float an = _pid_n.update(_vel_target.x, vel_ned.x, dt, was_ne_limited);
	float ae = _pid_e.update(_vel_target.y, vel_ned.y, dt, was_ne_limited);

	// A tilt of theta buys (vertical thrust) * tan(theta) of horizontal
	// acceleration, so a cap on the lean angle is a cap on this vector -- but
	// the vertical thrust is g only at a steady hover.
	//
	// Capping against g regardless is the usual shortcut and it leaks: during
	// a DESCENT the vertical thrust is less than g, so the same horizontal
	// acceleration needs a bigger tilt, and the aircraft quietly exceeds the
	// lean angle it was promised -- exactly when it is already sinking. Using
	// the real vertical thrust makes tilt <= lean_max an invariant instead of
	// an approximation, and it earns back the extra lean a climb allows.
	const float vertical_thrust = VELOCITY_GRAVITY_MSS - ad;
	if (limitLengthNE(an, ae, vertical_thrust * tanf(_lean_max))) {
		_limits.accel_ne = true;
	}

	_accel_target = Vector3f(an, ae, ad);
	computeThrustVector();
	computeAttitudeTarget(_thrust_vector, yaw_rad, _roll_target, _pitch_target);

	// Counted AFTER computeThrustVector(), which is what sets the throttle
	// flags. Continuous, not cumulative: a hard climb saturates for a moment
	// and that means nothing, while the same flag standing for seconds on end
	// is a hover throttle the loop cannot trim to. See
	// getVerticalSaturationTime().
	const bool vert_saturated = _limits.accel_up || _limits.accel_down
			|| _limits.throttle_upper || _limits.throttle_lower;
	_vert_sat_s = vert_saturated ? (_vert_sat_s + dt) : 0.0f;
}

void Velocity::computeThrustVector(void) {
	// The one piece of arithmetic this class exists for.
	//
	// Thrust is the only force the aircraft controls, and it has to supply
	// everything gravity does not:
	//
	//     thrust_accel = desired_accel - gravity_ned
	//                  = (a_n, a_e, a_d) - (0, 0, +g)
	//
	// Hovering level that is (0, 0, -g): straight up, at one g. The z term is
	// at most -0.2g by construction (see VELOCITY_MAX_DOWN_ACCEL_FRAC), so the
	// vector always points skyward and the length is never zero.
	const Vector3f thrust_accel(_accel_target.x, _accel_target.y,
			_accel_target.z - VELOCITY_GRAVITY_MSS);

	const float magnitude = thrust_accel.length();

	// Direction: where the aircraft's own "up" has to point.
	_thrust_vector = thrust_accel / magnitude;

	// Tilt away from vertical. -z of a unit vector IS the cosine of that
	// angle; clamp before acos so rounding at exactly level cannot hand it an
	// out-of-domain argument.
	_tilt_target = acosf(clampf(-_thrust_vector.z, -1.0f, 1.0f));

	// Magnitude: how hard to push, scaled so that one g is a hover.
	//
	// The tilt boost falls out of this and is not applied separately. Leaning
	// 30 degrees to accelerate needs g*tan(30) horizontally, and the length of
	// (g*tan(30), 0, -g) is g/cos(30) -- exactly the 1/cos(tilt) a flight
	// controller has to add to stop the aircraft sinking when it banks.
	float throttle = _hover_throttle * (magnitude / VELOCITY_GRAVITY_MSS);
	if (throttle < 0.0f) {
		throttle = 0.0f;
		_limits.throttle_lower = true;
	} else if (throttle > 1.0f) {
		throttle = 1.0f;
		// Out of thrust. The tilt still stands: Motors gives up the collective
		// before it gives up roll and pitch, which is the right order -- an
		// aircraft that loses its attitude has lost everything.
		_limits.throttle_upper = true;
	}
	_throttle = throttle;
}

// ---------------------------------------------------------------------------
// Thrust vector -> roll and pitch
// ---------------------------------------------------------------------------
void Velocity::computeAttitudeTarget(const Vector3f &thrust_unit, float yaw_rad,
		float &roll_rad, float &pitch_rad) {
	roll_rad = 0.0f;
	pitch_rad = 0.0f;
	if (!finite3(thrust_unit) || !isfinite(yaw_rad)) {
		return;
	}

	// Body "up" is (0, 0, -1) in FRD, so the attitude we want is any R with
	//
	//     R * (0, 0, -1) = thrust_unit
	//
	// Rotating the thrust vector into the heading frame first turns that into
	// two independent scalars. With ZYX Euler and cos(roll) > 0 it comes out
	// exactly, with no small-angle assumption anywhere:
	//
	//     right   =  sin(roll)
	//     forward = -sin(pitch) * cos(roll)
	//     -z      =  cos(pitch) * cos(roll)
	//
	// The usual atan(accel / g) form is what you get by assuming the vertical
	// acceleration is exactly g. During a 2.5 m/s^2 climb it is not, and the
	// answer is out by several degrees at a large lean -- in the direction
	// that makes a climbing turn undershoot.
	const float cy = cosf(yaw_rad);
	const float sy = sinf(yaw_rad);
	const float forward = thrust_unit.x * cy + thrust_unit.y * sy;
	const float right = -thrust_unit.x * sy + thrust_unit.y * cy;

	// Right roll to accelerate right: sin(roll) is the sideways component.
	roll_rad = asinf(clampf(right, -1.0f, 1.0f));

	// atan2 of (sin(pitch)*cos(roll), cos(pitch)*cos(roll)), which is atan2 of
	// (sin(pitch), cos(pitch)) once the common positive cos(roll) cancels. The
	// negations are the two minus signs in the identities above.
	//
	// Nose DOWN to accelerate forward: a forward thrust component gives a
	// negative pitch, which is the sign convention the estimator uses.
	pitch_rad = atan2f(-forward, -thrust_unit.z);
}

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------
float Velocity::getMaxHorizontalAccel(void) const {
	return VELOCITY_GRAVITY_MSS * tanf(_lean_max);
}

void Velocity::setHorizontalGains(float kp, float ki, float kd) {
	if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
		return;
	}
	if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
		return;  // a negative gain is a sign error, and it drives divergence
	}
	_pid_n.setGains(kp, ki, kd, 0.0f);
	_pid_e.setGains(kp, ki, kd, 0.0f);
}

void Velocity::setVerticalGains(float kp, float ki, float kd) {
	if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
		return;
	}
	if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
		return;
	}
	_pid_d.setGains(kp, ki, kd, 0.0f);
}

void Velocity::setMaxLeanAngle(float rad) {
	// Below 45 degrees because tan runs away past it: at 60 the horizontal
	// demand is 1.7 g and the vertical component has dropped to half, so the
	// aircraft accelerates sideways while falling out of the sky.
	if (isfinite(rad) && rad > 0.0f && rad < 0.7853982f) {
		_lean_max = rad;
		// The horizontal integrators must not be able to hold more than the
		// lean limit can deliver, or they spend the whole time unwinding.
		const float imax = VELOCITY_GRAVITY_MSS * tanf(rad);
		_pid_n.setIMax(imax);
		_pid_e.setIMax(imax);
	}
}

void Velocity::setVerticalAccelLimits(float up, float down) {
	if (isfinite(up) && up > 0.0f) {
		_accel_up_max = up;
	}
	// Downward is capped well short of g. Past it the thrust vector would have
	// to point DOWN -- the aircraft would need to fly inverted to obey -- and
	// every "the thrust vector points skyward" assumption below breaks.
	if (isfinite(down) && down > 0.0f) {
		_accel_down_max = (down < VELOCITY_GRAVITY_MSS * VELOCITY_MAX_DOWN_ACCEL_FRAC)
				? down
				: VELOCITY_GRAVITY_MSS * VELOCITY_MAX_DOWN_ACCEL_FRAC;
	}

	// One symmetric clamp for both directions, so it has to cover the larger.
	// Sized from `up` alone -- as it was -- the integrator could never hold
	// enough to trim a steady descent when the down limit was the bigger of the
	// two, and the loop would quietly settle short of its own limit.
	_pid_d.setIMax((_accel_up_max > _accel_down_max) ? _accel_up_max : _accel_down_max);
}

void Velocity::getThrottleRange(float &lo, float &hi) const {
	// Level and steady, magnitude is |a_d - g| = g - a_d, so the extremes of
	// a_d give the extremes of the throttle. a_d is NEGATIVE climbing.
	lo = _hover_throttle * (VELOCITY_GRAVITY_MSS - _accel_down_max)
			/ VELOCITY_GRAVITY_MSS;
	hi = _hover_throttle * (VELOCITY_GRAVITY_MSS + _accel_up_max)
			/ VELOCITY_GRAVITY_MSS;
	if (lo < 0.0f) {
		lo = 0.0f;
	}
	if (hi > 1.0f) {
		hi = 1.0f;
	}
}

void Velocity::setHoverThrottle(float hover) {
	// Zero or one would make the whole throttle output degenerate: everything
	// is scaled from this, so a hover of zero can never produce any thrust at
	// all and a hover of one has no headroom to climb.
	if (isfinite(hover) && hover > 0.0f && hover < 1.0f) {
		_hover_throttle = hover;
	}
}
