/*
 * Position.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Position.hpp"

#include <math.h>   // sqrtf, fabsf, isfinite

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

// A finite input, or zero. Every entry point screens its arguments here: a NaN
// that reaches _pos_desired is permanent, because every later comparison
// against it is false and nothing can pull it back.
bool finite3(const Vector3f &v) {
	return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Scales (x, y) so its length is at most `limit`. Returns true if it had to.
// Scaling the VECTOR rather than each axis is what keeps a diagonal demand
// pointing where it was aimed -- clamping x and y separately turns a 45 degree
// demand into something else whenever only one axis is over.
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

Position::Position() :
		_active { false }, _pos_desired { }, _pos_offset { }, _pos_offset_target { },
		_vel_desired { }, _vel_offset { }, _pos_measured { }, _pos_error { },
		_vel_target { }, _limits { },
		_p_ne { POSITION_P_NE }, _p_d { POSITION_P_D },
		_speed_ne { POSITION_SPEED_NE }, _speed_up { POSITION_SPEED_UP },
		_speed_down { POSITION_SPEED_DOWN }, _accel_ne { POSITION_ACCEL_NE },
		_accel_d { POSITION_ACCEL_D }, _max_error_ne { POSITION_MAX_ERROR_NE },
		_max_error_d { POSITION_MAX_ERROR_D } {
}

// ---------------------------------------------------------------------------
// The square-root controller
//
// The whole controller is this function. Everything else is bookkeeping.
// ---------------------------------------------------------------------------
float Position::sqrtController(float error, float p, float accel_max, float dt) {
	if (!isfinite(error)) {
		return 0.0f;
	}

	float rate;

	if (accel_max <= 0.0f) {
		// No deceleration limit given, so there is nothing to shape against.
		rate = error * p;
	} else if (p <= 0.0f) {
		// No proportional gain: pure "fastest speed I can still stop from".
		// v = sqrt(2 * a * d) is just the constant-acceleration result solved
		// for velocity.
		rate = (error >= 0.0f) ? sqrtf(2.0f * accel_max * error)
		                       : -sqrtf(2.0f * accel_max * -error);
	} else {
		// Both. Inside `linear_dist` the P term is the gentler of the two, so
		// use it; outside, the square root is what keeps the approach
		// stoppable. The two meet with matching value AND slope at the join --
		// that is what the (linear_dist / 2) term is for, and without it the
		// output steps every time the vehicle crosses the boundary.
		const float linear_dist = accel_max / (p * p);
		if (error > linear_dist) {
			rate = sqrtf(2.0f * accel_max * (error - (linear_dist * 0.5f)));
		} else if (error < -linear_dist) {
			rate = -sqrtf(2.0f * accel_max * (-error - (linear_dist * 0.5f)));
		} else {
			rate = error * p;
		}
	}

	// Never ask for more than lands exactly on the target next cycle. Without
	// this the final step of an approach overshoots, the error changes sign,
	// and the controller sits there buzzing across the target forever.
	if (dt > 0.0f && isfinite(dt)) {
		const float max_rate = fabsf(error) / dt;
		rate = clampf(rate, -max_rate, max_rate);
	}
	return rate;
}

void Position::sqrtControllerNE(float error_n, float error_e, float p,
		float accel_max, float dt, float &out_n, float &out_e) {
	out_n = 0.0f;
	out_e = 0.0f;
	if (!isfinite(error_n) || !isfinite(error_e)) {
		return;
	}

	// Shape the DISTANCE, then point the result back along the error. Running
	// the 1D controller on each axis separately would let them decelerate on
	// their own schedules, and the path to a diagonal target would bow away
	// from the straight line between here and there.
	const float dist = sqrtf(error_n * error_n + error_e * error_e);
	if (dist <= 0.0f) {
		return;
	}
	const float rate = sqrtController(dist, p, accel_max, dt);
	out_n = error_n * (rate / dist);
	out_e = error_e * (rate / dist);
}

// ---------------------------------------------------------------------------
// Engaging
// ---------------------------------------------------------------------------
void Position::reset(const Vector3f &pos_ned) {
	if (!finite3(pos_ned)) {
		return;
	}
	_pos_desired = pos_ned;
	_pos_offset = Vector3f();
	_pos_offset_target = Vector3f();
	_vel_desired = Vector3f();
	_vel_offset = Vector3f();
	_pos_error = Vector3f();
	_vel_target = Vector3f();
	_pos_measured = pos_ned;
	_limits = PositionLimits { };
	_active = true;
}

// ---------------------------------------------------------------------------
// Moving the target
// ---------------------------------------------------------------------------
void Position::setDesiredPosition(const Vector3f &pos_ned) {
	if (finite3(pos_ned)) {
		_pos_desired = pos_ned;
	}
}

void Position::inputVelocity(const Vector3f &vel_ned, float dt) {
	if (!_active || !isfinite(dt) || dt <= 0.0f) {
		return;
	}

	// A NaN from a stick or a mode is a demand to hold, not a demand to do
	// something arbitrary -- and it must never reach the integrator.
	const Vector3f request = finite3(vel_ned) ? vel_ned : Vector3f();

	// --- Shape the horizontal demand ---------------------------------------
	// The change is limited as a VECTOR: a diagonal stick slam would otherwise
	// get the acceleration limit on each axis and so sqrt(2) times as much
	// acceleration as a straight one.
	float dn = request.x - _vel_desired.x;
	float de = request.y - _vel_desired.y;
	limitLengthNE(dn, de, _accel_ne * dt);
	_vel_desired.x += dn;
	_vel_desired.y += de;

	// --- and the vertical --------------------------------------------------
	const float dd_max = _accel_d * dt;
	_vel_desired.z += clampf(request.z - _vel_desired.z, -dd_max, dd_max);

	// Hold the shaped velocity inside the speed limits too, or a stick held
	// hard over would ramp the feed-forward past what the aircraft may fly and
	// the target would slide away faster than it can be followed.
	limitLengthNE(_vel_desired.x, _vel_desired.y, _speed_ne);
	_vel_desired.z = clampf(_vel_desired.z, -_speed_up, _speed_down);

	// The target slides at the SHAPED velocity, not the requested one. That is
	// what makes the feed-forward and the target agree: the position term is
	// then correcting only real tracking error, not the difference between
	// what was asked for and what was allowed.
	_pos_desired = _pos_desired + _vel_desired * dt;

	// And it is only ever the STICK that can run the target away, which is why
	// the cap lives here and not in update(). A mission leg set through
	// setDesiredPosition() is a hundred metres away on purpose; capping that
	// would quietly move the waypoint to five metres ahead of the aircraft and
	// call it arrived.
	capPositionError();
}

void Position::setOffsetTarget(const Vector3f &offset_ned) {
	if (finite3(offset_ned)) {
		_pos_offset_target = offset_ned;
	}
}

void Position::setOffset(const Vector3f &offset_ned) {
	if (finite3(offset_ned)) {
		_pos_offset = offset_ned;
		_pos_offset_target = offset_ned;
		_vel_offset = Vector3f();
	}
}

// ---------------------------------------------------------------------------
// Run
// ---------------------------------------------------------------------------
void Position::update(const Vector3f &pos_ned, float dt) {
	if (!_active) {
		return;
	}
	// A bad clock or a bad position must not be turned into a velocity demand.
	// Holding the last output is the conservative answer: it is at least a
	// number the aircraft was already flying.
	if (!isfinite(dt) || dt <= 0.0f || !finite3(pos_ned)) {
		return;
	}

	// pos_error is owned by inputVelocity(), which is where the target can run
	// away; clearing it here would wipe the flag between the two calls.
	_limits.speed_ne = false;
	_limits.speed_up = false;
	_limits.speed_down = false;

	_pos_measured = pos_ned;

	slewOffset(dt);

	_pos_error = (_pos_desired + _pos_offset) - pos_ned;

	// --- The position correction -------------------------------------------
	float corr_n = 0.0f;
	float corr_e = 0.0f;
	sqrtControllerNE(_pos_error.x, _pos_error.y, _p_ne, _accel_ne, dt, corr_n, corr_e);
	const float corr_d = sqrtController(_pos_error.z, _p_d, _accel_d, dt);

	// --- Plus what is already known to be needed ---------------------------
	// The feed-forward is the pilot's demand and the offset's own motion. The
	// correction above only has to make up the difference, which is why a
	// well-fed-forward loop sits at almost zero position error while moving.
	_vel_target = Vector3f(_vel_desired.x + _vel_offset.x + corr_n,
	                       _vel_desired.y + _vel_offset.y + corr_e,
	                       _vel_desired.z + _vel_offset.z + corr_d);

	limitVelocityTarget();
}

void Position::slewOffset(float dt) {
	// The offset arrives as a step -- "climb one metre" -- and stepping the
	// target is a step velocity demand. Slew it in under the same limits as
	// everything else, and hand its rate to the position loop as feed-forward
	// so the aircraft does not have to fall behind the moving offset first.
	const Vector3f error = _pos_offset_target - _pos_offset;

	float vn = 0.0f;
	float ve = 0.0f;
	sqrtControllerNE(error.x, error.y, _p_ne, _accel_ne, dt, vn, ve);
	limitLengthNE(vn, ve, _speed_ne);

	float vd = sqrtController(error.z, _p_d, _accel_d, dt);
	vd = clampf(vd, -_speed_up, _speed_down);

	_vel_offset = Vector3f(vn, ve, vd);
	_pos_offset = _pos_offset + _vel_offset * dt;
}

void Position::capPositionError(void) {
	_limits.pos_error = false;

	// See the note on target runaway. The target is dragged along with the
	// aircraft rather than allowed to run ahead of it, so the error can never
	// grow past what one deceleration can absorb, however long the aircraft
	// spends unable to keep up.
	//
	// _pos_measured is from the last update(). One cycle stale is the right
	// amount of stale: the alternative is making the caller pass a position to
	// a function whose whole job is to take a stick reading.
	const Vector3f error = (_pos_desired + _pos_offset) - _pos_measured;

	const float dist_ne = sqrtf(error.x * error.x + error.y * error.y);
	if (_max_error_ne > 0.0f && dist_ne > _max_error_ne) {
		const float excess = dist_ne - _max_error_ne;
		_pos_desired.x -= error.x * (excess / dist_ne);
		_pos_desired.y -= error.y * (excess / dist_ne);
		_limits.pos_error = true;
	}

	if (_max_error_d > 0.0f && fabsf(error.z) > _max_error_d) {
		const float excess = fabsf(error.z) - _max_error_d;
		_pos_desired.z -= (error.z >= 0.0f) ? excess : -excess;
		_limits.pos_error = true;
	}
}

void Position::limitVelocityTarget(void) {
	if (limitLengthNE(_vel_target.x, _vel_target.y, _speed_ne)) {
		_limits.speed_ne = true;
	}

	// NED: down is positive, so climbing is bounded below by -_speed_up and
	// descending above by +_speed_down. Two one-sided limits, not one
	// symmetric one -- a multirotor descends into its own downwash and does it
	// badly, so the two numbers are genuinely different.
	if (_vel_target.z < -_speed_up) {
		_vel_target.z = -_speed_up;
		_limits.speed_up = true;
	} else if (_vel_target.z > _speed_down) {
		_vel_target.z = _speed_down;
		_limits.speed_down = true;
	}
}

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------
void Position::setPositionGains(float p_ne, float p_d) {
	// A negative gain is a sign error that would drive the aircraft AWAY from
	// its target, accelerating as it went. Zero is legal: it means "pure
	// square root", which is a real configuration.
	if (isfinite(p_ne) && p_ne >= 0.0f) {
		_p_ne = p_ne;
	}
	if (isfinite(p_d) && p_d >= 0.0f) {
		_p_d = p_d;
	}
}

void Position::setSpeedLimits(float speed_ne, float speed_up, float speed_down) {
	if (isfinite(speed_ne) && speed_ne > 0.0f) {
		_speed_ne = speed_ne;
	}
	if (isfinite(speed_up) && speed_up > 0.0f) {
		_speed_up = speed_up;
	}
	if (isfinite(speed_down) && speed_down > 0.0f) {
		_speed_down = speed_down;
	}
}

void Position::setAccelLimits(float accel_ne, float accel_d) {
	if (isfinite(accel_ne) && accel_ne > 0.0f) {
		_accel_ne = accel_ne;
	}
	if (isfinite(accel_d) && accel_d > 0.0f) {
		_accel_d = accel_d;
	}
}

void Position::setMaxPositionError(float max_ne, float max_d) {
	// Zero disables the cap on that axis -- for a mission leg where the target
	// really is a hundred metres away and getting there is the point.
	if (isfinite(max_ne) && max_ne >= 0.0f) {
		_max_error_ne = max_ne;
	}
	if (isfinite(max_d) && max_d >= 0.0f) {
		_max_error_d = max_d;
	}
}
