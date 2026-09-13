/*
 * Attitude.cpp
 *
 *  Created on: Sep 13, 2026
 *      Author: KAVINDU
 *
 * See Attitude.hpp for what this loop is and why it is shaped the way it is.
 */

#include "Attitude.hpp"

namespace {

// Below this a vector or a quaternion has no usable direction, and every
// normalisation in this file would divide by it.
const float ATT_EPS = 1e-6f;
const float ATT_PI  = 3.14159265358979f;

bool finiteF(float v) { return std::isfinite(v); }

bool finiteV(const Vector3f &v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool finiteQ(const Quaternionf &q)
{
	return std::isfinite(q.w) && std::isfinite(q.x)
	    && std::isfinite(q.y) && std::isfinite(q.z);
}

float quatLength(const Quaternionf &q)
{
	return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Caller must have checked the length; this does not re-check.
Quaternionf quatNormalize(const Quaternionf &q)
{
	const float n = quatLength(q);
	return Quaternionf(q.w / n, q.x / n, q.y / n, q.z / n);
}

float clampf(float v, float lo, float hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

// Scale the (x, y) PAIR so its length fits `limit`, and say whether it had to.
//
// A pair, not two independent clamps: roll and pitch are two components of one
// rotation, and clamping them separately would let a demand at 45 degrees keep
// sqrt(2) times the authority of one along an axis -- the aircraft would roll
// harder diagonally than it ever does cardinally, which is not a property any
// airframe has.
bool limitLengthXY(float &x, float &y, float limit)
{
	if (!(limit >= 0.0f)) return false;   // no usable cap; leave the pair alone

	const float len = sqrtf(x * x + y * y);
	if (len <= limit || len < ATT_EPS) return false;

	const float scale = limit / len;
	x *= scale;
	y *= scale;
	return true;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Attitude::Attitude() :
		_active(false),
		_p_roll(ATTITUDE_P_ROLL),
		_p_pitch(ATTITUDE_P_PITCH),
		_p_yaw(ATTITUDE_P_YAW),
		_q_target(),
		_heading_target(0.0f),
		_rate_target(),
		_att_error(),
		_tilt_error(0.0f),
		_yaw_error(0.0f),
		_yaw_priority(1.0f),
		_ff_body(),
		_ff_supplied(),
		_ff_has_supplied(false),
		_limits(),
		_rate_rp_max(ATTITUDE_RATE_RP_MAX),
		_rate_yaw_max(ATTITUDE_RATE_YAW_MAX),
		_accel_rp_max(ATTITUDE_ACCEL_RP_MAX),
		_accel_yaw_max(ATTITUDE_ACCEL_YAW_MAX),
		_yaw_priority_angle(ATTITUDE_YAW_PRIORITY_ANGLE),
		_rejected(0)
{
	_limits = AttitudeLimits();
}

// ---------------------------------------------------------------------------
// Engaging
// ---------------------------------------------------------------------------

void Attitude::reset(const Quaternionf &attitude)
{
	_rate_target = Vector3f();
	_att_error = Vector3f();
	_tilt_error = 0.0f;
	_yaw_error = 0.0f;
	_yaw_priority = 1.0f;
	_ff_body = Vector3f();
	_ff_supplied = Vector3f();
	_ff_has_supplied = false;
	_limits = AttitudeLimits();
	_rejected = 0;

	// Rubbish in means DISENGAGED, not a target seeded from rubbish. An
	// inactive loop commands zero, which is a state the caller can see and
	// react to; a loop holding a NaN-derived target is not.
	if (!finiteQ(attitude) || !(quatLength(attitude) > ATT_EPS)) {
		_active = false;
		return;
	}

	_q_target = quatNormalize(attitude);
	_heading_target = headingFromAttitude(_q_target);
	_active = true;
}

// ---------------------------------------------------------------------------
// Saying what attitude is wanted
// ---------------------------------------------------------------------------

void Attitude::applyTarget(const Quaternionf &q_target, float heading_rad)
{
	_q_target = quatNormalize(q_target);
	_heading_target = wrapPi(heading_rad);
}

void Attitude::setThrustVectorHeading(const Vector3f &thrust_ned,
		float heading_rad)
{
	if (!finiteV(thrust_ned) || !finiteF(heading_rad)
			|| !(thrust_ned.length() > ATT_EPS)) {
		_rejected++;
		return;
	}

	applyTarget(attitudeFromThrustVector(thrust_ned, heading_rad), heading_rad);
}

void Attitude::setEuler(float roll_rad, float pitch_rad, float heading_rad)
{
	if (!finiteF(roll_rad) || !finiteF(pitch_rad) || !finiteF(heading_rad)) {
		_rejected++;
		return;
	}

	applyTarget(attitudeFromEuler(roll_rad, pitch_rad, heading_rad),
			heading_rad);
}

void Attitude::setTargetAngularVelocity(const Vector3f &ang_vel_body)
{
	if (!finiteV(ang_vel_body)) {
		_rejected++;
		return;
	}

	_ff_supplied = ang_vel_body;
	_ff_has_supplied = true;
}

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------

void Attitude::update(const Quaternionf &attitude, float dt)
{
	// `dt` is validated but not otherwise used: with no differencing and no
	// integrator, every term below is a function of the error alone. It stays
	// in the signature because a non-positive or non-finite dt means the clock
	// upstream is broken, and a control loop must refuse that sample rather
	// than act on it -- the same contract every other loop in the cascade has.
	if (!finiteQ(attitude) || !finiteF(dt) || dt <= 0.0f
			|| !(quatLength(attitude) > ATT_EPS)) {
		_rejected++;   // outputs HOLD; see getRejectedCount()
		return;
	}

	// Consumed whether or not the loop is engaged: a feed-forward that
	// survives into a later cycle is a rate command nobody asked for.
	const bool have_ff = _ff_has_supplied;
	const Vector3f ff_target = _ff_supplied;
	_ff_has_supplied = false;
	_ff_supplied = Vector3f();

	if (!_active) {
		_rate_target = Vector3f();
		_ff_body = Vector3f();
		return;
	}

	_limits = AttitudeLimits();

	// One rotation from where the aircraft is to where it should be, in BODY
	// axes -- the frame the rate controller commands in.
	const Quaternionf q_meas = quatNormalize(attitude);
	const Quaternionf q_err = q_meas.conjugate() * _q_target;

	Vector3f tilt;
	float yaw_err;
	decompose(q_err, tilt, yaw_err);

	_tilt_error = sqrtf(tilt.x * tilt.x + tilt.y * tilt.y);  // tilt.z is 0
	_yaw_error = yaw_err;
	_att_error = Vector3f(tilt.x, tilt.y, yaw_err);

	// Heading is the degree of freedom that gets given up, and only this one:
	// the tilt terms below are never faded, because tilt is what keeps the
	// aircraft the right way up.
	_yaw_priority = yawPriority(_tilt_error);
	_limits.yaw_priority = (_yaw_priority < 1.0f);

	float wx = _p_roll * tilt.x;
	float wy = _p_pitch * tilt.y;
	float wz = _p_yaw * yaw_err * _yaw_priority;

	// Cap the CORRECTION at the rate it can still stop from in the angle
	// remaining. Not the output -- the feed-forward is added after this, for
	// the reason set out in the header.
	_limits.accel_rp = limitLengthXY(wx, wy,
			stoppingRate(_tilt_error, _accel_rp_max));

	const float yaw_stop = stoppingRate(yaw_err, _accel_yaw_max);
	if (fabsf(wz) > yaw_stop) {
		wz = (wz < 0.0f) ? -yaw_stop : yaw_stop;
		_limits.accel_yaw = true;
	}

	// The rate the TARGET is already turning at, rotated out of the target's
	// body frame into the current one. Zero unless a caller supplied it --
	// there is deliberately no differencing fallback.
	_ff_body = have_ff ? q_err.rotate(ff_target) : Vector3f();

	wx += _ff_body.x;
	wy += _ff_body.y;
	wz += _ff_body.z;

	// The rate limits DO apply to the total: they are what the airframe can
	// actually be asked to do, feed-forward included.
	_limits.rate_rp = limitLengthXY(wx, wy, _rate_rp_max);

	if (fabsf(wz) > _rate_yaw_max) {
		wz = (wz < 0.0f) ? -_rate_yaw_max : _rate_yaw_max;
		_limits.rate_yaw = true;
	}

	_rate_target = Vector3f(wx, wy, wz);
}

float Attitude::yawPriority(float tilt_error) const
{
	if (!(_yaw_priority_angle > 0.0f)) return 1.0f;
	if (tilt_error <= _yaw_priority_angle) return 1.0f;
	if (tilt_error >= 2.0f * _yaw_priority_angle) return 0.0f;

	return 1.0f - (tilt_error - _yaw_priority_angle) / _yaw_priority_angle;
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

Vector3f Attitude::getThrustVectorTarget(void) const
{
	return thrustVectorOf(_q_target);
}

// ---------------------------------------------------------------------------
// Tuning. Every setter REFUSES a value that would break the loop rather than
// storing it -- a zero gain is a loop that does not fly, and a negative one is
// a loop that flies away from its target.
// ---------------------------------------------------------------------------

void Attitude::setGains(float p_roll, float p_pitch, float p_yaw)
{
	if (finiteF(p_roll)  && p_roll  >= 0.0f) _p_roll  = p_roll;
	if (finiteF(p_pitch) && p_pitch >= 0.0f) _p_pitch = p_pitch;
	if (finiteF(p_yaw)   && p_yaw   >= 0.0f) _p_yaw   = p_yaw;
}

void Attitude::setRateLimits(float rp_max, float yaw_max)
{
	if (finiteF(rp_max)  && rp_max  > 0.0f) _rate_rp_max  = rp_max;
	if (finiteF(yaw_max) && yaw_max > 0.0f) _rate_yaw_max = yaw_max;
}

void Attitude::setAccelLimits(float rp_max, float yaw_max)
{
	if (finiteF(rp_max)  && rp_max  > 0.0f) _accel_rp_max  = rp_max;
	if (finiteF(yaw_max) && yaw_max > 0.0f) _accel_yaw_max = yaw_max;
}

void Attitude::setYawPriorityAngle(float rad)
{
	if (finiteF(rad) && rad > 0.0f) _yaw_priority_angle = rad;
}

// ---------------------------------------------------------------------------
// The rotation maths
// ---------------------------------------------------------------------------

float Attitude::wrapPi(float rad)
{
	if (!finiteF(rad)) return 0.0f;

	while (rad >  ATT_PI) rad -= 2.0f * ATT_PI;
	while (rad < -ATT_PI) rad += 2.0f * ATT_PI;
	return rad;
}

Quaternionf Attitude::attitudeFromEuler(float roll_rad, float pitch_rad,
		float yaw_rad)
{
	if (!finiteF(roll_rad) || !finiteF(pitch_rad) || !finiteF(yaw_rad)) {
		return Quaternionf();
	}

	// Bit-for-bit ESEKF::quaternionFromEuler(). The two MUST agree: this class
	// is handed the estimator's quaternion and hands back angles measured in
	// the same convention, and a mismatch would be a silent frame error rather
	// than anything that fails to build.
	const float cr = cosf(roll_rad  * 0.5f), sr = sinf(roll_rad  * 0.5f);
	const float cp = cosf(pitch_rad * 0.5f), sp = sinf(pitch_rad * 0.5f);
	const float cy = cosf(yaw_rad   * 0.5f), sy = sinf(yaw_rad   * 0.5f);

	const Quaternionf q(cr * cp * cy + sr * sp * sy,
	                    sr * cp * cy - cr * sp * sy,
	                    cr * sp * cy + sr * cp * sy,
	                    cr * cp * sy - sr * sp * cy);

	return quatNormalize(q);
}

Quaternionf Attitude::attitudeFromThrustVector(const Vector3f &thrust_ned,
		float heading_rad)
{
	if (!finiteV(thrust_ned) || !finiteF(heading_rad)) return Quaternionf();

	const float n = thrust_ned.length();
	if (!(n > ATT_EPS)) return Quaternionf();

	// Thrust acts along body -Z, so the BODY Z axis in NED is the negated,
	// normalised thrust direction. Call it (a, b, c) and it is the third
	// column of the ZYX rotation matrix:
	//
	//   a = cos(h) sin(p) cos(r) + sin(h) sin(r)
	//   b = sin(h) sin(p) cos(r) - cos(h) sin(r)
	//   c = cos(p) cos(r)
	//
	// Rotating (a, b) by -h separates them:  -a sin(h) + b cos(h) = -sin(r)
	// and  a cos(h) + b sin(h) = sin(p) cos(r),  which is the solve below.
	//
	// The thrust direction is a CONSTRAINT here, not something `heading_rad`
	// rotates. Every heading yields a different (roll, pitch) naming the SAME
	// direction in the world -- pitch -10 at heading 0, roll -10 at heading
	// 90 -- which is exactly why a heading target that leads the aircraft
	// cannot disturb where the thrust points.
	const float a = -thrust_ned.x / n;
	const float b = -thrust_ned.y / n;
	const float c = -thrust_ned.z / n;

	const float sh = sinf(heading_rad);
	const float ch = cosf(heading_rad);

	const float roll  = asinf(clampf(a * sh - b * ch, -1.0f, 1.0f));
	const float pitch = atan2f(a * ch + b * sh, c);

	return attitudeFromEuler(roll, pitch, heading_rad);
}

float Attitude::headingFromAttitude(const Quaternionf &q)
{
	if (!finiteQ(q) || !(quatLength(q) > ATT_EPS)) return 0.0f;

	const Quaternionf n = quatNormalize(q);

	// The ZYX yaw, matching ESEKF::eulerFromQuaternion().z -- NOT the bearing
	// of the projected body X axis, which is a different angle once the
	// airframe is pitched and would disagree with attitudeFromEuler().
	return atan2f(2.0f * (n.w * n.z + n.x * n.y),
	              1.0f - 2.0f * (n.y * n.y + n.z * n.z));
}

Vector3f Attitude::thrustVectorOf(const Quaternionf &q)
{
	if (!finiteQ(q) || !(quatLength(q) > ATT_EPS)) {
		return Vector3f(0.0f, 0.0f, -1.0f);
	}

	return quatNormalize(q).rotate(Vector3f(0.0f, 0.0f, -1.0f));
}

Vector3f Attitude::rotationVector(const Quaternionf &q)
{
	if (!finiteQ(q) || !(quatLength(q) > ATT_EPS)) return Vector3f();

	Quaternionf n = quatNormalize(q);

	// q and -q are the same ATTITUDE but name rotations differing by a full
	// turn. Only the short one is a correction worth commanding: the other
	// would send the aircraft the long way round to the place it already is.
	if (n.w < 0.0f) n = Quaternionf(-n.w, -n.x, -n.y, -n.z);

	const float s = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
	if (s < ATT_EPS) {
		// sin(t/2) ~ t/2 here, so the axis-angle vector is just 2v, and this
		// branch avoids the 0/0 the general form would hit.
		return Vector3f(2.0f * n.x, 2.0f * n.y, 2.0f * n.z);
	}

	const float angle = 2.0f * atan2f(s, n.w);
	return Vector3f(n.x / s * angle, n.y / s * angle, n.z / s * angle);
}

void Attitude::decompose(const Quaternionf &q_err, Vector3f &tilt,
		float &yaw_rad)
{
	tilt = Vector3f();
	yaw_rad = 0.0f;

	if (!finiteQ(q_err) || !(quatLength(q_err) > ATT_EPS)) return;

	const Quaternionf qe = quatNormalize(q_err);

	// Where the TARGET wants the thrust axis, expressed in the CURRENT body
	// frame. Using body +Z (not -Z) keeps the algebra in the same sense as the
	// rotation matrix's third column.
	const Vector3f zt = qe.rotate(Vector3f(0.0f, 0.0f, 1.0f));

	// Shortest rotation taking the current body Z onto it. The axis is
	// cross((0,0,1), zt) = (-zt.y, zt.x, 0), which has no Z component by
	// construction -- that is what makes this the pure TILT part.
	float ax = -zt.y;
	float ay =  zt.x;
	const float s = sqrtf(ax * ax + ay * ay);
	const float c = clampf(zt.z, -1.0f, 1.0f);
	const float angle = atan2f(s, c);

	Quaternionf q_tilt;   // identity

	if (s > ATT_EPS) {
		ax /= s;
		ay /= s;
		tilt = Vector3f(ax * angle, ay * angle, 0.0f);

		const float h = sinf(angle * 0.5f);
		q_tilt = Quaternionf(cosf(angle * 0.5f), ax * h, ay * h, 0.0f);
	} else if (c < 0.0f) {
		// Thrust axis exactly inverted: every axis in the XY plane is an
		// equally short way round, so the shortest rotation is not unique.
		// Pick body X. Any choice is arbitrary; a deterministic arbitrary one
		// beats the 0/0 the general form would produce.
		tilt = Vector3f(ATT_PI, 0.0f, 0.0f);
		q_tilt = Quaternionf(0.0f, 1.0f, 0.0f, 0.0f);
	}
	// else: already aligned, tilt stays zero and q_tilt stays identity.

	// Whatever is left once the tilt is taken out is a pure rotation about
	// body Z -- the heading error, and the ONLY part of the error that a
	// heading target running ahead of the aircraft can land in.
	const Quaternionf q_yaw = q_tilt.conjugate() * qe;
	yaw_rad = wrapPi(2.0f * atan2f(q_yaw.z, q_yaw.w));
}

float Attitude::stoppingRate(float angle_error, float accel_max)
{
	if (!finiteF(angle_error) || !finiteF(accel_max) || accel_max <= 0.0f) {
		return 0.0f;
	}

	return sqrtf(2.0f * accel_max * fabsf(angle_error));
}
