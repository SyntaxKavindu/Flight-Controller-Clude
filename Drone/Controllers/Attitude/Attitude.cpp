/*
 * Attitude.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Attitude.hpp"

#include <math.h>   // sqrtf, sinf, cosf, acosf, atan2f, fabsf, isfinite

namespace {

constexpr float PI_F     = 3.14159265358979f;
constexpr float TWO_PI_F = 6.28318530717959f;

// Below this the rotation axis is numerically meaningless: it is the length of
// a cross product between two vectors that are all but parallel, so every
// component is rounding noise and normalising it amplifies that noise by
// 1/length. A rotation this small is not worth naming an axis for -- the
// small-angle branch handles it exactly.
constexpr float AXIS_EPS = 1e-6f;

// A quaternion shorter than this cannot be normalised into an attitude. Not a
// tolerance on "nearly unit": it is the line between a slightly stale
// quaternion, which normalises fine, and one that carries no direction at all.
constexpr float QUAT_EPS = 1e-6f;

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

bool finiteQuat(const Quaternionf &q) {
	return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

float quatNorm(const Quaternionf &q) {
	return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// The one test for "is there an attitude in this": finite, and long enough to
// normalise. A quaternion failing it is not a bad attitude, it is no attitude.
bool usableQuat(const Quaternionf &q) {
	return finiteQuat(q) && quatNorm(q) >= QUAT_EPS;
}

// Wrap to +/-pi, so a heading error is always the short way round. Without it
// a target crossing north makes the aircraft spin 358 degrees to correct 2.
float wrapPi(float rad) {
	if (!isfinite(rad)) {
		return 0.0f;
	}
	while (rad > PI_F) {
		rad -= TWO_PI_F;
	}
	while (rad < -PI_F) {
		rad += TWO_PI_F;
	}
	return rad;
}

// Returns the identity for anything that cannot be normalised, which is the
// only safe answer: a zero-length quaternion has no attitude in it, and a NaN
// one would poison every rotation it touched afterwards.
Quaternionf normalized(const Quaternionf &q) {
	if (!finiteQuat(q)) {
		return Quaternionf();
	}
	const float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n < QUAT_EPS) {
		return Quaternionf();
	}
	const float s = 1.0f / n;
	return Quaternionf(q.w * s, q.x * s, q.y * s, q.z * s);
}

Quaternionf fromAxisAngle(const Vector3f &axis_unit, float angle) {
	const float h = angle * 0.5f;
	const float s = sinf(h);
	return Quaternionf(cosf(h), axis_unit.x * s, axis_unit.y * s,
			axis_unit.z * s);
}

// Scales (x, y) so its length is at most `limit`. Returns true if it had to.
//
// The VECTOR, not each axis. Roll and pitch are two components of ONE tilt
// correction, and clamping them separately rotates it: a 45 degree correction
// with both axes over the limit comes out along the diagonal, but one with
// only x over comes out swung toward y. The aircraft would recover from an
// upset in a direction nobody asked for, and the direction would change as the
// error shrank past each axis limit in turn.
bool limitLengthXY(float &x, float &y, float limit) {
	if (!isfinite(x) || !isfinite(y)) {
		x = 0.0f;
		y = 0.0f;
		return true;
	}
	const float len = sqrtf(x * x + y * y);

	// Tested BEFORE the zero-limit case on purpose. A limit of zero with a
	// demand of zero is not a saturation -- and the stopping cap is exactly
	// that on every cycle of a settled hover, where the error and the rate it
	// allows both go to zero together. Reporting it would light a limit flag
	// and hold the integrators frozen for the whole of a perfect hover.
	if (len <= limit) {
		return false;
	}
	if (limit <= 0.0f) {
		x = 0.0f;
		y = 0.0f;
		return true;
	}
	const float scale = limit / len;
	x *= scale;
	y *= scale;
	return true;
}

// Same first-order low-pass as PID::lowPassAlpha(). A cutoff at or below zero
// means no filter, which is alpha == 1 -- pass the input straight through --
// and never alpha == 0, which would freeze the filter on its initial value.
float lowPassAlpha(float cutoff_hz, float dt) {
	if (cutoff_hz <= 0.0f || dt <= 0.0f) {
		return 1.0f;
	}
	const float rc = 1.0f / (TWO_PI_F * cutoff_hz);
	return dt / (dt + rc);
}

} // namespace

Attitude::Attitude() :
		_active { false },
		_pid_roll { ATTITUDE_P_ROLL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
		_pid_pitch { ATTITUDE_P_PITCH, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
		_pid_yaw { ATTITUDE_P_YAW, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
		_q_target { }, _q_target_prev { }, _yaw_target { 0.0f },
		_rate_target { }, _att_error { }, _tilt_error { 0.0f },
		_yaw_error { 0.0f }, _yaw_priority { 1.0f },
		_ff_target { }, _ff_body { }, _ff_supplied { },
		_ff_has_supplied { false }, _ff_seed { true }, _ff_enabled { true },
		_ff_cutoff_hz { ATTITUDE_FF_CUTOFF_HZ },
		_limits { },
		_rate_rp_max { ATTITUDE_RATE_RP_MAX },
		_rate_yaw_max { ATTITUDE_RATE_YAW_MAX },
		_accel_rp_max { ATTITUDE_ACCEL_RP_MAX },
		_accel_yaw_max { ATTITUDE_ACCEL_YAW_MAX },
		_yaw_priority_angle { ATTITUDE_YAW_PRIORITY_ANGLE },
		_rejected { 0u } {
}

// ---------------------------------------------------------------------------
// Engaging
// ---------------------------------------------------------------------------
void Attitude::reset(const Quaternionf &attitude) {
	if (!usableQuat(attitude)) {
		return;
	}
	const Quaternionf q = normalized(attitude);

	// Integrators first -- they are off by default, but a tuner who has
	// enabled one must not have it carried across a mode change, where it
	// becomes a rate demand applied before any error has been measured.
	_pid_roll.reset();
	_pid_pitch.reset();
	_pid_yaw.reset();

	// Hold where it is already pointing, so the first cycle sees zero error.
	_q_target = q;
	_q_target_prev = q;
	_yaw_target = headingFromAttitude(q);

	_rate_target = Vector3f();
	_att_error = Vector3f();
	_tilt_error = 0.0f;
	_yaw_error = 0.0f;
	_yaw_priority = 1.0f;

	// Seed rather than difference: the gap between the last cycle of the
	// previous mode and this one is not a rotation the target performed.
	_ff_target = Vector3f();
	_ff_body = Vector3f();
	_ff_supplied = Vector3f();
	_ff_has_supplied = false;
	_ff_seed = true;

	_limits = AttitudeLimits { };
	_rejected = 0u;
	_active = true;
}

// ---------------------------------------------------------------------------
// Saying what attitude is wanted
// ---------------------------------------------------------------------------
void Attitude::setAttitudeTarget(const Quaternionf &q_target) {
	if (!usableQuat(q_target)) {
		return;   // no attitude in it -- keep the last good one
	}
	_q_target = normalized(q_target);
	_yaw_target = headingFromAttitude(_q_target);
}

void Attitude::setThrustVectorHeading(const Vector3f &thrust_ned,
		float yaw_rad) {
	if (!finite3(thrust_ned) || !isfinite(yaw_rad)
			|| thrust_ned.length() < AXIS_EPS) {
		return;
	}
	_q_target = attitudeFromThrustVector(thrust_ned, yaw_rad);
	_yaw_target = wrapPi(yaw_rad);
}

void Attitude::setThrustVectorYawRate(const Vector3f &thrust_ned,
		float yaw_rate, float dt) {
	if (!isfinite(yaw_rate) || !isfinite(dt) || dt <= 0.0f) {
		return;
	}
	// Integrate from where the TARGET heading already is, not from where the
	// aircraft is: taking the measured heading here would let a heading error
	// leak into the target every cycle, and the nose would creep.
	setThrustVectorHeading(thrust_ned, wrapPi(_yaw_target + yaw_rate * dt));
}

void Attitude::setEuler(float roll_rad, float pitch_rad, float yaw_rad) {
	if (!isfinite(roll_rad) || !isfinite(pitch_rad) || !isfinite(yaw_rad)) {
		return;
	}
	_q_target = attitudeFromEuler(roll_rad, pitch_rad, yaw_rad);
	_yaw_target = wrapPi(yaw_rad);
}

void Attitude::setTargetAngularVelocity(const Vector3f &ang_vel) {
	if (!finite3(ang_vel)) {
		return;
	}
	_ff_supplied = ang_vel;
	_ff_has_supplied = true;
}

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------
void Attitude::update(const Quaternionf &attitude, float dt) {
	if (!_active) {
		return;
	}

	// A bad clock or a bad attitude estimate must not become a rate demand.
	// The last output stands -- it is at least something the aircraft was
	// already flying -- and the refusal is counted, because a frozen loop is
	// otherwise indistinguishable from a healthy one holding still.
	if (!isfinite(dt) || dt <= 0.0f || !usableQuat(attitude)) {
		_rejected++;
		// Never difference the target across a refused sample: the gap is not
		// a rotation, and dividing it by one cycle's dt would invent a rate.
		_ff_seed = true;
		return;
	}
	const Quaternionf q_m = normalized(attitude);

	// Anti-windup uses LAST cycle's saturation, which is the only one that has
	// happened yet: the integrators are about to be advanced by an error the
	// output could not act on. They are off by default, so this costs nothing
	// until someone enables one -- and then it is already right.
	const bool was_rp_limited = _limits.rate_rp || _limits.accel_rp;
	const bool was_yaw_limited = _limits.rate_yaw || _limits.accel_yaw;
	_limits = AttitudeLimits { };

	// The error: one rotation, from where the aircraft is to where it should
	// be, expressed in BODY axes -- which is the frame the rate controller
	// commands in, so its components are directly a roll, pitch and yaw
	// demand.
	//
	// q_err maps vectors from the TARGET's body frame into the current one,
	// which is what the feed-forward below needs to rotate the target's own
	// angular velocity across.
	const Quaternionf q_err = normalized(q_m.conjugate() * _q_target);

	Vector3f tilt;
	float yaw_err;
	decompose(q_err, tilt, yaw_err);

	_tilt_error = tilt.length();
	_yaw_error = yaw_err;
	_att_error = Vector3f(tilt.x, tilt.y, yaw_err);

	// Heading is the degree of freedom a multirotor can give up, and under a
	// large tilt error it must: the yaw demand and the tilt demand are made
	// from the same motor thrust. See the header.
	_yaw_priority = yawPriority(_tilt_error);
	if (_yaw_priority < 1.0f) {
		_limits.yaw_priority = true;
	}

	// Angle error in, angular rate out. updateError() rather than update():
	// there is no "target minus measurement" for an attitude, the error came
	// out of a quaternion difference.
	//
	// The priority fade is applied to the ERROR, not to the output, so that an
	// enabled yaw integrator does not wind up on a correction the controller
	// has decided not to make.
	float wx = _pid_roll.updateError(tilt.x, dt, was_rp_limited);
	float wy = _pid_pitch.updateError(tilt.y, dt, was_rp_limited);
	float wz = _pid_yaw.updateError(yaw_err * _yaw_priority, dt, was_yaw_limited);

	// Cap the CORRECTION at the rate it can still stop from. Not the total --
	// the feed-forward is added afterwards, because the stopping rate goes to
	// zero exactly when the error does and capping the sum would stall a
	// perfectly tracked moving target. See the header.
	if (_accel_rp_max > 0.0f) {
		if (limitLengthXY(wx, wy, stoppingRate(_tilt_error, _accel_rp_max))) {
			_limits.accel_rp = true;
		}
	}
	if (_accel_yaw_max > 0.0f) {
		const float stop = stoppingRate(_yaw_error, _accel_yaw_max);
		if (fabsf(wz) > stop) {
			wz = clampf(wz, -stop, stop);
			_limits.accel_yaw = true;
		}
	}

	// The rate the target is itself turning at, so tracking it does not
	// require a standing error to pay for.
	_ff_body = feedForward(q_err, dt);
	wx += _ff_body.x;
	wy += _ff_body.y;
	wz += _ff_body.z;

	// What the airframe is allowed to be asked for, applied to the total.
	if (limitLengthXY(wx, wy, _rate_rp_max)) {
		_limits.rate_rp = true;
	}
	if (fabsf(wz) > _rate_yaw_max) {
		wz = clampf(wz, -_rate_yaw_max, _rate_yaw_max);
		_limits.rate_yaw = true;
	}

	_rate_target = Vector3f(wx, wy, wz);
}

float Attitude::yawPriority(float tilt_error) const {
	if (!isfinite(tilt_error) || _yaw_priority_angle <= 0.0f) {
		return 1.0f;
	}
	const float from = _yaw_priority_angle;
	if (tilt_error <= from) {
		return 1.0f;
	}
	// Linear from full authority at `from` to none at twice it. A hard switch
	// would step the yaw demand as the tilt error wandered across the
	// threshold, which on the innermost angle loop is felt as a twitch.
	const float to = from * 2.0f;
	if (tilt_error >= to) {
		return 0.0f;
	}
	return (to - tilt_error) / from;
}

Vector3f Attitude::feedForward(const Quaternionf &q_err, float dt) {
	// Consumed whether or not it is used: a supplied rate is good for the
	// cycle it was set in, and carrying it forward would be a rate command
	// standing long after the mode that meant it stopped asking.
	const bool have_supplied = _ff_has_supplied;
	const Vector3f supplied = _ff_supplied;
	_ff_has_supplied = false;

	const Quaternionf target_now = _q_target;
	const Quaternionf target_prev = _q_target_prev;
	_q_target_prev = target_now;

	if (!_ff_enabled) {
		_ff_target = Vector3f();
		return Vector3f();
	}

	if (have_supplied) {
		// The caller knows the rate exactly. No differencing, and no filter --
		// there is nothing here to smooth, and a filter would only add the lag
		// the feed-forward exists to remove.
		_ff_target = supplied;
	} else if (_ff_seed) {
		// First cycle after engaging: there is no previous target to difference
		// against that means anything.
		_ff_seed = false;
		_ff_target = Vector3f();
	} else {
		// How far the target moved since last cycle, as a rotation in the
		// target's own frame, over the time it took. Low-passed: this is a
		// numerical derivative, and a target that steps differentiates to an
		// impulse.
		const Quaternionf dq = normalized(target_prev.conjugate() * target_now);
		const Vector3f measured = rotationVector(dq) / dt;
		if (finite3(measured)) {
			const float alpha = lowPassAlpha(_ff_cutoff_hz, dt);
			_ff_target = _ff_target + (measured - _ff_target) * alpha;
		}
	}

	// The rate is in the TARGET's body frame; the output is in the aircraft's.
	// They differ by exactly the attitude error, which is what q_err is.
	const Vector3f body = q_err.rotate(_ff_target);
	return finite3(body) ? body : Vector3f();
}

// ---------------------------------------------------------------------------
// The rotation maths
// ---------------------------------------------------------------------------
Quaternionf Attitude::attitudeFromThrustVector(const Vector3f &thrust_ned,
		float yaw_rad) {
	// Body "up" in FRD is (0, 0, -1), so this is the attitude R with
	//
	//     R * (0, 0, -1) = thrust_unit      and      heading(R) = yaw
	//
	// Rotating the thrust vector into the heading frame first turns the vector
	// equation into two independent scalars. With ZYX Euler and cos(roll) > 0
	// it comes out exactly, with no small-angle assumption anywhere:
	//
	//     right   =  sin(roll)
	//     forward = -sin(pitch) * cos(roll)
	//     -z      =  cos(pitch) * cos(roll)
	//
	// This is the same solve as Velocity::computeAttitudeTarget(), and
	// deliberately so -- the velocity controller reports the roll and pitch it
	// is asking for, and if this class flew a different attitude for the same
	// thrust vector, the number on the telemetry line and the number in the
	// air would quietly disagree. They are kept apart rather than shared so
	// that the inner loops carry no dependency on the outer ones, and pinned
	// together by a test instead.
	//
	// The tempting alternative -- the minimal rotation from straight up onto
	// the thrust vector, composed with a rotation about NED down -- gets the
	// thrust axis just as exactly and is one line shorter. It was tried and
	// dropped: a minimal tilt about a diagonal axis carries its own heading
	// with it, so the attitude it builds is out by up to 8.7 degrees of
	// heading at a 45 degree lean, against the ZYX convention the estimator,
	// the compass and every other angle in this tree are written in.
	Vector3f t(0.0f, 0.0f, -1.0f);
	const float len = thrust_ned.length();
	if (finite3(thrust_ned) && len >= AXIS_EPS) {
		t = thrust_ned / len;
	}
	const float yaw = isfinite(yaw_rad) ? yaw_rad : 0.0f;

	const float cy = cosf(yaw);
	const float sy = sinf(yaw);
	const float forward = t.x * cy + t.y * sy;
	const float right = -t.x * sy + t.y * cy;

	// Right roll to lean right: sin(roll) is the sideways component. The clamp
	// saturates at +/- 90 degrees of roll, which is a thrust axis lying in the
	// horizontal plane -- past that ZYX cannot name the attitude at all, and
	// an aircraft there has bigger problems than which Euler triple describes
	// it.
	const float roll = asinf(clampf(right, -1.0f, 1.0f));

	// atan2 of (sin(pitch)*cos(roll), cos(pitch)*cos(roll)), which is atan2 of
	// (sin(pitch), cos(pitch)) once the common cos(roll) cancels. Nose DOWN to
	// lean forward, which is the sign convention the estimator uses.
	const float pitch = atan2f(-forward, -t.z);

	return attitudeFromEuler(roll, pitch, yaw);
}

Quaternionf Attitude::attitudeFromEuler(float roll_rad, float pitch_rad,
		float yaw_rad) {
	// ZYX, the estimator's convention: q = qz(yaw) * qy(pitch) * qx(roll).
	const float cr = cosf(roll_rad * 0.5f), sr = sinf(roll_rad * 0.5f);
	const float cp = cosf(pitch_rad * 0.5f), sp = sinf(pitch_rad * 0.5f);
	const float cy = cosf(yaw_rad * 0.5f), sy = sinf(yaw_rad * 0.5f);
	return Quaternionf(
			cy * cp * cr + sy * sp * sr,
			cy * cp * sr - sy * sp * cr,
			cy * sp * cr + sy * cp * sr,
			sy * cp * cr - cy * sp * sr);
}

float Attitude::headingFromAttitude(const Quaternionf &q) {
	if (!finiteQuat(q)) {
		return 0.0f;
	}
	// The ZYX yaw, matching ESEKF::eulerFromQuaternion().
	const float siny = 2.0f * (q.w * q.z + q.x * q.y);
	const float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	if (siny == 0.0f && cosy == 0.0f) {
		return 0.0f;   // atan2(0, 0) is undefined, and pointing nowhere is north
	}
	return atan2f(siny, cosy);
}

Vector3f Attitude::rotationVector(const Quaternionf &q) {
	if (!finiteQuat(q)) {
		return Vector3f();
	}
	float w = q.w, x = q.x, y = q.y, z = q.z;

	// q and -q are the same attitude but name rotations differing by a full
	// turn. Forcing w >= 0 picks the one at most half a turn from identity --
	// the short way round, and the only one worth commanding.
	if (w < 0.0f) {
		w = -w;
		x = -x;
		y = -y;
		z = -z;
	}

	const float s = sqrtf(x * x + y * y + z * z);   // |sin(angle/2)|
	if (s < AXIS_EPS) {
		// Tiny rotation. sin(a/2) ~ a/2, so the vector part is half the answer
		// -- and the general form below would divide by very nearly zero.
		return Vector3f(x, y, z) * 2.0f;
	}
	// atan2 rather than asin or acos: it stays accurate through the whole
	// range, including the half turn where w reaches zero.
	const float angle = 2.0f * atan2f(s, w);
	return Vector3f(x, y, z) * (angle / s);
}

void Attitude::decompose(const Quaternionf &q_err, Vector3f &tilt,
		float &yaw_rad) {
	tilt = Vector3f();
	yaw_rad = 0.0f;
	if (!finiteQuat(q_err)) {
		return;
	}

	// Where the TARGET's "down" axis lies, seen from the current body frame.
	// If the thrust axis is already right this is (0, 0, 1) and the whole
	// error is heading.
	const Vector3f z_t = q_err.rotate(Vector3f(0.0f, 0.0f, 1.0f));
	if (!finite3(z_t)) {
		return;
	}

	// The shortest rotation putting body down onto it. For (0,0,1) x (a,b,c)
	// the cross product is just (-b, a, 0) and the dot product is just c, so
	// the axis and angle both fall out with no trigonometry beyond the arc
	// cosine.
	const float angle = acosf(clampf(z_t.z, -1.0f, 1.0f));
	const float s = sqrtf(z_t.x * z_t.x + z_t.y * z_t.y);   // == sin(angle)

	Quaternionf q_tilt;   // identity: thrust axis already correct
	if (s >= AXIS_EPS) {
		const Vector3f axis(-z_t.y / s, z_t.x / s, 0.0f);
		tilt = axis * angle;
		q_tilt = fromAxisAngle(axis, angle);
	} else if (z_t.z < 0.0f) {
		// Upside down with respect to the target: half a turn, about an axis
		// the geometry does not pick out. See the note in
		// attitudeFromThrustVector().
		tilt = Vector3f(angle, 0.0f, 0.0f);
		q_tilt = fromAxisAngle(Vector3f(1.0f, 0.0f, 0.0f), angle);
	}

	// What is left once the tilt is taken out is a pure rotation about body z,
	// by construction: q_err == q_tilt * q_yaw.
	const Quaternionf q_yaw = normalized(q_tilt.conjugate() * q_err);
	float w = q_yaw.w, z = q_yaw.z;
	if (w < 0.0f) {
		w = -w;
		z = -z;
	}
	// With w >= 0 this lands in +/-pi on its own: the short way round, with no
	// wrapping needed afterwards.
	yaw_rad = 2.0f * atan2f(z, w);
}

float Attitude::stoppingRate(float angle_error, float accel_max) {
	if (!isfinite(angle_error) || !isfinite(accel_max) || accel_max <= 0.0f) {
		return 0.0f;
	}
	// v^2 = 2*a*d solved for v: the constant-acceleration result, and the
	// fastest this axis can be turning and still arrive stopped.
	return sqrtf(2.0f * accel_max * fabsf(angle_error));
}

Vector3f Attitude::getThrustVectorTarget(void) const {
	// Body up is (0, 0, -1) in FRD; rotating it out to NED is where the target
	// says the thrust must point.
	return _q_target.rotate(Vector3f(0.0f, 0.0f, -1.0f));
}

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------
void Attitude::setGains(float p_roll, float p_pitch, float p_yaw) {
	if (!isfinite(p_roll) || !isfinite(p_pitch) || !isfinite(p_yaw)) {
		return;
	}
	if (p_roll < 0.0f || p_pitch < 0.0f || p_yaw < 0.0f) {
		return;   // a negative angle gain drives the aircraft away from the target
	}
	// Only the P term. I and D stay wherever a tuner has deliberately put them
	// -- see the header on why both default to off.
	_pid_roll.setGains(p_roll, _pid_roll.getKi(), _pid_roll.getKd(), 0.0f);
	_pid_pitch.setGains(p_pitch, _pid_pitch.getKi(), _pid_pitch.getKd(), 0.0f);
	_pid_yaw.setGains(p_yaw, _pid_yaw.getKi(), _pid_yaw.getKd(), 0.0f);
}

void Attitude::setRateLimits(float rp_max, float yaw_max) {
	if (isfinite(rp_max) && rp_max > 0.0f) {
		_rate_rp_max = rp_max;
	}
	if (isfinite(yaw_max) && yaw_max > 0.0f) {
		_rate_yaw_max = yaw_max;
	}
}

void Attitude::setAccelLimits(float rp_max, float yaw_max) {
	// Zero is meaningful here and means "no stopping cap": the rate limits
	// still hold, and a caller that does not know its airframe's angular
	// acceleration is better off not pretending to.
	if (isfinite(rp_max) && rp_max >= 0.0f) {
		_accel_rp_max = rp_max;
	}
	if (isfinite(yaw_max) && yaw_max >= 0.0f) {
		_accel_yaw_max = yaw_max;
	}
}

void Attitude::setYawPriorityAngle(float rad) {
	// Zero disables the fade -- heading is then corrected at full gain however
	// badly the aircraft is tilted, which is only sensible on an airframe with
	// yaw authority to spare.
	if (isfinite(rad) && rad >= 0.0f && rad < PI_F) {
		_yaw_priority_angle = rad;
	}
}

void Attitude::setFeedForwardEnabled(bool enabled) {
	_ff_enabled = enabled;
	if (!enabled) {
		_ff_target = Vector3f();
		_ff_body = Vector3f();
	} else {
		// Re-seed: the target has been moving while this was off, and the
		// first difference after switching back on would span all of it.
		_ff_seed = true;
	}
}

void Attitude::setFeedForwardCutoffHz(float hz) {
	if (isfinite(hz) && hz >= 0.0f) {
		_ff_cutoff_hz = hz;
	}
}
