/*
 * Rate.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Rate.hpp"

#include <math.h>   // fabsf, isfinite

namespace {

constexpr float TWO_PI_F = 6.28318530717959f;

bool finite3(const Vector3f &v) {
	return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool validGains(float p, float i, float d) {
	if (!isfinite(p) || !isfinite(i) || !isfinite(d)) {
		return false;
	}
	// A negative gain on any term is a sign error, and on the innermost loop
	// it is the one that cannot be flown out of: the loop drives the error it
	// is meant to remove, and the aircraft departs in the time it takes to
	// notice.
	return p >= 0.0f && i >= 0.0f && d >= 0.0f;
}

} // namespace

Rate::Rate() :
		_active { false },
		_pid_roll { RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, 0.0f, RATE_IMAX,
		            RATE_FILT_D_HZ },
		_pid_pitch { RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, 0.0f, RATE_IMAX,
		             RATE_FILT_D_HZ },
		_pid_yaw { RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, 0.0f, RATE_IMAX,
		           RATE_FILT_D_HZ },
		_target { }, _gyro { }, _error { }, _output { }, _output_prev { },
		_oscillation { },
		_limits { }, _mixer_rp { false }, _mixer_yaw { false },
		_gyro_hz { RATE_FILT_GYRO_HZ }, _output_max { RATE_OUTPUT_MAX },
		_reset_filter { true }, _rejected { 0u } {
	// The mixer's range IS the output range: asking for more than 1 is asking
	// for thrust the airframe does not have, and the integrator must know that
	// the moment it happens rather than discovering it downstream.
	_pid_roll.setOutputLimit(RATE_OUTPUT_MAX);
	_pid_pitch.setOutputLimit(RATE_OUTPUT_MAX);
	_pid_yaw.setOutputLimit(RATE_OUTPUT_MAX);

	// The second and third poles; the gyro's own is applied in update(),
	// before the PIDs see anything.
	_pid_roll.setTargetCutoffHz(RATE_FILT_TARGET_HZ);
	_pid_pitch.setTargetCutoffHz(RATE_FILT_TARGET_HZ);
	_pid_yaw.setTargetCutoffHz(RATE_FILT_TARGET_HZ);
}

float Rate::lowPassAlpha(float cutoff_hz, float dt) {
	if (cutoff_hz <= 0.0f || dt <= 0.0f) {
		return 1.0f;   // no filter is alpha 1 -- pass through, never freeze
	}
	const float rc = 1.0f / (TWO_PI_F * cutoff_hz);
	return dt / (dt + rc);
}

// ---------------------------------------------------------------------------
// Engaging
// ---------------------------------------------------------------------------
void Rate::reset(void) {
	_pid_roll.reset();
	_pid_pitch.reset();
	_pid_yaw.reset();

	_target = Vector3f();
	_gyro = Vector3f();
	_error = Vector3f();
	_output = Vector3f();
	_output_prev = Vector3f();
	_oscillation = Vector3f();

	_limits = RateLimits { };
	// The mixer's report is cleared too. It describes a cycle that happened
	// before this loop was engaged, and carrying it in would freeze the
	// integrators on the first cycle of the new one.
	_mixer_rp = false;
	_mixer_yaw = false;

	_reset_filter = true;
	_rejected = 0u;
	_active = true;
}

void Rate::resetIntegrators(void) {
	_pid_roll.resetIntegrator();
	_pid_pitch.resetIntegrator();
	_pid_yaw.resetIntegrator();
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------
void Rate::setRateTarget(const Vector3f &rate_body) {
	if (finite3(rate_body)) {
		_target = rate_body;
	}
}

void Rate::setMixerSaturation(bool roll_pitch, bool yaw) {
	_mixer_rp = roll_pitch;
	_mixer_yaw = yaw;
}

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------
void Rate::update(const Vector3f &gyro_body, float dt) {
	if (!_active) {
		return;
	}

	// A dead gyro or a bad clock must not be turned into torque. The last
	// demand stands -- it is at least what the aircraft was already being
	// given -- and the refusal is counted, because at this depth a frozen loop
	// is invisible from the outside: the motors keep doing something
	// plausible, and nothing in the output says the sensor stopped.
	if (!isfinite(dt) || dt <= 0.0f || !finite3(gyro_body)) {
		_rejected++;
		return;
	}

	// The gyro's pole, ahead of everything. Both P and D read the result, so
	// filtering here rather than inside the PIDs is what keeps propeller noise
	// out of the P term as well -- PID's own filter only cleans the derivative
	// path.
	const bool first = _reset_filter;
	if (first) {
		// Seed rather than slide toward it from zero, which would read as a
		// full-scale step and put a large false derivative through D on the
		// first cycle after every arm.
		_reset_filter = false;
		_gyro = gyro_body;
	} else {
		const float alpha = lowPassAlpha(_gyro_hz, dt);
		_gyro = _gyro + (gyro_body - _gyro) * alpha;
	}

	// Anti-windup, from both directions.
	//
	// The mixer's report is about the demand it was given LAST cycle, and this
	// loop's own clipping likewise: the integrators are about to be advanced
	// by an error that the actuator -- ours or the mixer's -- could not act
	// on. The mixer flag covers roll and pitch together because that is how
	// the mixer gives them up; our own clipping is per axis, because that is
	// how it happens.
	const bool roll_limited = _mixer_rp || _limits.roll;
	const bool pitch_limited = _mixer_rp || _limits.pitch;
	const bool yaw_limited = _mixer_yaw || _limits.yaw;

	_limits = RateLimits { };
	// Reported back so the wiring between this class and Motors is visible. An
	// aircraft that visibly saturates while these stay false has a
	// setMixerSaturation() call nobody ever made.
	_limits.mixer_roll_pitch = _mixer_rp;
	_limits.mixer_yaw = _mixer_yaw;

	// Rate error in, torque out. update() rather than updateError(): the
	// derivative is taken on the MEASUREMENT, so a stick step does not put a
	// one-sample spike through D and into the motors. See the header.
	const float roll = _pid_roll.update(_target.x, _gyro.x, dt, roll_limited);
	const float pitch = _pid_pitch.update(_target.y, _gyro.y, dt, pitch_limited);
	const float yaw = _pid_yaw.update(_target.z, _gyro.z, dt, yaw_limited);

	// The PIDs clamp to the output range themselves and report it, so there is
	// no second clamp here -- one place that decides, and it is the same place
	// that decides whether the integrator may grow.
	_limits.roll = _pid_roll.isSaturated();
	_limits.pitch = _pid_pitch.isSaturated();
	_limits.yaw = _pid_yaw.isSaturated();

	// The error the loop ACTED on, which is not the raw difference of the
	// inputs: the target has been through its own pole and the measurement
	// through the gyro's.
	_error = Vector3f(_pid_roll.getError(), _pid_pitch.getError(),
			_pid_yaw.getError());

	_output = Vector3f(roll, pitch, yaw);
	updateOscillation(dt, first);
}

void Rate::updateOscillation(float dt, bool first) {
	// How fast the output is moving, averaged. Oscillation is the output
	// reversing as hard and as often as the loop can drive it, so its slew
	// rate is large where a loop flying an aircraft smoothly has a small one.
	//
	// The magnitude, not the signed rate: a ringing axis averages to nothing
	// signed, which is exactly the case this has to catch.
	if (first) {
		_oscillation = Vector3f();
	} else {
		const Vector3f slew = (_output - _output_prev) / dt;
		const float alpha = lowPassAlpha(RATE_OSC_FILT_HZ, dt);
		_oscillation.x += (fabsf(slew.x) - _oscillation.x) * alpha;
		_oscillation.y += (fabsf(slew.y) - _oscillation.y) * alpha;
		_oscillation.z += (fabsf(slew.z) - _oscillation.z) * alpha;
	}
	_output_prev = _output;
}

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------
void Rate::setRollGains(float p, float i, float d) {
	if (validGains(p, i, d)) {
		_pid_roll.setGains(p, i, d, _pid_roll.getKff());
	}
}

void Rate::setPitchGains(float p, float i, float d) {
	if (validGains(p, i, d)) {
		_pid_pitch.setGains(p, i, d, _pid_pitch.getKff());
	}
}

void Rate::setYawGains(float p, float i, float d) {
	if (validGains(p, i, d)) {
		_pid_yaw.setGains(p, i, d, _pid_yaw.getKff());
	}
}

void Rate::setFeedForward(float roll, float pitch, float yaw) {
	if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) {
		return;
	}
	_pid_roll.setGains(_pid_roll.getKp(), _pid_roll.getKi(), _pid_roll.getKd(),
			roll);
	_pid_pitch.setGains(_pid_pitch.getKp(), _pid_pitch.getKi(),
			_pid_pitch.getKd(), pitch);
	_pid_yaw.setGains(_pid_yaw.getKp(), _pid_yaw.getKi(), _pid_yaw.getKd(),
			yaw);
}

void Rate::setIMax(float imax) {
	if (isfinite(imax) && imax >= 0.0f) {
		_pid_roll.setIMax(imax);
		_pid_pitch.setIMax(imax);
		_pid_yaw.setIMax(imax);
	}
}

void Rate::setGyroCutoffHz(float hz) {
	// Zero is meaningful and means no filter: an IMU layer that already
	// low-passes the gyro should not be filtered a second time here, and
	// saying so must not be mistaken for "filter everything out".
	if (isfinite(hz) && hz >= 0.0f) {
		_gyro_hz = hz;
	}
}

void Rate::setTargetCutoffHz(float hz) {
	if (isfinite(hz) && hz >= 0.0f) {
		_pid_roll.setTargetCutoffHz(hz);
		_pid_pitch.setTargetCutoffHz(hz);
		_pid_yaw.setTargetCutoffHz(hz);
	}
}

void Rate::setDerivativeCutoffHz(float hz) {
	if (isfinite(hz) && hz >= 0.0f) {
		_pid_roll.setDerivativeCutoffHz(hz);
		_pid_pitch.setDerivativeCutoffHz(hz);
		_pid_yaw.setDerivativeCutoffHz(hz);
	}
}

void Rate::setOutputLimit(float limit) {
	// Zero would disable the clamp inside PID entirely, which on the loop that
	// feeds the mixer means handing it numbers it cannot use and losing the
	// saturation report that anti-windup depends on.
	if (isfinite(limit) && limit > 0.0f) {
		_output_max = limit;
		_pid_roll.setOutputLimit(limit);
		_pid_pitch.setOutputLimit(limit);
		_pid_yaw.setOutputLimit(limit);
	}
}
