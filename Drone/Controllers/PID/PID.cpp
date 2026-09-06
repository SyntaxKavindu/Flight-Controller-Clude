/*
 * PID.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "PID.hpp"

#include <cmath>

namespace {

constexpr float TWO_PI = 6.283185307179586f;

float clamp(float v, float lo, float hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

bool isValid(float v) {
	return std::isfinite(v);
}

} // namespace

PID::PID() :
		PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) {
}

PID::PID(float kp, float ki, float kd, float kff, float imax, float filt_d_hz) :
		_kp { kp }, _ki { ki }, _kd { kd }, _kff { kff },
		_imax { std::fabs(imax) }, _output_limit { 0.0f },
		_filt_t_hz { 0.0f }, _filt_e_hz { 0.0f }, _filt_d_hz { filt_d_hz },
		_target { 0.0f }, _error { 0.0f }, _measurement { 0.0f },
		_derivative { 0.0f }, _integrator { 0.0f }, _p_out { 0.0f },
		_d_out { 0.0f }, _ff_out { 0.0f }, _output { 0.0f },
		_saturated { false }, _reset_filters { true } {
}

// First-order low-pass, as the discrete form of a single RC pole:
//   alpha = dt / (dt + 1/(2*pi*fc))
// A cutoff at or below zero means "no filter", which must be alpha == 1 --
// pass the input straight through -- not alpha == 0, which would freeze the
// filter on its initial value and silently kill the signal.
float PID::lowPassAlpha(float cutoff_hz, float dt) {
	if (cutoff_hz <= 0.0f || dt <= 0.0f) {
		return 1.0f;
	}
	const float rc = 1.0f / (TWO_PI * cutoff_hz);
	return dt / (dt + rc);
}

void PID::reset(void) {
	_integrator = 0.0f;
	_target = 0.0f;
	_error = 0.0f;
	_measurement = 0.0f;
	_derivative = 0.0f;
	_p_out = 0.0f;
	_d_out = 0.0f;
	_ff_out = 0.0f;
	_output = 0.0f;
	_saturated = false;
	_reset_filters = true;
}

void PID::resetIntegrator(void) {
	_integrator = 0.0f;
}

void PID::setGains(float kp, float ki, float kd, float kff) {
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_kff = kff;
}

void PID::setIMax(float imax) {
	_imax = std::fabs(imax);
	_integrator = clamp(_integrator, -_imax, _imax);
}

void PID::setOutputLimit(float limit) {
	_output_limit = std::fabs(limit);
}

void PID::setDerivativeCutoffHz(float hz) { _filt_d_hz = (hz > 0.0f) ? hz : 0.0f; }
void PID::setTargetCutoffHz(float hz)     { _filt_t_hz = (hz > 0.0f) ? hz : 0.0f; }
void PID::setErrorCutoffHz(float hz)      { _filt_e_hz = (hz > 0.0f) ? hz : 0.0f; }

void PID::integrate(float dt, bool limit) {
	if (_ki == 0.0f || dt <= 0.0f) {
		// With no I gain there is nothing to accumulate, and leaving a stale
		// integrator behind would keep contributing through getI() and through
		// any later gain change.
		_integrator = 0.0f;
		return;
	}

	// The actuator is on its stop. Let the integrator shrink -- that is how it
	// comes back off the limit -- but never grow, or it winds up to its clamp
	// during the saturation and then has to unwind all of it before the output
	// leaves the stop. That lag is the overshoot anti-windup exists to stop.
	const bool would_shrink = (_integrator > 0.0f && _error < 0.0f)
			|| (_integrator < 0.0f && _error > 0.0f);
	if (limit && !would_shrink) {
		return;
	}

	_integrator += _error * _ki * dt;
	_integrator = clamp(_integrator, -_imax, _imax);
}

float PID::finish(float target, float dt, bool limit) {
	integrate(dt, limit);

	_p_out = _error * _kp;
	_d_out = _derivative * _kd;
	_ff_out = target * _kff; // feed-forward rides the TARGET, not the error

	float out = _p_out + _integrator + _d_out + _ff_out;

	_saturated = false;
	if (_output_limit > 0.0f) {
		const float clamped = clamp(out, -_output_limit, _output_limit);
		_saturated = (clamped != out);
		out = clamped;
	}

	_output = out;
	return out;
}

float PID::update(float target, float measurement, float dt, bool limit) {
	// A NaN reaching the integrator is permanent: every later output is NaN
	// and no amount of good input recovers it. Refuse the sample instead.
	if (!isValid(target) || !isValid(measurement) || !isValid(dt) || dt <= 0.0f) {
		return _output;
	}

	if (_reset_filters) {
		// Seed from the first sample rather than filtering toward it from
		// zero, which would read as a full-scale step and produce a huge false
		// derivative on the first cycle after every arm.
		_reset_filters = false;
		_target = target;
		_measurement = measurement;
		_error = _target - _measurement;
		_derivative = 0.0f;
		return finish(_target, dt, limit);
	}

	_target += lowPassAlpha(_filt_t_hz, dt) * (target - _target);

	const float error = _target - measurement;
	_error += lowPassAlpha(_filt_e_hz, dt) * (error - _error);

	// Derivative of the MEASUREMENT, negated -- not of the error. See the note
	// in the header: differentiating the error makes a setpoint step produce a
	// one-sample spike that slams the output.
	//
	// The low-pass is applied to the measurement BEFORE differencing, rather
	// than to the derivative after: differentiating raw noise and then trying
	// to clean up the result is strictly worse, because the differencing has
	// already multiplied the noise by 1/dt. Same pole either way.
	const float measurement_prev = _measurement;
	_measurement += lowPassAlpha(_filt_d_hz, dt) * (measurement - _measurement);
	_derivative = -(_measurement - measurement_prev) / dt;

	return finish(_target, dt, limit);
}

float PID::updateError(float error, float dt, bool limit) {
	if (!isValid(error) || !isValid(dt) || dt <= 0.0f) {
		return _output;
	}

	if (_reset_filters) {
		_reset_filters = false;
		_error = error;
		_derivative = 0.0f;
		_target = 0.0f;
		return finish(0.0f, dt, limit);
	}

	const float error_prev = _error;
	_error += lowPassAlpha(_filt_e_hz, dt) * (error - _error);

	// Only the error exists here, so the derivative has to come from it. Its
	// caller is an attitude loop whose target is itself the smoothed output of
	// an outer loop, so there is no step to kick against.
	const float raw_derivative = (_error - error_prev) / dt;
	_derivative += lowPassAlpha(_filt_d_hz, dt) * (raw_derivative - _derivative);

	// No target to feed forward from -- the caller owns that if it wants it.
	return finish(0.0f, dt, limit);
}
