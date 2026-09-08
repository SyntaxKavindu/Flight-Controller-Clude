/*
 * PID.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * One PID for every loop in the stack: rate, attitude, velocity, position.
 * Nothing in here knows what it is controlling -- the units of the error are
 * the caller's business, and the gains carry them.
 *
 * Two ways in
 * -----------
 *   update(target, measurement, dt, limit)
 *       The usual one. Rate, velocity and position controllers all have a
 *       setpoint and a measurement of the same quantity.
 *
 *   updateError(error, dt, limit)
 *       For a controller that computes its own error because "target minus
 *       measurement" is not defined. The attitude controller is exactly this:
 *       its error comes from a quaternion difference turned into an axis-angle
 *       vector, and there is no meaningful subtraction to hand this class.
 *
 * The derivative differs between them, deliberately -- see below.
 *
 * Derivative on measurement, not on error
 * ---------------------------------------
 * update() takes the derivative of the MEASUREMENT and negates it, rather than
 * the derivative of the error. The two are the same while the setpoint is
 * still, and completely different the moment it moves: d(error)/dt contains
 * d(target)/dt, so a stick step -- an instant setpoint change -- produces an
 * enormous one-sample derivative spike that slams the output. "Derivative
 * kick" is the standard name, and on a rate controller it is felt as a jolt on
 * every sharp stick input.
 *
 * updateError() has only the error, so it has no choice but to differentiate
 * it. That is fine for its intended caller: an attitude error moves smoothly
 * because the attitude target is itself the output of an outer loop.
 *
 * Anti-windup
 * -----------
 * Two mechanisms, and both matter:
 *
 *   1. The integrator is clamped to +/-iMax.
 *   2. `limit` -- set it when the actuator is saturated (motors at full, lean
 *      angle at its cap). While it is set, the integrator may SHRINK but not
 *      grow. Clamping alone is not enough: a vehicle held against a limit for
 *      a few seconds winds the integrator to its clamp, and then has to unwind
 *      all of it before the output comes off the stop. That lag is the classic
 *      windup overshoot.
 *
 * Filtering
 * ---------
 * D on an unfiltered gyro is unusable -- differentiation multiplies noise by
 * frequency, so the D term ends up mostly hiss. A first-order low-pass on the
 * derivative is not optional on a real airframe. Filters on the target and the
 * error are available too, and default to off.
 *
 * The first call after construction or reset() seeds the filters from the
 * input instead of filtering toward it from zero, which would otherwise
 * produce a large false derivative on the first sample of every arm.
 */

#ifndef CONTROLLERS_PID_PID_HPP_
#define CONTROLLERS_PID_PID_HPP_

#include "common.hpp"

class PID {
public:
	PID();

	// kFF multiplies the TARGET, not the error: it is open-loop feed-forward,
	// the part of the output you already know is needed without waiting for an
	// error to develop. iMax bounds the integrator; 0 disables the I term
	// entirely. Cutoffs are in Hz; 0 means no filtering on that signal.
	PID(float kp, float ki, float kd, float kff, float imax, float filt_d_hz);

	// Setpoint and measurement of the same quantity. Derivative is taken on
	// the measurement, so a step in `target` does not kick the output.
	float update(float target, float measurement, float dt, bool limit = false);

	// For an error the caller computed itself. Derivative is taken on the
	// error, because that is all there is.
	float updateError(float error, float dt, bool limit = false);

	// Zeroes the integrator and re-seeds the filters from the next sample.
	// Call it whenever the loop is (re-)engaged -- on arm, or on a mode change
	// that hands this controller a different job -- or the first output
	// carries an integrator wound up during a period it was not flying.
	void reset(void);

	// Drops the integrator but keeps the filter state.
	void resetIntegrator(void);

	void setGains(float kp, float ki, float kd, float kff);
	void setIMax(float imax);
	void setOutputLimit(float limit);      // 0 disables the clamp
	void setDerivativeCutoffHz(float hz);
	void setTargetCutoffHz(float hz);
	void setErrorCutoffHz(float hz);

	float getKp(void) const { return _kp; }
	float getKi(void) const { return _ki; }
	float getKd(void) const { return _kd; }
	float getKff(void) const { return _kff; }
	float getIMax(void) const { return _imax; }

	// The last output broken into its parts. Not decoration: tuning a PID
	// without seeing which term is doing the work is guesswork, and these are
	// what a tuning telemetry line should carry.
	float getError(void) const { return _error; }
	float getP(void) const { return _p_out; }
	float getI(void) const { return _integrator; }
	float getD(void) const { return _d_out; }
	float getFF(void) const { return _ff_out; }
	float getDerivative(void) const { return _derivative; }
	float getOutput(void) const { return _output; }

	// True when the last update() had its output clipped by setOutputLimit().
	bool isSaturated(void) const { return _saturated; }

	// Samples update()/updateError() REFUSED because an input was not finite
	// or dt was not positive. Refusing is right -- a NaN reaching the
	// integrator is permanent and no later good sample recovers it -- but the
	// loop then holds its last output indefinitely, and a frozen loop is
	// otherwise indistinguishable from a healthy one commanding that value.
	//
	// So this is the evidence that the freeze happened. On a rate loop it is
	// the difference between "the aircraft is holding attitude" and "the
	// aircraft has been repeating one torque demand since the gyro died".
	// Non-zero means a sensor or a clock upstream is broken; it should read
	// zero for the life of the aircraft. Cleared by reset().
	uint32_t getRejectedCount(void) const { return _rejected; }

private:
	// Shared tail: integrate, sum, clamp. Both entry points converge here once
	// they have produced an error and a derivative.
	float finish(float target, float dt, bool limit);
	void integrate(float dt, bool limit);
	static float lowPassAlpha(float cutoff_hz, float dt);

	float _kp;
	float _ki;
	float _kd;
	float _kff;
	float _imax;
	float _output_limit;   // 0 = unclamped

	float _filt_t_hz;
	float _filt_e_hz;
	float _filt_d_hz;

	float _target;         // filtered
	float _error;          // filtered
	float _measurement;    // filtered-input history for the derivative
	float _derivative;     // filtered
	float _integrator;

	float _p_out;
	float _d_out;
	float _ff_out;
	float _output;
	bool _saturated;
	uint32_t _rejected;

	// Set by reset() and on construction: the next update seeds the filters
	// from its input rather than sliding toward it from zero.
	bool _reset_filters;
};

#endif /* CONTROLLERS_PID_PID_HPP_ */
