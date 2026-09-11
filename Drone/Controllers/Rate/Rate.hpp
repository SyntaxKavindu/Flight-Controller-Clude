/*
 * Rate.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * The innermost loop, and the only one that touches the gyro. Everything above
 * it has been narrowing a want -- go there, at this speed, pointing that way --
 * down to one question, and this is it: how hard to twist, right now.
 *
 * Where it sits
 * -------------
 *   RC sticks -> mode -> Position -> Velocity -> Attitude -> RATE -> Motors
 *
 * In:  a body-frame angular rate target in rad/s from the attitude controller,
 *      and the measured body rate -- the bias-corrected gyro.
 * Out: three normalised torque demands in [-1, 1], which is exactly what
 *      Motors::setControlInput() takes.
 *
 * It does NOT produce the collective. Throttle comes from the velocity
 * controller or straight from a mode and goes to Motors alongside these three.
 * This class has no opinion about how hard the aircraft is pushing, only about
 * which way it is turning.
 *
 * Signs are the estimator's body frame, FRD, and they line up with the mixer's
 * without a conversion anywhere: +x is a right roll, +y is nose up, +z is nose
 * right, for the rate coming in and the torque going out alike.
 *
 * Why the units work out
 * ----------------------
 * The gains map rad/s of error onto a fraction of full torque, so P = 0.135
 * means "one radian per second of error is worth 13.5% of what the airframe
 * can twist with". Nothing here knows the aircraft's moment of inertia, the
 * thrust of a motor, or the length of an arm. It does not need to: the mixer
 * below normalises all of it away, and what is left is a number a tuner turns
 * until the aircraft feels right. That is also why these gains do not carry
 * over between airframes the way the loops above do -- Position's "one metre
 * asks for one metre per second" is true of any vehicle, and P = 0.135 is true
 * of one.
 *
 * PID, and why the full set belongs HERE
 * --------------------------------------
 * Attitude is deliberately P only. This loop is where the other two terms earn
 * their keep, and the reason is that this is the innermost loop -- the first
 * one that sees a disturbance and the last that can do anything about it.
 *
 *   I   is what trims a standing torque: a battery taped off-centre, a bent
 *       arm, a motor down a few percent, a wind that keeps pushing. Without it
 *       the aircraft flies with a permanent rate error, because a pure P
 *       controller needs an error to produce the very output that cancels it.
 *       This is the only integrator in the whole cascade that sees the
 *       disturbance directly, which is why Attitude does not have one -- two
 *       integrators winding against the same steady torque fight each other.
 *
 *   D   is damping, and it is what stops the aircraft ringing. Rate error is
 *       already the derivative of attitude error, so D here is the second
 *       derivative -- the term that anticipates. It is also the term that
 *       tears an airframe apart if it is too high, which is what the
 *       oscillation reading below is for.
 *
 * Derivative on the measurement, not on the error
 * -----------------------------------------------
 * update() rather than updateError(), and that choice is the whole reason
 * PID has two entry points. The rate target comes from a stick, and a stick
 * moves in steps. d(error)/dt contains d(target)/dt, so a flicked stick would
 * produce a one-sample derivative spike straight into the motors -- derivative
 * kick, felt as a jolt on every sharp input. Differentiating the measurement
 * instead gives the same damping with none of that, because the gyro cannot
 * step.
 *
 * Filtering is not optional here
 * ------------------------------
 * A gyro bolted to an airframe with four propellers on it is a noisy thing:
 * motor imbalance, blade passes and frame resonance all land in the signal,
 * and differentiation multiplies noise by frequency. D on a raw gyro is mostly
 * hiss, and hiss in the D term is motors heating up and an airframe buzzing.
 *
 * So there are three poles, and they do different jobs:
 *
 *   gyro cutoff     on the measurement, before anything sees it. P and D both
 *                   get a clean signal. The equivalent of ArduPilot's
 *                   INS_GYRO_FILTER, which is upstream there and here is not,
 *                   because nothing upstream of this class filters.
 *   target cutoff   on the demand, so a step from above is a ramp by the time
 *                   it reaches the motors.
 *   derivative      a second pole on the D path alone, where the noise hurts
 *                   most.
 *
 * Anti-windup has to come from the mixer
 * --------------------------------------
 * Four motors cannot always deliver what roll, pitch, yaw and collective all
 * ask for at once. When they cannot, Motors gives up yaw first, then the
 * collective, and roll and pitch last -- and it records what it gave up in
 * getLimits().
 *
 * That report has to come back here, through setMixerSaturation(), or this
 * loop integrates an error it was never allowed to correct. A quad pinned
 * against its yaw authority for a few seconds -- which is every hard climb,
 * because a climb spends the headroom yaw needs -- winds its yaw integrator to
 * the clamp, and then has to unwind all of it before the output comes off the
 * stop. The aircraft snaps round when the climb ends.
 *
 * This class cannot work that out for itself. It knows what it ASKED for, not
 * what the mixer managed, and the difference between the two is the entire
 * problem. Its own output clipping it does see, and handles.
 *
 * Either way the detection is one cycle late, and unavoidably so: the
 * integrator is advanced by an error whose output has not been clamped yet,
 * and the clamp is what proves the demand was impossible. So a saturation
 * costs one cycle of integration -- 2.5 ms at 400 Hz, a fiftieth of the clamp
 * even on an error far larger than anything the attitude loop can command --
 * and then nothing further, however long it lasts. Growing once and stopping
 * is the behaviour; creeping is the bug, and there is a test for it.
 *
 * What is NOT here
 * ----------------
 * Three things a mature rate loop grows, in the order they are likely to
 * matter:
 *
 *   Battery compensation. As a pack sags, the same demand produces less
 *   torque, so the loop quietly detunes over a flight -- sharp on a full
 *   battery, soggy on an empty one. ArduPilot scales the gains by voltage.
 *   Nothing here reads the battery yet.
 *
 *   A harmonic notch. Blade-pass noise is narrowband and moves with throttle;
 *   a notch tracking it removes far more than a low-pass can without the lag.
 *
 *   Automatic gain reduction on oscillation (ArduPilot's SMAX). The reading
 *   below measures the thing it would act on, but this class only reports it.
 *   Backing gains off automatically is a decision, and it belongs somewhere
 *   that can also tell the pilot it happened.
 */

#ifndef CONTROLLERS_RATE_RATE_HPP_
#define CONTROLLERS_RATE_RATE_HPP_

#include "common.hpp"
#include "PID.hpp"

// ArduPilot's ATC_RAT_* defaults for a multirotor. They are already in the
// units this class works in -- normalised torque out per rad/s of error in --
// so they carry over unchanged, and they are a sane place to start on an
// airframe nobody has tuned yet. They are NOT a substitute for tuning one.
#define RATE_ROLL_P             0.135f
#define RATE_ROLL_I             0.135f
#define RATE_ROLL_D             0.0036f

#define RATE_PITCH_P            0.135f
#define RATE_PITCH_I            0.135f
#define RATE_PITCH_D            0.0036f

// Yaw is PI, with no D at all. Yaw is made from motor torque reaction rather
// than thrust differential, so it is an order of magnitude weaker and a great
// deal slower than roll and pitch -- there is no fast transient for a D term
// to damp, and a D term on an axis this sluggish only feeds noise to the
// motors. I is small for the same reason: a weak axis winds up slowly, and a
// big integrator on it takes a long time to come back.
#define RATE_YAW_P              0.180f
#define RATE_YAW_I              0.018f
#define RATE_YAW_D              0.0f

// Integrator clamp, as a fraction of full torque. Half the available authority
// is a lot to hand to a term that exists to trim a steady error; more than
// this and a wound-up integrator is flying the aircraft.
#define RATE_IMAX               0.5f

// Filter cutoffs, Hz. See the header note -- these are the three poles.
#define RATE_FILT_GYRO_HZ       20.0f
#define RATE_FILT_TARGET_HZ     20.0f
#define RATE_FILT_D_HZ          20.0f

// The mixer's input range. Anything past this is thrust the airframe does not
// have.
#define RATE_OUTPUT_MAX         1.0f

// Cutoff for the oscillation reading, Hz. Slow on purpose: it is an average of
// how violently the output is moving, not a signal to control on.
#define RATE_OSC_FILT_HZ        2.0f

// What the last update() could not deliver, and what the mixer told us it
// could not deliver either. Both matter to a tuner: an axis clipping at +/-1
// is a loop asking for torque that does not exist, and there is no gain that
// fixes it -- it is a heavier aircraft, or a bigger motor.
struct RateLimits {
	bool roll;               // roll demand was clipped at the output range
	bool pitch;
	bool yaw;
	bool mixer_roll_pitch;   // the MIXER reported it could not deliver these
	bool mixer_yaw;          // ... or this
};

class Rate {
public:
	Rate();

	// -----------------------------------------------------------------------
	// Engaging
	// -----------------------------------------------------------------------

	// Clears the integrators, re-seeds every filter from the next sample, and
	// starts from a target of zero. Call it on arm, and on any mode change
	// that hands this loop a different job.
	//
	// The integrators above all: one wound up while the aircraft sat on the
	// ground -- where it could not rotate however hard the loop pushed -- is
	// torque applied the instant the motors have authority, before any error
	// has been measured. That is a flip on takeoff, and it is the classic way
	// to have one.
	void reset(void);

	// Drop the integrators but keep the filters running. For the case reset()
	// is too heavy for: the aircraft is armed and the loop is running, but the
	// motors are at ground idle and cannot act, so the I terms must not
	// accumulate. Call it every cycle the mixer is not at flight authority.
	void resetIntegrators(void);

	// Before reset(), update() does nothing and the outputs read zero. A
	// caller that forgets to engage this loop must get no torque, not the
	// demands left over from whatever it was doing last.
	bool isActive(void) const { return _active; }

	// -----------------------------------------------------------------------
	// Input
	// -----------------------------------------------------------------------

	// The body-frame rate the attitude controller wants, rad/s, FRD. Held
	// until overwritten.
	void setRateTarget(const Vector3f &rate_body);
	Vector3f getRateTarget(void) const { return _target; }

	// What the mixer could not deliver, from Motors::getLimits(). Pass
	// `roll_pitch` and `yaw` straight through; this class needs no other part
	// of that struct, and taking two bools rather than the struct is what
	// keeps the control law free of any dependency on the output stage.
	//
	// Held until overwritten, like the mixer holds its own inputs: a caller
	// that stops reporting leaves the last state standing. That is the safe
	// direction -- a stale "saturated" only freezes an integrator, while a
	// stale "clear" lets it wind up against a stop that is still there.
	//
	// Not calling this AT ALL is the one genuinely dangerous option, and it is
	// why getLimits() reports these back: if mixer_roll_pitch and mixer_yaw
	// never come true on an aircraft that visibly saturates, the wiring
	// between the two classes was never made.
	void setMixerSaturation(bool roll_pitch, bool yaw);

	// -----------------------------------------------------------------------
	// Run it. `gyro_body` is the measured body rate in rad/s -- the estimator's
	// bias-corrected rate, not the raw sensor, which still carries the bias
	// the estimator exists to find. A non-finite input or a non-positive dt is
	// REFUSED: the last demand holds and getRejectedCount() counts it.
	// -----------------------------------------------------------------------
	void update(const Vector3f &gyro_body, float dt);

	// -----------------------------------------------------------------------
	// Outputs -- normalised torque, [-1, 1], for Motors::setControlInput().
	// Zero until reset() engages the loop.
	// -----------------------------------------------------------------------
	Vector3f getOutput(void) const { return _active ? _output : Vector3f(); }
	float getRollOutput(void) const { return _active ? _output.x : 0.0f; }
	float getPitchOutput(void) const { return _active ? _output.y : 0.0f; }
	float getYawOutput(void) const { return _active ? _output.z : 0.0f; }

	// The error the loop actually acted on, rad/s: filtered target minus
	// filtered measurement, not the raw difference of what came in.
	Vector3f getRateError(void) const { return _error; }

	// The gyro after the low-pass -- what the loop believes the aircraft is
	// doing. Worth having next to the raw sample when a tuning session starts
	// arguing about noise.
	Vector3f getFilteredGyro(void) const { return _gyro; }

	const RateLimits &getLimits(void) const { return _limits; }

	// How hard each axis is working the motors, in output units per second,
	// averaged slowly. A diagnostic, not a control signal.
	//
	// This is the number that says D is too high. A loop flying an aircraft
	// through a manoeuvre moves its output steadily and reads small; one that
	// has started to ring reads one to two ORDERS of magnitude higher, because
	// oscillation is by definition the output reversing as fast as the loop
	// can drive it. Against a 25 Hz ring it reads around 12, where the same
	// loop flying a step reads about 0.03.
	//
	// Absolute numbers are airframe-specific -- a noisy gyro lifts the floor
	// for everyone -- so the way to use it is as a trend, not a threshold:
	// hover, note the reading, raise D, note it again. The point where it
	// starts climbing faster than the aircraft feels better is the point D has
	// gone past what the airframe will take.
	Vector3f getOscillation(void) const { return _oscillation; }

	// Samples update() REFUSED because the gyro was not finite or dt was not
	// positive. The loop then holds its last torque demand indefinitely, and
	// at the innermost loop that is the most dangerous freeze in the stack:
	// the aircraft keeps twisting at whatever it was last told, and nothing
	// about the output says the gyro died. Should read zero for the life of
	// the aircraft. Cleared by reset().
	uint32_t getRejectedCount(void) const { return _rejected; }

	// -----------------------------------------------------------------------
	// Tuning
	// -----------------------------------------------------------------------
	void setRollGains(float p, float i, float d);
	void setPitchGains(float p, float i, float d);
	void setYawGains(float p, float i, float d);

	// Feed-forward, which multiplies the TARGET rather than the error.
	//
	// Zero by default, and that is not laziness. A rigid body needs torque to
	// CHANGE its rate, not to hold one: at a steady roll rate with no
	// aerodynamic damping the correct steady torque is nothing at all, so
	// feeding the rate forward asks for a twist the physics does not want. It
	// is left available because a real airframe has some damping, and an
	// airframe with a lot of it -- large props, high drag -- can use a little.
	void setFeedForward(float roll, float pitch, float yaw);

	void setIMax(float imax);
	void setGyroCutoffHz(float hz);
	void setTargetCutoffHz(float hz);
	void setDerivativeCutoffHz(float hz);
	void setOutputLimit(float limit);

	float getGyroCutoffHz(void) const { return _gyro_hz; }
	float getOutputLimit(void) const { return _output_max; }

	// Read-only, for a tuning telemetry line. Seeing which term is doing the
	// work is the difference between tuning and guessing -- and on this loop
	// it is also how a runaway integrator is caught before it flies the
	// aircraft.
	const PID &getRollPID(void) const { return _pid_roll; }
	const PID &getPitchPID(void) const { return _pid_pitch; }
	const PID &getYawPID(void) const { return _pid_yaw; }

private:
	// One first-order pole, shared by the gyro filter and the oscillation
	// reading. The same form PID uses, and for the same reason: a cutoff at or
	// below zero must pass the signal through untouched, not freeze it.
	static float lowPassAlpha(float cutoff_hz, float dt);

	void updateOscillation(float dt, bool first);

	bool _active;

	PID _pid_roll;
	PID _pid_pitch;
	PID _pid_yaw;

	Vector3f _target;
	Vector3f _gyro;          // filtered
	Vector3f _error;
	Vector3f _output;
	Vector3f _output_prev;
	Vector3f _oscillation;

	RateLimits _limits;
	bool _mixer_rp;          // last reported mixer saturation, roll/pitch
	bool _mixer_yaw;         // ... and yaw

	float _gyro_hz;
	float _output_max;

	// Set by reset(): the next update seeds the gyro filter from its input
	// rather than sliding toward it from zero, which would read as a
	// full-scale step and put a false derivative through D on the first cycle
	// of every arm.
	bool _reset_filter;

	uint32_t _rejected;
};

#endif /* CONTROLLERS_RATE_RATE_HPP_ */
