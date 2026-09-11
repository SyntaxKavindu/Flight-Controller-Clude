/*
 * Attitude.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * Turns "point that way" into "turn at this rate". The last loop that thinks
 * in angles -- everything below it is rates, torques and motors.
 *
 * Where it sits
 * -------------
 *   RC sticks -> mode -> Position -> Velocity -> ATTITUDE -> Rate -> Motors
 *
 * In:  a target attitude, and the estimator's measured attitude.
 * Out: a body-frame angular rate in rad/s, which is exactly what the rate
 *      controller wants and exactly what a gyro measures.
 *
 * Quaternions, and why not three Euler errors
 * -------------------------------------------
 * The obvious implementation is to subtract three angles -- roll error, pitch
 * error, yaw error -- and put a gain on each. It is wrong, and not subtly:
 *
 *   - Three Euler differences are not a rotation. Euler angles compose in a
 *     fixed order about axes that move as you go, so "10 degrees of roll error
 *     and 10 of yaw error" does not name one turn the aircraft can make.
 *   - Near vertical, yaw and roll become the same axis and the difference is
 *     undefined. Gimbal lock is not an exotic case for a multirotor that gets
 *     upset.
 *   - The errors wrap. Subtracting 179 from -179 gives 358 degrees of yaw
 *     error, and the aircraft spins the long way round.
 *
 * A quaternion difference has none of those problems. q_err = q_measured^-1 *
 * q_target is one rotation -- the one that takes the aircraft from where it is
 * to where it should be -- and turning it into an axis-angle vector gives a
 * vector in BODY axes whose components are directly a roll, pitch and yaw
 * demand. That vector is the error this controller acts on, and it is why
 * PID::updateError() exists: there is no "target minus measurement" to hand
 * the usual entry point.
 *
 * Tilt and heading are not equally important
 * ------------------------------------------
 * The error is split in two before any gain touches it:
 *
 *   TILT     where the thrust axis points. Two of the three degrees of
 *            freedom, and the two that decide whether the aircraft accelerates
 *            the way the loop above asked -- or falls.
 *   HEADING  which way the nose points. One degree of freedom, and the only
 *            one a multirotor can simply give up.
 *
 * They compete for the same actuator. A quad yaws by unbalancing the torque
 * reaction between its two motor pairs, which means every bit of yaw authority
 * is thrust taken off one diagonal and added to the other -- the same thrust
 * that holds the aircraft up and points it. Under a large upset the two
 * demands cannot both be met, and a controller that treats them as three equal
 * axes will spend its remaining authority correcting the nose while the
 * aircraft is still falling.
 *
 * So the heading correction is faded out as tilt error grows: full below
 * yawPriorityAngle, nothing above twice it. Recovery attitude first, heading
 * afterwards. Nobody has ever crashed from pointing the wrong way.
 *
 * P, not PID
 * ----------
 * The gains default to P only, and that is the normal way to fly this. In a
 * cascade the integrator belongs in the INNERMOST loop that sees the
 * disturbance: the rate controller's I term is what trims out a heavy battery
 * or a bent arm, and a second integrator here would fight it -- two loops
 * winding against the same steady torque, each undoing the other. D is no
 * better: the derivative of attitude error is angular rate, which is precisely
 * what the loop below already measures and controls, so D here is a slower,
 * noisier copy of the rate loop's P term.
 *
 * Full PIDs are used anyway, rather than three floats, because they bring the
 * filtering, the finite-input guard, the anti-windup and the term-by-term
 * telemetry with them -- and because if an airframe ever does need a trickle
 * of I here, setGains() is the whole change.
 *
 * The stopping rate, and what it must not touch
 * ---------------------------------------------
 * A plain P on a big error asks for a rate the aircraft cannot stop from
 * before it arrives, and it overshoots. The fastest rate it can still stop
 * from in the angle remaining is
 *
 *     w = sqrt(2 * a * angle)
 *
 * -- the constant-acceleration result solved for rate, the same argument
 * Position makes about distances one loop further out -- so the CORRECTION is
 * capped there.
 *
 * The correction, not the output. The feed-forward below is added afterwards,
 * on purpose: it is the rate the target is already moving at, and the stopping
 * cap goes to zero exactly when the error does. Capping the sum would mean
 * that a target rotating steadily with the aircraft tracking it perfectly --
 * zero error -- gets a rate demand of zero, and the aircraft stops dead and
 * falls behind until enough error builds up to earn the rate back. A
 * commanded yaw would stutter. The cap answers "how fast may I close this
 * error", which is not a question about the feed-forward at all.
 *
 * Feed-forward
 * ------------
 * A P controller only outputs while there is an error, so tracking a MOVING
 * target it must run permanently behind it -- error = rate / P, a fixed lag
 * proportional to how fast the target turns. On a yaw stick that is the
 * difference between the nose following the stick and the nose dragging.
 *
 * So the target's own angular velocity is measured (by differencing the target
 * attitude), rotated into the body frame and added to the output. At steady
 * state the error goes to zero and the feed-forward alone carries the rate,
 * which is what it should be.
 *
 * Differencing a target is the one place this class can be surprised: a target
 * that STEPS differentiates to an impulse. Three things hold that down -- the
 * result is low-passed, the total is clamped to the rate limits, and reset()
 * re-seeds rather than differencing across the gap. A caller that knows its
 * target's rate exactly can skip the estimate entirely with
 * setTargetAngularVelocity(), which is always better when it is available.
 *
 * What is NOT here
 * ----------------
 * Input shaping. ArduPilot slews its attitude target under rate and
 * acceleration limits so the target never steps, and gets its feed-forward out
 * of that shaping for free. Here the target comes from the velocity
 * controller, whose own filters and limits already make it move smoothly, and
 * shaping it a second time would add lag to the innermost angle loop. If a
 * mode ever drives this class from raw sticks, shaping belongs in that mode.
 *
 * Nothing in this class knows about motors, throttle, or what the aircraft is
 * trying to achieve. It converts one orientation error into one angular rate,
 * and that is all.
 */

#ifndef CONTROLLERS_ATTITUDE_ATTITUDE_HPP_
#define CONTROLLERS_ATTITUDE_ATTITUDE_HPP_

#include "common.hpp"
#include "PID.hpp"

// Angle P gains, 1/s: "one radian of error asks for this many radians per
// second of correction". ArduPilot's ATC_ANG_*_P defaults, which are already
// in radians and carry over unchanged.
#define ATTITUDE_P_ROLL               4.5f
#define ATTITUDE_P_PITCH              4.5f
#define ATTITUDE_P_YAW                4.5f

// Rate limits, rad/s. Roll and pitch share one: on a symmetric airframe they
// are the same number, and sharing it lets the pair be limited as a VECTOR --
// see the note on limitLengthXY() in the implementation.
#define ATTITUDE_RATE_RP_MAX          6.2831853f   // 360 deg/s
#define ATTITUDE_RATE_YAW_MAX         2.0943951f   // 120 deg/s

// Angular acceleration limits, rad/s^2 -- what the stopping rate is computed
// against. ArduPilot's ATC_ACCEL_R/P_MAX (110000 cdeg/s^2) and ATC_ACCEL_Y_MAX
// (27000 cdeg/s^2). Yaw is an order of magnitude weaker than roll and pitch
// because it is made from motor torque reaction rather than thrust
// differential, and that is a real property of the airframe, not a tuning
// choice.
#define ATTITUDE_ACCEL_RP_MAX         19.198622f   // 1100 deg/s^2
#define ATTITUDE_ACCEL_YAW_MAX        4.7123890f   // 270 deg/s^2

// Tilt error at which the heading correction starts being given up; at twice
// this it is given up entirely. ArduPilot's AC_ATTITUDE_THRUST_ERROR_ANGLE.
#define ATTITUDE_YAW_PRIORITY_ANGLE   0.5235988f   // 30 deg

// Cutoff for the low-pass on the measured target rate, Hz. High enough not to
// add lag to a feed-forward whose whole purpose is removing lag, low enough
// that one noisy target sample does not become a rate command.
#define ATTITUDE_FF_CUTOFF_HZ         10.0f

// What the last update() could not deliver. Not decoration: a rate controller
// handed a demand that was clipped here has to know, or its own integrator
// winds up against a limit this class imposed; and a mode that cannot tell
// "holding attitude" from "recovering, heading abandoned" cannot report
// honestly to the pilot.
struct AttitudeLimits {
	bool rate_rp;       // roll/pitch demand was cut to the rate limit
	bool rate_yaw;      // yaw demand was cut to the rate limit
	bool accel_rp;      // roll/pitch correction was cut to the stopping rate
	bool accel_yaw;     // yaw correction was cut to the stopping rate
	bool yaw_priority;  // heading correction was faded out by the tilt error
};

class Attitude {
public:
	Attitude();

	// -----------------------------------------------------------------------
	// Engaging
	// -----------------------------------------------------------------------

	// Snap the target to the attitude the aircraft is ALREADY at, clear the
	// integrators, and re-seed the feed-forward. Call it on every mode entry.
	//
	// This is the whole answer to "what does it hold when the mode starts":
	// wherever it is pointing. Anything else -- a target left over from the
	// last mode, or an identity quaternion, which means wings level facing
	// north -- is a lurch the moment the mode engages, and at the innermost
	// angle loop that lurch is immediate.
	void reset(const Quaternionf &attitude);

	// Before this, update() does nothing and the rate target reads zero. A
	// caller that forgets to engage this controller must be told to hold
	// still, not handed the corrections from whatever it was doing last.
	bool isActive(void) const { return _active; }

	// -----------------------------------------------------------------------
	// Saying what attitude is wanted. Four ways in; they all end up setting
	// the same target quaternion.
	// -----------------------------------------------------------------------

	// The one the velocity controller feeds: where the thrust axis must point,
	// as a vector in NED, plus an absolute heading. Need not be a unit vector.
	// This is the singularity-free pairing -- a thrust direction and a heading
	// are always well defined, even pointing straight down.
	void setThrustVectorHeading(const Vector3f &thrust_ned, float yaw_rad);

	// The same, with the heading given as a RATE -- a yaw stick. The heading
	// is integrated internally from wherever the target already was, so the
	// nose sweeps rather than jumping, and releasing the stick holds the
	// heading it reached.
	void setThrustVectorYawRate(const Vector3f &thrust_ned, float yaw_rate,
			float dt);

	// Angles directly, ZYX, radians. For a stabilize-style mode, and for
	// anything that has a lean angle rather than a thrust vector.
	void setEuler(float roll_rad, float pitch_rad, float yaw_rad);

	// The target as a quaternion, body -> NED, for a caller that already has
	// one. Normalised on the way in; a zero-length quaternion is refused.
	void setAttitudeTarget(const Quaternionf &q_target);

	// The target's own angular velocity, body frame of the TARGET, rad/s.
	// Optional, and strictly better than the internal estimate when the caller
	// knows it: a mode that commands a yaw rate knows that rate exactly, while
	// this class can only measure it by differencing, which costs a filter's
	// worth of lag.
	//
	// Valid for ONE cycle -- it must be set every cycle it applies, or the
	// next update() falls back to differencing. That is deliberate: a stale
	// feed-forward is a rate command nobody asked for, and this way forgetting
	// it degrades to the estimate rather than to a runaway.
	void setTargetAngularVelocity(const Vector3f &ang_vel);

	// -----------------------------------------------------------------------
	// Run it. `attitude` is the estimator's orientation (body -> NED) and `dt`
	// the time since the last call. A non-finite input or a non-positive dt is
	// REFUSED, not acted on -- the outputs hold and getRejectedCount() counts
	// it. See the note on that counter.
	// -----------------------------------------------------------------------
	void update(const Quaternionf &attitude, float dt);

	// -----------------------------------------------------------------------
	// Outputs
	// -----------------------------------------------------------------------

	// What the rate controller should fly: body-frame FRD angular velocity in
	// rad/s. Zero until reset() engages the loop.
	Vector3f getRateTarget(void) const {
		return _active ? _rate_target : Vector3f();
	}

	// The attitude error as an axis-angle vector in body axes, rad -- the
	// tilt part and the heading part recombined. The number to watch when
	// tuning, and the one a telemetry line should carry.
	Vector3f getAttitudeError(void) const { return _att_error; }

	// Angle between where the thrust axis points and where it should, rad.
	// One number, no singularity, and the honest measure of how much trouble
	// the aircraft is in -- it is what the heading fade is computed from, and
	// what a mode should check before deciding it is still flying.
	float getTiltError(void) const { return _tilt_error; }

	// Heading error, rad, wrapped to +/-pi so it is always the short way
	// round. NOT scaled by the priority fade -- this is the angle that is
	// actually wrong, whatever the controller is currently allowed to do
	// about it.
	float getYawError(void) const { return _yaw_error; }

	// How much of the heading correction survived the fade: 1 normally, 0
	// while recovering from a large tilt error.
	float getYawPriority(void) const { return _yaw_priority; }

	Quaternionf getAttitudeTarget(void) const { return _q_target; }

	// Where the target says the thrust axis points, unit vector in NED. The
	// round trip back out of the quaternion, for telemetry and for checking
	// against what the velocity controller asked for.
	Vector3f getThrustVectorTarget(void) const;

	// Heading of the target, rad. What setThrustVectorYawRate() integrates.
	float getHeadingTarget(void) const { return _yaw_target; }

	// The feed-forward part of the last output, body frame, rad/s.
	Vector3f getFeedForward(void) const { return _ff_body; }

	const AttitudeLimits &getLimits(void) const { return _limits; }

	// Samples update() REFUSED because the attitude was not finite, the
	// quaternion had no length, or dt was not positive. Refusing is right, but
	// the loop then holds its last rate demand indefinitely, and a frozen
	// attitude loop looks exactly like a healthy one holding still. This is
	// the evidence. It should read zero for the life of the aircraft; cleared
	// by reset().
	uint32_t getRejectedCount(void) const { return _rejected; }

	// -----------------------------------------------------------------------
	// Tuning
	// -----------------------------------------------------------------------
	void setGains(float p_roll, float p_pitch, float p_yaw);
	void setRateLimits(float rp_max, float yaw_max);
	void setAccelLimits(float rp_max, float yaw_max);
	void setYawPriorityAngle(float rad);
	void setFeedForwardEnabled(bool enabled);
	void setFeedForwardCutoffHz(float hz);

	float getRateLimitRP(void) const { return _rate_rp_max; }
	float getRateLimitYaw(void) const { return _rate_yaw_max; }
	float getAccelLimitRP(void) const { return _accel_rp_max; }
	float getAccelLimitYaw(void) const { return _accel_yaw_max; }
	float getYawPriorityAngle(void) const { return _yaw_priority_angle; }
	bool isFeedForwardEnabled(void) const { return _ff_enabled; }

	// Read-only, for a tuning telemetry line -- which term is doing the work.
	const PID &getRollPID(void) const { return _pid_roll; }
	const PID &getPitchPID(void) const { return _pid_pitch; }
	const PID &getYawPID(void) const { return _pid_yaw; }

	// -----------------------------------------------------------------------
	// The rotation maths, exposed because it IS the controller and deserves
	// testing with no vehicle state anywhere near it.
	// -----------------------------------------------------------------------

	// Thrust direction (NED, any length) plus a heading -> the attitude that
	// achieves both, solved exactly in ZYX Euler -- the same solve, and so the
	// same answer, as Velocity::computeAttitudeTarget().
	static Quaternionf attitudeFromThrustVector(const Vector3f &thrust_ned,
			float yaw_rad);

	// ZYX Euler angles -> quaternion, body -> NED. The estimator's convention.
	static Quaternionf attitudeFromEuler(float roll_rad, float pitch_rad,
			float yaw_rad);

	// Heading of an attitude, rad. The ZYX yaw, which is the azimuth the nose
	// projects onto the horizontal plane.
	static float headingFromAttitude(const Quaternionf &q);

	// A rotation as an axis-angle vector, rad. Always the SHORT way round: a
	// quaternion and its negation are the same attitude but name rotations
	// differing by a full turn, and only one of them is a correction worth
	// commanding.
	static Vector3f rotationVector(const Quaternionf &q);

	// Split an attitude error into the part that moves the thrust axis and the
	// part that turns about it: q_err == (tilt) * (heading).
	//
	// `tilt` comes out as an axis-angle vector with no z component -- the
	// shortest rotation putting the thrust axis where it belongs -- and
	// `yaw_rad` as what is left, a pure rotation about body z, wrapped to
	// +/-pi. Both are in the CURRENT body frame, which is the frame the rate
	// controller commands in.
	static void decompose(const Quaternionf &q_err, Vector3f &tilt,
			float &yaw_rad);

	// The fastest you can be turning through `angle_error` and still stop
	// exactly on the target: sqrt(2 * a * angle). Position makes the same
	// argument about distances and velocities one loop further out; the two
	// are deliberately not shared, so that the inner loops carry no dependency
	// on the outermost one.
	//
	// `accel_max` must be positive -- "no limit" is the caller not applying
	// the cap, not a value passed in here.
	static float stoppingRate(float angle_error, float accel_max);

private:
	// 1 below the priority angle, fading linearly to 0 at twice it.
	float yawPriority(float tilt_error) const;

	// The target's angular velocity, in the CURRENT body frame. Either the
	// value a caller supplied, or the target attitude differenced and
	// low-passed. Consumes the supplied value: it is good for one cycle.
	Vector3f feedForward(const Quaternionf &q_err, float dt);

	bool _active;

	PID _pid_roll;
	PID _pid_pitch;
	PID _pid_yaw;

	Quaternionf _q_target;
	Quaternionf _q_target_prev;   // last cycle's, for differencing
	float _yaw_target;            // heading of the target, integrated by yaw rate

	Vector3f _rate_target;
	Vector3f _att_error;
	float _tilt_error;
	float _yaw_error;
	float _yaw_priority;

	Vector3f _ff_target;          // filtered, in the target's body frame
	Vector3f _ff_body;            // as applied, in the current body frame
	Vector3f _ff_supplied;        // what setTargetAngularVelocity() last gave
	bool _ff_has_supplied;
	bool _ff_seed;                // next cycle seeds rather than differences
	bool _ff_enabled;
	float _ff_cutoff_hz;

	AttitudeLimits _limits;

	float _rate_rp_max;
	float _rate_yaw_max;
	float _accel_rp_max;
	float _accel_yaw_max;
	float _yaw_priority_angle;

	uint32_t _rejected;
};

#endif /* CONTROLLERS_ATTITUDE_ATTITUDE_HPP_ */
