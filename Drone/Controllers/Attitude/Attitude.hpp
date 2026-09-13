/*
 * Attitude.hpp
 *
 *  Created on: Sep 13, 2026
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
 * demand. That vector is the error this controller acts on.
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
 * that holds the aircraft up. Under a large upset the two demands cannot both
 * be met, and a controller that treats them as three equal axes will spend its
 * remaining authority correcting the nose while the aircraft is still falling.
 *
 * So the heading correction is faded out as tilt error grows: full below
 * yawPriorityAngle, nothing above twice it. Recovery attitude first, heading
 * afterwards. Nobody has ever crashed from pointing the wrong way.
 *
 * The split has a second consequence, and it is the one that makes the input
 * API below safe. A heading error is a rotation ABOUT the thrust axis, and a
 * rotation about an axis cannot move that axis. So however far the heading
 * target has run ahead of the aircraft, NONE of that gap reaches the thrust
 * direction -- it lands entirely in the third degree of freedom, which is
 * geometrically incapable of touching the first two. See the note on
 * setThrustVectorHeading().
 *
 * P, not PID
 * ----------
 * The gains are P only, and that is the normal way to fly this. In a cascade
 * the integrator belongs in the INNERMOST loop that sees the disturbance: the
 * rate controller's I term is what trims out a heavy battery or a bent arm,
 * and a second integrator here would fight it -- two loops winding against the
 * same steady torque, each undoing the other. D is no better: the derivative
 * of attitude error is angular rate, which is precisely what the loop below
 * already measures and controls, so D here is a slower, noisier copy of the
 * rate loop's P term.
 *
 * The stopping rate, and what it must not touch
 * ---------------------------------------------
 * A plain P on a big error asks for a rate the aircraft cannot stop from
 * before it arrives, and it overshoots. The fastest rate it can still stop
 * from in the angle remaining is
 *
 *     w = sqrt(2 * a * angle)
 *
 * -- the constant-acceleration result solved for rate -- so the CORRECTION is
 * capped there.
 *
 * The correction, not the output. The feed-forward is added afterwards, on
 * purpose: it is the rate the target is already moving at, and the stopping
 * cap goes to zero exactly when the error does. Capping the sum would mean
 * that a target rotating steadily with the aircraft tracking it perfectly --
 * zero error -- gets a rate demand of zero, and the aircraft stops dead and
 * falls behind until enough error builds up to earn the rate back. A commanded
 * yaw would stutter. The cap answers "how fast may I close this error", which
 * is not a question about the feed-forward at all.
 *
 * What is NOT here, and lives in the mode layer instead
 * -----------------------------------------------------
 * Both setters below take an ABSOLUTE heading. There is deliberately no
 * "setYawRate" -- it would be one line (integrate, then call the absolute
 * form) carrying no control law of its own, and putting it here would mean two
 * objects holding a heading target that can disagree. So the heading target is
 * the mode's, and three responsibilities travel with it:
 *
 *   1. SEED IT ON MODE ENTRY from the measured heading, or the nose snaps to
 *      whatever the last mode left behind.
 *   2. CLAMP ITS LEAD over the measured heading to ~30-45 deg. The mixer gives
 *      up yaw FIRST when four motors cannot meet four demands (see Motors),
 *      so holding yaw stick through a saturation winds the target away from
 *      the aircraft; release it and the nose snaps round to catch up.
 *   3. LIMIT LEAN AS A VECTOR, not per axis, or a full diagonal stick gets
 *      sqrt(2) times the lean of a cardinal one.
 *
 * Input shaping is also absent. The target comes from the velocity controller
 * or from sticks the mode has already filtered, and shaping it a second time
 * would add lag to the innermost angle loop.
 *
 * Nothing in this class knows about motors, throttle, sticks, or what the
 * aircraft is trying to achieve. It converts one orientation error into one
 * angular rate, and that is all.
 */

#ifndef CONTROLLERS_ATTITUDE_ATTITUDE_HPP_
#define CONTROLLERS_ATTITUDE_ATTITUDE_HPP_

// MathTypes, NOT common.hpp: this class is pure algorithm and must stay
// liftable onto another MCU or a host test harness. common.hpp would drag
// stm32f7xx_hal.h in behind it for nothing. If this header ever starts
// needing the HAL, a board dependency has crept into the control law.
#include "MathTypes.hpp"

// Angle P gains, 1/s: "one radian of error asks for this many radians per
// second of correction". ArduPilot's ATC_ANG_*_P defaults, which are already
// in radians and carry over unchanged.
#define ATTITUDE_P_ROLL               4.5f
#define ATTITUDE_P_PITCH              4.5f
#define ATTITUDE_P_YAW                4.5f

// Rate limits, rad/s. Roll and pitch share one: on a symmetric airframe they
// are the same number, and sharing it lets the pair be limited as a VECTOR --
// a diagonal demand must not get sqrt(2) times the authority of a cardinal one.
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
	// feed-forward and the limit flags, and engage the loop. Call it on every
	// mode entry.
	//
	// This is the whole answer to "what does it hold when the mode starts":
	// wherever it is pointing. Anything else -- a target left over from the
	// last mode, or an identity quaternion, which means wings level facing
	// north -- is a lurch the moment the mode engages, and at the innermost
	// angle loop that lurch is immediate.
	//
	// A non-finite or zero-length attitude leaves the loop DISENGAGED rather
	// than seeding it with rubbish.
	void reset(const Quaternionf &attitude);

	// Before reset(), update() does nothing and the rate target reads zero. A
	// caller that forgets to engage this controller must be told to hold
	// still, not handed the corrections from whatever it was doing last.
	bool isActive(void) const { return _active; }

	// -----------------------------------------------------------------------
	// Saying what attitude is wanted. Two ways in; both end up setting the
	// same target quaternion, and both take an ABSOLUTE heading -- see the
	// note on the mode layer at the top of this file.
	// -----------------------------------------------------------------------

	// Where the thrust axis must point, as a vector in NED, plus an absolute
	// heading. Need not be a unit vector. This is the primitive: a thrust
	// direction and a heading are always well defined, even pointing straight
	// down, and it is what the velocity controller and every stick-driven mode
	// should use.
	//
	// `thrust_ned` IS IN NED AND CARRIES NO HEADING. Whatever heading a mode
	// used to rotate a stick into this vector has been consumed by that
	// rotation and is gone -- so `heading` here may differ freely from the
	// aircraft's measured heading with no effect on the thrust direction. The
	// solve holds the thrust axis fixed and chooses whichever (roll, pitch)
	// realises it at the heading asked for; at heading 0 that is pitch -10 and
	// at heading 90 it is roll -10, the same direction in the world both
	// times. This is the property that makes a swept heading target safe.
	//
	// Do NOT rotate a stick into NED and then pass the result to setEuler():
	// setEuler() rotates by its heading argument, so the lean would be rotated
	// twice and the error would equal the heading exactly -- zero pointing
	// north, and exactly backwards at 180.
	//
	// A non-finite or zero-length vector is REFUSED: the previous target
	// stands and getRejectedCount() counts it.
	void setThrustVectorHeading(const Vector3f &thrust_ned, float heading_rad);

	// Angles directly, ZYX, radians. For a stabilize-style mode whose sticks
	// are already BODY-frame lean angles, and for anything that has a lean
	// angle rather than a thrust vector.
	//
	// roll and pitch are interpreted IN THE FRAME OF `heading_rad`. That is
	// the difference from setThrustVectorHeading(), and it is only safe while
	// the lean and the heading come from the same frame. With a swept heading
	// target they do not, so prefer the thrust-vector form wherever the
	// heading target can lead the measurement.
	//
	// Non-finite inputs are REFUSED, as above.
	void setEuler(float roll_rad, float pitch_rad, float heading_rad);

	// The target's own angular velocity, body frame, rad/s. Optional, and
	// strictly better than any internal estimate: a mode that commands a yaw
	// rate knows that rate exactly, while differencing the target attitude to
	// recover it costs a filter's worth of lag in the one place lag is least
	// affordable. There is no differencing fallback here for that reason --
	// without this, the feed-forward is simply zero.
	//
	// Valid for ONE cycle -- it must be set every cycle it applies. That is
	// deliberate: a stale feed-forward is a rate command nobody asked for, and
	// this way forgetting it degrades to a slightly laggy nose rather than to
	// a runaway.
	//
	// It does NOT protect the thrust direction -- nothing needs to, see the
	// note on the tilt/heading split above. Its whole job is keeping the NOSE
	// with the stick instead of trailing it by rate/P.
	void setTargetAngularVelocity(const Vector3f &ang_vel_body);

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

	// The attitude error as an axis-angle vector in body axes, rad -- the tilt
	// part and the heading part recombined. The number to watch when tuning,
	// and the one a telemetry line should carry.
	Vector3f getAttitudeError(void) const { return _att_error; }

	// Angle between where the thrust axis points and where it should, rad. One
	// number, no singularity, and the honest measure of how much trouble the
	// aircraft is in -- it is what the heading fade is computed from, and what
	// a mode should check before deciding it is still flying.
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

	// Heading of the target, rad -- the ZYX yaw of whatever was last set.
	float getHeadingTarget(void) const { return _heading_target; }

	// The feed-forward part of the last output, body frame, rad/s.
	Vector3f getFeedForward(void) const { return _ff_body; }

	const AttitudeLimits &getLimits(void) const { return _limits; }

	// Inputs REFUSED because a value was not finite, a vector had no length,
	// or dt was not positive. Refusing is right, but the loop then holds its
	// last rate demand indefinitely, and a frozen attitude loop looks exactly
	// like a healthy one holding still. This is the evidence. It should read
	// zero for the life of the aircraft; cleared by reset().
	uint32_t getRejectedCount(void) const { return _rejected; }

	// -----------------------------------------------------------------------
	// Tuning
	// -----------------------------------------------------------------------
	void setGains(float p_roll, float p_pitch, float p_yaw);
	void setRateLimits(float rp_max, float yaw_max);
	void setAccelLimits(float rp_max, float yaw_max);
	void setYawPriorityAngle(float rad);

	float getRateLimitRP(void) const { return _rate_rp_max; }
	float getRateLimitYaw(void) const { return _rate_yaw_max; }
	float getAccelLimitRP(void) const { return _accel_rp_max; }
	float getAccelLimitYaw(void) const { return _accel_yaw_max; }
	float getYawPriorityAngle(void) const { return _yaw_priority_angle; }

	// -----------------------------------------------------------------------
	// The rotation maths, exposed because it IS the controller and deserves
	// testing with no vehicle state anywhere near it.
	// -----------------------------------------------------------------------

	// Thrust direction (NED, any length) plus a heading -> the attitude that
	// achieves both, solved exactly in ZYX Euler:
	//
	//     roll  = asin ( a sin(h) - b cos(h) )
	//     pitch = atan2( a cos(h) + b sin(h), c )     (a,b,c) = -thrust_ned
	//
	// The thrust direction is a CONSTRAINT, not something the heading rotates:
	// every heading gives a different (roll, pitch) naming the same direction
	// in the world. Returns identity for a zero-length or non-finite input.
	static Quaternionf attitudeFromThrustVector(const Vector3f &thrust_ned,
			float heading_rad);

	// ZYX Euler angles -> quaternion, body -> NED. Bit-for-bit the estimator's
	// ESEKF::quaternionFromEuler(), so the two agree on what an angle means.
	static Quaternionf attitudeFromEuler(float roll_rad, float pitch_rad,
			float yaw_rad);

	// Heading of an attitude, rad: the ZYX yaw, matching the estimator's
	// eulerFromQuaternion().z.
	static float headingFromAttitude(const Quaternionf &q);

	// Where an attitude points its thrust axis (body -Z), unit vector in NED.
	static Vector3f thrustVectorOf(const Quaternionf &q);

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
	// exactly on the target: sqrt(2 * a * |angle|), always positive.
	//
	// `accel_max` must be positive -- "no limit" is the caller not applying
	// the cap, not a value passed in here.
	static float stoppingRate(float angle_error, float accel_max);

	// Angle wrapped to +/-pi.
	static float wrapPi(float rad);

private:
	// 1 below the priority angle, fading linearly to 0 at twice it.
	float yawPriority(float tilt_error) const;

	// Common tail of both setters: normalise, store, and cache the heading.
	void applyTarget(const Quaternionf &q_target, float heading_rad);

	bool _active;

	float _p_roll;
	float _p_pitch;
	float _p_yaw;

	Quaternionf _q_target;
	float _heading_target;        // ZYX yaw of _q_target, as it was set

	Vector3f _rate_target;
	Vector3f _att_error;
	float _tilt_error;
	float _yaw_error;
	float _yaw_priority;

	Vector3f _ff_body;            // as applied, in the current body frame
	Vector3f _ff_supplied;        // what setTargetAngularVelocity() last gave
	bool _ff_has_supplied;

	AttitudeLimits _limits;

	float _rate_rp_max;
	float _rate_yaw_max;
	float _accel_rp_max;
	float _accel_yaw_max;
	float _yaw_priority_angle;

	uint32_t _rejected;
};

#endif /* CONTROLLERS_ATTITUDE_ATTITUDE_HPP_ */
