/*
 * Velocity.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * Turns a velocity target into a direction to point the thrust and an amount
 * of it. The step where the problem stops being about navigation and starts
 * being about the airframe.
 *
 * Where it sits
 * -------------
 *   RC sticks -> mode -> Position -> VELOCITY -> Attitude -> Rate -> Motors
 *
 * In:  a velocity target in NED (m/s), the estimator's measured velocity, and
 *      the current heading.
 * Out: a thrust vector -- which way the aircraft's "up" must point -- plus a
 *      collective throttle. The attitude controller takes the first, the
 *      motors take the second.
 *
 * How a multirotor accelerates
 * ----------------------------
 * It has one actuator direction: thrust, straight out of the top. It cannot
 * push sideways at all. To accelerate north it must TILT north, so that some
 * of that one thrust vector points north -- and then thrust harder, because
 * the part still holding it up is now only the vertical component.
 *
 * So the whole controller is one piece of vector arithmetic. Ask the velocity
 * PIDs what acceleration is wanted, add the acceleration needed to cancel
 * gravity, and the sum is the acceleration the thrust must produce:
 *
 *     thrust_accel = desired_accel - gravity
 *                  = (a_n, a_e, a_d) - (0, 0, +g)
 *                  = (a_n, a_e, a_d - g)
 *
 * Its DIRECTION is where "up" has to point, and its MAGNITUDE is how hard to
 * push. Hovering level, that is (0, 0, -g): straight up, at one g. Nothing
 * else in this class is more than bookkeeping around those two lines.
 *
 * The throttle boost comes out for free
 * -------------------------------------
 * Because the magnitude is |thrust_accel|, tilting automatically asks for more
 * throttle. At 30 degrees of lean, holding altitude, the horizontal demand is
 * g*tan(30) and the magnitude works out to g/cos(30) -- exactly the 1/cos(tilt)
 * boost a flight controller has to apply to stop the aircraft sinking when it
 * banks. It is not a correction bolted on afterwards; it is what the vector
 * length already says.
 *
 * Lean angle, and why it is a limit on ACCELERATION
 * ------------------------------------------------
 * A tilt of theta buys a horizontal acceleration of g*tan(theta). Past about
 * 45 degrees that grows faster than the airframe can supply thrust, and near
 * 90 it goes to infinity while the vertical component goes to zero -- the
 * aircraft stops holding itself up. So the horizontal acceleration is capped
 * at g*tan(lean_max), as a VECTOR, and the cap is fed back into the PIDs so
 * they do not wind up against it.
 *
 * Roll and pitch, exactly
 * -----------------------
 * The thrust vector plus a heading determines the attitude. This class solves
 * for it exactly rather than using the usual atan(a/g) approximation, which
 * quietly assumes the vertical acceleration is g and is wrong by several
 * degrees during a climb. See computeAttitudeTarget().
 *
 * What is NOT here
 * ----------------
 * ArduPilot puts a second PID INSIDE this one, closing a loop on measured
 * vertical acceleration, because throttle-to-thrust has real lag and a battery
 * that sags changes the relationship mid-flight. Here the loop closes at the
 * velocity level only, and the vertical PID's integrator is what absorbs a
 * wrong hover throttle. That is enough to fly and it is one fewer thing to
 * tune; if altitude hold turns out sloppy under aggressive throttle changes,
 * an inner accel loop is the first place to look.
 */

#ifndef CONTROLLERS_VELOCITY_VELOCITY_HPP_
#define CONTROLLERS_VELOCITY_VELOCITY_HPP_

#include "common.hpp"
#include "PID.hpp"

#define VELOCITY_GRAVITY_MSS          9.80665f

// Horizontal velocity PID: m/s of error in, m/s^2 of acceleration out.
// ArduPilot's PSC_VELXY defaults, which are in cm and so carry over unchanged.
// I matters more here than in most loops -- it is what holds the aircraft
// still against a steady wind, and without it a hover in any breeze drifts.
#define VELOCITY_NE_P                 2.0f
#define VELOCITY_NE_I                 1.0f
#define VELOCITY_NE_D                 0.5f

// Vertical velocity PID: m/s in, m/s^2 out. Stiffer than horizontal because
// the axis is stiffer -- there is no tilt to wait for, the thrust is already
// pointing the right way.
#define VELOCITY_D_P                  5.0f
#define VELOCITY_D_I                  2.0f
#define VELOCITY_D_D                  0.0f

// Cutoff for the D terms, Hz. Differentiating an estimator output that is
// itself fusing a noisy accelerometer is a good way to turn D into hiss.
#define VELOCITY_FILT_D_HZ            5.0f

// Radians. 30 degrees is ArduPilot's ANGLE_MAX default and a sane ceiling for
// anything that is not being flown aerobatically.
#define VELOCITY_MAX_LEAN_ANGLE       0.5235988f   // 30 deg

// Vertical acceleration limits, m/s^2, both positive. Kept well under g: the
// aircraft cannot accelerate downward faster than gravity without inverting,
// and long before that the thrust vector would have to point DOWN.
#define VELOCITY_ACCEL_UP             2.5f
#define VELOCITY_ACCEL_DOWN           2.5f

// A downward acceleration demand this close to g would flip the thrust vector
// over and ask the aircraft to fly upside down. The setter refuses to go past
// it, so the rest of the class can take a skyward thrust vector for granted.
#define VELOCITY_MAX_DOWN_ACCEL_FRAC  0.8f

// Throttle that holds a hover, as Motors sees it (0 = spin_min, 1 = spin_max).
// A guess until measured: hover the aircraft, read the throttle, put it here.
// The vertical integrator will cover the error, but only after it has wound
// up, and that wind-up is felt as a sag or a balloon on every takeoff.
#define VELOCITY_HOVER_THROTTLE       0.5f

// What the last update() could not deliver. The point of reporting it is
// anti-windup and honest mode logic: a controller pinned against its lean
// limit is not holding position, it is losing ground, and something above it
// should know.
struct VelocityLimits {
	bool accel_ne;        // horizontal demand was cut to the lean-angle limit
	bool accel_up;        // upward acceleration was cut
	bool accel_down;      // downward acceleration was cut
	bool throttle_lower;  // throttle demand fell below zero
	bool throttle_upper;  // throttle demand exceeded full
};

class Velocity {
public:
	Velocity();

	// -----------------------------------------------------------------------
	// Engaging
	// -----------------------------------------------------------------------

	// Clears the integrators, seeds the filters, and starts with the target
	// set to the velocity the aircraft already has -- so the first cycle sees
	// zero error rather than a step. Call on every mode entry.
	//
	// The integrators especially: one wound up during a period this loop was
	// not flying is a lean angle applied the instant it takes over.
	void reset(const Vector3f &vel_ned);

	// Before this, update() does nothing and the outputs stay at "level, at
	// hover throttle" -- the least surprising thing an unengaged controller
	// can say.
	bool isActive(void) const { return _active; }

	// -----------------------------------------------------------------------
	// Input -- from the position controller, or straight from a mode flying
	// velocities (which is most of them).
	// -----------------------------------------------------------------------
	void setVelocityTarget(const Vector3f &vel_ned);
	Vector3f getVelocityTarget(void) const { return _vel_target; }

	// `yaw_rad` is the aircraft's CURRENT heading, not a target. It is only
	// used to say which way "north" is in body terms -- the thrust vector is
	// computed in NED and does not depend on it.
	void update(const Vector3f &vel_ned, float yaw_rad, float dt);

	// -----------------------------------------------------------------------
	// Outputs
	// -----------------------------------------------------------------------

	// Where the aircraft's "up" must point, as a unit vector in NED. This is
	// what the attitude controller should take: it is singularity-free, unlike
	// the roll/pitch pair below, and it is what ArduPilot's
	// input_thrust_vector_* entry points want.
	Vector3f getThrustVector(void) const { return _thrust_vector; }

	// Collective for Motors, 0..1.
	float getThrottle(void) const { return _throttle; }

	// The same attitude as the thrust vector, as ZYX Euler angles about the
	// current heading. For telemetry, for a controller that wants angles, and
	// because a lean angle is the number a human can sanity-check.
	float getRollTarget(void) const { return _roll_target; }
	float getPitchTarget(void) const { return _pitch_target; }

	// Angle between the thrust vector and straight up. One number, no
	// singularity -- what a tilt limit or a safety check actually wants.
	float getTiltTarget(void) const { return _tilt_target; }

	// The acceleration the PIDs asked for, NED, m/s^2, gravity NOT included.
	// The intermediate everything else is derived from.
	Vector3f getAccelTarget(void) const { return _accel_target; }

	Vector3f getVelocityError(void) const { return _vel_error; }
	const VelocityLimits &getLimits(void) const { return _limits; }

	// -----------------------------------------------------------------------
	// Tuning
	// -----------------------------------------------------------------------
	void setHorizontalGains(float kp, float ki, float kd);
	void setVerticalGains(float kp, float ki, float kd);
	void setMaxLeanAngle(float rad);
	void setVerticalAccelLimits(float up, float down);
	void setHoverThrottle(float hover);

	float getMaxLeanAngle(void) const { return _lean_max; }
	// The horizontal acceleration the lean limit allows AT A HOVER, g *
	// tan(lean_max). The limit actually applied each cycle uses the real
	// vertical thrust rather than g -- more during a climb, less during a
	// descent -- so that the tilt never exceeds lean_max. This is the number
	// for a tuning display; getTiltTarget() is the one that is guaranteed.
	float getMaxHorizontalAccel(void) const;
	float getHoverThrottle(void) const { return _hover_throttle; }
	float getAccelUpLimit(void) const { return _accel_up_max; }
	float getAccelDownLimit(void) const { return _accel_down_max; }

	// Read-only, for a tuning telemetry line. Seeing which term is doing the
	// work is the difference between tuning and guessing.
	const PID &getNorthPID(void) const { return _pid_n; }
	const PID &getEastPID(void) const { return _pid_e; }
	const PID &getVerticalPID(void) const { return _pid_d; }

	// -----------------------------------------------------------------------
	// The vector arithmetic, exposed because it is the whole controller and
	// deserves testing with no vehicle state anywhere near it.
	// -----------------------------------------------------------------------

	// Solves R(roll, pitch, yaw) * (0,0,-1) = thrust_unit for roll and pitch,
	// exactly. Not the atan(a/g) approximation, which assumes the vertical
	// acceleration is g and is wrong by degrees during a climb.
	static void computeAttitudeTarget(const Vector3f &thrust_unit, float yaw_rad,
			float &roll_rad, float &pitch_rad);

private:
	void computeThrustVector(void);

	bool _active;

	PID _pid_n;
	PID _pid_e;
	PID _pid_d;

	Vector3f _vel_target;
	Vector3f _vel_error;
	Vector3f _accel_target;

	Vector3f _thrust_vector;
	float _throttle;
	float _roll_target;
	float _pitch_target;
	float _tilt_target;

	VelocityLimits _limits;

	float _lean_max;
	float _accel_up_max;
	float _accel_down_max;
	float _hover_throttle;
};

#endif /* CONTROLLERS_VELOCITY_VELOCITY_HPP_ */
