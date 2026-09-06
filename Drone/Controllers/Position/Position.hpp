/*
 * Position.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * The outermost flight loop: holds a position in NED and hands the velocity
 * controller a velocity target.
 *
 * Where it sits
 * -------------
 *   RC sticks -> mode -> POSITION -> Velocity -> Attitude -> Rate -> Motors
 *
 * It answers one question -- "how fast, and which way, to get to where I am
 * supposed to be" -- and nothing else. It does not know about lean angles,
 * thrust, or motors, and it never looks at attitude.
 *
 * Everything is metres and metres per second in the NED frame the estimator
 * uses: +x north, +y east, +z DOWN. So climbing is NEGATIVE z, and that is why
 * the two vertical speed limits are separate and both given as positive
 * numbers -- see setSpeedLimits().
 *
 * Why a square root, not a plain P
 * --------------------------------
 * A plain proportional controller asks for velocity = P * error. Twice as far
 * away, twice as fast. That is fine near the target and wrong far from it: at
 * 50 m out with P = 1 it demands 50 m/s, and even if the vehicle could reach
 * it, stopping from 50 m/s inside 50 m needs an acceleration the airframe does
 * not have. The vehicle arrives fast, overshoots, comes back, overshoots
 * again.
 *
 * What you actually want is the fastest speed from which the vehicle can still
 * stop in the distance remaining, which is v = sqrt(2 * a * d). That is the
 * square-root controller: sqrt shaping far out where deceleration is the
 * binding constraint, linear P near the target where sqrt's infinite slope at
 * zero would chatter, and a smooth join between them. ArduPilot's
 * sqrt_controller(), and the reason its position loops settle rather than
 * hunt.
 *
 * Desired, offset, target
 * -----------------------
 * Three positions, and they are not the same thing:
 *
 *   desired   where the PILOT (or the mission) has put the target. Sticks move
 *             it; nothing else does.
 *   offset    a correction owned by something else -- an auto-takeoff climb,
 *             terrain following, a landing descent. Added on top, slewed in
 *             gently rather than stepped.
 *   target    desired + offset. What the controller actually flies to.
 *
 * They are separate so that an offset can appear and disappear without
 * disturbing where the pilot put the aircraft. Fold them into one variable and
 * a takeoff offset that ends leaves the aircraft holding a position the pilot
 * never asked for.
 *
 * Feed-forward, and why the stick moves the target
 * ------------------------------------------------
 * A stick is a VELOCITY demand, not a position: inputVelocity() integrates it
 * into `desired`, so the target slides along and the aircraft follows. Without
 * a feed-forward term the aircraft would have to fall behind before the P term
 * produced any speed at all -- a permanent lag proportional to the speed. So
 * the shaped stick velocity is added to the output directly, and the position
 * term is left to correct what the feed-forward gets wrong.
 *
 * inputVelocity() must be called EVERY cycle while a mode is using it, with
 * zero when the sticks are centred. Zero is how you say "stop": it decelerates
 * over the acceleration limit rather than dropping the demand instantly. A
 * mode that simply stops calling it leaves the last feed-forward standing.
 *
 * Target runaway
 * --------------
 * If the sticks ask for 5 m/s into a wind the aircraft can only make 3 m/s
 * against, the target slides away at 5 m/s and the aircraft never catches it.
 * The error grows without bound, and when the stick centres the aircraft
 * charges at a target far ahead of it. So the error is CAPPED: past the cap
 * the desired position is dragged along with the aircraft rather than running
 * ahead of it. ArduPilot solves the same problem by feeding a saturation
 * signal back from the lean-angle limit; the cap needs nothing from
 * downstream, which matters while the velocity controller does not exist yet.
 *
 * The cap applies to inputVelocity() ONLY. setDesiredPosition() is exempt,
 * because a mission leg is a hundred metres away on purpose and capping it
 * would quietly move the waypoint to just ahead of the aircraft.
 */

#ifndef CONTROLLERS_POSITION_POSITION_HPP_
#define CONTROLLERS_POSITION_POSITION_HPP_

#include "common.hpp"

// Proportional gains, 1/s. A gain of 1.0 means "one metre of error asks for
// one metre per second", which is ArduPilot's default for both axes and a
// sensible place to start. These only act near the target -- past the linear
// distance the square root takes over.
#define POSITION_P_NE                 1.0f
#define POSITION_P_D                  1.0f

// Speed limits, m/s, all POSITIVE. Up and down are separate because a
// multirotor descends badly: it falls into its own downwash, and the usual
// answer is to come down at about half the speed it climbs.
#define POSITION_SPEED_NE             5.0f
#define POSITION_SPEED_UP             2.5f
#define POSITION_SPEED_DOWN           1.5f

// Acceleration limits, m/s^2. These do two jobs: they shape the stick input so
// a slammed stick does not become a step velocity demand, and they set where
// the square-root controller stops being a square root.
#define POSITION_ACCEL_NE             2.5f
#define POSITION_ACCEL_D              2.5f

// How far the target may get ahead of the aircraft before it stops running --
// see the note on target runaway. Roughly the stopping distance at the speed
// limit (v^2 / 2a), which is the furthest ahead a target can be while still
// being reachable in one deceleration.
#define POSITION_MAX_ERROR_NE         5.0f
#define POSITION_MAX_ERROR_D          3.0f

// What the last update() could not deliver. Not decoration: a velocity
// controller that keeps integrating toward a speed the aircraft is not allowed
// to fly winds up, and a mode that cannot tell "holding position" from
// "pinned against the speed limit" cannot report honestly to the pilot.
struct PositionLimits {
	bool speed_ne;    // horizontal velocity target was cut to the speed limit
	bool speed_up;    // climb rate was cut
	bool speed_down;  // descent rate was cut
	bool pos_error;   // stick input was held back to stop the target running away
};

class Position {
public:
	Position();

	// -----------------------------------------------------------------------
	// Engaging
	// -----------------------------------------------------------------------

	// Snap the target to where the aircraft is NOW and clear everything else:
	// offset, feed-forward, limit flags. Call it on every mode entry.
	//
	// This is the whole answer to "what position does it hold when the mode
	// starts": wherever it was. Anything else -- a target left over from the
	// last mode, or a zero that means the origin -- is a lurch the moment the
	// mode engages.
	void reset(const Vector3f &pos_ned);

	// True once reset() has been called. Before that update() does nothing and
	// the outputs stay zero, rather than flying to the NED origin.
	bool isActive(void) const { return _active; }

	// -----------------------------------------------------------------------
	// Moving the target
	// -----------------------------------------------------------------------

	// Put the target somewhere directly. For a mission leg or a return point,
	// not for sticks. A step here IS a step, and it is deliberately NOT capped
	// -- the square-root controller is what makes a distant target civilised,
	// by approaching it no faster than it can stop from.
	void setDesiredPosition(const Vector3f &pos_ned);

	// The pilot's stick, as a velocity in m/s. Integrates into the desired
	// position and becomes the feed-forward. The requested velocity is
	// ACCELERATION SHAPED first, so a slammed stick ramps rather than steps.
	//
	// Call every cycle while the mode uses it, with zero for centred sticks.
	void inputVelocity(const Vector3f &vel_ned, float dt);

	// -----------------------------------------------------------------------
	// The offset -- see the header note. Owned by whatever is doing the
	// takeoff, the terrain follow or the landing, not by the pilot.
	// -----------------------------------------------------------------------

	// Where the offset should end up. It slews there under the same speed and
	// acceleration limits as everything else, so asking for +1 m of takeoff
	// climb produces a climb, not a jump.
	void setOffsetTarget(const Vector3f &offset_ned);

	// Put the offset somewhere with no slew at all. For mode entry, where
	// there is nothing to be smooth about yet.
	void setOffset(const Vector3f &offset_ned);

	// -----------------------------------------------------------------------
	// Run it. `pos_ned` is the estimator's position, `dt` the time since the
	// last call. A non-finite or non-positive dt is ignored: the outputs hold
	// their last value rather than being computed from a bad clock.
	// -----------------------------------------------------------------------
	void update(const Vector3f &pos_ned, float dt);

	// -----------------------------------------------------------------------
	// Outputs
	// -----------------------------------------------------------------------

	// What the velocity controller should fly. Feed-forward plus the offset's
	// own motion plus the position correction, held inside the speed limits.
	Vector3f getVelocityTarget(void) const { return _vel_target; }

	// target - measured, in metres. The number to watch when tuning.
	Vector3f getPositionError(void) const { return _pos_error; }

	const PositionLimits &getLimits(void) const { return _limits; }

	Vector3f getDesiredPosition(void) const { return _pos_desired; }
	Vector3f getOffset(void) const { return _pos_offset; }
	Vector3f getOffsetTarget(void) const { return _pos_offset_target; }
	Vector3f getTargetPosition(void) const { return _pos_desired + _pos_offset; }

	// The shaped stick velocity -- what inputVelocity() actually acted on
	// after the acceleration limit, which is not what was asked for.
	Vector3f getDesiredVelocity(void) const { return _vel_desired; }

	// -----------------------------------------------------------------------
	// Tuning
	// -----------------------------------------------------------------------
	void setPositionGains(float p_ne, float p_d);
	void setSpeedLimits(float speed_ne, float speed_up, float speed_down);
	void setAccelLimits(float accel_ne, float accel_d);
	void setMaxPositionError(float max_ne, float max_d);

	float getPNE(void) const { return _p_ne; }
	float getPD(void) const { return _p_d; }
	float getSpeedNE(void) const { return _speed_ne; }
	float getSpeedUp(void) const { return _speed_up; }
	float getSpeedDown(void) const { return _speed_down; }
	float getAccelNE(void) const { return _accel_ne; }
	float getAccelD(void) const { return _accel_d; }

	// -----------------------------------------------------------------------
	// The square-root controller, exposed because it IS the controller and
	// deserves testing on its own, away from any vehicle state.
	//
	// Returns the correction rate for `error`, shaped so the vehicle can still
	// stop within the distance remaining under `accel_max`:
	//
	//   |error| <= accel_max / p^2     linear, p * error
	//   |error| >  accel_max / p^2     sqrt(2 * accel_max * (|error| - d/2))
	//
	// With accel_max <= 0 it is a plain P controller. With p <= 0 it is pure
	// sqrt. `dt` bounds the result to error/dt -- the rate that lands exactly
	// on the target next cycle -- so the last step of an approach cannot
	// overshoot and start a limit cycle.
	// -----------------------------------------------------------------------
	static float sqrtController(float error, float p, float accel_max, float dt);

	// The same thing for a horizontal error, applied to the VECTOR rather than
	// to each axis. Per-axis shaping would corner-cut on a diagonal: the two
	// axes decelerate independently and the path bows away from the straight
	// line to the target.
	static void sqrtControllerNE(float error_n, float error_e, float p,
			float accel_max, float dt, float &out_n, float &out_e);

private:
	// Moves _pos_offset toward _pos_offset_target under the limits, and leaves
	// the rate it did it at in _vel_offset so the position loop does not have
	// to lag a moving offset.
	void slewOffset(float dt);

	// Drags _pos_desired back toward the aircraft if the error has grown past
	// the cap. Sets _limits.pos_error when it does. Called from
	// inputVelocity(), because the stick is the only thing that can run the
	// target away -- see the note there.
	void capPositionError(void);

	// Holds _vel_target inside the speed limits, NE as a vector and D with its
	// two separate one-sided limits.
	void limitVelocityTarget(void);

	bool _active;

	Vector3f _pos_desired;        // where the pilot or the mission put it
	Vector3f _pos_offset;         // the current offset
	Vector3f _pos_offset_target;  // where the offset is heading
	Vector3f _vel_desired;        // shaped stick velocity, the feed-forward
	Vector3f _vel_offset;         // the offset's own rate of change

	Vector3f _pos_measured;       // last position update() was given
	Vector3f _pos_error;
	Vector3f _vel_target;
	PositionLimits _limits;

	float _p_ne;
	float _p_d;
	float _speed_ne;
	float _speed_up;
	float _speed_down;
	float _accel_ne;
	float _accel_d;
	float _max_error_ne;
	float _max_error_d;
};

#endif /* CONTROLLERS_POSITION_POSITION_HPP_ */
