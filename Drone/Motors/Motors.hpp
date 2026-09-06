/*
 * Motors.hpp
 *
 *  Created on: Sep 2, 2026
 *      Author: KAVINDU
 *
 * The output stage: owns every ESC, mixes the controllers' roll/pitch/yaw/
 * throttle demands into per-motor thrust, and gates the whole thing behind a
 * spool state machine modelled on ArduPilot's AP_MotorsMulticopter.
 *
 * Where it sits
 * -------------
 *   RC sticks -> modes -> Position -> Velocity -> Attitude -> Rate -> MOTORS
 *
 * The rate controller is the last thing to run before this. It produces three
 * normalised torque demands, and the flight mode produces a collective thrust
 * demand; setControlInput() takes all four, and update() turns them into four
 * DShot frames. Nothing above this layer knows how many motors there are or
 * where they sit.
 *
 * The spool state machine
 * -----------------------
 * Motors do not go from stopped to flying in one step, and the reason is not
 * comfort: a propeller that jumps to a commanded thrust is a propeller that
 * can lift a corner of an aircraft nobody is holding. So there is a ramp, and
 * a state machine that owns it:
 *
 *   SHUT_DOWN            nothing turns. The only state reachable while
 *                        disarmed, and the state a disarm forces immediately
 *                        from anywhere.
 *   GROUND_IDLE          every motor at MOTORS_SPIN_ARM, attitude mixing OFF.
 *                        "Armed and you can see it" -- the visible signal to
 *                        anyone near the aircraft that it is live.
 *   SPOOLING_UP          ramping to full authority over MOTORS_SPOOL_UP_TIME.
 *                        Stabilised, but the collective is capped by the ramp.
 *   THROTTLE_UNLIMITED   flying.
 *   SPOOLING_DOWN        ramping back to idle over MOTORS_SPOOL_DOWN_TIME.
 *
 * Two rules give the machine its shape. SHUT_DOWN can only be left through
 * GROUND_IDLE, so nothing can command thrust straight out of stopped; and
 * disarming cuts to SHUT_DOWN instantly rather than ramping, because a disarm
 * is a kill and a kill that takes half a second is not a kill. (ArduPilot
 * ramps down on disarm instead. This is a deliberate difference.)
 *
 * Who decides the state
 * ---------------------
 * Two inputs, and this class derives the rest:
 *
 *   setArmed()          the arming interlock, from the arming class when it
 *                       exists. Disarmed is always SHUT_DOWN, no exceptions.
 *   notifyPilotInput()  "a stick actually moved just now", from the RC layer.
 *
 * Armed with recent stick input means THROTTLE_UNLIMITED, so the aircraft
 * spools up. Armed with no stick input for MOTORS_PILOT_TIMEOUT drops back to
 * GROUND_IDLE, so an aircraft left armed on the ground settles to idle instead
 * of sitting at flight authority waiting for a knock. Arming alone does NOT
 * spool up: a freshly armed aircraft starts with its input already stale, so
 * it goes to GROUND_IDLE and waits for the pilot.
 *
 * What "thrust" means here
 * ------------------------
 * Every number in this class is normalised. Inputs are torque demands in
 * [-1, 1] and a collective in [0, 1]; outputs are per-motor thrust in [0, 1]
 * BEFORE the spin_min/spin_max band is applied. The band is applied last, on
 * the way to the ESC, because it is a property of the motor and propeller --
 * where they actually start turning, and where they stop giving more -- and
 * not of the control problem.
 *
 * Saturation is reported, not hidden
 * ----------------------------------
 * Four motors cannot always deliver what all four demands ask for. When they
 * cannot, the mixer gives things up in a fixed order -- yaw first, then the
 * collective, and roll/pitch last, because roll and pitch are what keeps the
 * aircraft the right way up -- and records what it gave up in getLimits().
 * A rate controller that integrates an error it was never allowed to correct
 * winds up; the limit flags are what lets it stop.
 */

#ifndef MOTORS_MOTORS_HPP_
#define MOTORS_MOTORS_HPP_

#include "common.hpp"
#include "ESC.hpp"

enum class MOTORS_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1
};

// Where the machine actually is.
enum class SpoolState : uint8_t {
	SHUT_DOWN = 0,
	GROUND_IDLE = 1,
	SPOOLING_UP = 2,
	THROTTLE_UNLIMITED = 3,
	SPOOLING_DOWN = 4
};

// Where it has been asked to be. Deliberately a SMALLER set than SpoolState:
// the two ramping states are how the machine gets somewhere, not somewhere
// anything is allowed to ask for.
enum class DesiredSpoolState : uint8_t {
	SHUT_DOWN = 0,
	GROUND_IDLE = 1,
	THROTTLE_UNLIMITED = 2
};

// What the mixer could not deliver on the last update(). Feed these to the
// rate controller's anti-windup: an axis that is limited must not keep
// integrating an error it has no authority left to correct.
struct MotorLimits {
	bool roll_pitch;      // roll/pitch demand was scaled down to fit
	bool yaw;             // yaw demand was scaled down or dropped
	bool throttle_lower;  // collective was raised to keep a motor above zero
	bool throttle_upper;  // collective was cut to keep a motor below full
};

#define MOTORS_MAX_MOTORS       4u

// Motor and propeller limits, as fractions of full output.
//
// SPIN_ARM is the ground-idle level: turning, clearly visible, not enough to
// go anywhere. SPIN_MIN is the lowest level in flight -- below it a motor can
// stall or desynchronise, and a motor that has stopped cannot be commanded
// back quickly enough to save an attitude. SPIN_MAX leaves headroom at the top
// so the mixer still has somewhere to go for roll and pitch at full throttle.
//
// All three are AIRFRAME properties. These are conservative starting points
// and every one of them should be measured on the bench before flight.
#define MOTORS_SPIN_ARM         0.10f
#define MOTORS_SPIN_MIN         0.15f
#define MOTORS_SPIN_MAX         0.95f

// Ramp durations, seconds. Down is faster than up: spooling up slowly is a
// safety margin, coming down slowly is just a delay.
#define MOTORS_SPOOL_UP_TIME    0.50f
#define MOTORS_SPOOL_DOWN_TIME  0.30f

// How long the sticks may sit still before the aircraft drops back to ground
// idle. Long enough not to trip while a pilot lines up a shot, short enough
// that an aircraft nobody is flying does not stay at flight authority.
#define MOTORS_PILOT_TIMEOUT_S  3.0f

class Motors {
public:
	// One entry per motor, in the frame's motor order -- see the table in
	// Motors.cpp for which physical corner each index is.
	struct Channel {
		TIM_HandleTypeDef *htim;
		uint32_t channel;   // a TIM_CHANNEL_x constant
	};

	explicit Motors(const Channel (&channels)[MOTORS_MAX_MOTORS]);

	// Brings up every ESC and computes the mixing table. ERROR if any ESC
	// refuses the rate -- a partly-working set of motors is not flyable, and
	// this is the last moment it can be said so cheaply.
	MOTORS_StatusTypeDef init(DShotRate rate, uint32_t timer_clock_hz);

	// -----------------------------------------------------------------------
	// Control inputs. Set these AFTER the rate controller has run, then call
	// update(). They are held until overwritten, so a controller that stops
	// running leaves its last demand standing -- which is why the pilot
	// timeout below exists.
	//
	// Sign conventions match the estimator's body frame: +roll is right wing
	// down, +pitch is nose up, +yaw is nose right.
	// -----------------------------------------------------------------------
	void setRoll(float roll);          // [-1, 1] torque demand
	void setPitch(float pitch);        // [-1, 1]
	void setYaw(float yaw);            // [-1, 1]
	void setThrottle(float throttle);  // [0, 1] collective thrust demand

	// All four at once. This is what the rate controller will call.
	void setControlInput(float roll, float pitch, float yaw, float throttle);

	// -----------------------------------------------------------------------
	// Arming and pilot activity
	// -----------------------------------------------------------------------

	// The arming interlock. Disarming takes effect on the spot: the state goes
	// to SHUT_DOWN and the next update() stops every motor.
	void setArmed(bool armed);
	bool isArmed(void) const { return _armed; }

	// "A stick moved." Call from the RC layer whenever the pilot's input is
	// genuinely live -- past the deadzone on any axis, not merely a receiver
	// frame arriving, or a failsafe holding the last stick positions would
	// look exactly like a pilot holding them.
	void notifyPilotInput(void);

	// Seconds since the last notifyPilotInput(), as counted by update(). Stops
	// growing once it is past the timeout.
	float getPilotIdleTime(void) const { return _pilot_idle_s; }

	// -----------------------------------------------------------------------
	// The fast-loop entry point. Advances the spool ramp by `dt` seconds, mixes,
	// and puts a frame on every ESC -- every call, in every state, including
	// SHUT_DOWN. An ESC that stops hearing frames cuts the motor on its own,
	// so silence is never how this class says "stop"; a MOTOR_STOP frame is.
	//
	// A non-finite or non-positive dt freezes the ramps for that call but
	// still drives the ESCs, because a bad clock is not a reason to go quiet.
	// -----------------------------------------------------------------------
	void update(float dt);

	// -----------------------------------------------------------------------
	// State
	// -----------------------------------------------------------------------
	SpoolState getSpoolState(void) const { return _spool_state; }
	DesiredSpoolState getDesiredSpoolState(void) const { return _desired_state; }

	// 0 at ground idle, 1 at full authority, in between while ramping.
	float getSpoolRatio(void) const { return _spool_ratio; }

	// Per-motor output for the last update(), in [0, 1], AFTER the spin
	// band -- what the ESC was actually told. 0 for an out-of-range index.
	float getThrust(uint8_t motor) const;

	const MotorLimits &getLimits(void) const { return _limits; }

	// The mixing factors, for the boot report and for tests.
	float getRollFactor(uint8_t motor) const;
	float getPitchFactor(uint8_t motor) const;
	float getYawFactor(uint8_t motor) const;

	static uint8_t getMotorCount(void) { return (uint8_t) MOTORS_MAX_MOTORS; }

	// Direct access, for a boot report or a diagnostic that wants an ESC's
	// own counters. nullptr for an out-of-range index.
	const ESC *getEsc(uint8_t motor) const;

	// -----------------------------------------------------------------------
	// Tuning. Every one of these is an airframe property with a conservative
	// default; see the #defines above.
	// -----------------------------------------------------------------------
	void setSpinLimits(float spin_arm, float spin_min, float spin_max);
	void setSpoolTimes(float up_s, float down_s);
	void setPilotTimeout(float timeout_s);

private:
	// Fills the roll/pitch/yaw factor tables from the frame geometry and
	// normalises each axis so a demand of 1 means "all the authority this
	// frame has on that axis".
	void buildMixingTable(void);

	// Chooses where the machine should be, from armed state and pilot idle.
	DesiredSpoolState chooseDesiredState(void) const;

	// One step of the state machine.
	void advanceSpool(float dt);

	// Mixes the held demands into _thrust[], honouring `throttle_max` (the
	// spool ramp's cap) and `floor` (the lowest output a spinning motor may
	// be given), and records what had to be given up in _limits.
	void mix(float throttle_max, float floor);

	// Pushes _thrust[] to the ESCs.
	void writeEscs(void);

	ESC _escs[MOTORS_MAX_MOTORS];

	float _roll_factor[MOTORS_MAX_MOTORS];
	float _pitch_factor[MOTORS_MAX_MOTORS];
	float _yaw_factor[MOTORS_MAX_MOTORS];

	float _roll_in;
	float _pitch_in;
	float _yaw_in;
	float _throttle_in;

	float _thrust[MOTORS_MAX_MOTORS];
	MotorLimits _limits;

	bool _initialized;
	bool _armed;

	SpoolState _spool_state;
	DesiredSpoolState _desired_state;
	float _spool_ratio;
	float _pilot_idle_s;

	float _spin_arm;
	float _spin_min;
	float _spin_max;
	float _spool_up_time_s;
	float _spool_down_time_s;
	float _pilot_timeout_s;
};

#endif /* MOTORS_MOTORS_HPP_ */
