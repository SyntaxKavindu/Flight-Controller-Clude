/*
 * Motors.cpp
 *
 *  Created on: Sep 2, 2026
 *      Author: KAVINDU
 */

#include "Motors.hpp"

#include <math.h>   // cosf, fabsf, isfinite

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;

// ===========================================================================
// FRAME GEOMETRY -- quad X, ArduPilot motor numbering and spin directions.
//
//              front                  M1  front right, CCW
//        M3            M1             M2  rear left,   CCW
//           \        /                M3  front left,  CW
//             \    /                  M4  rear right,  CW
//              +--+
//             /    \                  Seen from ABOVE, nose up the page.
//           /        \                CCW and CW are the propellers.
//        M2            M4
//
// angle_deg is measured CLOCKWISE FROM THE NOSE, so it is where the motor
// sits, not which way it points.
//
// yaw_factor is +1 for a COUNTER-clockwise propeller. A propeller spinning one
// way drags the airframe the other, so speeding up the CCW pair turns the nose
// RIGHT, which is +yaw. Getting a sign wrong here does not show up as a gentle
// drift -- the yaw axis runs away the instant it leaves the ground -- so treat
// this table as the one thing to check against the physical build.
// ===========================================================================
struct FrameMotor {
	float angle_deg;
	float yaw_factor;
};

const FrameMotor kFrame[MOTORS_MAX_MOTORS] = {
	{   45.0f,  1.0f },   // M1 front right, CCW
	{ -135.0f,  1.0f },   // M2 rear left,   CCW
	{  -45.0f, -1.0f },   // M3 front left,  CW
	{  135.0f, -1.0f },   // M4 rear right,  CW
};

float clampf(float v, float lo, float hi) {
	if (!isfinite(v)) {
		return lo;
	}
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

// Same, but for a CONTROL INPUT, where the two differ on NaN. A NaN means the
// controller upstream has come apart, and on a symmetric axis the low end is
// FULL DEFLECTION -- clamping to it would turn a broken controller into a hard
// roll. Neutral is the only safe reading. (For throttle the two agree, since
// neutral and the low end are both zero.)
float sanitiseInput(float v, float lo, float hi) {
	if (!isfinite(v)) {
		return 0.0f;
	}
	return clampf(v, lo, hi);
}

} // namespace

Motors::Motors(const Channel (&channels)[MOTORS_MAX_MOTORS]) :
		_escs { ESC(channels[0].htim, channels[0].channel),
		        ESC(channels[1].htim, channels[1].channel),
		        ESC(channels[2].htim, channels[2].channel),
		        ESC(channels[3].htim, channels[3].channel) },
		_roll_factor { }, _pitch_factor { }, _yaw_factor { },
		_roll_in { 0.0f }, _pitch_in { 0.0f }, _yaw_in { 0.0f }, _throttle_in { 0.0f },
		_thrust { }, _limits { }, _initialized { false }, _armed { false },
		_spool_state { SpoolState::SHUT_DOWN },
		_desired_state { DesiredSpoolState::SHUT_DOWN },
		_spool_ratio { 0.0f }, _pilot_idle_s { MOTORS_PILOT_TIMEOUT_S },
		_spin_arm { MOTORS_SPIN_ARM }, _spin_min { MOTORS_SPIN_MIN },
		_spin_max { MOTORS_SPIN_MAX }, _spool_up_time_s { MOTORS_SPOOL_UP_TIME },
		_spool_down_time_s { MOTORS_SPOOL_DOWN_TIME },
		_pilot_timeout_s { MOTORS_PILOT_TIMEOUT_S } {
	buildMixingTable();
}

MOTORS_StatusTypeDef Motors::init(DShotRate rate, uint32_t timer_clock_hz) {
	_initialized = false;
	_armed = false;
	_spool_state = SpoolState::SHUT_DOWN;
	_desired_state = DesiredSpoolState::SHUT_DOWN;
	_spool_ratio = 0.0f;

	// Already stale, so the first thing an arm does is settle to ground idle.
	_pilot_idle_s = _pilot_timeout_s;

	buildMixingTable();

	// Every ESC, or none. Flying three quarters of a quadcopter is not a
	// degraded mode, it is a crash, and this is the cheapest place to say so.
	bool all_ok = true;
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		if (_escs[i].init(rate, timer_clock_hz) != ESC_StatusTypeDef::OK) {
			all_ok = false;
		}
		_thrust[i] = 0.0f;
	}
	if (!all_ok) {
		return MOTORS_StatusTypeDef::ERROR;
	}

	_initialized = true;
	return MOTORS_StatusTypeDef::OK;
}

void Motors::buildMixingTable(void) {
	// ArduPilot's formulation: a motor's leverage on an axis is the cosine of
	// the angle between its arm and that axis. Pitch acts along the nose, so
	// it is cos(angle); roll acts across the airframe, 90 degrees round.
	float max_roll = 0.0f;
	float max_pitch = 0.0f;
	float max_yaw = 0.0f;

	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		_roll_factor[i] = cosf((kFrame[i].angle_deg + 90.0f) * DEG_TO_RAD);
		_pitch_factor[i] = cosf(kFrame[i].angle_deg * DEG_TO_RAD);
		_yaw_factor[i] = kFrame[i].yaw_factor;

		if (fabsf(_roll_factor[i]) > max_roll) {
			max_roll = fabsf(_roll_factor[i]);
		}
		if (fabsf(_pitch_factor[i]) > max_pitch) {
			max_pitch = fabsf(_pitch_factor[i]);
		}
		if (fabsf(_yaw_factor[i]) > max_yaw) {
			max_yaw = fabsf(_yaw_factor[i]);
		}
	}

	// Normalise each axis independently so that a demand of 1.0 means "all the
	// authority this frame has on this axis". Without it the meaning of a
	// controller output would change with the frame geometry, and the same PID
	// gains would behave differently on a quad and a hex.
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		if (max_roll > 0.0f) {
			_roll_factor[i] /= max_roll;
		}
		if (max_pitch > 0.0f) {
			_pitch_factor[i] /= max_pitch;
		}
		if (max_yaw > 0.0f) {
			_yaw_factor[i] /= max_yaw;
		}
	}
}

// ---------------------------------------------------------------------------
// Control inputs
// ---------------------------------------------------------------------------
void Motors::setRoll(float roll)         { _roll_in = sanitiseInput(roll, -1.0f, 1.0f); }
void Motors::setPitch(float pitch)       { _pitch_in = sanitiseInput(pitch, -1.0f, 1.0f); }
void Motors::setYaw(float yaw)           { _yaw_in = sanitiseInput(yaw, -1.0f, 1.0f); }
void Motors::setThrottle(float throttle) { _throttle_in = sanitiseInput(throttle, 0.0f, 1.0f); }

void Motors::setControlInput(float roll, float pitch, float yaw, float throttle) {
	setRoll(roll);
	setPitch(pitch);
	setYaw(yaw);
	setThrottle(throttle);
}

// ---------------------------------------------------------------------------
// Arming and pilot activity
// ---------------------------------------------------------------------------
void Motors::setArmed(bool armed) {
	if (armed) {
		// Arming motors that were never brought up would leave the state
		// machine claiming to be live while every write() fails.
		if (!_initialized) {
			return;
		}
		// Arming is itself usually a stick gesture, and treating it as pilot
		// input would spool the aircraft up the moment it armed. Start stale:
		// the pilot has to move a stick AFTER arming to get thrust.
		_pilot_idle_s = _pilot_timeout_s;
		_armed = true;
		return;
	}

	// Disarm cuts. Not a ramp down, not a state the machine walks through --
	// a disarm is the one command whose whole purpose is to stop the motors
	// NOW, and one that took MOTORS_SPOOL_DOWN_TIME to arrive would not be
	// worth reaching for. (ArduPilot ramps here instead; this is deliberate.)
	_armed = false;
	_spool_state = SpoolState::SHUT_DOWN;
	_desired_state = DesiredSpoolState::SHUT_DOWN;
	_spool_ratio = 0.0f;
}

void Motors::notifyPilotInput(void) {
	_pilot_idle_s = 0.0f;
}

// ---------------------------------------------------------------------------
// The fast-loop entry point
// ---------------------------------------------------------------------------
void Motors::update(float dt) {
	if (!_initialized) {
		return;
	}

	// A bad dt must not move the ramps -- a NaN would poison _spool_ratio
	// permanently -- but it is not a reason to stop feeding the ESCs.
	const bool dt_ok = isfinite(dt) && (dt > 0.0f);

	if (dt_ok) {
		// Stop accumulating once it is past the timeout. Left unbounded this
		// grows for the whole flight and eventually loses its resolution
		// against the small dt being added to it.
		if (_pilot_idle_s < _pilot_timeout_s) {
			_pilot_idle_s += dt;
			if (_pilot_idle_s > _pilot_timeout_s) {
				_pilot_idle_s = _pilot_timeout_s;
			}
		}
	}

	_desired_state = chooseDesiredState();
	advanceSpool(dt_ok ? dt : 0.0f);

	switch (_spool_state) {
	case SpoolState::SHUT_DOWN:
		// Nothing turns, and nothing accumulates: the limit flags describe a
		// mix that did not happen.
		_limits = MotorLimits { };
		for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
			_thrust[i] = 0.0f;
		}
		break;

	case SpoolState::GROUND_IDLE:
		// Deliberately NOT stabilised. On the ground the aircraft is held by
		// its own legs, and a mixer correcting a tilt it cannot fix would
		// spin one motor up against the ground while the others sit at idle.
		_limits = MotorLimits { };
		for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
			_thrust[i] = _spin_arm;
		}
		break;

	case SpoolState::SPOOLING_UP:
	case SpoolState::SPOOLING_DOWN:
		// Stabilised, but on a leash: the collective cannot exceed the ramp,
		// and the floor slides from the ground-idle level up to the in-flight
		// minimum so there is no step at either end of the ramp.
		mix(_spool_ratio, _spin_arm + (_spin_min - _spin_arm) * _spool_ratio);
		break;

	case SpoolState::THROTTLE_UNLIMITED:
	default:
		mix(1.0f, _spin_min);
		break;
	}

	writeEscs();
}

DesiredSpoolState Motors::chooseDesiredState(void) const {
	if (!_armed) {
		return DesiredSpoolState::SHUT_DOWN;
	}
	// Armed but nobody is flying it. Settling to idle costs a spool-up when
	// the pilot comes back, and buys an aircraft that is not sitting at full
	// authority waiting for a nudge or a stale controller output.
	if (_pilot_idle_s >= _pilot_timeout_s) {
		return DesiredSpoolState::GROUND_IDLE;
	}
	return DesiredSpoolState::THROTTLE_UNLIMITED;
}

void Motors::advanceSpool(float dt) {
	switch (_spool_state) {
	case SpoolState::SHUT_DOWN:
		_spool_ratio = 0.0f;
		// Disarmed always means SHUT_DOWN -- chooseDesiredState() is the one
		// place that decides, so this is the only test needed. A second
		// _armed check here would be unreachable, and unreachable safety
		// code is code nothing keeps honest.
		if (_desired_state == DesiredSpoolState::SHUT_DOWN) {
			break;
		}
		// Everything leaves SHUT_DOWN through GROUND_IDLE. Nothing gets to
		// jump from stopped to commanding thrust, however urgent it thinks
		// it is.
		_spool_state = SpoolState::GROUND_IDLE;
		break;

	case SpoolState::GROUND_IDLE:
		_spool_ratio = 0.0f;
		if (_desired_state == DesiredSpoolState::SHUT_DOWN) {
			_spool_state = SpoolState::SHUT_DOWN;
		} else if (_desired_state == DesiredSpoolState::THROTTLE_UNLIMITED) {
			_spool_state = SpoolState::SPOOLING_UP;
		}
		break;

	case SpoolState::SPOOLING_UP:
		if (_desired_state != DesiredSpoolState::THROTTLE_UNLIMITED) {
			_spool_state = SpoolState::SPOOLING_DOWN;
			break;
		}
		// A zero or negative spool time would divide by zero; treat it as
		// "no ramp wanted" rather than trapping.
		if (_spool_up_time_s > 0.0f) {
			_spool_ratio += dt / _spool_up_time_s;
		} else {
			_spool_ratio = 1.0f;
		}
		if (_spool_ratio >= 1.0f) {
			_spool_ratio = 1.0f;
			_spool_state = SpoolState::THROTTLE_UNLIMITED;
		}
		break;

	case SpoolState::THROTTLE_UNLIMITED:
		_spool_ratio = 1.0f;
		if (_desired_state != DesiredSpoolState::THROTTLE_UNLIMITED) {
			_spool_state = SpoolState::SPOOLING_DOWN;
		}
		break;

	case SpoolState::SPOOLING_DOWN:
	default:
		// A pilot who comes back mid-ramp picks up from where the ramp got to,
		// rather than starting again from idle.
		if (_desired_state == DesiredSpoolState::THROTTLE_UNLIMITED) {
			_spool_state = SpoolState::SPOOLING_UP;
			break;
		}
		if (_spool_down_time_s > 0.0f) {
			_spool_ratio -= dt / _spool_down_time_s;
		} else {
			_spool_ratio = 0.0f;
		}
		if (_spool_ratio <= 0.0f) {
			_spool_ratio = 0.0f;
			_spool_state = SpoolState::GROUND_IDLE;
		}
		break;
	}
}

// ---------------------------------------------------------------------------
// The mixer
//
// Four motors have four degrees of freedom and are being asked for four
// things, so on paper it always fits. It does not, because every motor is also
// stuck inside [0, 1]: ask for a big roll at full throttle and the motors that
// should go up have nowhere to go. Something has to give, and WHICH thing
// gives is a safety decision, not an arithmetic one:
//
//   roll and pitch keep the aircraft the right way up      -- given up last
//   the collective decides whether it climbs or descends   -- given up second
//   yaw only decides which way it is pointing              -- given up first
//
// A yaw that lags is a nuisance. A roll that lags is a crash. So yaw is what
// gets squeezed, and everything squeezed is recorded in _limits so the rate
// controller can stop integrating an error it was never allowed to correct.
// ---------------------------------------------------------------------------
void Motors::mix(float throttle_max, float floor) {
	_limits = MotorLimits { };

	// --- 1. Roll and pitch, and the swing they need -------------------------
	float rp[MOTORS_MAX_MOTORS];
	float rp_low = 0.0f;
	float rp_high = 0.0f;
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		rp[i] = _roll_in * _roll_factor[i] + _pitch_in * _pitch_factor[i];
		if (i == 0 || rp[i] < rp_low) {
			rp_low = rp[i];
		}
		if (i == 0 || rp[i] > rp_high) {
			rp_high = rp[i];
		}
	}

	// The gap between the highest and lowest motor is the swing roll and pitch
	// need. More than the whole output range and it cannot fit at ANY
	// collective -- scale both axes down together, which keeps the demanded
	// direction and only loses magnitude.
	const float rp_span = rp_high - rp_low;
	if (rp_span > 1.0f) {
		const float scale = 1.0f / rp_span;
		for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
			rp[i] *= scale;
		}
		rp_low *= scale;
		rp_high *= scale;
		_limits.roll_pitch = true;
	}

	// The lowest motor sits at (collective + rp_low), so the collective has to
	// be at least -rp_low to keep it off zero. While spooling up the collective
	// is capped below that, and the cap wins -- it is the safety limit -- so
	// roll and pitch give up the difference.
	if (rp_low < -throttle_max) {
		const float scale = (rp_low < 0.0f) ? (throttle_max / -rp_low) : 1.0f;
		for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
			rp[i] *= scale;
		}
		rp_low *= scale;
		rp_high *= scale;
		_limits.roll_pitch = true;
	}

	// --- 2. Place the collective -------------------------------------------
	float throttle = _throttle_in;
	if (throttle > throttle_max) {
		throttle = throttle_max;
		_limits.throttle_upper = true;
	}
	const float throttle_floor = -rp_low;        // lowest motor lands on zero
	const float throttle_ceiling = 1.0f - rp_high;  // highest lands on one
	if (throttle < throttle_floor) {
		throttle = throttle_floor;
		_limits.throttle_lower = true;
	}
	if (throttle > throttle_ceiling) {
		throttle = throttle_ceiling;
		_limits.throttle_upper = true;
	}

	// --- 3. Fit yaw into whatever is left ----------------------------------
	float yaw[MOTORS_MAX_MOTORS];
	float yaw_scale = 1.0f;
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		yaw[i] = _yaw_in * _yaw_factor[i];

		// Headroom at THIS motor, in the direction its yaw term is pushing.
		const float base = throttle + rp[i];
		const float room = (yaw[i] >= 0.0f) ? (1.0f - base) : base;
		const float need = fabsf(yaw[i]);
		if (need > room) {
			const float scale = (need > 0.0f) ? (room / need) : 1.0f;
			if (scale < yaw_scale) {
				yaw_scale = scale;
			}
		}
	}
	if (yaw_scale < 1.0f) {
		if (yaw_scale < 0.0f) {
			yaw_scale = 0.0f;
		}
		_limits.yaw = true;
	}

	// --- 4. Out, and onto the motor's own band ------------------------------
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		// The arithmetic above already keeps this inside [0, 1]; the clamp is
		// against rounding at the ends, not against a mistake.
		const float out = clampf(throttle + rp[i] + yaw_scale * yaw[i], 0.0f, 1.0f);

		// `floor` upwards, because below it a motor stalls or desynchronises
		// and stops answering at all. Everything above is the control problem;
		// this last step is the motor and the propeller.
		_thrust[i] = floor + out * (_spin_max - floor);
	}
}

void Motors::writeEscs(void) {
	// Every motor, every update, in every state. An ESC that stops hearing
	// frames cuts its motor on its own after a few tens of milliseconds, so
	// going quiet is never how this class says anything -- a MOTOR_STOP frame
	// is an instruction, silence is a fault.
	for (uint8_t i = 0; i < MOTORS_MAX_MOTORS; i++) {
		_escs[i].write(_thrust[i]);
	}
}

// ---------------------------------------------------------------------------
// State and tuning
// ---------------------------------------------------------------------------
float Motors::getThrust(uint8_t motor) const {
	return (motor < MOTORS_MAX_MOTORS) ? _thrust[motor] : 0.0f;
}

float Motors::getRollFactor(uint8_t motor) const {
	return (motor < MOTORS_MAX_MOTORS) ? _roll_factor[motor] : 0.0f;
}

float Motors::getPitchFactor(uint8_t motor) const {
	return (motor < MOTORS_MAX_MOTORS) ? _pitch_factor[motor] : 0.0f;
}

float Motors::getYawFactor(uint8_t motor) const {
	return (motor < MOTORS_MAX_MOTORS) ? _yaw_factor[motor] : 0.0f;
}

const ESC *Motors::getEsc(uint8_t motor) const {
	return (motor < MOTORS_MAX_MOTORS) ? &_escs[motor] : nullptr;
}

void Motors::setSpinLimits(float spin_arm, float spin_min, float spin_max) {
	_spin_arm = clampf(spin_arm, 0.0f, 1.0f);
	_spin_min = clampf(spin_min, 0.0f, 1.0f);
	_spin_max = clampf(spin_max, 0.0f, 1.0f);

	// An inverted band would make the mixer run motors backwards through the
	// arithmetic -- more demand giving less output. Order them instead of
	// trusting the caller.
	if (_spin_min < _spin_arm) {
		_spin_min = _spin_arm;
	}
	if (_spin_max < _spin_min) {
		_spin_max = _spin_min;
	}
}

void Motors::setSpoolTimes(float up_s, float down_s) {
	_spool_up_time_s = isfinite(up_s) && (up_s > 0.0f) ? up_s : 0.0f;
	_spool_down_time_s = isfinite(down_s) && (down_s > 0.0f) ? down_s : 0.0f;
}

void Motors::setPilotTimeout(float timeout_s) {
	if (!isfinite(timeout_s) || timeout_s < 0.0f) {
		return;
	}
	_pilot_timeout_s = timeout_s;
	if (_pilot_idle_s > _pilot_timeout_s) {
		_pilot_idle_s = _pilot_timeout_s;
	}
}
