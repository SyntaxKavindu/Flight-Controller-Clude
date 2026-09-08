/*
 * Indicator.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Indicator.hpp"

namespace {

// Slot the given phase is in, derived from the clock rather than counted, so
// the pattern cannot drift and a missed update() shows a late edge rather than
// a stalled or skipped cycle.
//
// The subtraction is on unsigned values, so it stays correct across the
// HAL_GetTick() wrap at 2^32 ms (~49 days) -- which is inside the life of a
// tethered bench session, never mind an airframe left powered.
inline uint8_t slotOf(uint32_t now, uint32_t phase_start) {
	const uint32_t elapsed = now - phase_start;
	return (uint8_t) ((elapsed / INDICATOR_SLOT_MS) % INDICATOR_SLOTS);
}

} // namespace

Indicator::Indicator(const IndicatorLed &system, const IndicatorLed &arm,
		const IndicatorLed &gps) :
		_out { { system, INDICATOR_PAT_SYS_ERROR, 0u, -1 },
		       { arm,    INDICATOR_PAT_ARM_SAFE,  0u, -1 },
		       { gps,    INDICATOR_PAT_GPS_SEARCH, 0u, -1 } },
		// Pessimistic on purpose. Until something tells this class otherwise
		// the aircraft has no verified health, no arming interlock and no fix,
		// and every one of those is safer shown as the bad case: an operator
		// who sees "OK" because nobody has reported yet is being lied to.
		_systemState { SystemState::ERROR },
		_armState { ArmState::DISARMED },
		_gpsState { GPSState::UNLOCKED },
		_initialized { false }, _lampTest { false }, _lampTestStart { 0u } {
}

void Indicator::init() {
	_lampTestStart = HAL_GetTick();
	_lampTest = true;
	_initialized = true;

	// Straight to the GPIO rather than through drive(): the lamp test is the
	// one moment the patterns are deliberately ignored, and forcing `written`
	// here means the first post-test update() writes whatever the pattern
	// actually wants instead of assuming the LED is already there.
	for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
		Output &o = _out[i];
		HAL_GPIO_WritePin(o.led.port, o.led.pin,
				o.led.active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
		o.written = 1;
		// Every channel starts its cycle when the lamp test ends, so the three
		// patterns come up in phase and read as one panel rather than three
		// unrelated lights.
		o.phase_start = _lampTestStart + INDICATOR_LAMP_TEST_MS;
	}
}

void Indicator::update() {
	// Called before init(), the pins may not be configured yet. Doing nothing
	// is right: init() is what claims them.
	if (!_initialized) {
		return;
	}

	const uint32_t now = HAL_GetTick();

	if (_lampTest) {
		// Unsigned difference, so this is wrap-safe.
		if ((now - _lampTestStart) < INDICATOR_LAMP_TEST_MS) {
			return; // everything stays lit
		}
		_lampTest = false;
	}

	for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
		Output &o = _out[i];
		const uint8_t slot = slotOf(now, o.phase_start);
		drive(o, ((o.pattern >> slot) & 1u) != 0u);
	}
}

void Indicator::drive(Output &o, bool on) {
	const int8_t want = on ? 1 : 0;
	// The GPIO write is cheap, but at 50 Hz across three channels it is 150
	// pointless bus accesses a second for a panel that changes maybe twice a
	// minute. Skipping the unchanged ones costs one comparison.
	if (o.written == want) {
		return;
	}
	o.written = want;
	const bool level = on == o.led.active_high;
	HAL_GPIO_WritePin(o.led.port, o.led.pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Indicator::setPattern(Channel ch, uint16_t pattern) {
	Output &o = _out[ch];
	// The dispatcher pushes the state every pass, so most calls here are a
	// no-op. Restarting the phase on those would hold the LED at slot 0 for
	// ever and every pattern would render as a solid light.
	if (o.pattern == pattern) {
		return;
	}
	o.pattern = pattern;
	o.phase_start = HAL_GetTick();
}

void Indicator::setSystemState(SystemState state) {
	_systemState = state;
	setPattern(CHANNEL_SYSTEM, (state == SystemState::OK)
			? INDICATOR_PAT_SYS_OK : INDICATOR_PAT_SYS_ERROR);
}

void Indicator::setArmState(ArmState state) {
	_armState = state;
	setPattern(CHANNEL_ARM, (state == ArmState::ARMED)
			? INDICATOR_PAT_ARM_LIVE : INDICATOR_PAT_ARM_SAFE);
}

void Indicator::setGPSState(GPSState state) {
	_gpsState = state;
	setPattern(CHANNEL_GPS, (state == GPSState::LOCKED)
			? INDICATOR_PAT_GPS_FIX : INDICATOR_PAT_GPS_SEARCH);
}
