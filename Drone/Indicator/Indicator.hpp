/*
 * Indicator.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * Three status LEDs, one per concern: system health, arming, GPS fix.
 *
 * Each LED is driven by a repeating 16-slot bit pattern, one slot per
 * INDICATOR_SLOT_MS, so a "state" is just a pattern word and the whole class is
 * a table lookup plus a shift. That is deliberate: an indicator that costs
 * anything measurable is an indicator that gets called less often and then lies
 * about the state of the aircraft.
 *
 * Timing comes from HAL_GetTick() only -- no cycle counter, no timer channel.
 *
 * NOT hot-loop code, but it is on the ONE path an operator has when telemetry
 * is not connected, which is most of the time on a real airframe. Two
 * properties follow from that and are worth keeping:
 *
 *   - update() never blocks and never allocates. It is safe to call from the
 *     main dispatcher at any rate above ~2x the slot rate.
 *   - a state the class was never told about is DISARMED / UNLOCKED / ERROR,
 *     never OK. Reporting "OK" because nobody has said otherwise is a lie an
 *     operator acts on.
 *   - no state is dark. An unlit LED is indistinguishable from a dead LED, a
 *     wrong pin, or a hung loop, so darkness is reserved for meaning exactly
 *     that. See the pattern table.
 *
 * Wiring lives at the definition of the global `indicator` in Globals.cpp.
 */

#ifndef INDICATOR_INDICATOR_HPP_
#define INDICATOR_INDICATOR_HPP_

#include "common.hpp"

enum class SystemState {
	OK,
	ERROR
};

enum class ArmState {
	ARMED,
	DISARMED
};

enum class GPSState {
	LOCKED,
	UNLOCKED
};

// One LED's wiring. active_high is per channel rather than per board because
// the two halves of a bicolour part are often driven opposite ways, and getting
// it wrong inverts the meaning of the light rather than breaking anything --
// the failure mode that survives a bench test and misleads in the field.
struct IndicatorLed {
	GPIO_TypeDef *port;
	uint16_t pin;
	bool active_high;
};

// Length of one pattern slot, and how many slots make a cycle. 16 x 100 ms is a
// 1.6 s period: long enough to fit a distinguishable code, short enough that an
// operator glancing at the airframe sees the whole thing.
#define INDICATOR_SLOT_MS        100u
#define INDICATOR_SLOTS          16u

// All three LEDs are lit for this long at init(). A lamp test is not decoration
// here: every healthy pattern below is mostly-dark, so without one there is no
// way to tell a working LED reporting "fine" from an LED that is not connected,
// is on the wrong pin, or has the wrong active level. Non-blocking -- init()
// only records the start, update() ends it.
#define INDICATOR_LAMP_TEST_MS   700u

// Patterns, LSB first: bit n is slot n, 1 = lit. Written MSB-left in the
// comments so the drawing reads the way the light does.
//
// One rule across all three channels:
//
//     SOLID = the good state    BLINK = wanting attention
//
// and NOTHING is dark. That is the property worth protecting, not the
// prettiness: a dark LED would otherwise mean either "this state" or "dead LED
// / wrong pin / wrong active level / hung loop / no power", and no glance can
// separate those. With every state lit, one look settles it -- a dark LED is
// always a fault, and a lit one has already proved the whole chain works before
// it says anything about the aircraft.
//
// So do not give any state INDICATOR_PAT_OFF. It exists to initialise and to
// say "no pattern", not as something a state maps to. The lamp test at init()
// still lights everything for INDICATOR_LAMP_TEST_MS at power-up, which now
// confirms the panel rather than being the only evidence it works.
//
// The three blink patterns differ from each other on purpose. They are on
// separate LEDs so they never have to be told apart, but an urgent strobe, a
// calm slow blink and a searching double-blink read as different KINDS of
// attention, which is free to provide and hard to add later.
//
//                                        slot 0 is the RIGHTMOST bit
#define INDICATOR_PAT_OFF        0x0000u  // ................  not a state; see above
#define INDICATOR_PAT_SOLID      0xFFFFu  // XXXXXXXXXXXXXXXX  lit, the good case

#define INDICATOR_PAT_SYS_OK     INDICATOR_PAT_SOLID
#define INDICATOR_PAT_SYS_ERROR  0x5555u  // .X.X.X.X.X.X.X.X  100 ms strobe: urgent
#define INDICATOR_PAT_ARM_SAFE   0x00FFu  // ........XXXXXXXX  800/800: calm, safe
#define INDICATOR_PAT_ARM_LIVE   INDICATOR_PAT_SOLID          // props live
#define INDICATOR_PAT_GPS_SEARCH 0x0303u  // ......XX......XX  double blink: searching
#define INDICATOR_PAT_GPS_FIX    INDICATOR_PAT_SOLID          // has a fix

class Indicator {
public:
	Indicator(const IndicatorLed &system, const IndicatorLed &arm,
			const IndicatorLed &gps);

	// Drives all three LEDs to a known level and starts the lamp test. The GPIO
	// pins themselves are configured by CubeMX's MX_GPIO_Init(), exactly as the
	// SPI chip selects are -- this class only writes them.
	void init();

	// Renders the current slot. Call it faster than twice the slot rate; the
	// dispatcher runs it at DRONE_INDICATOR_HZ. Cheap enough to call more often
	// and harmless to call less: a late call shows a late edge, never a wrong
	// pattern, because the slot is derived from the clock rather than counted.
	void update();

	void setSystemState(SystemState state);
	void setArmState(ArmState state);
	void setGPSState(GPSState state);

	SystemState getSystemState() const { return _systemState; }
	ArmState getArmState() const { return _armState; }
	GPSState getGPSState() const { return _gpsState; }

private:
	// Channel identifiers, and the count. Ordered as declared so a loop over
	// them reads top-to-bottom the way the states are documented above.
	enum Channel : uint8_t {
		CHANNEL_SYSTEM = 0,
		CHANNEL_ARM    = 1,
		CHANNEL_GPS    = 2,
		CHANNEL_COUNT  = 3
	};

	// One LED and everything needed to render it.
	struct Output {
		IndicatorLed led;
		uint16_t pattern;
		// Tick the current pattern started at. Reset whenever the state
		// changes, so a change is visible immediately instead of waiting out
		// however much of the old cycle was left.
		uint32_t phase_start;
		// Last level actually written, so update() only touches the GPIO on a
		// change. -1 means nothing has been written yet, which forces the first
		// write and makes init() ordering irrelevant.
		int8_t written;
	};

	Output _out[CHANNEL_COUNT];

	SystemState _systemState;
	ArmState _armState;
	GPSState _gpsState;

	bool _initialized;
	bool _lampTest;
	uint32_t _lampTestStart;

	// Point a channel at a new pattern and restart its cycle. Does nothing if
	// the pattern is unchanged, so a setter called every loop -- which is how
	// the dispatcher uses them -- does not pin the LED to slot 0 forever.
	void setPattern(Channel ch, uint16_t pattern);

	// Write a level, skipping the GPIO if it already holds it.
	void drive(Output &o, bool on);
};

// The one indicator. Defined in Globals.cpp, which is also where the pins are.
extern Indicator indicator;

#endif /* INDICATOR_INDICATOR_HPP_ */
