/*
 * ESC.hpp
 *
 *  Created on: Sep 2, 2026
 *      Author: KAVINDU
 *
 * One electronic speed controller, driven with DShot over a timer PWM channel
 * fed by DMA.
 *
 * Why DShot, and why DMA
 * ----------------------
 * DShot sends a 16-bit digital frame instead of an analogue pulse width. Each
 * bit is one timer period whose DUTY CYCLE carries the value: 37.5% is a zero,
 * 75% is a one. So a frame is 16 back-to-back compare values that have to land
 * on consecutive timer update events with no gap and no jitter -- a hard real-
 * time requirement no software loop can meet. The DMA does it: one burst per
 * frame, the CPU touches nothing while it goes out.
 *
 * That is also why this class programs the timer's period itself. The bit rate
 * IS the timer period, so ARR and PSC are protocol parameters, not board
 * configuration, and init() derives them from the timer clock you pass in.
 *
 * Wiring it up
 * ------------
 * In CubeMX, on the timer channel each ESC is on:
 *
 *   PWM Generation CHx, PWM mode 1, no output compare preload needed
 *   A DMA request for that channel (TIMx_CHx), mode NORMAL, memory to
 *     peripheral, increment memory only, both widths WORD, priority high
 *   Enable the timer's global interrupt AND the DMA stream interrupt -- the
 *     HAL's transfer-complete handler is what returns the channel to READY,
 *     and without it the second frame and every one after it is refused
 *
 * NORMAL, not circular: a frame is a one-shot burst, and a circular DMA would
 * repeat the same frame forever behind your back.
 *
 * Then:
 *
 *     ESC esc { &htim3, TIM_CHANNEL_1 };
 *     esc.init(DShotRate::DSHOT300, 108000000u);   // APB1 timer clock
 *     ...
 *     esc.write(0.42f);                            // every loop
 *
 * Keep calling write()
 * --------------------
 * An ESC expects a continuous stream of frames and will cut the motor if the
 * signal stops -- that is its own failsafe and it is the last line of defence.
 * So write() must be called every loop even when the demand has not changed,
 * and even when the demand is zero: a MOTOR_STOP frame is an instruction, but
 * NO frame is a fault. Motors::update() is what guarantees this; nothing in
 * this class repeats a frame on its own.
 *
 * If a burst is still going out when the next write() arrives, the frame is
 * DROPPED rather than queued or truncated, and getBusySkipCount() counts it.
 * At DShot300 a frame takes 60 us, so a loop that trips this is running faster
 * than the wire, which is a configuration mistake worth seeing rather than
 * hiding.
 */

#ifndef DRIVERS_ESC_ESC_HPP_
#define DRIVERS_ESC_ESC_HPP_

#include "common.hpp"

enum class ESC_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1,
	BUSY = 2   // previous frame still on the wire; this one was dropped
};

// The four standard bit rates. The value IS the rate in bits per second, which
// is what init() divides the timer clock by.
enum class DShotRate : uint32_t {
	DSHOT150  = 150000u,
	DSHOT300  = 300000u,
	DSHOT600  = 600000u,
	DSHOT1200 = 1200000u
};

// Frame layout, MSB first:
//   [15..5] 11-bit value
//   [4]     telemetry request
//   [3..0]  CRC, xor of the three nibbles above
#define DSHOT_FRAME_BITS        16u

// Two trailing zero slots hold the line low after the last bit. Without them
// the compare register keeps the final bit's duty cycle and the idle line sits
// at 37.5% or 75%, which the ESC reads as the start of a frame that never ends.
#define DSHOT_BUFFER_LEN        (DSHOT_FRAME_BITS + 2u)

#define DSHOT_CMD_MOTOR_STOP    0u
#define DSHOT_MAX_COMMAND       47u
#define DSHOT_MIN_THROTTLE      48u
#define DSHOT_MAX_THROTTLE      2047u
#define DSHOT_THROTTLE_RANGE    (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

// Duty cycles, as parts per thousand of the bit period.
#define DSHOT_BIT0_PERMILLE     375u
#define DSHOT_BIT1_PERMILLE     750u

// A bit period is a whole number of timer ticks, so the achieved bit rate is
// the timer clock divided by an integer and will not land exactly on the
// nominal one. A couple of percent is fine -- the ESC samples each bit in the
// middle of its window -- but past that it stops decoding, and it stops
// decoding INTERMITTENTLY, which is the worst way for it to fail.
//
// This also bounds the resolution of the two duty cycles, because the error is
// about half a tick: staying inside 2% means at least 25 ticks per bit, which
// is ample to tell 37.5% from 75%. So it is one check, not two.
#define DSHOT_RATE_TOLERANCE_PERMILLE  20u   // 2%

class ESC {
public:
	// `channel` is a TIM_CHANNEL_x constant. The handle must already be
	// initialised by CubeMX's MX_TIMx_Init().
	ESC(TIM_HandleTypeDef *htim, uint32_t channel);

	// Programs ARR and PSC for `rate`, works out the two bit duty cycles, and
	// sends one MOTOR_STOP frame so the line is driven from the moment this
	// returns. ERROR if the timer clock cannot produce the rate closely
	// enough -- see DSHOT_RATE_TOLERANCE_PERMILLE.
	//
	// `timer_clock_hz` is the clock reaching the TIMER, which on an STM32F7 is
	// usually TWICE the APB bus clock. Getting it wrong shows up here as a
	// rejected rate or silently as an ESC that never decodes anything, so it
	// is worth checking against the CubeMX clock tree rather than assuming.
	ESC_StatusTypeDef init(DShotRate rate, uint32_t timer_clock_hz);

	// Normalised thrust in [0, 1]; out-of-range values are clamped, NaN is
	// rejected. Exactly 0 sends MOTOR_STOP -- the motor is commanded STILL,
	// not slow. What idle means is Motors' decision, not this driver's.
	ESC_StatusTypeDef write(float thrust);

	// The DShot value itself: 0 is MOTOR_STOP, 1..47 are commands (beeps,
	// direction, save settings), 48..2047 is the throttle range. Anything
	// above 2047 does not fit the 11-bit field and is rejected.
	ESC_StatusTypeDef writeValue(uint16_t value, bool request_telemetry = false);

	// MOTOR_STOP. Still a frame on the wire -- see the note above about why
	// silence is not the same instruction.
	ESC_StatusTypeDef stop(void);

	bool isInitialized(void) const { return _initialized; }

	// The last value actually put on the wire. A dropped frame does not
	// change it, because nothing was sent.
	uint16_t getLastValue(void) const { return _last_value; }

	// Diagnostics. Frames counts what went out; the other two are the ways
	// this can go wrong and are worth reporting if either is climbing.
	uint32_t getFrameCount(void) const { return _frame_count; }
	uint32_t getBusySkipCount(void) const { return _busy_skip_count; }
	uint32_t getErrorCount(void) const { return _error_count; }

	// What init() worked out. Exposed for the boot report and for tests that
	// decode the wire back to a frame.
	uint32_t getPeriodTicks(void) const { return _period_ticks; }
	uint32_t getPrescaler(void) const { return _prescaler; }
	uint32_t getBit0Ticks(void) const { return _bit0_ticks; }
	uint32_t getBit1Ticks(void) const { return _bit1_ticks; }

	// Packs a value and its telemetry bit into the 16-bit frame, CRC included.
	// Static and public because the encoding is worth testing on its own, with
	// no timer anywhere near it.
	static uint16_t buildFrame(uint16_t value, bool request_telemetry);

private:
	// Expands `frame` into per-bit compare values and hands the buffer to the
	// DMA. Everything above funnels through here.
	ESC_StatusTypeDef send(uint16_t value, uint16_t frame);

	TIM_HandleTypeDef *_htim;
	uint32_t _channel;

	bool _initialized;

	uint32_t _period_ticks;   // timer counts in one DShot bit
	uint32_t _prescaler;      // PSC value, not PSC+1
	uint32_t _bit0_ticks;     // compare value for a zero bit
	uint32_t _bit1_ticks;     // compare value for a one bit

	uint16_t _last_value;

	uint32_t _frame_count;
	uint32_t _busy_skip_count;
	uint32_t _error_count;

	// Handed to the DMA and read by it after send() returns, so it has to
	// outlive the call -- which is why it is a member and not a local.
	uint32_t _dma_buffer[DSHOT_BUFFER_LEN];
};

#endif /* DRIVERS_ESC_ESC_HPP_ */
