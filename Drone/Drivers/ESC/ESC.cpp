/*
 * ESC.cpp
 *
 *  Created on: Sep 2, 2026
 *      Author: KAVINDU
 */

#include "ESC.hpp"

#include <math.h>   // lrintf, isfinite

ESC::ESC(TIM_HandleTypeDef *htim, uint32_t channel) :
		_htim { htim }, _channel { channel }, _initialized { false },
		_period_ticks { 0 }, _prescaler { 0 }, _bit0_ticks { 0 }, _bit1_ticks { 0 },
		_last_value { 0 }, _frame_count { 0 }, _busy_skip_count { 0 },
		_error_count { 0 }, _dma_buffer { } {
}

ESC_StatusTypeDef ESC::init(DShotRate rate, uint32_t timer_clock_hz) {
	_initialized = false;

	if (_htim == nullptr || timer_clock_hz == 0u) {
		return ESC_StatusTypeDef::ERROR;
	}

	const uint32_t bit_rate = (uint32_t) rate;

	// One DShot bit is one timer period, so the period is the whole of the
	// rate calculation. Divide the timer clock down until the count fits a
	// 16-bit ARR; on any sane F7 clock at any of the four rates the prescaler
	// stays at zero, and the loop is here for the boards where it does not.
	uint32_t prescaler = 0u;
	uint32_t period = 0u;
	uint32_t divided = 0u;
	for (;;) {
		divided = timer_clock_hz / (prescaler + 1u);
		period = (divided + (bit_rate / 2u)) / bit_rate;  // rounded, not truncated
		if (period <= 0x10000u) {
			break;
		}
		prescaler++;
		if (prescaler > 0xFFFFu) {
			return ESC_StatusTypeDef::ERROR;  // clock far too fast for this rate
		}
	}

	// A clock slower than the bit rate rounds the period to nothing, and ARR
	// would then underflow to a period of 65536.
	if (period == 0u) {
		return ESC_StatusTypeDef::ERROR;
	}

	// The period had to be a whole number of ticks, so the achieved rate is
	// divided/period, not exactly the nominal one. Compare the two as a ratio
	// rather than working out the achieved rate and rounding that too:
	//
	//   |divided/period - rate| / rate  <=  tolerance
	//   |divided - rate*period| * 1000  <=  rate * period * tolerance
	//
	// A clock that cannot get close does not fail cleanly on the bench -- it
	// decodes most frames and drops the rest -- so it is worth refusing here.
	const uint64_t ideal = (uint64_t) bit_rate * period;
	const uint64_t actual = (uint64_t) divided;
	const uint64_t error = (actual > ideal) ? (actual - ideal) : (ideal - actual);
	if ((error * 1000u) > (ideal * DSHOT_RATE_TOLERANCE_PERMILLE)) {
		return ESC_StatusTypeDef::ERROR;
	}

	_prescaler = prescaler;
	_period_ticks = period;
	_bit0_ticks = (period * DSHOT_BIT0_PERMILLE + 500u) / 1000u;
	_bit1_ticks = (period * DSHOT_BIT1_PERMILLE + 500u) / 1000u;

	// ARR is period - 1: the counter runs 0..ARR inclusive.
	__HAL_TIM_SET_PRESCALER(_htim, _prescaler);
	__HAL_TIM_SET_AUTORELOAD(_htim, _period_ticks - 1u);

	_initialized = true;

	// Drive the line from here on rather than leaving it floating until the
	// first update(): an ESC powering up next to an undriven signal pin is
	// exactly the case where it can latch onto noise.
	return stop();
}

uint16_t ESC::buildFrame(uint16_t value, bool request_telemetry) {
	// 11 bits of value, then the telemetry request, then a 4-bit CRC over the
	// three nibbles above it. The CRC is what lets the ESC reject a frame it
	// half-received rather than acting on it.
	const uint16_t payload = (uint16_t) ((uint16_t) ((value & 0x07FFu) << 1)
			| (uint16_t) (request_telemetry ? 1u : 0u));
	const uint16_t crc = (uint16_t) ((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0Fu);
	return (uint16_t) ((uint16_t) (payload << 4) | crc);
}

ESC_StatusTypeDef ESC::writeValue(uint16_t value, bool request_telemetry) {
	if (!_initialized) {
		return ESC_StatusTypeDef::ERROR;
	}
	// The value field is 11 bits. Anything wider would silently wrap into a
	// different -- and possibly much faster -- command.
	if (value > DSHOT_MAX_THROTTLE) {
		_error_count++;
		return ESC_StatusTypeDef::ERROR;
	}
	return send(value, buildFrame(value, request_telemetry));
}

ESC_StatusTypeDef ESC::write(float thrust) {
	if (!_initialized) {
		return ESC_StatusTypeDef::ERROR;
	}
	// A NaN thrust means the controller upstream has come apart. Refusing the
	// frame keeps the last command on the motor; turning NaN into a number
	// here would hide the fault and pick an arbitrary throttle.
	if (!isfinite(thrust)) {
		_error_count++;
		return ESC_StatusTypeDef::ERROR;
	}

	if (thrust <= 0.0f) {
		return stop();
	}
	if (thrust > 1.0f) {
		thrust = 1.0f;
	}

	const uint16_t value = (uint16_t) (DSHOT_MIN_THROTTLE
			+ (uint32_t) lrintf(thrust * (float) DSHOT_THROTTLE_RANGE));
	return writeValue(value, false);
}

ESC_StatusTypeDef ESC::stop(void) {
	if (!_initialized) {
		return ESC_StatusTypeDef::ERROR;
	}
	return send(DSHOT_CMD_MOTOR_STOP, buildFrame(DSHOT_CMD_MOTOR_STOP, false));
}

ESC_StatusTypeDef ESC::send(uint16_t value, uint16_t frame) {
	// MSB first: the ESC clocks the value in from bit 15 down.
	for (uint32_t i = 0; i < DSHOT_FRAME_BITS; i++) {
		_dma_buffer[i] = ((frame & 0x8000u) != 0u) ? _bit1_ticks : _bit0_ticks;
		frame = (uint16_t) (frame << 1);
	}
	// The line rests low between frames -- see DSHOT_BUFFER_LEN.
	_dma_buffer[DSHOT_FRAME_BITS] = 0u;
	_dma_buffer[DSHOT_FRAME_BITS + 1u] = 0u;

	const HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(_htim, _channel,
			_dma_buffer, (uint16_t) DSHOT_BUFFER_LEN);

	if (status == HAL_BUSY) {
		// The previous frame has not finished. Dropping this one is the only
		// safe answer: restarting the DMA mid-burst would splice two frames
		// together, and the ESC would decode the join as some third value.
		_busy_skip_count++;
		return ESC_StatusTypeDef::BUSY;
	}
	if (status != HAL_OK) {
		_error_count++;
		return ESC_StatusTypeDef::ERROR;
	}

	_last_value = value;
	_frame_count++;
	return ESC_StatusTypeDef::OK;
}
