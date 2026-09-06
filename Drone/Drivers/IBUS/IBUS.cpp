/*
 * IBUS.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "IBUS.hpp"

IBUS::IBUS(UART_HandleTypeDef *huart) :
		_huart { huart }, _data { }, _rx_buffer { }, _read_index { 0 },
		_frame { }, _frame_pos { 0 }, _state { ParseState::WAIT_LENGTH },
		_have_frame { false }, _last_frame_ms { 0 }, _frame_count { 0 },
		_checksum_error_count { 0 }, _overrun_count { 0 },
		_dma_restart_count { 0 } {
	// Everything is brought up zeroed rather than left indeterminate:
	// getData() before the first frame arrives would otherwise hand the flight
	// code stick positions made of stack garbage. It reports ERROR instead,
	// but a caller that ignores the return still gets zeros, not noise.
}

RC_StatusTypeDef IBUS::init(void) {
	if (_huart == nullptr) {
		return RC_StatusTypeDef::ERROR;
	}

	_read_index = 0;
	_frame_pos = 0;
	_state = ParseState::WAIT_LENGTH;

	// One circular DMA, started once and left running for the life of the
	// vehicle. There is no per-frame re-arm because i-BUS gives no frame
	// boundary to re-arm on -- the parser finds frames in the byte stream.
	if (HAL_UART_Receive_DMA(_huart, _rx_buffer, IBUS_RX_BUFFER_LEN) != HAL_OK) {
		return RC_StatusTypeDef::ERROR;
	}
	return RC_StatusTypeDef::OK;
}

RC_StatusTypeDef IBUS::restartDma(void) {
	// A framing, noise or overrun error aborts the HAL's DMA reception, and
	// nothing restarts it on its own: without this the link dies permanently
	// the first time the receiver is unplugged, brown-outs, or the wire is
	// touched. Drop everything half-parsed and start the stream again.
	HAL_UART_DMAStop(_huart);
	_huart->ErrorCode = HAL_UART_ERROR_NONE;
	_read_index = 0;
	_frame_pos = 0;
	_state = ParseState::WAIT_LENGTH;
	_dma_restart_count++;

	if (HAL_UART_Receive_DMA(_huart, _rx_buffer, IBUS_RX_BUFFER_LEN) != HAL_OK) {
		return RC_StatusTypeDef::ERROR;
	}
	return RC_StatusTypeDef::OK;
}

void IBUS::update(void) {
	if (_huart == nullptr || _huart->hdmarx == nullptr) {
		return;
	}

	// If the DMA is no longer running, a UART error killed it. Restart and
	// come back next time -- there is nothing in the buffer worth trusting.
	if (_huart->RxState != HAL_UART_STATE_BUSY_RX) {
		restartDma();
		return;
	}

	// NDTR counts DOWN from the transfer length, so the DMA has written
	// (LEN - NDTR) bytes into the buffer and is about to write at that index.
	const uint32_t remaining = __HAL_DMA_GET_COUNTER(_huart->hdmarx);
	uint16_t write_index =
			(remaining > IBUS_RX_BUFFER_LEN) ?
					0 : (uint16_t) (IBUS_RX_BUFFER_LEN - remaining);
	if (write_index >= IBUS_RX_BUFFER_LEN) {
		write_index = 0; // NDTR == 0 at the exact wrap point
	}

	const uint16_t available = (uint16_t) ((write_index + IBUS_RX_BUFFER_LEN
			- _read_index) % IBUS_RX_BUFFER_LEN);

	// Within one frame of being lapped means update() is not being called
	// often enough and the DMA may already have overwritten bytes we never
	// read. This is a heuristic -- circular DMA gives no lap counter, so a
	// full lap between two calls is indistinguishable from no data at all.
	// Correctness does not depend on catching it (the checksum does that);
	// it is here so a too-slow loop shows up as a number instead of as
	// occasional glitchy sticks.
	if (available > (IBUS_RX_BUFFER_LEN - IBUS_FRAME_LEN)) {
		_overrun_count++;
	}

	for (uint16_t i = 0; i < available; i++) {
		parseByte(_rx_buffer[_read_index]);
		_read_index = (uint16_t) ((_read_index + 1) % IBUS_RX_BUFFER_LEN);
	}

	// Age the link. Done here rather than in getData() so that the flag is a
	// property of the last update(), and two reads in the same loop iteration
	// cannot disagree about whether the link is alive.
	if (_have_frame) {
		if ((HAL_GetTick() - _last_frame_ms) > IBUS_SIGNAL_LOST_MS) {
			_data.flags |= IBUS_FLAG_SIGNAL_LOST;
		} else {
			_data.flags &= (uint8_t) ~IBUS_FLAG_SIGNAL_LOST;
		}
	}
}

void IBUS::parseByte(uint8_t byte) {
	switch (_state) {
	case ParseState::WAIT_LENGTH:
		if (byte == IBUS_HEADER_LENGTH) {
			_frame[0] = byte;
			_frame_pos = 1;
			_state = ParseState::WAIT_COMMAND;
		}
		// Anything else is mid-frame garbage or the tail of a frame we joined
		// late. Drop it and keep hunting; there is no cost to resynchronising.
		break;

	case ParseState::WAIT_COMMAND:
		if (byte == IBUS_HEADER_COMMAND) {
			_frame[1] = byte;
			_frame_pos = 2;
			_state = ParseState::PAYLOAD;
		} else if (byte == IBUS_HEADER_LENGTH) {
			// 0x20 0x20: the first was stream noise that happened to look
			// like a header. Stay here and let THIS one be the length byte
			// rather than throwing both away -- otherwise any frame preceded
			// by a stray 0x20 is lost. _frame[0] and _frame_pos already hold
			// what this byte would write, so not resetting IS the action.
		} else {
			_state = ParseState::WAIT_LENGTH;
			_frame_pos = 0;
		}
		break;

	case ParseState::PAYLOAD:
		_frame[_frame_pos++] = byte;
		if (_frame_pos >= IBUS_FRAME_LEN) {
			commitFrame();
			_state = ParseState::WAIT_LENGTH;
			_frame_pos = 0;
		}
		break;
	}
}

bool IBUS::commitFrame(void) {
	// Checksum is 0xFFFF minus the sum of every byte before it, sent
	// little-endian. It is the only integrity check i-BUS has, and it is what
	// stops a mid-stream 0x20 0x40 from being decoded as a frame.
	uint16_t sum = 0;
	for (uint8_t i = 0; i < (IBUS_FRAME_LEN - 2); i++) {
		sum = (uint16_t) (sum + _frame[i]);
	}
	const uint16_t expected = (uint16_t) (0xFFFFu - sum);
	const uint16_t received = (uint16_t) (_frame[30] | (_frame[31] << 8));

	if (expected != received) {
		_checksum_error_count++;
		return false;
	}

	for (uint8_t ch = 0; ch < IBUS_CHANNEL_COUNT; ch++) {
		const uint8_t lo = _frame[2 + (ch * 2)];
		const uint8_t hi = _frame[3 + (ch * 2)];
		_data.channels[ch] = (uint16_t) (lo | (hi << 8));
	}

	_data.flags = IBUS_FLAG_VALID; // a fresh frame clears SIGNAL_LOST
	_have_frame = true;
	_last_frame_ms = HAL_GetTick();
	_frame_count++;
	return true;
}

RC_StatusTypeDef IBUS::getData(IBUS_Data *data) {
	if (data == nullptr || !_have_frame) {
		return RC_StatusTypeDef::ERROR;
	}
	*data = _data;
	return RC_StatusTypeDef::OK;
}

bool IBUS::isReceiving(void) const {
	return _have_frame && ((_data.flags & IBUS_FLAG_SIGNAL_LOST) == 0);
}

uint32_t IBUS::getLastFrameAgeMs(void) const {
	if (!_have_frame) {
		return UINT32_MAX;
	}
	return HAL_GetTick() - _last_frame_ms;
}
