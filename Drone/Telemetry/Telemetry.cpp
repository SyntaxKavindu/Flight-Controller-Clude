/*
 * Telemetry.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "Telemetry.hpp"
#include "Calibrator.hpp"

#include <cstdio>
#include <cstdarg>
#include <cstdlib> // strtof, for CALLEVEL's optional yaw argument

// Declared here rather than by including usbd_cdc_if.h so this unit does not
// drag the USB middleware in. Returns USBD_OK (0) on success, USBD_BUSY (1)
// while the previous packet is still in flight.
extern "C" uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

// Implemented in Drone.cpp. Declared here rather than by including Drone.hpp,
// which would be circular: Drone.hpp pulls in Globals.hpp, which pulls in this
// header.
void Drone_ReportDiagnostics(void);

extern "C" void Telemetry_Receive(const uint8_t *data, uint32_t len) {
	if (data == nullptr) {
		return;
	}
	telemetry.receive(reinterpret_cast<const char*>(data), (uint16_t) len);
}

namespace {

constexpr uint16_t RX_MASK = TELEMETRY_RX_RING_SIZE - 1;
static_assert((TELEMETRY_RX_RING_SIZE & RX_MASK) == 0, "RX ring must be a power of two");

char toUpper(char c) {
	return (c >= 'a' && c <= 'z') ? (char) (c - 'a' + 'A') : c;
}

bool isSpace(char c) {
	return c == ' ' || c == '\t';
}

// Case-insensitive compare of a NUL-terminated token against a literal.
bool tokenEquals(const char *token, const char *literal) {
	if (token == nullptr) return false;
	size_t i = 0;
	for (; token[i] != '\0' && literal[i] != '\0'; i++) {
		if (toUpper(token[i]) != literal[i]) return false;
	}
	return token[i] == '\0' && literal[i] == '\0';
}

// Trim leading and trailing blanks in place.
char *trim(char *s) {
	while (*s != '\0' && isSpace(*s)) s++;
	for (char *e = s + strlen(s); e > s && isSpace(*(e - 1)); e--) {
		*(e - 1) = '\0';
	}
	return s;
}

} // namespace

Telemetry::Telemetry() :
		_rx_ring { }, _rx_head { 0 }, _rx_tail { 0 }, _rx_dropped { 0 },
		_rx_dropped_reported { 0 }, _rx_bytes { 0 }, _lines { 0 },
		_line_len { 0 }, _line_dropped { false },
		_tx_buf { }, _tx_which { 0 }, _tx_dropped { 0 } {
	_line[0] = '\0';
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

void Telemetry::receive(const char *data, uint16_t len) {
	if (data == nullptr) {
		return;
	}

	// Counted before anything can reject a byte, so this is proof the interrupt
	// reached us at all -- which is the one thing a dead command link cannot
	// otherwise distinguish from a mis-typed command. See getRxByteCount().
	_rx_bytes += len;

	// INTERRUPT CONTEXT. Bounded, allocation-free, no reentrancy into anything
	// the main loop owns: copy and leave. See the note in the header for why.
	uint16_t head = _rx_head; // only this producer writes it
	for (uint16_t i = 0; i < len; i++) {
		const uint16_t next = (uint16_t) ((head + 1) & RX_MASK);
		if (next == _rx_tail) {
			// Ring full: poll() has not run recently enough. Drop the rest of
			// the packet rather than overwrite unread bytes -- a half-eaten
			// command is worse than a missing one, because it can still parse.
			_rx_dropped += (uint32_t) (len - i);
			break;
		}
		_rx_ring[head] = data[i];
		head = next;
	}

	// Publish once, after the bytes are in place. _rx_head is volatile, so the
	// compiler cannot hoist this store above the writes it covers, and on one
	// core that is the entire requirement.
	_rx_head = head;
}

void Telemetry::poll(void) {
	const uint16_t head = _rx_head;
	uint16_t tail = _rx_tail; // only this consumer writes it

	while (tail != head) {
		const char c = _rx_ring[tail];
		tail = (uint16_t) ((tail + 1) & RX_MASK);
		consumeByte(c);
	}
	_rx_tail = tail;

	// Report a loss once per occurrence rather than every poll. A dropped byte
	// means a command was silently mangled, which the operator has to know:
	// they saw no ACK and would otherwise assume the link is merely slow.
	const uint32_t dropped = _rx_dropped;
	if (dropped != _rx_dropped_reported) {
		_rx_dropped_reported = dropped;
		send("$ERR,RX OVERFLOW, %lu BYTES LOST", (unsigned long) dropped);
	}
}

void Telemetry::consumeByte(char c) {
	if (c == '\n' || c == '\r') {
		if (_line_dropped) {
			// Refuse rather than truncate: a shortened line can parse as a
			// different, valid command.
			_line_dropped = false;
			_line_len = 0;
			send("$NAK,?,LINETOOLONG");
			return;
		}
		if (_line_len > 0) {
			_line[_line_len] = '\0';
			_lines++;
			dispatchLine(_line, _line_len);
			_line_len = 0;
		}
		return; // blank lines and bare CRLF are ignored
	}

	if (_line_dropped) {
		return;
	}
	if (_line_len >= TELEMETRY_LINE_MAX) {
		_line_dropped = true;
		_line_len = 0;
		return;
	}
	_line[_line_len++] = c;
}

// ---------------------------------------------------------------------------
// Parse and act
// ---------------------------------------------------------------------------

void Telemetry::dispatchLine(char *line, uint16_t len) {
	// Keep the original text for the ACK/NAK echo before the line is chopped
	// into NUL-terminated tokens in place.
	char echo[TELEMETRY_LINE_MAX + 1];
	const uint16_t echo_len = (len < TELEMETRY_LINE_MAX) ? len : TELEMETRY_LINE_MAX;
	memcpy(echo, line, echo_len);
	echo[echo_len] = '\0';

	// Split into "NAME" and one optional argument.
	char *arg = nullptr;
	for (uint16_t i = 0; i < len; i++) {
		if (line[i] == ',') {
			line[i] = '\0';
			arg = trim(&line[i + 1]);
			break;
		}
	}
	char *name = trim(line);
	if (*name == '\0') {
		return;
	}

	// Every procedure, not just the two the original checks knew about. A
	// levelling run left out of this was startable on top of an accelerometer
	// calibration, and CANCEL answered "$NAK,IDLE" while one was in progress.
	const bool busy = calibrator.isCalibrating();

	if (tokenEquals(name, "CALIMU")) {
		const bool tumble = (arg != nullptr) && tokenEquals(arg, "TUMBLE");
		if (arg != nullptr && !tumble && *arg != '\0') {
			send("$NAK,%s,UNKNOWN", echo);
			return;
		}
		if (busy) { send("$NAK,%s,BUSY", echo); return; }
		send("$ACK,%s", echo);
		if (tumble) {
			calibrator.startAccelerometerTumbleCalibration();
		} else {
			calibrator.startAccelerometerCalibration();
		}
		return;
	}

	if (tokenEquals(name, "CALMAG")) {
		if (busy) { send("$NAK,%s,BUSY", echo); return; }
		send("$ACK,%s", echo);
		calibrator.startCompassCalibration();
		return;
	}

	if (tokenEquals(name, "CALLEVEL")) {
		if (busy) { send("$NAK,%s,BUSY", echo); return; }

		// Optional mounting yaw, in degrees. Gravity cannot observe rotation
		// about the vertical, so if the board is bolted in at a known angle it
		// has to be supplied here -- see LevelCalibrator::begin(). Absent, the
		// run corrects roll and pitch only, which is the common case.
		float yaw_deg = 0.0f;
		if (arg != nullptr && *arg != '\0') {
			char *end = nullptr;
			const float parsed = strtof(arg, &end);
			// Reject trailing junk rather than silently levelling against half a
			// number: strtof stops at the first bad character and reports
			// success for "12abc".
			if (end == arg || *end != '\0') {
				send("$NAK,%s,BADARG", echo);
				return;
			}
			yaw_deg = parsed;
		}
		send("$ACK,%s", echo);
		calibrator.startLevelCalibration(yaw_deg);
		return;
	}

	if (tokenEquals(name, "READY")) {
		if (!calibrator.isAwaitingPosition()) { send("$NAK,%s,NOTWAITING", echo); return; }
		send("$ACK,%s", echo);
		calibrator.confirmReady();
		return;
	}

	if (tokenEquals(name, "CANCEL")) {
		if (!busy) { send("$NAK,%s,IDLE", echo); return; }
		send("$ACK,%s", echo);
		calibrator.cancelCalibration();
		return;
	}

	if (tokenEquals(name, "CALCLEAR")) {
		if (busy) { send("$NAK,%s,BUSY", echo); return; }
		send("$ACK,%s", echo);
		if (calibrator.clearStoredCalibration() == Calibrator_StatusTypeDef::OK) {
			send("$INFO,STORED CALIBRATION ERASED");
		} else {
			send("$ERR,ERASE FAILED (NO STORAGE?)");
		}
		return;
	}

	if (tokenEquals(name, "DIAG")) {
		// Deliberately answerable at any time, calibrating or not: it exists
		// for the case where nothing else is talking.
		send("$ACK,%s", echo);
		Drone_ReportDiagnostics();
		return;
	}

	if (tokenEquals(name, "CALDUMP")) {
		// Answerable at any time, calibrating or not: knowing what is applied
		// right now is most useful precisely when something is going wrong.
		send("$ACK,%s", echo);
		calibrator.reportCalibration();
		return;
	}

	if (tokenEquals(name, "CALSTATUS")) {
		send("$ACK,%s", echo);
		send("$STATUS,ACCL,%s,%s", calibrator.isAcclCalibrated() ? "CAL" : "UNCAL",
				calibrator.isAcclCalibrating() ? "BUSY" : "IDLE");
		send("$STATUS,MAG,%s,%s", calibrator.isCompassCalibrated() ? "CAL" : "UNCAL",
				calibrator.isCompassCalibrating() ? "BUSY" : "IDLE");
		send("$STATUS,LEVEL,%s,%s", calibrator.isLevelCalibrated() ? "CAL" : "UNCAL",
				calibrator.isLevelCalibrating() ? "BUSY" : "IDLE");
		return;
	}

	send("$NAK,%s,UNKNOWN", echo);
}

// ---------------------------------------------------------------------------
// Send
// ---------------------------------------------------------------------------

void Telemetry::send(const char *fmt, ...) {
	// Alternate buffers: the one in flight is never the one being written.
	// See the note on _tx_buf -- handing CDC_Transmit_FS() a local array is
	// the tempting version of this function and it hands the USB DMA a dead
	// stack frame.
	char *buf = _tx_buf[_tx_which];

	va_list args;
	va_start(args, fmt);
	const int n = vsnprintf(buf, TELEMETRY_TX_MAX - 2, fmt, args);
	va_end(args);

	if (n <= 0) {
		return; // encoding error, or nothing to say
	}

	// vsnprintf returns what it WOULD have written, so a long line reports
	// more than the buffer holds. Clamp before using it as a length or the
	// transmit runs off the end of the buffer.
	uint16_t len = ((size_t) n < (TELEMETRY_TX_MAX - 2))
			? (uint16_t) n : (uint16_t) (TELEMETRY_TX_MAX - 2);
	buf[len++] = '\r';
	buf[len++] = '\n';

	// Wait a moment for the endpoint, then give up. The host polls about once
	// a millisecond, so without any patience a burst loses most of itself --
	// but stalling the flight loop to print is never the right trade, so this
	// is bounded and a dropped line is counted rather than waited out.
	const uint32_t start = HAL_GetTick();
	for (;;) {
		if (CDC_Transmit_FS(reinterpret_cast<uint8_t*>(buf), len) == 0) {
			_tx_which ^= 1u;
			return;
		}
		if ((HAL_GetTick() - start) >= TELEMETRY_TX_TIMEOUT_MS) {
			_tx_dropped++;
			return;
		}
	}
}
