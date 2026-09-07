/*
 * Telemetry.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * USB CDC command/telemetry link. Two calls: bytes in, lines out.
 *
 *   receive() -- fed by CDC_Receive_FS() through Telemetry_Receive() in
 *                TelemetryCBridge.h, since usbd_cdc_if.c is plain C and
 *                cannot call a C++ method. Assembles bytes into lines and
 *                acts on each complete command as soon as it arrives.
 *   send()    -- frames a record and pushes it to CDC_Transmit_FS().
 *
 * Commands act on the global `calibrator`.
 *
 * !! receive() is called from the USB RX interrupt, so command handling runs
 * !! in interrupt context: it mutates calibrator state the main loop also
 * !! reads, and it transmits from inside the RX interrupt. Acceptable while
 * !! calibration is a ground-only, operator-driven mode; if commands ever need
 * !! to be handled in flight, buffer here and drain from the main loop.
 *
 * Wire format
 * -----------
 * In  : one ASCII command per line, terminated by LF or CR (both tolerated).
 *       "NAME" or "NAME,ARG", case insensitive, spaces trimmed.
 *
 *         CALIMU           six-position accelerometer calibration
 *         CALIMU,TUMBLE    tumble (ellipsoid) accelerometer calibration
 *         CALMAG           magnetometer calibration
 *         CALLEVEL         board levelling; needs a finished CALIMU behind it
 *         CALLEVEL,<deg>   ... with a known mounting yaw, degrees
 *         READY            in position and holding still (six-position step)
 *         CANCEL           abort whatever calibration is running
 *         CALCLEAR         erase the stored calibration
 *         CALSTATUS        report what is calibrated / in progress
 *         DIAG             dump loop counters and estimator health
 *
 * Out : one record per line, "$KEY,fields...\r\n". Callers build the record;
 *       this class frames and sends it.
 */

#ifndef TELEMETRY_TELEMETRY_HPP_
#define TELEMETRY_TELEMETRY_HPP_

#include "common.hpp"

// Longest command line accepted; anything longer is refused with a $NAK.
#define TELEMETRY_LINE_MAX      64
// Longest line send() will emit; longer output is truncated, never overflows.
#define TELEMETRY_TX_MAX        160
// How long send() will wait for the endpoint to free up before giving up on a
// line. CDC_Transmit_FS() refuses a packet while the previous one is in
// flight, and the host only polls every millisecond or so, so without a little
// patience a burst -- the boot banner, a calibration's replies -- loses most
// of itself. Bounded, because stalling the flight loop to print is never worth
// it: past this, the line is dropped and counted.
#define TELEMETRY_TX_TIMEOUT_MS 2u
// Bytes buffered between the USB interrupt and poll(). Must be a power of two.
// A CDC packet is at most 64 bytes and poll() runs every loop, so this is four
// packets of slack -- enough that a burst while the loop is busy is not lost.
#define TELEMETRY_RX_RING_SIZE  256

class Telemetry {
public:
	Telemetry();

	// Bytes from the USB CDC RX callback. RUNS IN INTERRUPT CONTEXT, and does
	// nothing but copy them into the RX ring: no parsing, no dispatch, no
	// send(). Commands are acted on later, by poll(), on the main loop.
	//
	// This split is not tidiness. Everything reachable from a command --
	// Calibrator state, and through CALCLEAR an EEPROM write that blocks for
	// milliseconds -- is also touched by the main loop. Acting here would mean
	// two writers with no lock, and a mutex cannot be taken in an ISR anyway.
	// One writer is cheaper and correct.
	void receive(const char *data, uint16_t len);

	// Drains the RX ring, assembles lines and acts on them. MAIN LOOP ONLY.
	// Call it before the flight tasks so a CANCEL takes effect before the next
	// sample is fed to a running calibration, not after.
	void poll(void);

	// Queue and push one line, printf-style, CRLF appended. Main loop only.
	void send(const char *fmt, ...);

	// Bytes the ISR had to throw away because poll() had not drained the ring.
	// Non-zero means the loop stalled long enough to lose a command.
	uint32_t getRxDroppedCount(void) const { return _rx_dropped; }

	// Total bytes receive() has been OFFERED since boot, and complete lines
	// poll() has dispatched. Reported in the heartbeat, because they answer the
	// one question a silent command link otherwise cannot:
	//
	//   rx = 0            nothing is reaching receive() at all. The USB RX
	//                     interrupt is not calling Telemetry_Receive() -- see
	//                     TelemetryCBridge.h, which has to be wired into
	//                     CDC_Receive_FS() by hand. Nothing in the C++ side can
	//                     detect that omission; only this counter can.
	//   rx > 0, cmd = 0   bytes arrive but no line is ever completed: the host
	//                     is not sending a CR or LF terminator.
	//   rx > 0, cmd > 0   the link works and the command was acted on; an
	//                     unrecognised one answers $NAK.
	//
	// Counted as offered rather than as stored, so a full ring shows up as
	// rx climbing while getRxDroppedCount() climbs with it.
	uint32_t getRxByteCount(void) const { return _rx_bytes; }
	uint32_t getLineCount(void) const { return _lines; }

	// Lines dropped because the endpoint stayed busy past the timeout. Output
	// is expendable; the flight loop is not.
	uint32_t getTxDroppedCount(void) const { return _tx_dropped; }

private:
	// Written by the ISR, read by poll(). Single producer, single consumer,
	// power-of-two masked: the producer only advances _rx_head and the
	// consumer only advances _rx_tail, so neither needs a lock.
	//
	// No memory barrier either. This is one core, and the ISR PREEMPTS the
	// main loop rather than running beside it -- there is no second observer
	// to publish to. The indices are volatile so the compiler cannot reorder
	// or cache them across the accesses, and that is the whole requirement.
	char _rx_ring[TELEMETRY_RX_RING_SIZE];
	volatile uint16_t _rx_head;
	volatile uint16_t _rx_tail;
	volatile uint32_t _rx_dropped;
	uint32_t _rx_dropped_reported;
	// Written by the ISR, read by the main loop for the heartbeat. A torn read
	// of a 32-bit counter is not possible on this core, and an off-by-one in a
	// diagnostic would not matter anyway.
	volatile uint32_t _rx_bytes;
	uint32_t _lines;

	// Line assembly. Touched only by poll(), so it needs no guarding.
	char _line[TELEMETRY_LINE_MAX + 1];
	uint16_t _line_len;
	bool _line_dropped; // current line overran; discard until the terminator

	// TWO buffers, used alternately, and members rather than locals.
	//
	// CDC_Transmit_FS() is ASYNCHRONOUS: it registers the pointer and returns,
	// and the USB DMA reads from it until the transfer completes. Formatting
	// into a local `char buffer[256]` and handing that over -- the obvious way
	// to write this -- gives the DMA a pointer into a stack frame that is gone
	// the moment send() returns. It usually appears to work, because nothing
	// has overwritten that stack yet, and it corrupts output the moment
	// something does.
	//
	// Alternating two buffers means the one being formatted is never the one
	// in flight: only one transfer can be outstanding, so the other buffer is
	// always free.
	char _tx_buf[2][TELEMETRY_TX_MAX];
	uint8_t _tx_which;
	uint32_t _tx_dropped;

	// One byte of the RX stream, assembled into _line and dispatched on a
	// terminator. Main loop only, via poll().
	void consumeByte(char c);

	void dispatchLine(char *line, uint16_t len);
};

// The one link. Calibrator reports through it; TelemetryCBridge feeds it.
// Defined in Globals.cpp.
extern Telemetry telemetry;

#endif /* TELEMETRY_TELEMETRY_HPP_ */
