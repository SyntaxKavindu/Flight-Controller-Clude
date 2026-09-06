/*
 * IBUS.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * FlySky i-BUS receiver input, over UART with circular DMA.
 *
 * Wiring it up
 * ------------
 * In CubeMX, configure the UART the receiver is on as:
 *
 *   115200 baud, 8 data bits, no parity, 1 stop bit
 *   RX DMA stream, mode CIRCULAR, byte to byte, priority high
 *
 * CIRCULAR is not optional. In Normal mode the DMA stops after one pass and
 * the link goes dead a few frames in; this driver never re-arms per frame,
 * because there is no frame boundary to re-arm on.
 *
 * Then:
 *
 *     IBUS receiver { &huart2 };
 *     receiver.init();                 // AFTER MX_DMA_Init() and MX_USART2_UART_Init()
 *     ...
 *     receiver.update();               // every loop, see the timing note below
 *     IBUS_Data sticks;
 *     if (receiver.getData(&sticks) == RC_StatusTypeDef::OK) { ... }
 *
 * No interrupt callback is needed and none is provided. The DMA writes into
 * _rx_buffer on its own; update() just looks at how far it has got and parses
 * whatever is new. Nothing in this driver runs in interrupt context, so there
 * is no shared state to guard.
 *
 * Call update() often enough
 * --------------------------
 * The receiver sends a 32-byte frame every ~7.7 ms. _rx_buffer holds 4 frames,
 * so update() must run at least every ~30 ms (33 Hz) or the DMA laps the
 * buffer and unread bytes are overwritten. The mid loop at 50 Hz clears this,
 * the fast loop clears it comfortably. Falling behind is not silent: it is
 * counted by getOverrunCount(), and a lapped buffer costs at most a frame or
 * two because the parser resynchronises on the header and the checksum.
 *
 * Failsafe
 * --------
 * getData() keeps handing back the last good stick positions when the link
 * drops -- it does NOT zero them or invent a neutral. Whether that means cut
 * the throttle, hold, or return to launch is the flight code's decision, not
 * the receiver driver's. What this driver guarantees is that you are told:
 * IBUS_FLAG_SIGNAL_LOST is set once no valid frame has arrived for
 * IBUS_SIGNAL_LOST_MS. Check the flag on every read. Stick values with the
 * flag set are history, not input.
 */

#ifndef DRIVERS_IBUS_IBUS_HPP_
#define DRIVERS_IBUS_IBUS_HPP_

#include "common.hpp"

enum class RC_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1
};

// Protocol constants. A frame is:
//   [0]     0x20  length, always 32
//   [1]     0x40  command, "servo/RC data"
//   [2..29] 14 channels, little-endian uint16, microseconds
//   [30,31] checksum, little-endian: 0xFFFF - sum of bytes [0..29]
#define IBUS_CHANNEL_COUNT     14
#define IBUS_FRAME_LEN         32
#define IBUS_HEADER_LENGTH     0x20
#define IBUS_HEADER_COMMAND    0x40

// Four frames of slack -- see the timing note above.
#define IBUS_RX_BUFFER_LEN     (IBUS_FRAME_LEN * 4)

// About 20 missed frames. Long enough that a burst of line noise does not
// trip it, short enough to be well inside any sane failsafe reaction.
#define IBUS_SIGNAL_LOST_MS    150u

// IBUS_Data::flags
#define IBUS_FLAG_VALID        0x01u  // channels came from a checksum-verified frame
#define IBUS_FLAG_SIGNAL_LOST  0x02u  // ...but the last one was too long ago

struct IBUS_Data {
	uint16_t channels[IBUS_CHANNEL_COUNT];  // microseconds, typically 1000..2000
	uint8_t flags;
};

class IBUS {
public:
	explicit IBUS(UART_HandleTypeDef *huart);

	// Starts the circular DMA. Call after the UART and DMA peripherals exist.
	RC_StatusTypeDef init(void);

	// Drains whatever the DMA has written since last time, parses it, and
	// updates the stored channels. Cheap: it touches only the new bytes.
	void update(void);

	// Hands back the last checksum-verified frame. ERROR (and `data` left
	// untouched) if no valid frame has ever arrived -- a caller that ignores
	// the return must not be handed stick positions made of nothing.
	RC_StatusTypeDef getData(IBUS_Data *data);

	// True once a frame has been decoded and the link has not since gone quiet.
	bool isReceiving(void) const;

	// Milliseconds since the last valid frame. UINT32_MAX if there has not
	// been one yet.
	uint32_t getLastFrameAgeMs(void) const;

	// Diagnostics. Frames counts good frames; the other three are the ways
	// this can go wrong, and are worth reporting if any of them is climbing.
	uint32_t getFrameCount(void) const { return _frame_count; }
	uint32_t getChecksumErrorCount(void) const { return _checksum_error_count; }
	uint32_t getOverrunCount(void) const { return _overrun_count; }
	uint32_t getDmaRestartCount(void) const { return _dma_restart_count; }

private:
	// One byte of the stream, fed to the frame state machine.
	void parseByte(uint8_t byte);

	// Validates the checksum of the assembled frame and, if it passes,
	// publishes its channels. Returns true if the frame was accepted.
	bool commitFrame(void);

	// Re-arms the DMA after a UART error aborted it, and resets the parser.
	RC_StatusTypeDef restartDma(void);

	enum class ParseState : uint8_t {
		WAIT_LENGTH,   // hunting for 0x20
		WAIT_COMMAND,  // saw 0x20, expecting 0x40
		PAYLOAD        // in a frame, collecting the remaining bytes
	};

	UART_HandleTypeDef *_huart;
	IBUS_Data _data;

	// DMA lands here. Never read by anything but update().
	uint8_t _rx_buffer[IBUS_RX_BUFFER_LEN];
	uint16_t _read_index;

	// Frame under construction.
	uint8_t _frame[IBUS_FRAME_LEN];
	uint8_t _frame_pos;
	ParseState _state;

	bool _have_frame;
	uint32_t _last_frame_ms;

	uint32_t _frame_count;
	uint32_t _checksum_error_count;
	uint32_t _overrun_count;
	uint32_t _dma_restart_count;
};

#endif /* DRIVERS_IBUS_IBUS_HPP_ */
