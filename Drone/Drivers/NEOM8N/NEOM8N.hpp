/*
 * NEOM8N.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 *
 * u-blox NEO-M8N over UART with circular DMA, parsing NMEA 0183.
 *
 * Wiring it up
 * ------------
 * In CubeMX, the UART the module is on:
 *
 *   9600 baud, 8 data bits, no parity, 1 stop bit
 *   RX DMA stream, mode CIRCULAR, byte to byte
 *
 * CIRCULAR is not optional -- in Normal mode the DMA stops after one pass and
 * the link dies. Same arrangement as IBUS, and for the same reason: NMEA is a
 * byte stream with no frame boundary to re-arm on.
 *
 *     NEOM8N gps { &huart1 };
 *     gps.init();          // AFTER MX_DMA_Init() and MX_USARTn_UART_Init()
 *     ...
 *     gps.update();        // every loop; see the timing note below
 *     GPS_Data fix;
 *     if (gps.readData(fix) == GPS_StatusTypeDef::OK) { ... }
 *
 * Why NMEA and not UBX
 * --------------------
 * An M8N on an APM/Pixhawk carrier ships talking NMEA at 9600, so this works
 * with no configuration at all -- plug it in and it parses. UBX (NAV-PVT in
 * particular) is denser, carries velocity in 3 axes and a proper accuracy
 * estimate, and would be the right upgrade once the position controller needs
 * those. It also has to be configured into the module first, which is a
 * failure mode on a fresh or replaced unit. NMEA first, UBX when it earns it.
 *
 * Two sentences are read, and between them they fill every field of GPS_Data:
 *
 *   GGA  fix quality, satellites, MSL altitude, position
 *   RMC  position, ground speed, course over ground, and the A/V valid flag
 *
 * Any talker ID is accepted -- the M8N is multi-GNSS, so it emits $GNGGA when
 * it has a combined solution and $GPGGA when only GPS is in it. Matching the
 * talker rather than the last three letters is a classic way to end up with a
 * driver that works on the bench and stops when GLONASS comes into view.
 *
 * Call update() often enough
 * --------------------------
 * At 9600 baud the module sends about 960 bytes a second and the RX buffer
 * holds 512, so update() must run at least every ~500 ms. Anything in the
 * scheduler clears that by two orders of magnitude. Falling behind is not
 * silent: getOverrunCount() counts it.
 */

#ifndef DRIVERS_NEOM8N_NEOM8N_HPP_
#define DRIVERS_NEOM8N_NEOM8N_HPP_

#include "common.hpp"

enum class GPS_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1
};

// NMEA caps a sentence at 82 characters including the leading '$' and the
// trailing CRLF. The extra room is slack for a module that ignores that.
#define NEOM8N_LINE_MAX          96
#define NEOM8N_MAX_FIELDS        24

// Must be a power of two -- the ring index masking depends on it. 512 bytes is
// over half a second of traffic at 9600 baud.
#define NEOM8N_RX_BUFFER_LEN     512

// No valid fix for this long and the data is stale. The module reports at
// 1 Hz by default, so this is three missed reports.
#define NEOM8N_FIX_TIMEOUT_MS    3000u

// GPS_Data::flags
#define GPS_FLAG_VALID           0x01u  // came from a checksum-verified fix
#define GPS_FLAG_STALE           0x02u  // ...but the last one was too long ago

struct GPS_Data {
	double latitude;       // degrees, + north
	double longitude;      // degrees, + east
	double altitude;       // metres above mean sea level
	double speed;          // m/s over ground
	double course;         // degrees true, 0..360
	uint8_t num_satellites;
	uint8_t fix_quality;   // 0 = no fix, 1 = GPS fix, 2 = DGPS fix
	uint8_t flags;
};

class NEOM8N {
public:
	explicit NEOM8N(UART_HandleTypeDef *huart);

	// Starts the circular DMA. Call after the UART and DMA peripherals exist.
	GPS_StatusTypeDef init(void);

	// Drains whatever the DMA has written since last time and parses it.
	void update(void);

	// The last fix assembled from checksum-verified sentences. ERROR, with
	// `data` untouched, until one has arrived -- a caller that ignores the
	// return must not be handed a position made of nothing.
	GPS_StatusTypeDef readData(GPS_Data &data);

	// True once a fix has been decoded and it has not since gone stale.
	bool hasFix(void) const;

	// Milliseconds since the last valid fix; UINT32_MAX if there has not been
	// one yet.
	uint32_t getLastFixAgeMs(void) const;

	// Diagnostics. Sentences counts accepted ones; the rest are the ways this
	// degrades, and are worth reporting if any of them climbs.
	uint32_t getSentenceCount(void) const { return _sentence_count; }
	uint32_t getChecksumErrorCount(void) const { return _checksum_error_count; }
	uint32_t getOverrunCount(void) const { return _overrun_count; }
	uint32_t getDmaRestartCount(void) const { return _dma_restart_count; }

private:
	void parseByte(char c);
	bool parseSentence(char *line, uint16_t len);
	void parseGGA(char *fields[], uint8_t count);
	void parseRMC(char *fields[], uint8_t count);
	GPS_StatusTypeDef restartDma(void);

	UART_HandleTypeDef *_huart;

	uint8_t _rx_buffer[NEOM8N_RX_BUFFER_LEN];
	uint16_t _read_index;

	// Sentence under construction, from '$' to the terminator.
	char _line[NEOM8N_LINE_MAX + 1];
	uint16_t _line_len;
	bool _in_sentence;
	bool _line_dropped; // overran NEOM8N_LINE_MAX; discard to the terminator

	GPS_Data _data;
	bool _have_fix;
	uint32_t _last_fix_ms;

	uint32_t _sentence_count;
	uint32_t _checksum_error_count;
	uint32_t _overrun_count;
	uint32_t _dma_restart_count;
};

#endif /* DRIVERS_NEOM8N_NEOM8N_HPP_ */
