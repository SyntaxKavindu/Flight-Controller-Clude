/*
 * NEOM8N.cpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#include "NEOM8N.hpp"

#include <cstring>

namespace {

constexpr uint16_t RX_MASK = NEOM8N_RX_BUFFER_LEN - 1;
static_assert((NEOM8N_RX_BUFFER_LEN & RX_MASK) == 0,
		"NEOM8N RX buffer must be a power of two");

constexpr double KNOTS_TO_MPS = 0.514444444;

// Hand-rolled rather than strtod(). NMEA fields are a tiny, fixed grammar --
// optional sign, digits, optional fraction -- and newlib's strtod drags in
// locale handling and a good deal of flash for generality none of this needs.
// It also rejects trailing rubbish, which strtod silently accepts.
bool parseDouble(const char *s, double &out) {
	if (s == nullptr || *s == '\0') {
		return false;
	}

	bool negative = false;
	if (*s == '-') {
		negative = true;
		s++;
	} else if (*s == '+') {
		s++;
	}

	double value = 0.0;
	bool any_digits = false;
	while (*s >= '0' && *s <= '9') {
		value = value * 10.0 + (double) (*s - '0');
		any_digits = true;
		s++;
	}
	if (*s == '.') {
		s++;
		double scale = 0.1;
		while (*s >= '0' && *s <= '9') {
			value += (double) (*s - '0') * scale;
			scale *= 0.1;
			any_digits = true;
			s++;
		}
	}

	if (!any_digits || *s != '\0') {
		return false; // empty, or trailing rubbish -- either way not a number
	}
	out = negative ? -value : value;
	return true;
}

bool parseUint8(const char *s, uint8_t &out) {
	if (s == nullptr || *s == '\0') {
		return false;
	}
	uint32_t value = 0;
	for (; *s != '\0'; s++) {
		if (*s < '0' || *s > '9') {
			return false;
		}
		value = value * 10u + (uint32_t) (*s - '0');
		if (value > 255u) {
			return false;
		}
	}
	out = (uint8_t) value;
	return true;
}

// NMEA position is ddmm.mmmm / dddmm.mmmm -- degrees and minutes run together
// with no separator, which is the single most common thing to get wrong in an
// NMEA parser. 4807.038 is 48 degrees 07.038 minutes, NOT 4807.038 degrees.
bool parseCoordinate(const char *field, const char *hemisphere, double &degrees) {
	double raw = 0.0;
	if (!parseDouble(field, raw)) {
		return false;
	}
	if (hemisphere == nullptr || hemisphere[0] == '\0' || hemisphere[1] != '\0') {
		return false;
	}

	const double whole_degrees = (double) (long) (raw / 100.0);
	const double minutes = raw - whole_degrees * 100.0;
	if (minutes < 0.0 || minutes >= 60.0) {
		return false; // not a valid minutes field
	}

	double value = whole_degrees + minutes / 60.0;
	switch (hemisphere[0]) {
	case 'N':
	case 'E':
		break;
	case 'S':
	case 'W':
		value = -value;
		break;
	default:
		return false;
	}

	degrees = value;
	return true;
}

int hexDigit(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

// True when `type` (3 chars, e.g. "GGA") matches the sentence id, whatever the
// talker is. The M8N sends $GNGGA with a combined solution and $GPGGA with GPS
// alone, so matching "GPGGA" works on the bench and stops the day GLONASS
// comes into view.
bool sentenceIs(const char *id, const char *type) {
	const size_t len = strlen(id);
	if (len < 3) {
		return false;
	}
	return strncmp(id + (len - 3), type, 3) == 0;
}

} // namespace

NEOM8N::NEOM8N(UART_HandleTypeDef *huart) :
		_huart { huart }, _rx_buffer { }, _read_index { 0 }, _line { },
		_line_len { 0 }, _in_sentence { false }, _line_dropped { false },
		_data { }, _have_fix { false }, _last_fix_ms { 0 },
		_sentence_count { 0 }, _checksum_error_count { 0 },
		_overrun_count { 0 }, _dma_restart_count { 0 } {
	// Zeroed rather than left indeterminate: readData() before the first fix
	// reports ERROR, but a caller that ignores the return gets zeros rather
	// than a position made of stack garbage.
}

GPS_StatusTypeDef NEOM8N::init(void) {
	if (_huart == nullptr) {
		return GPS_StatusTypeDef::ERROR;
	}

	_read_index = 0;
	_line_len = 0;
	_in_sentence = false;
	_line_dropped = false;

	if (HAL_UART_Receive_DMA(_huart, _rx_buffer, NEOM8N_RX_BUFFER_LEN) != HAL_OK) {
		return GPS_StatusTypeDef::ERROR;
	}
	return GPS_StatusTypeDef::OK;
}

GPS_StatusTypeDef NEOM8N::restartDma(void) {
	// A framing, noise or overrun error aborts HAL reception and nothing
	// re-arms it; without this the module goes permanently silent the first
	// time the wire is touched or the module browns out on a power dip.
	HAL_UART_DMAStop(_huart);
	_huart->ErrorCode = HAL_UART_ERROR_NONE;
	_read_index = 0;
	_line_len = 0;
	_in_sentence = false;
	_line_dropped = false;
	_dma_restart_count++;

	if (HAL_UART_Receive_DMA(_huart, _rx_buffer, NEOM8N_RX_BUFFER_LEN) != HAL_OK) {
		return GPS_StatusTypeDef::ERROR;
	}
	return GPS_StatusTypeDef::OK;
}

void NEOM8N::update(void) {
	if (_huart == nullptr || _huart->hdmarx == nullptr) {
		return;
	}

	if (_huart->RxState != HAL_UART_STATE_BUSY_RX) {
		restartDma();
		return;
	}

	// NDTR counts DOWN from the transfer length, so the DMA has written
	// (LEN - NDTR) bytes and is about to write at that index.
	const uint32_t remaining = __HAL_DMA_GET_COUNTER(_huart->hdmarx);
	uint16_t write_index = (remaining > NEOM8N_RX_BUFFER_LEN)
			? 0 : (uint16_t) (NEOM8N_RX_BUFFER_LEN - remaining);
	if (write_index >= NEOM8N_RX_BUFFER_LEN) {
		write_index = 0; // NDTR == 0 at the exact wrap point
	}

	const uint16_t available = (uint16_t) ((write_index + NEOM8N_RX_BUFFER_LEN
			- _read_index) % NEOM8N_RX_BUFFER_LEN);

	// Within one sentence of being lapped means update() is not being called
	// often enough. A heuristic -- circular DMA gives no lap counter -- and
	// correctness does not depend on catching it, because a mangled sentence
	// fails its checksum. It is here so a too-slow loop shows up as a number
	// rather than as a fix that occasionally stutters.
	if (available > (NEOM8N_RX_BUFFER_LEN - NEOM8N_LINE_MAX)) {
		_overrun_count++;
	}

	for (uint16_t i = 0; i < available; i++) {
		parseByte((char) _rx_buffer[_read_index]);
		_read_index = (uint16_t) ((_read_index + 1) % NEOM8N_RX_BUFFER_LEN);
	}

	// Age the fix here rather than in readData(), so the flag is a property of
	// the last update() and two reads in one loop iteration cannot disagree
	// about whether the position is still current.
	if (_have_fix) {
		if ((HAL_GetTick() - _last_fix_ms) > NEOM8N_FIX_TIMEOUT_MS) {
			_data.flags |= GPS_FLAG_STALE;
		} else {
			_data.flags &= (uint8_t) ~GPS_FLAG_STALE;
		}
	}
}

void NEOM8N::parseByte(char c) {
	if (c == '$') {
		// Always resynchronise on '$', even mid-sentence. A dropped byte would
		// otherwise glue two sentences together and cost both of them.
		_in_sentence = true;
		_line_dropped = false;
		_line_len = 0;
		return;
	}

	if (!_in_sentence) {
		return; // between sentences, or joined the stream mid-line
	}

	if (c == '\r' || c == '\n') {
		if (!_line_dropped && _line_len > 0) {
			_line[_line_len] = '\0';
			parseSentence(_line, _line_len);
		}
		_in_sentence = false;
		_line_dropped = false;
		_line_len = 0;
		return;
	}

	if (_line_len >= NEOM8N_LINE_MAX) {
		// Refuse rather than truncate: a shortened sentence would fail its
		// checksum anyway, and this way it is not also counted as corrupt.
		_line_dropped = true;
		return;
	}
	_line[_line_len++] = c;
}

bool NEOM8N::parseSentence(char *line, uint16_t len) {
	// "<id>,<field>,...*<hh>". The checksum is an XOR of everything between
	// the '$' (already stripped) and the '*'.
	uint16_t star = 0;
	bool found_star = false;
	for (uint16_t i = 0; i < len; i++) {
		if (line[i] == '*') {
			star = i;
			found_star = true;
			break;
		}
	}
	if (!found_star || (uint16_t) (star + 2) >= len) {
		_checksum_error_count++;
		return false; // no checksum, or truncated before both hex digits
	}

	const int hi = hexDigit(line[star + 1]);
	const int lo = hexDigit(line[star + 2]);
	if (hi < 0 || lo < 0) {
		_checksum_error_count++;
		return false;
	}

	uint8_t sum = 0;
	for (uint16_t i = 0; i < star; i++) {
		sum = (uint8_t) (sum ^ (uint8_t) line[i]);
	}
	if (sum != (uint8_t) ((hi << 4) | lo)) {
		// The only integrity check NMEA has, and the thing that stops a
		// half-received sentence from being decoded as a plausible position.
		_checksum_error_count++;
		return false;
	}

	line[star] = '\0'; // drop the checksum; the payload is now NUL-terminated

	// Split on commas in place. Empty fields are normal and meaningful -- they
	// mean "the module has no value for this" -- so they are kept as "".
	char *fields[NEOM8N_MAX_FIELDS];
	uint8_t count = 0;
	fields[count++] = line;
	for (uint16_t i = 0; i < star && count < NEOM8N_MAX_FIELDS; i++) {
		if (line[i] == ',') {
			line[i] = '\0';
			fields[count++] = &line[i + 1];
		}
	}

	if (sentenceIs(fields[0], "GGA")) {
		parseGGA(fields, count);
	} else if (sentenceIs(fields[0], "RMC")) {
		parseRMC(fields, count);
	} else {
		return true; // valid sentence, just not one we use (GSV, GSA, VTG...)
	}

	_sentence_count++;
	return true;
}

// $--GGA,time,lat,N/S,lon,E/W,quality,sats,HDOP,alt,M,geoid,M,age,ref*cs
void NEOM8N::parseGGA(char *fields[], uint8_t count) {
	if (count < 10) {
		return;
	}

	uint8_t quality = 0;
	if (!parseUint8(fields[6], quality)) {
		return; // no fix quality field at all -- nothing here is trustworthy
	}
	_data.fix_quality = quality;

	uint8_t sats = 0;
	if (parseUint8(fields[7], sats)) {
		_data.num_satellites = sats;
	}

	if (quality == 0) {
		// No fix. The position and altitude fields are either empty or the
		// module's last guess; either way they are not a position. Leave the
		// stored fix alone and let it age out rather than overwriting a good
		// one with a bad one.
		return;
	}

	double latitude = 0.0;
	double longitude = 0.0;
	double altitude = 0.0;
	const bool have_position = parseCoordinate(fields[2], fields[3], latitude)
			&& parseCoordinate(fields[4], fields[5], longitude);
	if (!have_position) {
		return;
	}

	_data.latitude = latitude;
	_data.longitude = longitude;
	if (parseDouble(fields[9], altitude)) {
		_data.altitude = altitude;
	}

	_data.flags |= GPS_FLAG_VALID;
	_data.flags &= (uint8_t) ~GPS_FLAG_STALE;
	_have_fix = true;
	_last_fix_ms = HAL_GetTick();
}

// $--RMC,time,status,lat,N/S,lon,E/W,speed,course,date,magvar,E/W*cs
void NEOM8N::parseRMC(char *fields[], uint8_t count) {
	if (count < 9) {
		return;
	}

	// 'A' active, 'V' void. A void RMC carries stale or empty fields, so it
	// says nothing except "not now".
	if (fields[2][0] != 'A' || fields[2][1] != '\0') {
		return;
	}

	double latitude = 0.0;
	double longitude = 0.0;
	if (!parseCoordinate(fields[3], fields[4], latitude)
			|| !parseCoordinate(fields[5], fields[6], longitude)) {
		return;
	}
	_data.latitude = latitude;
	_data.longitude = longitude;

	// Knots on the wire; every consumer of this wants m/s.
	double speed_knots = 0.0;
	if (parseDouble(fields[7], speed_knots)) {
		_data.speed = speed_knots * KNOTS_TO_MPS;
	}

	// Course is undefined when stationary and the module leaves it empty --
	// keeping the previous heading is better than snapping it to zero, which
	// would read as "pointing due north".
	double course = 0.0;
	if (parseDouble(fields[8], course) && course >= 0.0 && course < 360.0) {
		_data.course = course;
	}

	_data.flags |= GPS_FLAG_VALID;
	_data.flags &= (uint8_t) ~GPS_FLAG_STALE;
	_have_fix = true;
	_last_fix_ms = HAL_GetTick();
}

GPS_StatusTypeDef NEOM8N::readData(GPS_Data &data) {
	if (!_have_fix) {
		return GPS_StatusTypeDef::ERROR;
	}
	data = _data;
	return GPS_StatusTypeDef::OK;
}

bool NEOM8N::hasFix(void) const {
	return _have_fix && ((_data.flags & GPS_FLAG_STALE) == 0)
			&& _data.fix_quality > 0;
}

uint32_t NEOM8N::getLastFixAgeMs(void) const {
	if (!_have_fix) {
		return UINT32_MAX;
	}
	return HAL_GetTick() - _last_fix_ms;
}
