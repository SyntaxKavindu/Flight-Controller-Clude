/*
 * EEPROM.hpp
 *
 *  Created on: Aug 30, 2026
 *      Author: KAVINDU
 */

#ifndef EEPROM_EEPROM_HPP_
#define EEPROM_EEPROM_HPP_

#include "M24C02.hpp"

// Total device size. M24C02::writeBuffer()/readBuffer() address the part with a
// uint8_t, so a transfer running past the end WRAPS to address 0 rather than
// failing -- it would silently overwrite whatever is stored at the bottom of
// the device. Every access is bounds-checked against this.
#define EEPROM_DEVICE_SIZE 256u

// M24C02 is 256 bytes. Slots are spaced 64 bytes apart for the three sensor
// calibration blobs because a full correction record (magic + version + 3-vector
// offset + 3x3 matrix + CRC) is 56 bytes -- the previous 32-byte spacing could
// not hold one, so ACCLCALIBRATEDAT would have run into GYROCALIBRATEDAT.
// Nothing persisted anything at the old offsets, so there is no stored data to
// migrate.
enum class EEPROMLocation : uint8_t {
    ACCLCALIBRATEDAT       = 0x00, // 0x00..0x3F  64   unchanged
    COMPASSCALIBRATEDAT    = 0x40, // 0x40..0x7F  64   unchanged
    GYROCALIBRATEDAT       = 0x80, // 0x80..0xBF  64   unchanged
    SYSTEM_CONFIG          = 0xC0, // 0xC0..0xDF  32   unchanged
    USER_SETTINGS          = 0xE0, // 0xE0..0xEF  16   was 32, now 16
    BOARDLEVELCALIBRATEDAT = 0xF0  // 0xF0..0xFF  16   new
};

class EEPROM {
public:
    explicit EEPROM(I2C_HandleTypeDef *hi2c, uint8_t device_addr_bits = 0x00);
    EEPROM_StatusTypeDef init(void);
    // Defined inline: a template defined in EEPROM.cpp would only exist for
    // instantiations made inside that translation unit, so every call from
    // elsewhere (Calibrator, for one) failed to link.
    template <typename T>
    EEPROM_StatusTypeDef write(EEPROMLocation location, const T *data) {
        static_assert(sizeof(T) <= 64, "EEPROM slots are spaced 64 bytes apart");
        if (data == nullptr) {
            return EEPROM_StatusTypeDef::ERROR;
        }
        if (!fits(location, sizeof(T))) {
            return EEPROM_StatusTypeDef::ERROR;
        }
        return _storage.writeBuffer(static_cast<uint8_t>(location),
                reinterpret_cast<const uint8_t*>(data), sizeof(T));
    }

    template <typename T>
    EEPROM_StatusTypeDef write(EEPROMLocation location, const T &data) {
        return write(location, &data);
    }

    template <typename T>
    EEPROM_StatusTypeDef read(EEPROMLocation location, T *data) {
        static_assert(sizeof(T) <= 64, "EEPROM slots are spaced 64 bytes apart");
        if (data == nullptr) {
            return EEPROM_StatusTypeDef::ERROR;
        }
        if (!fits(location, sizeof(T))) {
            return EEPROM_StatusTypeDef::ERROR;
        }
        return _storage.readBuffer(static_cast<uint8_t>(location),
                reinterpret_cast<uint8_t*>(data), sizeof(T));
    }

    template <typename T>
    EEPROM_StatusTypeDef read(EEPROMLocation location, T &data) {
        return read(location, &data);
    }
private:
    // Bytes available to a slot before the next one starts, or before the end
    // of the device for the last slot.
    //
    // Bounding by the device end alone is not enough, and sizeof(T) <= 64 is
    // not either: the last two slots are only 32 bytes apart. A 56-byte record
    // at SYSTEM_CONFIG (0xC0) fits the device but overwrites the first 24 bytes
    // of USER_SETTINGS, and one at USER_SETTINGS (0xE0) runs to 0x117 which,
    // because M24C02 addresses the part with a uint8_t, wraps to 0x17 and
    // corrupts the middle of the accelerometer calibration at 0x00. Both
    // report success.
    static size_t slotCapacity(EEPROMLocation location) {
        switch (location) {
        case EEPROMLocation::ACCLCALIBRATEDAT:    return 0x40; // 0x00..0x3F
        case EEPROMLocation::COMPASSCALIBRATEDAT: return 0x40; // 0x40..0x7F
        case EEPROMLocation::GYROCALIBRATEDAT:    return 0x40; // 0x80..0xBF
        case EEPROMLocation::SYSTEM_CONFIG:       return 0x20; // 0xC0..0xDF
        case EEPROMLocation::USER_SETTINGS:       return 0x20; // 0xE0..0xFF
        default:                                  return 0;
        }
    }

    // True when `len` bytes starting at `location` stay inside that slot, and
    // therefore inside the device.
    static bool fits(EEPROMLocation location, size_t len) {
        const size_t start = static_cast<size_t>(static_cast<uint8_t>(location));
        const size_t capacity = slotCapacity(location);
        if (capacity == 0 || len > capacity) {
            return false;
        }
        return start + len <= EEPROM_DEVICE_SIZE;
    }

    M24C02 _storage;
};


// The one EEPROM. Defined in Globals.cpp.
extern EEPROM storage;

#endif /* EEPROM_EEPROM_HPP_ */
