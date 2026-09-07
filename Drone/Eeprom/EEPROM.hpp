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

// M24C02 is 256 bytes, and a full correction record (magic + version +
// 3-vector offset + 3x3 matrix + CRC) is 56 bytes, so every slot that holds one
// must be at least 64. Three of them plus two 32-byte config slots is exactly
// 256 -- the device is full, and there is no room for a fourth calibration
// record without shrinking something else or fitting a larger part.
//
// BOARDLEVELCALIBRATEDAT was previously squeezed into 0xF0..0xFF, sixteen
// bytes, to hold a 56-byte record. It could never be written: fits() refused
// every attempt and the levelling calibration reported NOTSAVED on every run
// while working perfectly in RAM. slotCapacity() did not list it at all either,
// so its capacity came back as 0 -- the write was doubly impossible -- and the
// two disagreed about USER_SETTINGS, which slotCapacity() believed ran to 0xFF
// straight through the level slot.
//
// It now occupies 0x80, the 64-byte slot that GYROCALIBRATEDAT reserved and
// nothing ever used: there is no gyro calibration anywhere in this codebase.
// That enum entry is removed rather than left pointing at the same address,
// because an alias for a slot that now means something else is a trap.
//
// ACCL and COMPASS keep their addresses, so a calibration already stored at
// 0x00 or 0x40 survives this change. Nothing was ever stored at 0x80 or 0xF0.
enum class EEPROMLocation : uint8_t {
    ACCLCALIBRATEDAT       = 0x00, // 0x00..0x3F  64  holds a CalibrationRecord
    COMPASSCALIBRATEDAT    = 0x40, // 0x40..0x7F  64  holds a CalibrationRecord
    BOARDLEVELCALIBRATEDAT = 0x80, // 0x80..0xBF  64  holds a CalibrationRecord
    SYSTEM_CONFIG          = 0xC0, // 0xC0..0xDF  32
    USER_SETTINGS          = 0xE0  // 0xE0..0xFF  32
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
    // constexpr, and public, so a caller that knows what it is storing can
    // assert at COMPILE TIME that the slot is big enough -- see the checks at
    // the top of Calibrator.cpp. A slot missing from this switch returns 0 and
    // fits() then refuses every write to it, which is safe but silent: that is
    // exactly how the levelling record came to fail on every save with nothing
    // but a NOTSAVED to show for it.
    static constexpr size_t slotCapacity(EEPROMLocation location) {
        return (location == EEPROMLocation::ACCLCALIBRATEDAT)       ? 0x40  // 0x00..0x3F
             : (location == EEPROMLocation::COMPASSCALIBRATEDAT)    ? 0x40  // 0x40..0x7F
             : (location == EEPROMLocation::BOARDLEVELCALIBRATEDAT) ? 0x40  // 0x80..0xBF
             : (location == EEPROMLocation::SYSTEM_CONFIG)          ? 0x20  // 0xC0..0xDF
             : (location == EEPROMLocation::USER_SETTINGS)          ? 0x20  // 0xE0..0xFF
             : 0u;
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
