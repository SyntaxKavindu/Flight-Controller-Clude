/*
 * EEPROM.cpp
 *
 *  Created on: Aug 30, 2026
 *      Author: KAVINDU
 */

#include "EEPROM.hpp"

EEPROM::EEPROM(I2C_HandleTypeDef *hi2c, uint8_t device_addr_bits) :
		_storage(hi2c, device_addr_bits) {
}

EEPROM_StatusTypeDef EEPROM::init(void) {
	return _storage.init();
}

// The read/write templates live in EEPROM.hpp -- see the note there.
