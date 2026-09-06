/*
 * M24C02.hpp
 *
 *  Created on: Aug 15, 2026
 *      Author: KAVINDU
 */

#ifndef M24C02_M24C02_HPP_
#define M24C02_M24C02_HPP_

#include "main.h" // I2C_HandleTypeDef
#include <cstdint>
#include <cstddef>

enum class EEPROM_StatusTypeDef {
	OK, ERROR, TIMEOUT
};

class M24C02 {
public:
	explicit M24C02(I2C_HandleTypeDef *hi2c, uint8_t device_addr_bits = 0x00);

	EEPROM_StatusTypeDef init(void);
	EEPROM_StatusTypeDef writeByte(uint8_t mem_addr, uint8_t data);
	EEPROM_StatusTypeDef readByte(uint8_t mem_addr, uint8_t &data);
	EEPROM_StatusTypeDef writeBuffer(uint8_t mem_addr, const uint8_t *data, size_t len);
	EEPROM_StatusTypeDef readBuffer(uint8_t mem_addr, uint8_t *data, size_t len);

private:
	I2C_HandleTypeDef *_hi2c;
	uint8_t _dev_addr_8bit;

	static constexpr uint8_t PAGE_SIZE = 16;
	static constexpr uint32_t I2C_TIMEOUT_MS = 100;
	static constexpr uint32_t WRITE_CYCLE_TIMEOUT_MS = 20; // datasheet tWR ~5ms, margin

	EEPROM_StatusTypeDef waitWriteComplete(void);
};

#endif
