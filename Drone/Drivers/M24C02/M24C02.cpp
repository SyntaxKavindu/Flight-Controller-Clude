/*
 * M24C02.cpp
 *
 *  Created on: Aug 15, 2026
 *      Author: KAVINDU
 */

#include "M24C02.hpp"

M24C02::M24C02(I2C_HandleTypeDef *hi2c, uint8_t device_addr_bits) :
	_hi2c { hi2c } {
	uint8_t addr_7bit = 0x50 | (device_addr_bits & 0x07);
	_dev_addr_8bit = addr_7bit << 1;
}

EEPROM_StatusTypeDef M24C02::init(void) {
	if (HAL_I2C_IsDeviceReady(_hi2c, _dev_addr_8bit, 3, I2C_TIMEOUT_MS) != HAL_OK)
		return EEPROM_StatusTypeDef::ERROR;
	return EEPROM_StatusTypeDef::OK;
}

EEPROM_StatusTypeDef M24C02::waitWriteComplete(void) {
	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < WRITE_CYCLE_TIMEOUT_MS) {
		if (HAL_I2C_IsDeviceReady(_hi2c, _dev_addr_8bit, 1, 1) == HAL_OK)
			return EEPROM_StatusTypeDef::OK;
	}
	return EEPROM_StatusTypeDef::TIMEOUT;
}

EEPROM_StatusTypeDef M24C02::writeByte(uint8_t mem_addr, uint8_t data) {
	if (HAL_I2C_Mem_Write(_hi2c, _dev_addr_8bit, mem_addr, I2C_MEMADD_SIZE_8BIT,&data, 1, I2C_TIMEOUT_MS) != HAL_OK)
		return EEPROM_StatusTypeDef::ERROR;
	return waitWriteComplete();
}

EEPROM_StatusTypeDef M24C02::readByte(uint8_t mem_addr, uint8_t &data) {
	if (HAL_I2C_Mem_Read(_hi2c, _dev_addr_8bit, mem_addr, I2C_MEMADD_SIZE_8BIT,&data, 1, I2C_TIMEOUT_MS) != HAL_OK)
		return EEPROM_StatusTypeDef::ERROR;
	return EEPROM_StatusTypeDef::OK;
}

EEPROM_StatusTypeDef M24C02::writeBuffer(uint8_t mem_addr, const uint8_t *data, size_t len) {
	size_t written = 0;
	while (written < len) {
		uint8_t addr_now = static_cast<uint8_t>(mem_addr + written);
		size_t space_in_page = PAGE_SIZE - (addr_now % PAGE_SIZE);
		size_t remaining = len - written;
		size_t chunk = (remaining < space_in_page) ? remaining : space_in_page;

		if (HAL_I2C_Mem_Write(_hi2c, _dev_addr_8bit, addr_now, I2C_MEMADD_SIZE_8BIT,
				const_cast<uint8_t*>(&data[written]), chunk, I2C_TIMEOUT_MS) != HAL_OK)
			return EEPROM_StatusTypeDef::ERROR;

		EEPROM_StatusTypeDef st = waitWriteComplete();
		if (st != EEPROM_StatusTypeDef::OK)
			return st;

		written += chunk;
	}
	return EEPROM_StatusTypeDef::OK;
}

EEPROM_StatusTypeDef M24C02::readBuffer(uint8_t mem_addr, uint8_t *data, size_t len) {
	if (HAL_I2C_Mem_Read(_hi2c, _dev_addr_8bit, mem_addr, I2C_MEMADD_SIZE_8BIT,
			data, len, I2C_TIMEOUT_MS) != HAL_OK)
		return EEPROM_StatusTypeDef::ERROR;
	return EEPROM_StatusTypeDef::OK;
}
