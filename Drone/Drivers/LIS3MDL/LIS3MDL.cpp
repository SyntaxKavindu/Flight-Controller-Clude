/*
 * LIS3MDL.cpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#include "LIS3MDL.hpp"

// A dead device must not be able to stop the aircraft.
//
// Every transfer here used LIS3MDL_SPI_TIMEOUT_MS, which is not a long timeout -- it is
// "block forever". A sensor that stops completing a transfer therefore parks
// the CPU inside the HAL with interrupts still running but the flight loop
// never advancing: no telemetry, no commands, no scheduler, nothing. It looks
// exactly like a dead board, and it is how a marginal barometer took the whole
// vehicle down with it.
//
// The longest transfer in this driver is a handful of bytes -- microseconds at
// any sane clock -- so this can only ever elapse when something is genuinely
// broken. Every caller already checks the status and returns an error, and the
// frontends keep their last good sample, so a timeout degrades one sensor
// instead of stopping the aircraft.
#define LIS3MDL_SPI_TIMEOUT_MS   5u


LIS3MDL::LIS3MDL(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin) :
		_hspi { hspi }, _cs_port { cs_port }, _cs_pin { cs_pin } {
}

MAG_StatusTypeDef LIS3MDL::init(void) {

	HAL_StatusTypeDef status;
	uint8_t tx[2];
	uint8_t rx[2];

	// ---- Soft reset ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG2 & 0x3F; // write (bit7=0, bit6=0)
	tx[1] = LIS3MDL_REG::SOFT_RST;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	HAL_Delay(2); // datasheet turn-on/reset settling margin

	// ---- Read WHO_AM_I ----//
	tx[0] = LIS3MDL_REG::REG_WHO_AM_I | 0x80; // read: MSB=1
	tx[1] = 0x00;                            // dummy clock byte

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	if (rx[1] != 0x3D)
		return MAG_StatusTypeDef::ERROR; // WHO_AM_I mismatch

	// ---- CTRL_REG1: enable temp sensor, ultra-high perf X/Y, 80Hz ODR ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG1 & 0x3F; // write
	tx[1] = LIS3MDL_REG::TEMP_EN | LIS3MDL_REG::OM_XY_ULTRA_HIGH
			| LIS3MDL_REG::ODR_80HZ;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// ---- CTRL_REG2: +-4 gauss full scale ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG2 & 0x3F; // write
	tx[1] = LIS3MDL_REG::FS_4GAUSS;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// ---- CTRL_REG3: continuous-conversion mode ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG3 & 0x3F; // write
	tx[1] = LIS3MDL_REG::MD_CONTINUOUS;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// ---- CTRL_REG4: ultra-high performance Z axis ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG4 & 0x3F; // write
	tx[1] = LIS3MDL_REG::OMZ_ULTRA_HIGH;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// ---- CTRL_REG5: block data update (avoids torn H/L reads) ----
	tx[0] = LIS3MDL_REG::REG_CTRL_REG5 & 0x3F; // write
	tx[1] = LIS3MDL_REG::BDU_ENABLE;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// ---- Check WHO_AM_I again to verify communication ----
	tx[0] = LIS3MDL_REG::REG_WHO_AM_I | 0x80; // read: MSB=1
	tx[1] = 0x00;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 2, LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	if (rx[1] != 0x3D)
		return MAG_StatusTypeDef::ERROR; // WHO_AM_I mismatch

	return MAG_StatusTypeDef::OK;
}

MAG_StatusTypeDef LIS3MDL::readRawData(LIS3MDL_RawData &raw) {
	HAL_StatusTypeDef status;
	uint8_t tx[9] = { 0 };
	uint8_t rx[9] = { 0 };

	tx[0] = LIS3MDL_REG::REG_OUT_X_L | LIS3MDL_REG::READ_BIT
			| LIS3MDL_REG::AUTO_INCREMENT_BIT;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, sizeof(tx), LIS3MDL_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return MAG_StatusTypeDef::ERROR;

	// Registers are little-endian: *_L is the low byte, *_H is the high byte.
	raw.mag_x = (int16_t) ((rx[2] << 8) | rx[1]);
	raw.mag_y = (int16_t) ((rx[4] << 8) | rx[3]);
	raw.mag_z = (int16_t) ((rx[6] << 8) | rx[5]);
	raw.temp = (int16_t) ((rx[8] << 8) | rx[7]);

	return MAG_StatusTypeDef::OK;
}

MAG_StatusTypeDef LIS3MDL::readData(LIS3MDL_Data &data) {
	LIS3MDL_RawData raw;
	MAG_StatusTypeDef status = readRawData(raw);
	if (status != MAG_StatusTypeDef::OK)
		return status;

	float mx = raw.mag_x / _sensitivity_lsb_per_gauss;
	float my = raw.mag_y / _sensitivity_lsb_per_gauss;
	float mz = raw.mag_z / _sensitivity_lsb_per_gauss;

	// NOTE: native/electrical axes, SI-converted but NOT rotated to the
	// vehicle body frame -- see Magnetometer::remapMag() in the frontend
	// class, which now owns the board-mounting remap.
	data.x = mx;
	data.y = my;
	data.z = mz;

	// Per datasheet: 8 LSB/°C, 0 LSB (typ) at 25°C. NOTE: this is explicitly
	// documented as a non-calibrated, relative-only output -- don't treat it
	// as an absolute temperature reading (unlike the ICM42688P's temp sensor).
	data.temperature = 25.0f + (raw.temp / 8.0f);

	return MAG_StatusTypeDef::OK;
}
