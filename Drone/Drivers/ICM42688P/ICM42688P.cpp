/*
 * ICM42688P.cpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#include "ICM42688P.hpp"

// A dead device must not be able to stop the aircraft.
//
// Every transfer here used ICM42688P_SPI_TIMEOUT_MS, which is not a long timeout -- it is
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
#define ICM42688P_SPI_TIMEOUT_MS   5u


ICM42688P::ICM42688P(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
		uint16_t cs_pin) :
		_hspi { hspi }, _cs_port { cs_port }, _cs_pin { cs_pin } {
}

IMU_StatusTypeDef ICM42688P::init(void) {
	HAL_StatusTypeDef status;
	uint8_t tx[2];
	uint8_t rx[2];

	// ---- Soft reset ----
	tx[0] = ICM42688P_REG::REG_DEVICE_CONFIG & 0x7F; // write
	tx[1] = 0x01; // SOFT_RESET_CONFIG bit0=1

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	HAL_Delay(2); // datasheet only requires ~1ms; 2ms gives margin

	// ---- Read WHO_AM_I ----
	tx[0] = ICM42688P_REG::REG_WHO_AM_I | 0x80; // read: MSB=1
	tx[1] = 0x00;                            // dummy clock byte

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	if (rx[1] != 0x47)
		return IMU_StatusTypeDef::ERROR; // WHO_AM_I mismatch

	// ---- Select register bank 0 ----
	tx[0] = ICM42688P_REG::REG_BANK_SEL & 0x7F; // write
	tx[1] = ICM42688P_REG::REG_BANK_0; // select bank 0

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Enable Sensors ----
	tx[0] = ICM42688P_REG::REG_PWR_MGMT0 & 0x7F; // write
	tx[1] = ICM42688P_REG::PWR_MGMT0_GYRO_LN
			| ICM42688P_REG::PWR_MGMT0_ACCEL_LN; // enable gyro and accel
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Configure Gyroscope ----
	tx[0] = ICM42688P_REG::REG_GYRO_CONFIG0 & 0x7F; // write
	tx[1] = static_cast<uint8_t>(ICM42688P_REG::GYRO_FS_2000DPS)
			| static_cast<uint8_t>(ICM42688P_REG::GYRO_ODR_1KHZ); // gyro 2000dps, 1kHz

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Configure Accelerometer ----
	tx[0] = ICM42688P_REG::REG_ACCEL_CONFIG0 & 0x7F; // write
	tx[1] = static_cast<uint8_t>(ICM42688P_REG::ACCEL_FS_16G)
			| static_cast<uint8_t>(ICM42688P_REG::ACCEL_ODR_1KHZ); // accel 16g, 1kHz

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Configure Gyroscope Digital Filters ----
	tx[0] = ICM42688P_REG::REG_GYRO_CONFIG1 & 0x7F; // write
	tx[1] = ICM42688P_REG::GYRO_UI_FILT_ORD_2ND;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Configure Accelerometer Digital Filters ----
	tx[0] = ICM42688P_REG::REG_ACCEL_CONFIG1 & 0x7F; // write
	tx[1] = ICM42688P_REG::ACCEL_UI_FILT_ORD_2ND;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Configure Accelerometer and Gyroscope LPF filters Bandwidth ----
	tx[0] = ICM42688P_REG::REG_GYRO_ACCEL_CONFIG0 & 0x7F; // write
	tx[1] = static_cast<uint8_t>(ICM42688P_REG::ACCEL_UI_FILT_BW_ODR_4)
			| static_cast<uint8_t>(ICM42688P_REG::GYRO_UI_FILT_BW_ODR_4);

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, tx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	// ---- Check WHO_AM_I again to verify communication ----
	tx[0] = ICM42688P_REG::REG_WHO_AM_I | 0x80; // read: MSB=1
	tx[1] = 0x00; // dummy clock byte

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 2, ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	if (rx[1] != 0x47)
		return IMU_StatusTypeDef::ERROR; // WHO_AM_I mismatch

	return IMU_StatusTypeDef::OK;
}

IMU_StatusTypeDef ICM42688P::readRawData(IMU_RawData &raw) {
	HAL_StatusTypeDef status;
	uint8_t tx[15] = { 0 };
	uint8_t rx[15] = { 0 };

	tx[0] = ICM42688P_REG::REG_TEMP_DATA1 | 0x80; // read: MSB=1

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, sizeof(tx), ICM42688P_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return IMU_StatusTypeDef::ERROR;

	raw.temp = (int16_t) ((rx[1] << 8) | rx[2]);
	raw.accel_x = (int16_t) ((rx[3] << 8) | rx[4]);
	raw.accel_y = (int16_t) ((rx[5] << 8) | rx[6]);
	raw.accel_z = (int16_t) ((rx[7] << 8) | rx[8]);
	raw.gyro_x = (int16_t) ((rx[9] << 8) | rx[10]);
	raw.gyro_y = (int16_t) ((rx[11] << 8) | rx[12]);
	raw.gyro_z = (int16_t) ((rx[13] << 8) | rx[14]);

	return IMU_StatusTypeDef::OK;
}

IMU_StatusTypeDef ICM42688P::readData(IMU_Data &data) {
	IMU_RawData raw;
	IMU_StatusTypeDef status = readRawData(raw);
	if (status != IMU_StatusTypeDef::OK)
		return status;

	// raw -> g -> m/s^2
	float ax = (raw.accel_x / _accel_sensitivity_lsb_per_g)
			* STANDARD_GRAVITY_MSS;
	float ay = (raw.accel_y / _accel_sensitivity_lsb_per_g)
			* STANDARD_GRAVITY_MSS;
	float az = (raw.accel_z / _accel_sensitivity_lsb_per_g)
			* STANDARD_GRAVITY_MSS;

	float gx = (raw.gyro_x / _gyro_sensitivity_lsb_per_dps) * DEG_TO_RAD;
	float gy = (raw.gyro_y / _gyro_sensitivity_lsb_per_dps) * DEG_TO_RAD;
	float gz = (raw.gyro_z / _gyro_sensitivity_lsb_per_dps) * DEG_TO_RAD;

	// NOTE: this returns data in the chip's own native/electrical axes,
	// SI-converted but NOT rotated to the vehicle body frame. Board-mounting
	// axis remap now lives one layer up, in the IMU frontend class
	// (IMU::remapAccel()/remapGyro()) -- see that class for why it moved
	// and why accel/gyro must share one remap.
	data.accel.x = ax;
	data.accel.y = ay;
	data.accel.z = az;

	data.gyro.x = gx;
	data.gyro.y = gy;
	data.gyro.z = gz;

	// Per datasheet: Temp(°C) = (TEMP_DATA / 132.48) + 25
	data.temperature = (raw.temp / 132.48f) + 25.0f;

	return IMU_StatusTypeDef::OK;
}
