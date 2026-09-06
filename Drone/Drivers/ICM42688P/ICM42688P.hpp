/*
 * ICM42688P.hpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef ICM42688P_ICM42688P_HPP_
#define ICM42688P_ICM42688P_HPP_

#include "ICM42688P-reg.hpp"
#include "common.hpp"

enum class IMU_StatusTypeDef {
	OK = 0,
	ERROR = 1
};

// EKF-ready, SI-unit output.
struct IMU_Data {
	Vector3f accel;      // m/s^2, body frame
	Vector3f gyro;       // rad/s, body frame
	float temperature;   // deg C
};

struct IMU_RawData {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temp;
};

class ICM42688P {
public:
	/*
	 * SPI = hspi1
	 * CS_PORT = SPI1_CS_GPIO_Port
	 * CS = SPI1_CS_Pin
	 */
	explicit ICM42688P(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
	IMU_StatusTypeDef init(void);

	IMU_StatusTypeDef readData(IMU_Data &data);
	IMU_StatusTypeDef readRawData(IMU_RawData &raw);

private:
	SPI_HandleTypeDef *_hspi;
	GPIO_TypeDef *_cs_port;
	uint16_t _cs_pin;

	float _accel_sensitivity_lsb_per_g = 2048.0f;   // ACCEL_FS_16G
	float _gyro_sensitivity_lsb_per_dps = 16.4f;    // GYRO_FS_2000DPS

	// SI conversion constants
	static constexpr float STANDARD_GRAVITY_MSS = 9.80665f; // g -> m/s^2
	static constexpr float DEG_TO_RAD = 0.017453292519943295f; // dps -> rad/s
};

#endif /* ICM42688P_ICM42688P_HPP_ */
