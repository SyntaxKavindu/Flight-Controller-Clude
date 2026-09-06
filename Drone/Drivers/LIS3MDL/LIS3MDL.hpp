/*
 * LIS3MDL.hpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef LIS3MDL_LIS3MDL_HPP_
#define LIS3MDL_LIS3MDL_HPP_

#include "LIS3MDL-reg.hpp"
#include "common.hpp"

enum class MAG_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1
};

// EKF-ready output. Gauss is fine as-is for EKF mag fusion (compared
// directly against a world magnetic field model, e.g. WMM, also in Gauss) --
// no further conversion needed here, unlike accel/gyro.
struct LIS3MDL_Data {
	float x;         // Gauss, body frame
	float y;         // Gauss, body frame
	float z;         // Gauss, body frame
	float temperature;   // deg C (relative, see readData() note)
};

// Raw two's-complement register values, before scaling.
struct LIS3MDL_RawData {
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	int16_t temp;
};

class LIS3MDL {
public:
	/*
	 * SPI = hspi3
	 * CS_PORT = SPI3_CS_GPIO_Port
	 * CS = SPI3_CS_Pin
	*/
	explicit LIS3MDL(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
	MAG_StatusTypeDef init(void);
	MAG_StatusTypeDef readData(LIS3MDL_Data &data);
	MAG_StatusTypeDef readRawData(LIS3MDL_RawData &raw);
private:
	SPI_HandleTypeDef *_hspi;
	GPIO_TypeDef *_cs_port;
	uint16_t _cs_pin;

	float _sensitivity_lsb_per_gauss = LIS3MDL_REG::SENSITIVITY_4GAUSS;
};

#endif /* LIS3MDL_LIS3MDL_HPP_ */
