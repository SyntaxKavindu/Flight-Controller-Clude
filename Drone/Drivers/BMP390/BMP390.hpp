/*
 * BMP390.hpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef BMP390_BMP390_HPP_
#define BMP390_BMP390_HPP_

#include "BMP390-reg.hpp"
#include "common.hpp"

enum class BARO_StatusTypeDef : uint8_t {
	OK = 0,
	ERROR = 1
};

struct BMP390_Data {
	double pressure;    // Pa
	double temperature; // deg C
	double altitude;    // m, +up, relative to sea_level_pa passed to readData()
};

struct BMP390_RawData {
	uint32_t pressure;
	uint32_t temperature;
};

class BMP390 {
public:
	BMP390(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
	BARO_StatusTypeDef init(void);
	BARO_StatusTypeDef readData(BMP390_Data &data);
	BARO_StatusTypeDef readRawData(BMP390_RawData &raw);

	// Barometric formula (ISA model): pressure (Pa) -> altitude (m).
	float pressureToAltitude(double pressure_pa);
private:
	SPI_HandleTypeDef *_hspi;
	GPIO_TypeDef *_cs_port;
	uint16_t _cs_pin;

	BARO_StatusTypeDef readCalibrationData(void);
	float compensateTemperature(uint32_t uncomp_temp);
	float compensatePressure(uint32_t uncomp_press);

	double _par_t1 = 0.0, _par_t2 = 0.0, _par_t3 = 0.0;
	double _par_p1 = 0.0, _par_p2 = 0.0, _par_p3 = 0.0, _par_p4 = 0.0;
	double _par_p5 = 0.0, _par_p6 = 0.0, _par_p7 = 0.0, _par_p8 = 0.0;
	double _par_p9 = 0.0, _par_p10 = 0.0, _par_p11 = 0.0;

	float _t_lin = 0.0;

	constexpr static double SEA_LEVEL_PRESSURE_PA = 101325.0; // Papr
};

#endif /* BMP390_BMP390_HPP_ */
