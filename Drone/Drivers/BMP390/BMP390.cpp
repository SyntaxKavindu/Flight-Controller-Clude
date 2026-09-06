/*
 * BMP390.cpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#include "BMP390.hpp"

// A dead device must not be able to stop the aircraft.
//
// Every transfer here used BMP390_SPI_TIMEOUT_MS, which is not a long timeout -- it is
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
#define BMP390_SPI_TIMEOUT_MS   5u

// The part is not ready the instant power arrives, and it does not know it is
// on SPI yet. Both are why init() failed only SOMETIMES.
#define BMP390_STARTUP_DELAY_MS 5u
#define BMP390_ID_ATTEMPTS      5u

#include <cmath>

BMP390::BMP390(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) :
		_hspi { hspi }, _cs_port { cs_port }, _cs_pin { cs_pin } {
}

		BARO_StatusTypeDef BMP390::init(void) {

	HAL_StatusTypeDef status;
	uint8_t tx[3];
	uint8_t rx[3];

	// The BMP390 needs a few milliseconds after power-up before it answers,
	// and it chooses between SPI and I2C on the FIRST RISING EDGE of CSB --
	// so the very first transaction after power-on can be swallowed selecting
	// the interface rather than returning data. Bosch's own driver discards a
	// first CHIP_ID read for exactly this reason.
	//
	// Drone::init() runs within milliseconds of power-on, which is why this
	// failed intermittently rather than every time: whether the part was ready
	// depended on how fast the rest of the boot happened to be.
	HAL_Delay(BMP390_STARTUP_DELAY_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET); // CSB rising edge -> SPI
	HAL_Delay(1);

	// ---- Read CHIP_ID ----//
	// Retried, because the part may still be finishing its own startup. A
	// sensor that needs a second attempt is not a sensor that is broken, and
	// giving up on the first try is what turned a 5 ms timing margin into
	// "$ERR,BAROMETER INIT FAILED".
	bool identified = false;
	for (uint8_t attempt = 0; attempt < BMP390_ID_ATTEMPTS && !identified; attempt++) {
		tx[0] = BMP390_REG::REG_CHIP_ID | 0x80;  // 0x00 | RW=1 → 0x80
		tx[1] = 0x00;                            // dummy byte (BMP390-specific!)
		tx[2] = 0x00;                            // clock byte for real data

		HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
		status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 3, BMP390_SPI_TIMEOUT_MS);
		HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);

		if (status == HAL_OK && rx[2] == 0x60) { // note: rx[2], not rx[1]!
			identified = true;
		} else {
			HAL_Delay(2);
		}
	}

	if (!identified)
		return BARO_StatusTypeDef::ERROR; // no answer, or CHIP_ID mismatch

	// ---- Soft reset ----
	uint8_t cmd_tx[2];
	cmd_tx[0] = BMP390_REG::REG_CMD & 0x7F; // write
	cmd_tx[1] = BMP390_REG::SOFT_RESET_CMD;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, cmd_tx, 2, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	HAL_Delay(2); // let the reset complete and NVM trim reload before further access

	// ---- Read calibration (trim) data from NVM ----
	BARO_StatusTypeDef calib_status = readCalibrationData();
	if (calib_status != BARO_StatusTypeDef::OK)
		return calib_status;

	// ---- OSR: pressure x8, temperature x1 ----
	uint8_t cfg_tx[2];
	cfg_tx[0] = BMP390_REG::REG_OSR & 0x7F; // write
	cfg_tx[1] = static_cast<uint8_t>(BMP390_REG::PRESS_OS_8X) | static_cast<uint8_t>(BMP390_REG::TEMP_OS_1X);

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, cfg_tx, 2, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	// ---- ODR: 50Hz ----
	cfg_tx[0] = BMP390_REG::REG_ODR & 0x7F; // write
	cfg_tx[1] = BMP390_REG::ODR_50HZ;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, cfg_tx, 2, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	// ---- CONFIG: IIR filter coefficient 3 ----
	cfg_tx[0] = BMP390_REG::REG_CONFIG & 0x7F; // write
	cfg_tx[1] = BMP390_REG::IIR_FILTER_COEFF_3;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, cfg_tx, 2, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	// ---- PWR_CTRL: enable pressure + temp, normal (continuous) mode ----
	cfg_tx[0] = BMP390_REG::REG_PWR_CTRL & 0x7F; // write
	cfg_tx[1] = BMP390_REG::PRESS_EN | BMP390_REG::TEMP_EN | BMP390_REG::MODE_NORMAL;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(_hspi, cfg_tx, 2, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	// ---- Verify the OSR/ODR combination was accepted ----
	tx[0] = BMP390_REG::REG_ERR_REG | 0x80;
	tx[1] = 0x00;
	tx[2] = 0x00;

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, 3, BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	if (rx[2] & BMP390_REG::ERR_CONF)
		return BARO_StatusTypeDef::ERROR; // OSR/ODR combination invalid

	return BARO_StatusTypeDef::OK;
}

		BARO_StatusTypeDef BMP390::readCalibrationData(void) {
	HAL_StatusTypeDef status;
	// 1 addr byte + 1 dummy byte (BMP390 SPI quirk) + 21 calibration bytes
	uint8_t tx[2 + BMP390_REG::REG_CALIB_DATA_LEN] = { 0 };
	uint8_t rx[2 + BMP390_REG::REG_CALIB_DATA_LEN] = { 0 };

	tx[0] = BMP390_REG::REG_CALIB_DATA_START | 0x80; // read
	tx[1] = 0x00;                                    // dummy byte

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, sizeof(tx), BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	const uint8_t *d = &rx[2]; // real calibration data starts here

	uint16_t reg_t1 = (uint16_t) ((d[1] << 8) | d[0]);
	uint16_t reg_t2 = (uint16_t) ((d[3] << 8) | d[2]);
	int8_t reg_t3 = (int8_t) d[4];

	int16_t reg_p1 = (int16_t) ((d[6] << 8) | d[5]);
	int16_t reg_p2 = (int16_t) ((d[8] << 8) | d[7]);
	int8_t reg_p3 = (int8_t) d[9];
	int8_t reg_p4 = (int8_t) d[10];
	uint16_t reg_p5 = (uint16_t) ((d[12] << 8) | d[11]);
	uint16_t reg_p6 = (uint16_t) ((d[14] << 8) | d[13]);
	int8_t reg_p7 = (int8_t) d[15];
	int8_t reg_p8 = (int8_t) d[16];
	int16_t reg_p9 = (int16_t) ((d[18] << 8) | d[17]);
	int8_t reg_p10 = (int8_t) d[19];
	int8_t reg_p11 = (int8_t) d[20];

	_par_t1 = (double) reg_t1 / 0.00390625;               // / (1 / 2^8)
	_par_t2 = (double) reg_t2 / 1073741824.0;             // / 2^30
	_par_t3 = (double) reg_t3 / 281474976710656.0;        // / 2^48

	_par_p1 = (double) (reg_p1 - 16384) / 1048576.0;      // / 2^20
	_par_p2 = (double) (reg_p2 - 16384) / 536870912.0;    // / 2^29
	_par_p3 = (double) reg_p3 / 4294967296.0;             // / 2^32
	_par_p4 = (double) reg_p4 / 137438953472.0;           // / 2^37
	_par_p5 = (double) reg_p5 / 0.125;                    // / (1 / 2^3)
	_par_p6 = (double) reg_p6 / 64.0;                     // / 2^6
	_par_p7 = (double) reg_p7 / 256.0;                    // / 2^8
	_par_p8 = (double) reg_p8 / 32768.0;                  // / 2^15
	_par_p9 = (double) reg_p9 / 281474976710656.0;        // / 2^48
	_par_p10 = (double) reg_p10 / 281474976710656.0;      // / 2^48
	_par_p11 = (double) reg_p11 / 36893488147419103232.0; // / 2^65

	return BARO_StatusTypeDef::OK;
}

float BMP390::compensateTemperature(uint32_t uncomp_temp) {
	float partial_data1 = (float) (uncomp_temp - _par_t1);
	float partial_data2 = (float) (partial_data1 * _par_t2);

	_t_lin = partial_data2 + (partial_data1 * partial_data1) * _par_t3;

	return _t_lin;
}

float BMP390::compensatePressure(uint32_t uncomp_press) {
	double up = (double) uncomp_press;

	double partial_data1 = _par_p6 * _t_lin;
	double partial_data2 = _par_p7 * (_t_lin * _t_lin);
	double partial_data3 = _par_p8 * (_t_lin * _t_lin * _t_lin);
	double partial_out1 = _par_p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = _par_p2 * _t_lin;
	partial_data2 = _par_p3 * (_t_lin * _t_lin);
	partial_data3 = _par_p4 * (_t_lin * _t_lin * _t_lin);
	double partial_out2 = up * (_par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = up * up;
	partial_data2 = _par_p9 + _par_p10 * _t_lin;
	double partial_data3b = partial_data1 * partial_data2;
	double partial_data4 = partial_data3b + (up * up * up) * _par_p11;

	return (partial_out1 + partial_out2 + partial_data4);
}

BARO_StatusTypeDef BMP390::readRawData(BMP390_RawData &raw) {
	HAL_StatusTypeDef status;
	// STATUS (0x03) sits immediately below DATA_0..DATA_5 (0x04..0x09), so one
	// burst gets the data-ready flags and the sample they describe atomically:
	// 1 addr byte + 1 dummy byte + 1 status byte + 6 data bytes.
	//
	// Checking DRDY matters. The caller polls at control-loop rate (~1 kHz)
	// against a 50 Hz ODR, so all but one read in twenty lands on a register
	// set the sensor may be part-way through updating. A torn 24-bit pressure
	// word is not a small error: one wrong MSB is tens of kPa, which the
	// barometric formula turns into a jump of well over a kilometre.
	uint8_t tx[9] = { 0 };
	uint8_t rx[9] = { 0 };

	tx[0] = BMP390_REG::REG_STATUS | 0x80; // read
	tx[1] = 0x00;                          // dummy byte

	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(_hspi, tx, rx, sizeof(tx), BMP390_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
	if (status != HAL_OK)
		return BARO_StatusTypeDef::ERROR;

	const uint8_t status_reg = rx[2];
	const uint8_t ready = BMP390_REG::STATUS_DRDY_PRESS | BMP390_REG::STATUS_DRDY_TEMP;
	if ((status_reg & ready) != ready) {
		return BARO_StatusTypeDef::ERROR; // no new conversion yet -- do not
		                                   // hand back the previous sample as
		                                   // though it were fresh
	}

	const uint8_t *d = &rx[3]; // real data starts after the status byte

	// 24-bit values, LSB-first: *_0 = XLSB, *_1 = LSB, *_2 = MSB
	raw.pressure = ((uint32_t) d[2] << 16) | ((uint32_t) d[1] << 8) | d[0];
	raw.temperature = ((uint32_t) d[5] << 16) | ((uint32_t) d[4] << 8) | d[3];

	return BARO_StatusTypeDef::OK;
}

BARO_StatusTypeDef BMP390::readData(BMP390_Data &data) {
	BMP390_RawData raw;
	BARO_StatusTypeDef status = readRawData(raw);
	if (status != BARO_StatusTypeDef::OK)
		return status;

	data.temperature = compensateTemperature(raw.temperature);
	data.pressure = compensatePressure(raw.pressure);
	data.altitude = pressureToAltitude(data.pressure);

	return BARO_StatusTypeDef::OK;
}

float BMP390::pressureToAltitude(double pressure_pa) {
	// pow() of a negative base with a fractional exponent is NaN, and the
	// compensation can produce a non-positive pressure from a corrupt reading
	// or a bad NVM trim block. A NaN altitude propagates into the estimator
	// and every consumer downstream with nothing to catch it, so reject the
	// input here instead of returning one.
	if (!(pressure_pa > 0.0) || !std::isfinite(pressure_pa)) {
		return 0.0f;
	}
	return 44330.0 * (1.0 - std::pow(pressure_pa / SEA_LEVEL_PRESSURE_PA, 0.190294957));
}
