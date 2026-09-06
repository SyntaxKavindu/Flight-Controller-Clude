/*
 * Barometer.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#ifndef BAROMETER_BAROMETER_HPP_
#define BAROMETER_BAROMETER_HPP_

#include "BMP390.hpp"

// BMP390 operating range, per the datasheet: 300..1250 hPa. A compensated
// pressure outside this is not a real atmosphere, it is a corrupt reading.
#define BAROMETER_MIN_PRESSURE_PA   30000.0
#define BAROMETER_MAX_PRESSURE_PA  125000.0

// Largest altitude step accepted between two consecutive accepted samples, in
// metres. The barometer runs at 50 Hz, so even a fast dive moves well under a
// metre per sample; anything past this is a glitch, not a climb.
#define BAROMETER_MAX_STEP_M        20.0f

/*
 * Frontend for the BMP390. Same contract as Imu and Magnetometer: update()
 * reads once and keeps the result, getData() returns that stored sample, a
 * failed read keeps the previous one, and getData() returns ERROR without
 * touching the caller's buffer until the first successful read.
 *
 * No calibrator hook: Calibrator only corrects the accelerometer and the
 * compass. Barometric bias is handled by referencing altitude to a ground
 * pressure captured at arm time, which belongs to the estimator rather than
 * here.
 *
 * It does screen the reading, though. The estimator snaps its vertical
 * position straight to the barometer when height aiding has been gated out
 * for long enough, so one corrupt sample can teleport altitude by kilometres.
 * A reading is only accepted if the compensated pressure is inside the
 * sensor's own operating range and the altitude has not stepped further than
 * a barometer sampling at 50 Hz possibly could.
 */
class Barometer {
public:
	Barometer();

	BARO_StatusTypeDef init(void);
	void update(void);
	BARO_StatusTypeDef getData(BMP390_Data &data);

	BARO_StatusTypeDef getStatus(void) const { return _status; }
	bool hasData(void) const { return _hasData; }

	// Samples rejected as implausible since boot. Non-zero means the sensor is
	// producing corrupt readings; a rising count in flight is worth a failsafe.
	uint32_t getRejectedSampleCount(void) const { return _rejected; }

private:
	BMP390 _sensor;
	BMP390_Data _data;

	BARO_StatusTypeDef _status;
	bool _hasData;
	uint32_t _rejected;

	bool isPlausible(const BMP390_Data &sample) const;
};

// The one barometer. Defined in Globals.cpp.
extern Barometer barometer;

#endif /* BAROMETER_BAROMETER_HPP_ */
