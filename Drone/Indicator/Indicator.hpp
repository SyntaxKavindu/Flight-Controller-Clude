/*
 * Indicator.hpp
 *
 *  Created on: Aug 31, 2026
 *      Author: KAVINDU
 */

#ifndef INDICATOR_INDICATOR_HPP_
#define INDICATOR_INDICATOR_HPP_

#include "common.hpp"

enum class SystemState {
	OK,
	ERROR
};

enum class ArmState {
	ARMED,
	DISARMED
};

enum class GPSState {
	LOCKED,
	UNLOCKED
};

class Indicator {
public:
	explicit Indicator();
	void init();
	void update();

	void setSystemState(SystemState state);
	void setArmState(ArmState state);
	void setGPSState(GPSState state);

private:
	SystemState _systemState;
	ArmState _armState;
	GPSState _gpsState;
};

#endif /* INDICATOR_INDICATOR_HPP_ */
