/*
 * BMP390-reg.hpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef BMP390_BMP390_REG_HPP_
#define BMP390_BMP390_REG_HPP_

#include "stdint.h"

namespace BMP390_REG {
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_REV_ID = 0x01;
constexpr uint8_t REG_ERR_REG = 0x02;
constexpr uint8_t REG_STATUS = 0x03;

constexpr uint8_t REG_DATA_0 = 0x04; // press_7_0
constexpr uint8_t REG_DATA_1 = 0x05;  // press_15_8
constexpr uint8_t REG_DATA_2 = 0x06;  // press_23_16
constexpr uint8_t REG_DATA_3 = 0x07;  // temp_7_0
constexpr uint8_t REG_DATA_4 = 0x08;  // temp_15_8
constexpr uint8_t REG_DATA_5 = 0x09;  // temp_23_16

constexpr uint8_t REG_SENSORTIME_0 = 0x0C;
constexpr uint8_t REG_SENSORTIME_1 = 0x0D;
constexpr uint8_t REG_SENSORTIME_2 = 0x0E;

constexpr uint8_t REG_EVENT = 0x10;
constexpr uint8_t REG_INT_STATUS = 0x11;

constexpr uint8_t REG_FIFO_LENGTH_0 = 0x12;
constexpr uint8_t REG_FIFO_LENGTH_1 = 0x13;
constexpr uint8_t REG_FIFO_DATA = 0x14;
constexpr uint8_t REG_FIFO_WTM_0 = 0x15;
constexpr uint8_t REG_FIFO_WTM_1 = 0x16;
constexpr uint8_t REG_FIFO_CONFIG_1 = 0x17;
constexpr uint8_t REG_FIFO_CONFIG_2 = 0x18;

constexpr uint8_t REG_INT_CTRL = 0x19;
constexpr uint8_t REG_IF_CONF = 0x1A;
constexpr uint8_t REG_PWR_CTRL = 0x1B;
constexpr uint8_t REG_OSR = 0x1C;
constexpr uint8_t REG_ODR = 0x1D;
constexpr uint8_t REG_CONFIG = 0x1F;

constexpr uint8_t REG_CALIB_DATA_START = 0x31; // .. 0x45 (21 bytes: NVM_PAR_T1..P11)
constexpr uint8_t REG_CALIB_DATA_LEN = 21;

constexpr uint8_t REG_CMD = 0x7E;

// ---- CMD (0x7E) ----
constexpr uint8_t SOFT_RESET_CMD = 0xB6;
constexpr uint8_t FIFO_FLUSH_CMD = 0xB0;

// ---- ERR_REG (0x02) ----
constexpr uint8_t ERR_FATAL = (1 << 0);
constexpr uint8_t ERR_CMD = (1 << 1);
constexpr uint8_t ERR_CONF = (1 << 2); // set if the OSR/ODR combination can't fit in the ODR period

// ---- STATUS (0x03) ----
constexpr uint8_t STATUS_CMD_RDY = (1 << 4);
constexpr uint8_t STATUS_DRDY_PRESS = (1 << 5);
constexpr uint8_t STATUS_DRDY_TEMP = (1 << 6);

// ---- PWR_CTRL (0x1B) ----
constexpr uint8_t PRESS_EN = (1 << 0);
constexpr uint8_t TEMP_EN = (1 << 1);

// bits[5:4]: power mode
enum MODE : uint8_t {
	MODE_SLEEP = (0b00 << 4),  // default after reset/power-on
	MODE_FORCED = (0b01 << 4), // one measurement, then back to sleep
	MODE_NORMAL = (0b11 << 4)  // continuous measurement at the configured ODR
};

// ---- OSR (0x1C) ----
// bits[2:0]: pressure oversampling
enum PRESS_OS : uint8_t {
	PRESS_OS_1X = 0b000,
	PRESS_OS_2X = 0b001,
	PRESS_OS_4X = 0b010,
	PRESS_OS_8X = 0b011,
	PRESS_OS_16X = 0b100,
	PRESS_OS_32X = 0b101
};

// bits[5:3]: temperature oversampling
enum TEMP_OS : uint8_t {
	TEMP_OS_1X = (0b000 << 3),
	TEMP_OS_2X = (0b001 << 3),
	TEMP_OS_4X = (0b010 << 3),
	TEMP_OS_8X = (0b011 << 3),
	TEMP_OS_16X = (0b100 << 3),
	TEMP_OS_32X = (0b101 << 3)
};

// ---- ODR (0x1D) ----
// bits[4:0]: output data rate (subsampling of the 200Hz base rate)
enum ODR : uint8_t {
	ODR_200HZ = 0x00,
	ODR_100HZ = 0x01,
	ODR_50HZ = 0x02,
	ODR_25HZ = 0x03,
	ODR_12_5HZ = 0x04,
	ODR_6_25HZ = 0x05,
	ODR_3_1HZ = 0x06,
	ODR_1_5HZ = 0x07,
	ODR_0_78HZ = 0x08,
	ODR_0_39HZ = 0x09,
	ODR_0_2HZ = 0x0A,
	ODR_0_1HZ = 0x0B,
	ODR_0_05HZ = 0x0C,
	ODR_0_02HZ = 0x0D,
	ODR_0_01HZ = 0x0E,
	ODR_0_006HZ = 0x0F,
	ODR_0_003HZ = 0x10,
	ODR_0_001HZ = 0x11
};

// ---- CONFIG (0x1F) ----
// bits[3:1]: IIR filter coefficient
enum IIR_FILTER : uint8_t {
	IIR_FILTER_DISABLE = (0b000 << 1), // reset default
	IIR_FILTER_COEFF_1 = (0b001 << 1),
	IIR_FILTER_COEFF_3 = (0b010 << 1),
	IIR_FILTER_COEFF_7 = (0b011 << 1),
	IIR_FILTER_COEFF_15 = (0b100 << 1),
	IIR_FILTER_COEFF_31 = (0b101 << 1),
	IIR_FILTER_COEFF_63 = (0b110 << 1),
	IIR_FILTER_COEFF_127 = (0b111 << 1)
};

}

#endif /* BMP390_BMP390_REG_HPP_ */
