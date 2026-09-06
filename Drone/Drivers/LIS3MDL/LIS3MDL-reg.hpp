/*
 * LIS3MDL-reg.hpp
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef LIS3MDL_LIS3MDL_REG_HPP_
#define LIS3MDL_LIS3MDL_REG_HPP_

#include "stdint.h"

namespace LIS3MDL_REG {
constexpr uint8_t REG_OFFSET_X_REG_L_M = 0x05;
constexpr uint8_t REG_OFFSET_X_REG_H_M = 0x06;
constexpr uint8_t REG_OFFSET_Y_REG_L_M = 0x07;
constexpr uint8_t REG_OFFSET_Y_REG_H_M = 0x08;
constexpr uint8_t REG_OFFSET_Z_REG_L_M = 0x09;
constexpr uint8_t REG_OFFSET_Z_REG_H_M = 0x0A;

constexpr uint8_t REG_WHO_AM_I = 0x0F;

constexpr uint8_t REG_CTRL_REG1 = 0x20;
constexpr uint8_t REG_CTRL_REG2 = 0x21;
constexpr uint8_t REG_CTRL_REG3 = 0x22;
constexpr uint8_t REG_CTRL_REG4 = 0x23;
constexpr uint8_t REG_CTRL_REG5 = 0x24;

constexpr uint8_t REG_STATUS_REG = 0x27;

constexpr uint8_t REG_OUT_X_L = 0x28;
constexpr uint8_t REG_OUT_X_H = 0x29;
constexpr uint8_t REG_OUT_Y_L = 0x2A;
constexpr uint8_t REG_OUT_Y_H = 0x2B;
constexpr uint8_t REG_OUT_Z_L = 0x2C;
constexpr uint8_t REG_OUT_Z_H = 0x2D;

constexpr uint8_t REG_TEMP_OUT_L = 0x2E;
constexpr uint8_t REG_TEMP_OUT_H = 0x2F;

constexpr uint8_t REG_INT_CFG = 0x30;
constexpr uint8_t REG_INT_SRC = 0x31;
constexpr uint8_t REG_INT_THS_L = 0x32;
constexpr uint8_t REG_INT_THS_H = 0x33;

// SPI address-byte control bits (OR into the register address before sending).
constexpr uint8_t READ_BIT = 0x80;         // bit7 R/W: 1 = read
constexpr uint8_t AUTO_INCREMENT_BIT = 0x40; // bit6 MS: 1 = auto-increment addr for burst transfers

constexpr uint8_t TEMP_EN = (1 << 7);

// bits[6:5]: operating mode for X/Y axes
enum OM_XY : uint8_t {
	OM_XY_LOW_POWER = (0b00 << 5),
	OM_XY_MEDIUM = (0b01 << 5),
	OM_XY_HIGH = (0b10 << 5),
	OM_XY_ULTRA_HIGH = (0b11 << 5)
};

// bits[4:2]: output data rate (only valid when FAST_ODR is not set)
enum ODR : uint8_t {
	ODR_0_625HZ = (0b000 << 2),
	ODR_1_25HZ = (0b001 << 2),
	ODR_2_5HZ = (0b010 << 2),
	ODR_5HZ = (0b011 << 2),
	ODR_10HZ = (0b100 << 2),
	ODR_20HZ = (0b101 << 2),
	ODR_40HZ = (0b110 << 2),
	ODR_80HZ = (0b111 << 2)
};

constexpr uint8_t FAST_ODR = (1 << 1); // enables ODR > 80Hz (not used with the ODR enum above)
constexpr uint8_t SELF_TEST = (1 << 0);

// ---- CTRL_REG2 (0x21) ----
// bits[6:5]: full-scale range
enum FS : uint8_t {
	FS_4GAUSS = (0b00 << 5),
	FS_8GAUSS = (0b01 << 5),
	FS_12GAUSS = (0b10 << 5),
	FS_16GAUSS = (0b11 << 5)
};

constexpr uint8_t REBOOT = (1 << 3);
constexpr uint8_t SOFT_RST = (1 << 2);

// Sensitivity for each FS setting, in LSB per Gauss (reciprocal of the
// datasheet's Gauss/LSB figures).
constexpr float SENSITIVITY_4GAUSS = 6842.0f;
constexpr float SENSITIVITY_8GAUSS = 3421.0f;
constexpr float SENSITIVITY_12GAUSS = 2281.0f;
constexpr float SENSITIVITY_16GAUSS = 1711.0f;

// ---- CTRL_REG3 (0x22) ----
// bits[1:0]: system operating mode
enum MD : uint8_t {
	MD_CONTINUOUS = 0b00,
	MD_SINGLE = 0b01,
	MD_POWER_DOWN = 0b10 // 0b11 is also power-down
};

constexpr uint8_t SIM_3WIRE = (1 << 2); // 0 = 4-wire SPI (default), 1 = 3-wire
constexpr uint8_t LP_ENABLE = (1 << 5); // forces DO to 0.625Hz, minimum averaging

// ---- CTRL_REG4 (0x23) ----
// bits[3:2]: operating mode for the Z axis
enum OMZ : uint8_t {
	OMZ_LOW_POWER = (0b00 << 2),
	OMZ_MEDIUM = (0b01 << 2),
	OMZ_HIGH = (0b10 << 2),
	OMZ_ULTRA_HIGH = (0b11 << 2)
};

// Named ENDIAN_BIG, not BIG_ENDIAN: <endian.h> defines BIG_ENDIAN as a
// macro, and a macro ignores the namespace. Any translation unit that
// pulled it in first failed to compile this header with a confusing
// "expected unqualified-id" error.
constexpr uint8_t ENDIAN_BIG = (1 << 1); // 0 = little-endian (default)

// ---- CTRL_REG5 (0x24) ----
constexpr uint8_t BDU_ENABLE = (1 << 6); // block data update: hold H/L pair until both read
constexpr uint8_t FAST_READ_ENABLE = (1 << 7); // read only the 8 MSBs, single-byte

}

#endif /* LIS3MDL_LIS3MDL_REG_HPP_ */
