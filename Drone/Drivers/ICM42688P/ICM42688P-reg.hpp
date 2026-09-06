/*
 * ICM42688P-reg.h
 *
 *  Created on: Aug 7, 2026
 *      Author: KAVINDU
 */

#ifndef INC_ICM42688P_REG_HPP_
#define INC_ICM42688P_REG_HPP_

#include "stdint.h"

namespace ICM42688P_REG {

constexpr uint8_t REG_WHO_AM_I = 0x75; // R, reset value 0x47
constexpr uint8_t REG_DEVICE_CONFIG = 0x11; // bit4 SPI_MODE, bit0 SOFT_RESET_CONFIG
constexpr uint8_t REG_DRIVE_CONFIG = 0x13; // I2C/SPI slew rate
constexpr uint8_t REG_INT_CONFIG = 0x14; // INT1/INT2 polarity & drive
constexpr uint8_t REG_FIFO_CONFIG = 0x16; // FIFO_MODE
constexpr uint8_t REG_BANK_SEL = 0x76; // BANK_SEL

constexpr uint8_t REG_TEMP_DATA1 = 0x1D; // TEMP_DATA[15:8]
constexpr uint8_t REG_TEMP_DATA0 = 0x1E; // TEMP_DATA[7:0]

constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F; // X[15:8]
constexpr uint8_t REG_ACCEL_DATA_X0 = 0x20; // X[7:0]
constexpr uint8_t REG_ACCEL_DATA_Y1 = 0x21; // Y[15:8]
constexpr uint8_t REG_ACCEL_DATA_Y0 = 0x22; // Y[7:0]
constexpr uint8_t REG_ACCEL_DATA_Z1 = 0x23; // Z[15:8]
constexpr uint8_t REG_ACCEL_DATA_Z0 = 0x24; // Z[7:0]

constexpr uint8_t REG_GYRO_DATA_X1 = 0x25; // X[15:8]
constexpr uint8_t REG_GYRO_DATA_X0 = 0x26; // X[7:0]
constexpr uint8_t REG_GYRO_DATA_Y1 = 0x27; // Y[15:8]
constexpr uint8_t REG_GYRO_DATA_Y0 = 0x28; // Y[7:0]
constexpr uint8_t REG_GYRO_DATA_Z1 = 0x29; // Z[15:8]
constexpr uint8_t REG_GYRO_DATA_Z0 = 0x2A; // Z[7:0]

constexpr uint8_t REG_TMST_FSYNCH = 0x2B; // TMST_FSYNC_DATA[15:8]
constexpr uint8_t REG_TMST_FSYNCL = 0x2C; // TMST_FSYNC_DATA[7:0]

constexpr uint8_t REG_INT_STATUS = 0x2D; // R/C
constexpr uint8_t REG_FIFO_COUNTH = 0x2E;
constexpr uint8_t REG_FIFO_COUNTL = 0x2F;
constexpr uint8_t REG_FIFO_DATA = 0x30;
constexpr uint8_t REG_INT_STATUS2 = 0x37;
constexpr uint8_t REG_INT_STATUS3 = 0x38;

constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x4B; // FIFO_FLUSH, DMP resets, etc.
constexpr uint8_t REG_INTF_CONFIG0 = 0x4C; // FIFO/data endianness, UI_SIFS_CFG
constexpr uint8_t REG_INTF_CONFIG1 = 0x4D; // CLKSEL, RTC_MODE, ACCEL_LP_CLK_SEL

constexpr uint8_t REG_PWR_MGMT0 = 0x4E;

constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F; // reset 0x06
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50; // reset 0x06
constexpr uint8_t REG_GYRO_CONFIG1 = 0x51; // TEMP_FILT_BW, GYRO_UI_FILT_ORD, GYRO_DEC2_M2_ORD
constexpr uint8_t REG_GYRO_ACCEL_CONFIG0 = 0x52; // ACCEL_UI_FILT_BW / GYRO_UI_FILT_BW
constexpr uint8_t REG_ACCEL_CONFIG1 = 0x53; // ACCEL_UI_FILT_ORD, ACCEL_DEC2_M2_ORD
constexpr uint8_t REG_TMST_CONFIG = 0x54; // TMST_EN etc.

constexpr uint8_t SELF_TEST_CONFIG = 0x70; // Self-test

constexpr uint8_t REG_BANK_0 = 0b0000; // Bank 0
constexpr uint8_t REG_BANK_1 = 0b0001; // Bank 1
constexpr uint8_t REG_BANK_2 = 0b0010; // Bank 2
constexpr uint8_t REG_BANK_3 = 0b0011; // Bank 3
constexpr uint8_t REG_BANK_4 = 0b0100; // Bank 4

enum PWR_MGMT0 : uint8_t {
	// Gyro modes
	PWR_MGMT0_GYRO_OFF         = (0b00 << 2),
	PWR_MGMT0_GYRO_STANDBY     = (0b01 << 2),
	PWR_MGMT0_GYRO_LN          = (0b11 << 2),

	// Accelerometer modes
	PWR_MGMT0_ACCEL_OFF        = (0b00 << 0),
	PWR_MGMT0_ACCEL_LP         = (0b10 << 0),
	PWR_MGMT0_ACCEL_LN         = (0b11 << 0)
};

enum GYRO_FS : uint8_t {
	GYRO_FS_2000DPS = (0b000 << 5),
	GYRO_FS_1000DPS = (0b001 << 5),
	GYRO_FS_500DPS = (0b010 << 5),
	GYRO_FS_250DPS = (0b011 << 5),
	GYRO_FS_125DPS = (0b100 << 5),
	GYRO_FS_62_5DPS = (0b101 << 5),
	GYRO_FS_31_25DPS = (0b110 << 5),
	GYRO_FS_15_625DPS = (0b111 << 5)
};

enum GYRO_ODR : uint8_t {
	GYRO_ODR_32KHZ = 0b0001,
	GYRO_ODR_16KHZ = 0b0010,
	GYRO_ODR_8KHZ = 0b0011,
	GYRO_ODR_4KHZ = 0b0100,
	GYRO_ODR_2KHZ = 0b0101,
	GYRO_ODR_1KHZ = 0b0110,
	GYRO_ODR_200HZ = 0b0111,
	GYRO_ODR_100HZ = 0b1000,
	GYRO_ODR_50HZ = 0b1001,
	GYRO_ODR_25HZ = 0b1010,
	GYRO_ODR_12_5HZ = 0b1011,
	GYRO_ODR_500HZ = 0b1111
};

enum ACCEL_FS : uint8_t {
	ACCEL_FS_16G = (0b00 << 5),
	ACCEL_FS_8G = (0b01 << 5),
	ACCEL_FS_4G = (0b10 << 5),
	ACCEL_FS_2G = (0b11 << 5)
};

enum ACCEL_ODR : uint8_t {
	ACCEL_ODR_32KHZ = 0b0001,
	ACCEL_ODR_16KHZ = 0b0010,
	ACCEL_ODR_8KHZ = 0b0011,
	ACCEL_ODR_4KHZ = 0b0100,
	ACCEL_ODR_2KHZ = 0b0101,
	ACCEL_ODR_1KHZ = 0b0110,
	ACCEL_ODR_200HZ = 0b0111,
	ACCEL_ODR_100HZ = 0b1000,
	ACCEL_ODR_50HZ = 0b1001,
	ACCEL_ODR_25HZ = 0b1010,
	ACCEL_ODR_12_5HZ = 0b1011,
	ACCEL_ODR_6_25HZ = 0b1100,
	ACCEL_ODR_3_125HZ = 0b1101,
	ACCEL_ODR_1_5625HZ = 0b1110,
	ACCEL_ODR_500HZ = 0b1111
};

enum GYRO_UI_FILT_ORD : uint8_t {
	GYRO_UI_FILT_ORD_1ST = (0b00 << 2), // reset default
	GYRO_UI_FILT_ORD_2ND = (0b01 << 2),
	GYRO_UI_FILT_ORD_3RD = (0b11 << 2)
};

enum GYRO_UI_FILT_BW : uint8_t {
	GYRO_UI_FILT_BW_ODR_2 = 0x0, // reset default (BW = ODR/2)
	GYRO_UI_FILT_BW_ODR_4 = 0x1,
	GYRO_UI_FILT_BW_ODR_5 = 0x2,
	GYRO_UI_FILT_BW_ODR_8 = 0x3,
	GYRO_UI_FILT_BW_ODR_10 = 0x4,
	GYRO_UI_FILT_BW_ODR_16 = 0x5,
	GYRO_UI_FILT_BW_ODR_20 = 0x6,
	GYRO_UI_FILT_BW_ODR_40 = 0x7,
	GYRO_UI_FILT_BW_LL_ODR_4 = 0xE, // low-latency option
	GYRO_UI_FILT_BW_LL_ODR_8 = 0xF  // low-latency option
};

enum ACCEL_UI_FILT_BW : uint8_t {
	ACCEL_UI_FILT_BW_ODR_2 = (0x0 << 4), // reset default (BW = ODR/2)
	ACCEL_UI_FILT_BW_ODR_4 = (0x1 << 4),
	ACCEL_UI_FILT_BW_ODR_5 = (0x2 << 4),
	ACCEL_UI_FILT_BW_ODR_8 = (0x3 << 4),
	ACCEL_UI_FILT_BW_ODR_10 = (0x4 << 4),
	ACCEL_UI_FILT_BW_ODR_16 = (0x5 << 4),
	ACCEL_UI_FILT_BW_ODR_20 = (0x6 << 4),
	ACCEL_UI_FILT_BW_ODR_40 = (0x7 << 4),
	ACCEL_UI_FILT_BW_LL_ODR_4 = (0xE << 4), // low-latency option
	ACCEL_UI_FILT_BW_LL_ODR_8 = (0xF << 4)  // low-latency option
};

enum ACCEL_UI_FILT_ORD : uint8_t {
	ACCEL_UI_FILT_ORD_1ST = (0b00 << 3), // reset default
	ACCEL_UI_FILT_ORD_2ND = (0b01 << 3),
	ACCEL_UI_FILT_ORD_3RD = (0b11 << 3)
};

} // namespace ICM42688P_REG

#endif /* INC_ICM42688P_REG_HPP_ */
