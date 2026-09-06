# Host test suite for the flight stack's core: the ESEKF and the calibrators.
#
#   make          build and run everything
#   make esekf    just the estimator
#   make cal      just the three calibration engines (no HAL, no facade)
#   make integ    the Calibrator facade + sensor frontends + EEPROM record
#   make firmware link the whole flight stack, to prove it still builds
#   make clean
#
# No dependencies beyond a C++17 compiler. -ffast-math is deliberately absent:
# the estimator's guards are isfinite() checks and that flag deletes them.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter
ROOT     := ..
BUILD    := build

INC := -I. -I$(ROOT)/Drone/Common -I$(ROOT)/Drone/Ahrs -I$(ROOT)/Drone/Calibrators \
       -I$(ROOT)/Drone/Imu -I$(ROOT)/Drone/Magnetometer -I$(ROOT)/Drone/Barometer \
       -I$(ROOT)/Drone/Telemetry -I$(ROOT)/Drone/Eeprom -I$(ROOT)/Drone \
       -I$(ROOT)/Drone/Drivers/ICM42688P -I$(ROOT)/Drone/Drivers/LIS3MDL \
       -I$(ROOT)/Drone/Drivers/BMP390 -I$(ROOT)/Drone/Drivers/M24C02 -Istub

ESEKF_SRC := $(ROOT)/Drone/Ahrs/ESEKF.cpp
# The three engines, with NO facade and NO HAL: if this link needs anything from
# stub/, the portability split has regressed.
CAL_SRC   := $(ROOT)/Drone/Calibrators/AccelerometerCalibrator.cpp \
             $(ROOT)/Drone/Calibrators/CompassCalibrator.cpp \
             $(ROOT)/Drone/Calibrators/LevelCalibrator.cpp
STACK_SRC := $(ESEKF_SRC) $(ROOT)/Drone/Calibrators/Calibrator.cpp $(CAL_SRC) \
             $(ROOT)/Drone/Globals.cpp $(ROOT)/Drone/Imu/Imu.cpp \
             $(ROOT)/Drone/Magnetometer/Magnetometer.cpp \
             $(ROOT)/Drone/Barometer/Barometer.cpp \
             $(ROOT)/Drone/Telemetry/Telemetry.cpp $(ROOT)/Drone/Eeprom/EEPROM.cpp \
             $(ROOT)/Drone/Drivers/ICM42688P/ICM42688P.cpp \
             $(ROOT)/Drone/Drivers/LIS3MDL/LIS3MDL.cpp \
             $(ROOT)/Drone/Drivers/BMP390/BMP390.cpp \
             $(ROOT)/Drone/Drivers/M24C02/M24C02.cpp \
             stub/stub.cpp

.PHONY: all esekf cal integ firmware clean
all: esekf cal integ firmware
	@echo "=========================================="
	@echo " all suites passed"
	@echo "=========================================="

$(BUILD):
	@mkdir -p $(BUILD)

esekf: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/t_esekf test_esekf.cpp $(ESEKF_SRC)
	@$(BUILD)/t_esekf

cal: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/t_cal test_calibration.cpp $(CAL_SRC)
	@$(BUILD)/t_cal

integ: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/t_integ test_integration.cpp $(STACK_SRC)
	@$(BUILD)/t_integ

# Not a test: a link of the complete flight stack including Drone.cpp, so a
# change that compiles in isolation but breaks the real build is caught here.
firmware: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/firmware firmware_link.cpp \
		$(ROOT)/Drone/Drone.cpp $(STACK_SRC)
	@$(BUILD)/firmware && echo "firmware: links and runs"

clean:
	@rm -rf $(BUILD)
