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

# Discovered, not hand-listed: the project puts every header on the include path
# by directory, and a hand-maintained copy of that list here silently stops
# covering whatever is added next.
INC := -I. -Istub $(addprefix -I,$(shell find $(ROOT)/Drone -type d))

ESEKF_SRC := $(ROOT)/Drone/Ahrs/ESEKF.cpp
# The three engines, with NO facade and NO HAL: if this link needs anything from
# stub/, the portability split has regressed.
CAL_SRC   := $(ROOT)/Drone/Calibrators/AccelerometerCalibrator.cpp \
             $(ROOT)/Drone/Calibrators/CompassCalibrator.cpp \
             $(ROOT)/Drone/Calibrators/LevelCalibrator.cpp
# Drone.cpp is now part of the stack, not an extra: `drone` IS the globals, so
# anything that touches a subsystem needs it. That is the cost of dropping
# dependency injection, and it is why `integ` and `firmware` now link the same
# set. `cal` and `ctrl` below are unaffected -- those classes never name `drone`.
STACK_SRC := $(ESEKF_SRC) $(ROOT)/Drone/Calibrators/Calibrator.cpp $(CAL_SRC) \
             $(ROOT)/Drone/Drone.cpp $(ROOT)/Drone/Imu/Imu.cpp \
             $(ROOT)/Drone/Indicator/Indicator.cpp \
             $(ROOT)/Drone/Magnetometer/Magnetometer.cpp \
             $(ROOT)/Drone/Barometer/Barometer.cpp \
             $(ROOT)/Drone/Telemetry/Telemetry.cpp $(ROOT)/Drone/Eeprom/EEPROM.cpp \
             $(ROOT)/Drone/Drivers/ICM42688P/ICM42688P.cpp \
             $(ROOT)/Drone/Drivers/LIS3MDL/LIS3MDL.cpp \
             $(ROOT)/Drone/Drivers/BMP390/BMP390.cpp \
             $(ROOT)/Drone/Drivers/M24C02/M24C02.cpp \
             stub/stub.cpp

# Every .cpp under Drone/, including files nothing links yet. GPS.cpp sat broken
# for a long time -- it included a "Matrix3f.hpp" that does not exist -- purely
# because no build ever touched it. A syntax-only sweep costs seconds and is the
# only thing that catches that class of rot.
ALL_SRC := $(shell find $(ROOT)/Drone -name '*.cpp')

.PHONY: all esekf cal integ indicator ctrl firmware compileall clean
all: compileall esekf cal integ indicator ctrl firmware
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

# The control cascade's implemented half. Links NO stub: PID, Position and
# Velocity take dt as a parameter and touch no peripheral, so if this ever
# starts needing stub/ a board dependency has crept into the control law.
ctrl: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/t_ctrl test_controllers.cpp \
		$(ROOT)/Drone/Controllers/PID/PID.cpp \
		$(ROOT)/Drone/Controllers/Position/Position.cpp \
		$(ROOT)/Drone/Controllers/Velocity/Velocity.cpp
	@$(BUILD)/t_ctrl

# The status panel. Needs the HAL stub for GPIO and the clock, but nothing else
# from the flight stack -- the class talks to pins and HAL_GetTick() and that is
# all, which is what keeps it droppable into another project.
indicator: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/t_ind test_indicator.cpp \
		$(ROOT)/Drone/Indicator/Indicator.cpp stub/stub.cpp
	@$(BUILD)/t_ind

# Not a test: a link of the complete flight stack including Drone.cpp, so a
# change that compiles in isolation but breaks the real build is caught here.
firmware: | $(BUILD)
	@$(CXX) $(CXXFLAGS) $(INC) -o $(BUILD)/firmware firmware_link.cpp \
		$(STACK_SRC)
	@$(BUILD)/firmware && echo "firmware: links and runs"

compileall:
	@for f in $(ALL_SRC); do \
		$(CXX) $(CXXFLAGS) $(INC) -fsyntax-only $$f || exit 1; \
	done
	@echo "compileall: $(words $(ALL_SRC)) source files compile"

clean:
	@rm -rf $(BUILD)
