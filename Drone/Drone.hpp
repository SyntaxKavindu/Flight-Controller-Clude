/*
 * Drone.hpp
 *
 *  Created on: Aug 25, 2026
 *      Author: KAVINDU
 *
 * Top level of the flight stack. Owns the state estimator and runs the
 * sense -> estimate -> report cycle; everything else it works through is a
 * global (see Globals.hpp).
 *
 * Wiring it up
 * ------------
 * main.cpp is C++, so it holds a Drone and calls it directly:
 *
 *     #include "Drone.hpp"
 *
 *     static Drone drone;   // static, not a local in main(): it carries the
 *                           // ESEKF's 15x15 covariance and process noise,
 *                           // about 2 kB that has no business on the stack.
 *
 *     int main(void)
 *     {
 *         HAL_Init();
 *         SystemClock_Config();
 *         MX_GPIO_Init();
 *         MX_SPI1_Init();  MX_SPI2_Init();  MX_SPI3_Init();
 *         MX_I2C1_Init();
 *         MX_USB_DEVICE_Init();
 *
 *         drone.init();                 // AFTER the peripherals above exist
 *         while (1) { drone.loop(); }
 *     }
 *
 * init() must come after the SPI, I2C and USB peripherals are initialised: it
 * probes every sensor and the EEPROM, and a bus that does not exist yet
 * reports as a dead device. SysTick must also be running -- HAL_GetTick() is
 * what times the estimator's integration step and dispatches the mid and slow
 * loops.
 *
 * Serial commands are NOT handled here. usbd_cdc_if.c is CubeMX-generated
 * plain C and cannot call a C++ method, so it still goes through the one
 * remaining C shim:
 *
 *     // usbd_cdc_if.c, inside CDC_Receive_FS():
 *     Telemetry_Receive(Buf, *Len);   // see TelemetryCBridge.h
 *
 * Nothing else is required. loop() samples the sensors, feeds any running
 * calibration, advances the estimator and publishes
 *
 *     IMU: 1.23 Roll, -0.45 Pitch, 87.60 Yaw, 12.30 Altitude
 *     QUAT: 0.9993 W, 0.0107 X, -0.0039 Y, 0.0348 Z
 *     TILT: 1.31 Tilt
 *     EKF: OK Attitude, OK Height, OK Yaw, NO DeadReckon
 */

#ifndef DRONE_HPP_
#define DRONE_HPP_

#include "ESEKF.hpp"
#include "Globals.hpp"

// ===========================================================================
// MAGNETIC DECLINATION -- EDIT HERE, AND ONLY HERE
//
// !! UNVERIFIED placeholder for Sri Lanka. Angle from TRUE north to MAGNETIC
// !! north, degrees, positive east. Look yours up in a World Magnetic Model;
// !! it ranges roughly -25..+25 deg across the populated world.
//
// This is what makes yaw ABSOLUTE. With the wrong value the filter is not
// unhealthy and reports nothing wrong -- it simply flies confidently in a
// rotated frame, which for an autonomous mission means flying the right
// pattern in the wrong direction.
// ===========================================================================
#define DRONE_MAGNETIC_DECLINATION_DEG   (-2.0f)

// The airframe must be within this of 1 g, in m/s^2, before the estimator is
// seeded: initialize() derives the initial attitude from the accelerometer, so
// a moving vehicle would be levelled against its own acceleration.
#define DRONE_SEED_STILLNESS_MPS2        0.6f

// ---------------------------------------------------------------------------
// Loop rates.
//
// The fast loop runs on every call of loop(), which free-runs: it reads the
// IMU, propagates the estimator and fuses the accelerometer. Attitude is only
// as good as how often it is integrated, so this is the one that wants to be
// quick -- real flight controllers put it between 400 Hz and 8 kHz.
//
// The mid loop runs the magnetometer and barometer. Those parts are configured
// at 80 Hz and 50 Hz respectively, so polling them faster only re-reads
// registers that have not changed -- and re-fusing a sample the filter has
// already seen counts the same information twice and makes it overconfident,
// exactly the reason the accelerometer is only fused when a predict actually
// happened. 50 Hz matches the slower of the two sensors.
//
// The slow loop is for housekeeping that does not belong in either: battery
// monitoring, GPS status, logging, failsafe checks. It currently emits the
// heartbeat and reports sensor health, and is otherwise reserved.
//
// All three are dispatched from loop() off HAL_GetTick(), which is 1 ms. That
// is coarse -- it is why there is no rate gate on the fast group, and why the
// mid and slow periods are whole milliseconds. A cooperative scheduler with
// per-task budgets belongs here eventually; it needs a microsecond clock
// first, which is a timer that is not wired up yet.
// ---------------------------------------------------------------------------
#define DRONE_MID_LOOP_HZ                50
#define DRONE_SLOW_LOOP_HZ                1

#define DRONE_MID_LOOP_PERIOD_MS         (1000u / DRONE_MID_LOOP_HZ)
#define DRONE_SLOW_LOOP_PERIOD_MS        (1000u / DRONE_SLOW_LOOP_HZ)

// State is published on its own rate gate, off the same tick. At the default
// this is 10 Hz -- fast enough to plot, slow enough to read in a terminal.
//
// Published as:
//   IMU: 1.23 Roll, -0.45 Pitch, 87.60 Yaw, 12.30 Altitude
//
// Altitude is BAROMETRIC, in metres positive up, relative to the point where
// the estimator was seeded. The barometer is the only height source; no GPS
// is wired up.
//   QUAT: 0.9993 W, 0.0107 X, -0.0039 Y, 0.0348 Z
//
// The same attitude as the IMU line, but as the estimator actually holds it.
// The Euler triple is for reading; the quaternion is for using. ZYX Euler has
// a singularity at pitch = +/-90 deg where roll and yaw stop being separable,
// so a horizon display or an attitude controller driven from the triple will
// misbehave nose-up even though the estimate is fine. Nothing inside the
// filter ever forms an Euler triple -- getEulerAngles() converts on demand.
//   TILT: 1.31 Tilt
//
// Angle between the airframe's Down axis and true Down, degrees: 0 upright,
// 90 knife-edge or nose-vertical, 180 inverted. Heading drops out, and there
// is no singularity anywhere in the range -- this is the number a tilt limit
// or a crash detector wants, not roll and pitch separately.
//   EKF: OK Attitude, OK Height, OK Yaw, NO DeadReckon
#define DRONE_PUBLISH_HZ                 10

#define DRONE_PUBLISH_PERIOD_MS          (1000u / DRONE_PUBLISH_HZ)

// Seconds without an attitude estimate before the warning starts. Long enough
// that a normal boot -- which seeds within a second of the board being put
// down -- stays quiet, short enough that a real failure is obvious.
#define DRONE_NO_ESTIMATE_WARN_S         3u

class Drone {
public:
	Drone();

	// Brings up the globals and configures the estimator. Call once.
	void init(void);

	// Call it in a tight while(1). It free-runs rather than pacing itself:
	// the fast group runs every pass, and the mid, publish and slow groups are
	// dispatched off HAL_GetTick() when their periods come due.
	void loop(void);

	// Dumps the loop counters and the estimator's health on demand. Answers
	// the one question a silent vehicle cannot: is the loop running and every
	// group bailing out, or has the loop itself stopped?
	void reportDiagnostics(void);

private:
	ESEKF _esekf;

	uint8_t _bootStatus;        // GlobalsInitStatus bitmask from init()
	uint32_t _lastPredictTick;  // HAL_GetTick() at the last integrated step
	uint32_t _lastMidTick;
	uint32_t _lastSlowTick;
	uint32_t _lastPublishTick;
	float _lastReportedResetTime;   // so one estimator reset is reported once
	uint32_t _lastReportedBaroRejects;
	uint32_t _lastReportedEkfFaults;
	uint32_t _noEstimateSlowLoops;  // consecutive 1 Hz passes with no estimator

	// Pass counters. Not statistics -- diagnostics. A frozen _loopCount means
	// the loop stopped; a climbing _loopCount with a frozen _midCount means a
	// rate gate stopped firing; both climbing with no output means the group
	// itself is returning early. Those three need opposite responses and are
	// otherwise indistinguishable from a silent link.
	uint32_t _loopCount;
	uint32_t _midCount;
	uint32_t _slowCount;
	uint32_t _publishCount;

	// IMU, estimator propagation, accelerometer fusion. Runs on every call.
	void fastLoop(void);
	// Magnetometer and barometer fusion. DRONE_MID_LOOP_HZ.
	void midLoop(void);
	// Heartbeat and housekeeping. DRONE_SLOW_LOOP_HZ.
	void slowLoop(void);

	void reportBootStatus(void);

	// Says out loud what the estimator cannot do. Nothing here fixes anything --
	// the filter repairs itself now -- but a fault that heals silently is a fault
	// nobody ever finds, and an aircraft with no attitude must not present as an
	// aircraft with nothing to say.
	void reportEstimatorHealth(void);

	void seedEstimator(bool imu_ok, const IMU_Data &imu_data);
	void publishState(void);
};

// Free function so Telemetry can reach the diagnostics without including this
// header -- Drone.hpp pulls in Globals.hpp, which pulls in Telemetry.hpp.
// There is exactly one Drone; init() binds it.
void Drone_ReportDiagnostics(void);

#endif /* DRONE_HPP_ */
