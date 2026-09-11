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

// What the raw/corrected sample stream is currently emitting. Values are part
// of the wire protocol -- tools/stream_plot.py and the STREAM command both
// name them -- so do not renumber them.
enum DroneStreamMode {
	DRONE_STREAM_OFF   = 0,
	DRONE_STREAM_ACCEL = 1,
	DRONE_STREAM_MAG   = 2
};

// ---------------------------------------------------------------------------
// Loop rates.
//
// EVERY group is dispatched from loop() off HAL_GetTick(), the 1 ms SysTick.
// One clock, one place, no cycle counter: DWT is not enabled here, and nothing
// in this scheduler needs sub-millisecond resolution.
//
// The fast group reads the IMU, propagates the estimator and fuses the
// accelerometer. It is RATE GATED at 1 kHz rather than free-running, and that
// is the whole reason 1 ms resolution is enough: the ICM-42688-P is configured
// for a 1 kHz output data rate (see ICM42688P::init()), so one pass per
// millisecond reads each sample exactly once.
//
// It used to run on every call of loop() instead. On an F7 that is several
// thousand passes a second, so most of them spent a full SPI burst re-reading
// a register set that had not changed since the last one, then threw the result
// away -- predict() only integrates when the millisecond ticks over, and
// everything in between was discarded. Gating here is what makes the sample the
// estimator integrates the sample that was actually just read, and it hands the
// spare SPI bandwidth and CPU back.
//
// The mid group runs the magnetometer and barometer, configured at 80 Hz and
// 50 Hz. Polling them faster only re-reads unchanged registers -- and re-fusing
// a sample the filter has already seen counts the same information twice and
// makes it overconfident, exactly the reason the accelerometer is only fused
// when a predict actually happened. 50 Hz matches the slower of the two.
//
// The slow group is for housekeeping that belongs in neither: battery
// monitoring, GPS status, logging, failsafe checks. It currently emits the
// heartbeat and reports sensor health, and is otherwise reserved.
//
// Every period below must be a whole number of milliseconds, which is what
// bounds the fast group at 1 kHz. A faster inner loop needs a hardware timer
// as its time base, not a shorter tick -- but nothing on this airframe samples
// faster than 1 kHz, so there is nothing to gain from one yet.
// ---------------------------------------------------------------------------
#define DRONE_FAST_LOOP_HZ             1000
#define DRONE_MID_LOOP_HZ                50
#define DRONE_SLOW_LOOP_HZ                1

// Command polling. Gated like everything else, and for a measured reason:
// telemetry.poll() used to run on EVERY pass, and once -O2 took the loop up to
// ~178,000 passes per second that was 1.8 us x 178k = 31% OF THE CPU spent
// discovering an empty ring buffer. Commands arrive at human speed.
//
// 1 kHz is chosen against the ring, not against the operator. USB full-speed
// CDC delivers at most one 64-byte packet per 1 ms frame, and poll() drains the
// whole ring each time, so at 1 kHz the 256-byte ring can never hold more than
// a quarter of its capacity -- four frames of margin before a byte could be
// lost. It also keeps a CANCEL acting within a millisecond, which is the
// property the ordering below exists to protect.
#define DRONE_POLL_HZ                  1000

// ... but the time gate alone is not enough. A dead SysTick freezes
// HAL_GetTick(), so due() would never fire again and the command link would die
// WITH the clock -- taking DIAG, the one tool for diagnosing exactly that
// failure, down with it. So poll() also runs unconditionally every this many
// passes, whatever the clock is doing. At the loop rates seen here that is a
// poll every ~20 ms even with time stopped, and it costs about 0.02% of the CPU
// the ungated version was spending.
#define DRONE_POLL_MAX_SKIP            4096u

// Status LED refresh. INDICATOR_SLOT_MS is 100 ms, so this is 5x oversampled:
// enough that every pattern edge lands within 20 ms of where it belongs, which
// is well under what an eye resolves. Higher buys nothing -- the work is three
// compares and, on the rare pass that changes anything, one GPIO write.
//
// Deliberately its own gate in update() rather than a line in midLoop(). The
// mid group stands down while a calibration runs and again whenever the
// estimator is not ready, and those are precisely the two states an operator
// most needs the panel to keep reporting.
#define DRONE_INDICATOR_HZ               50

// Raw/corrected sample stream, for the calibration check plotted by
// tools/stream_plot.py. Ground-only: it is a diagnostic, not a flight feature.
//
// 25 Hz is chosen against the LINK, not against the plot. Each line is about
// 60 bytes, so this is ~1.5 kB/s against the ~64 kB/s a USB full-speed CDC
// endpoint can carry -- but Telemetry::send() blocks up to
// TELEMETRY_TX_TIMEOUT_MS waiting for the endpoint, and that cost lands in the
// flight loop. At 25 Hz the worst case is 5% of the CPU and the typical case is
// nothing. The magnetometer is only read at DRONE_MID_LOOP_HZ (50) anyway, so
// past 50 this would emit the same sample twice and make the plot look denser
// than the evidence actually is.
#define DRONE_STREAM_HZ                  25

#define DRONE_FAST_LOOP_PERIOD_MS        (1000u / DRONE_FAST_LOOP_HZ)
#define DRONE_MID_LOOP_PERIOD_MS         (1000u / DRONE_MID_LOOP_HZ)
#define DRONE_SLOW_LOOP_PERIOD_MS        (1000u / DRONE_SLOW_LOOP_HZ)
#define DRONE_POLL_PERIOD_MS             (1000u / DRONE_POLL_HZ)
#define DRONE_INDICATOR_PERIOD_MS        (1000u / DRONE_INDICATOR_HZ)
#define DRONE_STREAM_PERIOD_MS           (1000u / DRONE_STREAM_HZ)

static_assert(DRONE_FAST_LOOP_PERIOD_MS >= 1u,
		"HAL_GetTick() is 1 ms: the fast group cannot be dispatched faster than 1 kHz");

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

// Accelerometer motion gate, m/s^2: how far |accel| may sit from 1 g before the
// sample stops being treated as a gravity reference at all. This is the one
// estimator parameter that has to be measured on the airframe rather than
// reasoned about, so it is set here rather than left at the filter's generic
// default -- hover the vehicle, read `motion` from $DIAG,ACCEL, and put this
// just above the figure it settles at. Too low gates out all attitude aiding;
// too high lets a manoeuvre drag the attitude over. Samples inside the gate are
// de-weighted smoothly with how close to it they sit, so this is a limit rather
// than a cliff -- see ESEKF::getLastAccelNoiseInflation().
//
// Left at the value the estimator's own constructor used, so setting it here
// changes nothing today; the point is that it is now visible and tunable in the
// same file as the loop rates rather than buried in a filter default. Widening
// it is safer than it used to be -- a sample near the limit is de-weighted, not
// taken at face value -- but widen it against a measurement, not on principle.
#define DRONE_ACCEL_GATE_MPS2            1.0f

class Drone {
public:
	Drone();

	// Brings up the globals and configures the estimator. Call once.
	void init(void);

	// Call it in a tight while(1). Every group -- command polling, fast, mid,
	// publish, slow -- is dispatched off HAL_GetTick() when its period comes
	// due, from a single tick read per pass so they all share one clock.
	// Passes between due dates do nothing but check those gates.
	void loop(void);

	// Start or stop the raw/corrected sample stream. Ground diagnostic; see
	// DRONE_STREAM_HZ and tools/stream_plot.py.
	void setStreamMode(DroneStreamMode mode);
	DroneStreamMode getStreamMode(void) const {
		return (DroneStreamMode) _streamMode;
	}

	// Dumps the loop counters and the estimator's health on demand. Answers
	// the one question a silent vehicle cannot: is the loop running and every
	// group bailing out, or has the loop itself stopped?
	void reportDiagnostics(void);

private:
	ESEKF _esekf;

	uint8_t _bootStatus;        // GlobalsInitStatus bitmask from init()
	uint32_t _lastPredictTick;  // HAL_GetTick() at the last integrated step
	uint32_t _lastFastTick;
	uint32_t _lastPollTick;
	uint32_t _pollSkipped;      // passes since poll() last ran; see DRONE_POLL_MAX_SKIP
	uint32_t _lastMidTick;
	uint32_t _lastIndicatorTick;
	uint32_t _lastStreamTick;
	uint8_t _streamMode;        // a DroneStreamMode
	uint32_t _lastSlowTick;
	uint32_t _lastPublishTick;
	float _lastReportedResetTime;   // so one estimator reset is reported once
	uint32_t _lastReportedBaroRejects;
	uint32_t _lastReportedEkfFaults;
	uint32_t _noEstimateSlowLoops;  // consecutive 1 Hz passes with no estimator

	// Calibrator::getCalibrationEpoch() as of the last time the estimator was
	// seeded. A change means the applied correction moved under the filter --
	// see the re-seed in fastLoop().
	uint32_t _calibrationEpoch;

	// ---- Section profiler ----
	// Milliseconds accumulated inside each group since boot, summed from
	// HAL_GetTick() deltas. NOT a microsecond timer, and it does not need to be:
	// one sample quantises to 0 or 1 ms, but summed over thousands of calls the
	// quantisation averages out and the total is accurate to a tick. That is
	// what makes a 1 kHz SysTick enough to profile a loop running at hundreds of
	// hertz, with no DWT cycle counter and no spare hardware timer.
	//
	// Read them as a SHARE OF UPTIME: a group taking 60% of the clock is the one
	// to fix, whatever its mean looks like.
	uint32_t _pollMs;   // telemetry.poll()
	uint32_t _imuMs;    // imu.update() alone, inside the fast group
	uint32_t _fastMs;   // the whole fast group, imu.update() included
	uint32_t _midMs;
	uint32_t _pubMs;
	uint32_t _slowMs;

	// Pass counters. Not statistics -- diagnostics. A frozen _loopCount means
	// the loop stopped; a climbing _loopCount with a frozen _midCount means a
	// rate gate stopped firing; both climbing with no output means the group
	// itself is returning early. Those three need opposite responses and are
	// otherwise indistinguishable from a silent link.
	uint32_t _loopCount;
	uint32_t _pollCount;
	uint32_t _fastCount;
	uint32_t _midCount;
	uint32_t _slowCount;
	uint32_t _publishCount;

	// IMU, estimator propagation, accelerometer fusion. DRONE_FAST_LOOP_HZ.
	void fastLoop(void);
	// Magnetometer and barometer fusion. DRONE_MID_LOOP_HZ.
	void midLoop(void);

	// Map what this class knows about the aircraft onto the status panel, then
	// render it. Health only -- arming and GPS are pushed by whoever owns those
	// subsystems, which is nothing in Drone yet. See the note on the body.
	void updateIndicator(void);

	// Emit one raw/corrected pair. See DRONE_STREAM_HZ.
	void publishStream(void);
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

// Telemetry's STREAM command reaches the stream through this, for the same
// reason DIAG does: Telemetry.hpp cannot include Drone.hpp without a cycle.
// Takes a DroneStreamMode; typed as int so the shim stays declarable from a
// unit that has not seen the enum.
void Drone_SetStreamMode(int mode);

#endif /* DRONE_HPP_ */
