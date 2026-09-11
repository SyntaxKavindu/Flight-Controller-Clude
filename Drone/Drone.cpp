/*
 * Drone.cpp
 *
 *  Created on: Aug 25, 2026
 *      Author: KAVINDU
 */

#include "Drone.hpp"

#include <math.h>   // acosf, fabsf, sqrtf

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float RAD_TO_DEG = 57.29577951308232f;
constexpr float STANDARD_GRAVITY_MSS = 9.80665f;

// ICM42688P datasheet noise densities, converted to the units
// setImuNoiseParameters() wants. The bias random walks are not specified by
// the datasheet; these are conventional starting points, and are the first
// thing to tune if the filter's bias estimates wander.
constexpr float IMU_GYRO_NOISE_DENSITY      = 4.9e-5f;  // (rad/s)/sqrt(Hz), 0.0028 dps/sqrt(Hz)
constexpr float IMU_ACCEL_NOISE_DENSITY     = 6.9e-4f;  // (m/s^2)/sqrt(Hz), 70 ug/sqrt(Hz)
constexpr float IMU_GYRO_BIAS_RANDOM_WALK   = 1.0e-5f;  // (rad/s)/sqrt(s)
constexpr float IMU_ACCEL_BIAS_RANDOM_WALK  = 1.0e-4f;  // (m/s^2)/sqrt(s)

// Measurement noise, as standard deviations in each sensor's OWN units.
//
// The ESEKF's built-in defaults are 0.05 for both, as variances. For the
// accelerometer that is sigma = 0.22 m/s^2 against a 9.81 m/s^2 signal, which
// is reasonable. For the magnetometer it is sigma = 0.22 GAUSS against an
// Earth field of about 0.47 Gauss -- nearly half the signal -- so the default
// leaves the only yaw reference barely weighted at all. Set both explicitly
// here, where the sensor units are actually known.
constexpr float ACCEL_MEAS_SIGMA = 0.35f;  // m/s^2, covers vibration and tilt modelling error
constexpr float MAG_MEAS_SIGMA   = 0.05f;  // Gauss, ~10% of Earth field; matches EK3_MAG_M_NSE

// True when `period_ms` has elapsed since `last`, advancing `last` by whole
// periods so the schedule does not drift. After a long stall it resynchronises
// to now rather than firing a burst of catch-up iterations, which on a flight
// controller would be worse than the missed ticks.
bool due(uint32_t now, uint32_t &last, uint32_t period_ms) {
	const uint32_t elapsed = now - last; // unsigned: wraps correctly
	if (elapsed < period_ms) {
		return false;
	}
	last = (elapsed > 4u * period_ms) ? now : (uint32_t) (last + period_ms);
	return true;
}

// getLastPositionResetTime() returns a large negative sentinel until a reset
// has actually happened, so "no reset yet" has to be seeded with that same
// value rather than with zero. Seeded with 0.0f, the very first mid loop
// compared -1000 against 0, decided they differed, and announced a position
// reset on every boot -- reporting the estimator as discontinuous at exactly
// the moment it was fine. Any value below the sentinel's magnitude works as
// the guard; this matches it.
constexpr float ESEKF_NEVER_RESET = -1000.0f;

// There is exactly one Drone (main.cpp holds it static), so a file-scope
// pointer is all the binding the DIAG shim needs. init() sets it.
Drone *g_drone = nullptr;

} // namespace

Drone::Drone() :
		_esekf { }, _bootStatus { GLOBALS_INIT_OK }, _lastPredictTick { 0 },
		_lastFastTick { 0 }, _lastPollTick { 0 }, _pollSkipped { 0 },
		_lastMidTick { 0 },
		_lastStreamTick { 0 }, _streamMode { DRONE_STREAM_OFF },
		_lastSlowTick { 0 }, _lastPublishTick { 0 },
		_lastReportedResetTime { ESEKF_NEVER_RESET }, _lastReportedBaroRejects { 0 },
		_lastReportedEkfFaults { 0 }, _noEstimateSlowLoops { 0 },
		_calibrationEpoch { 0 },
		_pollMs { 0 }, _imuMs { 0 }, _fastMs { 0 }, _midMs { 0 }, _pubMs { 0 },
		_slowMs { 0 },
		_loopCount { 0 }, _pollCount { 0 }, _fastCount { 0 }, _midCount { 0 },
		_slowCount { 0 },
		_publishCount { 0 } {
}

void Drone::init(void) {
	_bootStatus = Globals_Init();
	reportBootStatus();

	// Must be set before the estimator is seeded: initialize() builds the NED
	// magnetic reference from it, and that is what makes yaw absolute rather
	// than merely repeatable.
	_esekf.setMagneticDeclination(DRONE_MAGNETIC_DECLINATION_DEG * DEG_TO_RAD);

	// Prefer datasheet noise densities over the constructor's generic defaults.
	_esekf.setImuNoiseParameters(IMU_GYRO_NOISE_DENSITY, IMU_ACCEL_NOISE_DENSITY,
			IMU_GYRO_BIAS_RANDOM_WALK, IMU_ACCEL_BIAS_RANDOM_WALK);

	// Measurement noise in the sensors' own units -- see the constants above
	// for why the estimator's generic defaults are wrong for a Gauss-scale
	// magnetometer.
	const float accel_var = ACCEL_MEAS_SIGMA * ACCEL_MEAS_SIGMA;
	const float mag_var = MAG_MEAS_SIGMA * MAG_MEAS_SIGMA;
	const float R_accel[3][3] = { { accel_var, 0.0f, 0.0f },
	                              { 0.0f, accel_var, 0.0f },
	                              { 0.0f, 0.0f, accel_var } };
	const float R_mag[3][3] = { { mag_var, 0.0f, 0.0f },
	                            { 0.0f, mag_var, 0.0f },
	                            { 0.0f, 0.0f, mag_var } };
	_esekf.setAccelNoise(R_accel);
	_esekf.setMagNoise(R_mag);

	// The motion gate is airframe-specific, so it is set from Drone.hpp rather
	// than left at the filter's generic default -- see DRONE_ACCEL_GATE_MPS2.
	_esekf.setAccelGateThreshold(DRONE_ACCEL_GATE_MPS2);

	// Every rate gate starts from the same tick, so nothing is spuriously due on
	// the first pass just because its `last` was left at zero.
	const uint32_t now = HAL_GetTick();
	_lastPredictTick = now;
	_lastFastTick = now;
	_lastPollTick = now;
	_pollSkipped = 0;
	_lastMidTick = now;
	_lastIndicatorTick = now;
	_lastStreamTick = now;
	// Never persists across a boot: a stream left running would spend link
	// bandwidth and loop time on a diagnostic nobody is watching.
	_streamMode = DRONE_STREAM_OFF;
	_lastSlowTick = now;
	_lastPublishTick = now;
	_lastReportedResetTime = ESEKF_NEVER_RESET;
	_lastReportedBaroRejects = barometer.getRejectedSampleCount();
	_lastReportedEkfFaults = _esekf.getFaultCount();
	_noEstimateSlowLoops = 0;

	// The accelerometer and levelling procedures are driven from fastLoop(), so
	// this is the rate their sample-count gates would otherwise be judged
	// against -- five times what they were characterised for. Tell the facade
	// the rate and it decimates; see CALIBRATOR_ACCEL_FEED_HZ. Nothing needs to
	// be said about the compass, which is fed from the 50 Hz mid group.
	calibrator.setAccelFeedRate(DRONE_FAST_LOOP_HZ);

	// Baseline: seeding is about to happen against exactly these gains, so only
	// a LATER change has to force a re-seed.
	_calibrationEpoch = calibrator.getCalibrationEpoch();

	// Bind the DIAG shim -- see Drone_ReportDiagnostics() at the bottom.
	g_drone = this;
}

void Drone::reportBootStatus(void) {
	telemetry.send("$BOOT,%s", (_bootStatus == GLOBALS_INIT_OK) ? "OK" : "DEGRADED");

	if (_bootStatus & GLOBALS_INIT_STORAGE_FAIL) {
		telemetry.send("$ERR,EEPROM INIT FAILED - CALIBRATION WILL NOT PERSIST");
	}
	if (_bootStatus & GLOBALS_INIT_IMU_FAIL) {
		telemetry.send("$ERR,IMU INIT FAILED");
	}
	if (_bootStatus & GLOBALS_INIT_MAG_FAIL) {
		telemetry.send("$ERR,MAGNETOMETER INIT FAILED");
	}
	if (_bootStatus & GLOBALS_INIT_BARO_FAIL) {
		telemetry.send("$ERR,BAROMETER INIT FAILED");
	}

	telemetry.send("$STATUS,ACCL,%s,IDLE", calibrator.isAcclCalibrated() ? "CAL" : "UNCAL");
	telemetry.send("$STATUS,MAG,%s,IDLE", calibrator.isCompassCalibrated() ? "CAL" : "UNCAL");
	// Reported like the other two. An uncalibrated board mounting is not an
	// error -- a squarely mounted board needs no rotation -- but it is the
	// difference between "roll and pitch are true" and "roll and pitch are
	// whatever angle the board happens to be bolted at", and nothing else in
	// the output distinguishes them.
	telemetry.send("$STATUS,LEVEL,%s,IDLE", calibrator.isLevelCalibrated() ? "CAL" : "UNCAL");
}

void Drone::loop(void) {
	_loopCount++;

	// One tick read for the whole pass, so every group is dispatched against a
	// single clock -- two reads either side of a millisecond boundary would let
	// the groups disagree about what time it is within one iteration.
	const uint32_t now = HAL_GetTick();

	// Host commands FIRST, ahead of every group below. A CANCEL that arrived
	// since the last pass has to take effect BEFORE the next sample is fed to a
	// running calibration, or the procedure finishes one sample into a run the
	// operator already stopped. That ordering is the reason this sits at the top
	// rather than anywhere else in the pass.
	//
	// This is also the only place commands are acted on: receive() runs in the
	// USB interrupt and does nothing but buffer bytes, which is what makes
	// Calibrator single-writer and lock-free. See Telemetry.hpp.
	//
	// Rate gated at 1 kHz, with a pass-count escape so a dead clock cannot take
	// the command link with it -- see DRONE_POLL_HZ and DRONE_POLL_MAX_SKIP.
	_pollSkipped++;
	if (due(now, _lastPollTick, DRONE_POLL_PERIOD_MS)
			|| _pollSkipped >= DRONE_POLL_MAX_SKIP) {
		_pollSkipped = 0;
		_pollCount++;
		const uint32_t t0 = HAL_GetTick();
		telemetry.poll();
		_pollMs += HAL_GetTick() - t0;
	}

	// The panel, on its own gate. Directly after poll() so a state a command
	// just changed is rendered on the same pass, and ahead of the flight groups
	// because it is the only output an operator has when telemetry is not
	// connected -- it should not queue behind a slow sensor read.
	if (due(now, _lastIndicatorTick, DRONE_INDICATOR_PERIOD_MS)) {
		updateIndicator();
	}

	// The fast group is gated like the rest. It used to run unconditionally,
	// which on this MCU meant several IMU reads per millisecond of which all but
	// the last were discarded -- the part only produces a new sample at 1 kHz.
	// See the loop-rate banner in Drone.hpp.
	if (due(now, _lastFastTick, DRONE_FAST_LOOP_PERIOD_MS)) {
		_fastCount++;
		const uint32_t t0 = HAL_GetTick();
		fastLoop();
		_fastMs += HAL_GetTick() - t0;
	}
	if (due(now, _lastMidTick, DRONE_MID_LOOP_PERIOD_MS)) {
		_midCount++;
		const uint32_t t0 = HAL_GetTick();
		midLoop();
		_midMs += HAL_GetTick() - t0;
	}
	// After the mid group above, so a magnetometer sample is at most one mid
	// cycle old when it goes out, and gated separately from publishState() so
	// turning the stream on cannot change the rate of the normal telemetry.
	if (_streamMode != DRONE_STREAM_OFF
			&& due(now, _lastStreamTick, DRONE_STREAM_PERIOD_MS)) {
		publishStream();
	}

	if (due(now, _lastPublishTick, DRONE_PUBLISH_PERIOD_MS)) {
		_publishCount++;
		const uint32_t t0 = HAL_GetTick();
		publishState();
		_pubMs += HAL_GetTick() - t0;
	}
	if (due(now, _lastSlowTick, DRONE_SLOW_LOOP_PERIOD_MS)) {
		_slowCount++;
		const uint32_t t0 = HAL_GetTick();
		slowLoop();
		_slowMs += HAL_GetTick() - t0;
	}
}

// ---------------------------------------------------------------------------
// Fast group: IMU, estimator propagation, accelerometer fusion.
// ---------------------------------------------------------------------------
void Drone::fastLoop(void) {
	// Timed separately from the rest of the group: a slow SPI bus and a slow
	// estimator need completely different fixes, and the totals cannot tell
	// them apart.
	const uint32_t t_imu = HAL_GetTick();
	imu.update(); // also feeds a running accelerometer calibration
	_imuMs += HAL_GetTick() - t_imu;

	IMU_Data imu_data;
	const bool imu_ok = (imu.getData(imu_data) == IMU_StatusTypeDef::OK);

	// A calibration is an operator deliberately handling the airframe, and the
	// sensors hand back uncorrected data while one runs. Feeding either to the
	// estimator would corrupt it, so stand down until the run finishes. The
	// sensor read above still happens -- it is what feeds the calibrator.
	//
	// isCalibrating() covers all three procedures. Spelled out as an OR of the
	// two flags, as it was here and at three other sites, the levelling run was
	// simply left out and the estimator kept integrating through it.
	if (calibrator.isCalibrating()) {
		// Keep the clock current so the first step afterwards is one cycle,
		// not the whole length of the calibration.
		_lastPredictTick = HAL_GetTick();
		return;
	}

	// A finished calibration silently changes what every later sample MEANS: a
	// different accelerometer scale, a different magnetic frame, or a board
	// rotation that was not being applied a moment ago. The filter's state was
	// built on the old gains, so the first corrected sample after the change
	// arrives looking like the vehicle moved -- the estimator would fuse a step
	// that never physically happened, and a large one would be gated out, which
	// is worse: it recovers slowly and silently.
	//
	// So re-align instead. This is a ground procedure on a stationary airframe,
	// which is exactly the condition seeding needs anyway.
	const uint32_t epoch = calibrator.getCalibrationEpoch();
	if (epoch != _calibrationEpoch) {
		_calibrationEpoch = epoch;
		_esekf.reset();
		_lastReportedResetTime = ESEKF_NEVER_RESET;
		telemetry.send("$INFO,CALIBRATION CHANGED - RESEEDING ESTIMATOR");
		// Falls through to the seeding path below on this same pass.
	}

	// NOTHING BELOW RESETS THE ESTIMATOR ON A FAULT, and that is deliberate.
	//
	// The re-seed above is a different thing entirely: it is triggered by an
	// operator finishing a calibration on the ground, where the airframe is
	// stationary and re-alignment is guaranteed to work. A fault is not.
	//
	// This used to call _esekf.reset() on a numerical fault and let the seeding
	// path pick it up again. On a bench that is fine. In the air it is a crash:
	// reset() leaves the IDENTITY quaternion, so the estimator claims to be
	// perfectly level, and a controller handed "level" while the aircraft is
	// banked 25 degrees commands a correction that makes the bank worse. And it
	// could not recover either -- seeding needs a still airframe to read gravity
	// from, and an aircraft departing controlled flight is the opposite of still,
	// so the filter would sit uninitialized and SILENT for the rest of the flight.
	//
	// The estimator now repairs itself from its last good state instead (see
	// checkFinite() in ESEKF.cpp) and keeps flying. All this loop does is report,
	// which is reportEstimatorHealth()'s job -- and hasDiverged() is now only
	// reachable from a garbage seed, on the ground, where refusing is correct.

	if (!_esekf.isInitialized()) {
		seedEstimator(imu_ok, imu_data);
		return;
	}

	if (!imu_ok) {
		return;
	}

	const uint32_t now = HAL_GetTick();
	const uint32_t elapsed_ms = now - _lastPredictTick; // unsigned: wraps correctly

	if (elapsed_ms > 0) {
		_lastPredictTick = now;
		_esekf.predict(imu_data.gyro, imu_data.accel, (float) elapsed_ms * 0.001f);

		// Fused after the propagation, and only when one happened: fusing the
		// same sample twice without a predict in between would count the same
		// information more than once and make the filter overconfident.
		_esekf.updateAccelerometer(imu_data.accel);
	}
	// elapsed_ms == 0 means HAL_GetTick()'s 1 ms resolution has not ticked over
	// yet. _lastPredictTick is deliberately left alone so the next cycle
	// integrates the whole interval instead of discarding it.
}

// ---------------------------------------------------------------------------
// Mid group: magnetometer and barometer fusion.
//
// These sensors run at 80 Hz and 50 Hz. Reading them from the fast loop only
// re-read registers that had not changed, and re-fusing an unchanged sample
// counts the same information twice -- the filter grows confident on evidence
// it already had.
// ---------------------------------------------------------------------------
void Drone::midLoop(void) {
	magnetometer.update(); // also feeds a running compass calibration
	barometer.update();

	// Same stand-down as the fast group, and for the same reason -- but only
	// after the reads above, which are what feed a compass calibration.
	if (calibrator.isCalibrating()) {
		return;
	}
	if (!_esekf.isInitialized() || _esekf.hasDiverged()) {
		return;
	}

	LIS3MDL_Data mag_data;
	if (magnetometer.getData(mag_data) == MAG_StatusTypeDef::OK) {
		_esekf.updateMagnetometer(Vector3f(mag_data.x, mag_data.y, mag_data.z));
	}

	BMP390_Data baro_data;
	if (barometer.getData(baro_data) == BARO_StatusTypeDef::OK) {
		_esekf.updateBarometer((float) baro_data.altitude);
	}

	// A height-aiding timeout makes the estimator SNAP vertical position to
	// the barometer rather than fuse its way back, so the estimate is
	// discontinuous. That is deliberate -- it is how the filter recovers from
	// a genuinely wrong state -- but it must never be silent: a controller
	// integrating altitude sees the jump as an enormous instantaneous error.
	const float reset_t = _esekf.getLastPositionResetTime();
	if (reset_t != _lastReportedResetTime) {
		_lastReportedResetTime = reset_t;
		telemetry.send("$ERR,POSITION RESET %.2f m - ALTITUDE IS DISCONTINUOUS",
				(double) _esekf.getLastPositionResetDelta().z);
	}

}

// ---------------------------------------------------------------------------
// Slow group: the heartbeat, then housekeeping. Reserved for battery
// monitoring, GPS status, logging and failsafe checks; currently reports
// sensor health.
// ---------------------------------------------------------------------------
void Drone::slowLoop(void) {
	// Unconditional 1 Hz heartbeat, before anything that can return early.
	//
	// Silence from a flight controller is ambiguous: it can mean "nothing to
	// report" or "the loop stopped", and those need opposite responses. One
	// line a second removes the ambiguity for about 45 bytes/s, and it is
	// gated by nothing -- not the estimator, not calibration -- so the only
	// thing that can stop it is the loop itself stopping.
	//
	// pass is the number that matters. If it climbs, the loop is running and
	// any missing output is a group bailing out. If it is frozen, or the line
	// stops arriving, the loop really has stopped.
	// rx and cmd are here rather than only in $DIAG deliberately: if the command
	// link is broken, DIAG is exactly the thing that cannot be asked for. The
	// heartbeat is the one line that arrives with no command at all, so the
	// evidence about why commands are not working has to travel on it. See
	// Telemetry::getRxByteCount() for how to read them.
	telemetry.send("$HB,pass=%lu tick=%lu init=%u div=%u rx=%lu cmd=%lu",
			(unsigned long) _loopCount,
			(unsigned long) HAL_GetTick(),
			(unsigned) (_esekf.isInitialized() ? 1 : 0),
			(unsigned) (_esekf.hasDiverged() ? 1 : 0),
			(unsigned long) telemetry.getRxByteCount(),
			(unsigned long) telemetry.getLineCount());

	reportEstimatorHealth();

	// A barometer that starts rejecting samples is failing, and the estimator
	// would simply coast on its last good height without saying anything.
	// Report only on change, so a healthy sensor stays silent.
	const uint32_t rejects = barometer.getRejectedSampleCount();
	if (rejects != _lastReportedBaroRejects) {
		_lastReportedBaroRejects = rejects;
		telemetry.send("$ERR,BAROMETER REJECTED %lu IMPLAUSIBLE SAMPLES",
				(unsigned long) rejects);
	}
}

void Drone::reportEstimatorHealth(void) {
	// A repaired numerical fault must never be silent. The filter heals itself
	// now rather than being torn down, and a filter that heals silently is one
	// whose bugs are never found -- this counter is the only evidence the repair
	// happened at all. It should read zero for the life of the aircraft.
	const uint32_t faults = _esekf.getFaultCount();
	if (faults != _lastReportedEkfFaults) {
		_lastReportedEkfFaults = faults;
		telemetry.send("$ERR,EKF NUMERICAL FAULT x%lu - RECOVERED FROM LAST GOOD STATE",
				(unsigned long) faults);
	}

	// hasDiverged() now means "went non-finite with nothing to fall back on",
	// which only a garbage seed can do -- on the ground, before anything flies.
	// Unrecoverable, so say it every second for as long as it lasts.
	if (_esekf.hasDiverged()) {
		telemetry.send("$ERR,EKF DIVERGED - NOT FLYABLE");
		return;
	}

	// No attitude at all. publishState() bails out silently in this state, and
	// silence must not be the only symptom: an operator needs to be told to put
	// the airframe down and hold it still, not left guessing why the link went
	// quiet. Seeding needs stillness, and nothing else in the output says so.
	if (!_esekf.isInitialized()) {
		_noEstimateSlowLoops++;
		if (_noEstimateSlowLoops >= DRONE_NO_ESTIMATE_WARN_S) {
			telemetry.send("$ERR,NO ATTITUDE ESTIMATE %lus - HOLD THE AIRFRAME STILL",
					(unsigned long) _noEstimateSlowLoops);
		}
		return;
	}
	_noEstimateSlowLoops = 0;
}

// Health, as the panel sees it. Deliberately the SAME definition
// reportEstimatorHealth() prints over telemetry -- two definitions of "is this
// aircraft well" that can disagree is worse than either one alone, because the
// operator then has to work out which to believe.
//
// ERROR means: a device did not come up, OR the filter is unrecoverable, OR it
// has repaired a numerical fault, OR it has had no attitude for longer than the
// telemetry warning allows.
//
// That last one is why this is not simply !isInitialized(). Seeding takes a
// moment on every single boot, and a panel that screams ERROR for the first few
// seconds of every power-up is a panel the operator learns to ignore -- which
// costs nothing right up until the boot where the seed never lands. The grace
// period is DRONE_NO_ESTIMATE_WARN_S, shared with the telemetry warning so the
// LED and the "$ERR,NO ATTITUDE ESTIMATE" line change at the same moment.
//
// ARM and GPS are NOT set here. Drone owns neither Motors nor GPS, so it has
// nothing truthful to say about them; the constructor leaves them at DISARMED
// and UNLOCKED, which is the safe reading of "nobody has reported". When those
// subsystems are wired in, one call each is all this needs:
//
//     indicator.setArmState(motors.isArmed() ? ArmState::ARMED
//                                            : ArmState::DISARMED);
//     indicator.setGPSState(gps.isFix() ? GPSState::LOCKED
//                                       : GPSState::UNLOCKED);
void Drone::updateIndicator(void) {
	const bool device_failed = (_bootStatus != GLOBALS_INIT_OK);
	const bool unrecoverable = _esekf.hasDiverged();
	const bool repaired_fault = (_esekf.getFaultCount() != 0u);
	const bool no_attitude = !_esekf.isInitialized()
			&& (_noEstimateSlowLoops >= DRONE_NO_ESTIMATE_WARN_S);

	indicator.setSystemState(
			(device_failed || unrecoverable || repaired_fault || no_attitude)
					? SystemState::ERROR : SystemState::OK);

	// Rendering is separate from deciding, and unconditional: the pattern has
	// to keep advancing even on the passes where nothing about the state moved.
	indicator.update();
}

void Drone::setStreamMode(DroneStreamMode mode) {
	_streamMode = (uint8_t) mode;
	// Re-baselined so the first sample goes out on the next due tick rather than
	// immediately, which keeps the spacing in the plot honest from the start.
	_lastStreamTick = HAL_GetTick();
}

// One line per sample, raw and corrected TOGETHER.
//
// Together is the point: emitted as two lines they could be one cycle apart,
// and a moving airframe -- which is exactly how this data is collected -- would
// then show a corrected point that does not correspond to the raw one beside
// it. The plot would blur in a way that looks like calibration error.
//
// Raw here means axis-remapped but uncorrected, which is the input the
// calibration actually fits. Corrected means what the flight stack uses, board
// rotation included -- a rotation does not change a sphere, so it cannot
// flatter the result.
void Drone::publishStream(void) {
	if (_streamMode == DRONE_STREAM_ACCEL) {
		IMU_Data d;
		if (imu.getData(d) != IMU_StatusTypeDef::OK) {
			return;   // no sample yet; say nothing rather than emit zeros
		}
		const Vector3f raw = imu.getRawAccel();
		telemetry.send("$STREAM,A,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
				(double) raw.x, (double) raw.y, (double) raw.z,
				(double) d.accel.x, (double) d.accel.y, (double) d.accel.z);
		return;
	}

	if (_streamMode == DRONE_STREAM_MAG) {
		LIS3MDL_Data d;
		if (magnetometer.getData(d) != MAG_StatusTypeDef::OK) {
			return;
		}
		const Vector3f raw = magnetometer.getRawField();
		telemetry.send("$STREAM,M,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
				(double) raw.x, (double) raw.y, (double) raw.z,
				(double) d.x, (double) d.y, (double) d.z);
	}
}

void Drone::seedEstimator(bool imu_ok, const IMU_Data &imu_data) {
	// initialize() levels the airframe from the accelerometer and takes its
	// heading from the magnetometer, so both are required -- seeding attitude
	// without a heading reference would leave yaw wherever it happened to land.
	//
	// The magnetometer and barometer are read by the mid loop; getData() hands
	// back the last good sample, which is what seeding wants anyway.
	LIS3MDL_Data mag_data;
	BMP390_Data baro_data;
	const bool mag_ok = (magnetometer.getData(mag_data) == MAG_StatusTypeDef::OK);
	const bool baro_ok = (barometer.getData(baro_data) == BARO_StatusTypeDef::OK);

	if (!imu_ok || !mag_ok) {
		return;
	}

	const Vector3f field(mag_data.x, mag_data.y, mag_data.z);
	if (field.length() < 1e-6f) {
		return; // no usable heading reference yet
	}

	// Only seed while stationary. Under acceleration the accelerometer is not
	// a gravity reference, and the resulting tilt error is baked in permanently
	// -- the filter has no way to know its starting attitude was wrong.
	const float specific_force = imu_data.accel.length();
	if (fabsf(specific_force - STANDARD_GRAVITY_MSS) > DRONE_SEED_STILLNESS_MPS2) {
		return;
	}

	// No GPS is wired up yet, so the local NED origin is the zero reference and
	// position is relative to wherever the vehicle was switched on. Altitude
	// still works: initialize() takes the barometer reading as the datum and
	// fuses it as a relative climb from there.
	const float altitude = baro_ok ? (float) baro_data.altitude : 0.0f;
	_esekf.initialize(imu_data.accel, field, altitude, Vector3f());

	// initialize() restarts the filter clock and clears its reset history, so
	// the previous alignment's reset timestamp is meaningless now -- keeping it
	// would make the next comparison in midLoop() fire against a stale value.
	_lastReportedResetTime = ESEKF_NEVER_RESET;

	// Whatever gains are in force now are the ones this alignment was built on.
	_calibrationEpoch = calibrator.getCalibrationEpoch();

	_lastPredictTick = HAL_GetTick();
	telemetry.send("$INFO,EKF INITIALIZED");
}

void Drone::publishState(void) {
	// Dispatched straight from loop() on its own rate gate, not from inside
	// midLoop()'s guard, so it carries its own stand-down: a group that
	// depends on another group's early-return publishes garbage the moment
	// the dispatch order changes. During a calibration the sensors hand back
	// uncorrected data, and before the estimator is seeded there is no
	// attitude at all; either way the numbers would be fiction.
	if (calibrator.isCalibrating()) {
		return;
	}
	if (!_esekf.isInitialized() || _esekf.hasDiverged()) {
		return;
	}

	const Vector3f euler = _esekf.getEulerAngles();
	const ESEKFStatus status = _esekf.getStatus();

	// Angles in degrees. Altitude is metres positive up, relative to wherever
	// the estimator was seeded -- it is BAROMETRIC: the barometer is the only
	// height source, fused with the accelerometer, and getAltitude() adds
	// gps_ref_.z which is zero because no GPS is wired up.
	telemetry.send("IMU: %.2f Roll, %.2f Pitch, %.2f Yaw, %.2f Altitude",
			(double) (euler.x * RAD_TO_DEG),
			(double) (euler.y * RAD_TO_DEG),
			(double) (euler.z * RAD_TO_DEG),
			(double) _esekf.getAltitude());

	// The Euler triple above is for a human reading a terminal. It is NOT a
	// safe basis for anything automatic: ZYX (3-2-1) Euler has a singularity at
	// pitch = +/-90 deg, where roll and yaw stop being separable and trade off
	// against each other. That is why a 90 deg pitch on the bench also swings
	// the reported roll by tens of degrees while the ACTUAL attitude is fine --
	// the estimator never forms an Euler triple internally, its state is the
	// quaternion below.
	//
	// So publish the quaternion too. It is singularity-free, it is what an AHRS
	// display or a ground station should draw from (this is exactly why MAVLINK
	// carries ATTITUDE_QUATERNION alongside ATTITUDE), and it is what a future
	// attitude controller will take its error from.
	const Quaternionf q = _esekf.getOrientation();
	telemetry.send("QUAT: %.4f W, %.4f X, %.4f Y, %.4f Z",
			(double) q.w, (double) q.x, (double) q.y, (double) q.z);

	// Tilt: the angle between the airframe's Down axis and true Down, i.e. how
	// far off level it is regardless of heading. R_b^n[2][2] is the Down-Down
	// term of the rotation matrix, which in quaternion terms is 1 - 2(x^2+y^2);
	// clamp before acos so rounding at exactly level or exactly inverted cannot
	// hand it an out-of-domain argument. 0 deg is upright, 90 deg is knife-edge
	// or nose-vertical, 180 deg is inverted -- one number, no singularity, and
	// it is the quantity a tilt limit or a crash detector actually wants.
	float cos_tilt = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	if (cos_tilt > 1.0f) {
		cos_tilt = 1.0f;
	} else if (cos_tilt < -1.0f) {
		cos_tilt = -1.0f;
	}
	telemetry.send("TILT: %.2f Tilt", (double) (acosf(cos_tilt) * RAD_TO_DEG));

	// NED position and velocity are deliberately not published: with no GPS
	// wired up they are unaided dead reckoning from the power-on point, and a
	// number that looks like a position but drifts without bound is worse than
	// no number at all.
	//
	// The health line stays, because the four numbers above say nothing about
	// whether to believe them: attitude valid, vertical position aided, yaw
	// bounded by the magnetometer, and dead reckoning.
	telemetry.send("EKF: %s Attitude, %s Height, %s Yaw, %s DeadReckon",
			status.attitude_valid ? "OK" : "BAD",
			status.vert_pos_valid ? "OK" : "BAD",
			status.mag_aiding ? "OK" : "BAD",
			status.dead_reckoning ? "YES" : "NO");
}

// ---------------------------------------------------------------------------
// On-demand diagnostics (DIAG command)
//
// telemetry.poll() runs at the top of every pass, ahead of every group, and its
// pass-count escape means it keeps running even if the clock stops -- so it is
// the one thing still reachable when every periodic group has gone quiet, which
// is exactly when you need to ask why. See DRONE_POLL_MAX_SKIP.
// ---------------------------------------------------------------------------
void Drone::reportDiagnostics(void) {
	// pass climbing means the loop is alive; tick is the clock it dispatches
	// on. A frozen tick with a climbing pass would be a dead SysTick, which
	// would stop every rate gate below.
	telemetry.send("$DIAG,LOOP pass=%lu tick=%lu",
			(unsigned long) _loopCount,
			(unsigned long) HAL_GetTick());

	// Each of these should climb at its declared rate. One stuck at 0 or 1
	// means its gate stopped firing; one climbing with no output on the link
	// means the group itself is returning early -- see $DIAG,EKF below.
	telemetry.send("$DIAG,RATE poll=%lu fast=%lu mid=%lu pub=%lu slow=%lu",
			(unsigned long) _pollCount,
			(unsigned long) _fastCount,
			(unsigned long) _midCount,
			(unsigned long) _publishCount,
			(unsigned long) _slowCount);

	// The CPU clock the HAL believes it configured. Report it because a loop
	// that is inexplicably slow is very often a clock tree that never left the
	// internal oscillator: an F7 running on 16 MHz HSI instead of a 216 MHz PLL
	// is 13x down on every figure above and nothing else in the output says so.
	// Also worth knowing that on an F7 the instruction cache and the ART
	// accelerator are OFF unless main() enables them, which alone is worth
	// several times the execution speed out of flash.
	telemetry.send("$DIAG,CLOCK hz=%lu tickhz=%lu",
			(unsigned long) SystemCoreClock,
			(unsigned long) (1000u / (HAL_GetTickFreq() ? HAL_GetTickFreq() : 1u)));

	// How this binary was BUILT, which is the difference between the estimator
	// costing 150 us and costing 2.5 ms. None of it is visible from any other
	// output, and all of it is a project setting rather than a code defect:
	//
	//   fpu=SOFT   every float operation is a library call. On a part with an
	//              FPU this is 10-50x. Fix: -mfloat-abi=hard -mfpu=fpv5-sp-d16.
	//   opt=OFF    built at -O0, which is CubeIDE's Debug default. Float-heavy
	//              code spills every intermediate to the stack; 5-15x. Fix:
	//              build the Release configuration, or set -O2 on Debug.
	//   ic=0/dc=0  instruction and data cache disabled. An F7 runs from flash
	//              with 7 wait states, so without the I-cache and the ART
	//              accelerator it stalls constantly; 3-6x. Fix: call
	//              SCB_EnableICache() and SCB_EnableDCache() early in main().
	//
	// Note that D-cache ON has a consequence of its own: DMA buffers then need
	// cache maintenance or MPU-marked non-cacheable memory, so enable it
	// deliberately rather than reflexively.
#if defined(__SOFTFP__)
	const char *build_fpu = "SOFT";
#elif defined(__ARM_FP)
	const char *build_fpu = "HARD";
#else
	const char *build_fpu = "?";
#endif
#if defined(__OPTIMIZE_SIZE__)
	const char *build_opt = "Os";
#elif defined(__OPTIMIZE__)
	const char *build_opt = "ON";
#else
	const char *build_opt = "OFF";
#endif
#if defined(SCB_CCR_IC_Msk) && defined(SCB_CCR_DC_Msk)
	const int build_ic = (SCB->CCR & SCB_CCR_IC_Msk) ? 1 : 0;
	const int build_dc = (SCB->CCR & SCB_CCR_DC_Msk) ? 1 : 0;
#else
	const int build_ic = -1; // host build: no Cortex-M control block
	const int build_dc = -1;
#endif
	telemetry.send("$DIAG,BUILD fpu=%s opt=%s ic=%d dc=%d",
			build_fpu, build_opt, build_ic, build_dc);

	// Accelerometer/levelling feed decimation. div=1 with a fast loop well above
	// CALIBRATOR_ACCEL_FEED_HZ means setAccelFeedRate() was never called, and
	// every sample-count gate in AccelerometerCalibrator and LevelCalibrator is
	// then running at a fraction of its documented time -- which shows up as a
	// six-position run that reports STALLED while the operator is still
	// settling the airframe after READY.
	// What the panel is being TOLD, which is the half worth reporting: it says
	// whether the states reaching the indicator are the ones expected, so a
	// wrong-looking LED can be blamed on the wiring or on the state feeding it.
	// The instantaneous lit/dark of each LED is deliberately not here -- it is
	// a snapshot of one 100 ms slot, so it says nothing on its own.
	telemetry.send("$DIAG,LED sys=%s arm=%s gps=%s",
			indicator.getSystemState() == SystemState::OK ? "OK" : "ERR",
			indicator.getArmState() == ArmState::ARMED ? "ARMED" : "SAFE",
			indicator.getGPSState() == GPSState::LOCKED ? "FIX" : "SEARCH");

	// A stream left running is a real cost -- see DRONE_STREAM_HZ -- and it is
	// otherwise invisible once the plotting tool is closed.
	telemetry.send("$DIAG,STREAM mode=%u hz=%u",
			(unsigned) _streamMode, (unsigned) DRONE_STREAM_HZ);

	telemetry.send("$DIAG,CALFEED div=%u hz=%u",
			(unsigned) calibrator.getAccelFeedDivider(),
			(unsigned) (DRONE_FAST_LOOP_HZ / (calibrator.getAccelFeedDivider() ?
					calibrator.getAccelFeedDivider() : 1u)));

	// Where the loop's time actually goes, as a share of uptime. Summed from
	// HAL_GetTick() deltas: one sample quantises to 0 or 1 ms, but over
	// thousands of calls that averages out, which is what makes a 1 ms tick
	// enough to profile this without a cycle counter.
	//
	// imu is inside fast, so `fast - imu` is the estimator's own cost. The group
	// with the largest share is the one to fix; if they sum to far less than
	// 100%, the time is going somewhere not measured here -- an interrupt
	// handler, or a HAL call blocking outside these brackets.
	const uint32_t up = HAL_GetTick() ? HAL_GetTick() : 1u;
	telemetry.send("$DIAG,TIME poll=%lu%% imu=%lu%% fast=%lu%% mid=%lu%% pub=%lu%% slow=%lu%%",
			(unsigned long) (100u * _pollMs / up),
			(unsigned long) (100u * _imuMs / up),
			(unsigned long) (100u * _fastMs / up),
			(unsigned long) (100u * _midMs / up),
			(unsigned long) (100u * _pubMs / up),
			(unsigned long) (100u * _slowMs / up));

	// The same totals in raw milliseconds, so a group that rounds to 0% is still
	// visible and the numbers can be checked against uptime by hand.
	telemetry.send("$DIAG,TIMEMS up=%lu poll=%lu imu=%lu fast=%lu mid=%lu pub=%lu slow=%lu",
			(unsigned long) up,
			(unsigned long) _pollMs, (unsigned long) _imuMs,
			(unsigned long) _fastMs, (unsigned long) _midMs,
			(unsigned long) _pubMs, (unsigned long) _slowMs);

	// The accelerometer's motion gate, which is the one estimator parameter
	// that has to be tuned against the airframe rather than reasoned about.
	// Hover the vehicle and read `motion`: that is the vibration floor, and
	// `gate` should sit just above it. Too low gates out all attitude aiding;
	// too high lets a manoeuvre drag the attitude over. `infl` is the extra
	// measurement sigma the last accepted sample was charged.
	telemetry.send("$DIAG,ACCEL motion=%.3f gate=%.3f infl=%.2f",
			(double) _esekf.getAccelMotion(),
			(double) _esekf.getAccelGateThreshold(),
			(double) sqrtf(_esekf.getLastAccelNoiseInflation()));

	// publishState() bails out silently on any of these, which looks exactly
	// like a task that never ran.
	const ESEKFStatus st = _esekf.getStatus();
	telemetry.send("$DIAG,EKF init=%u div=%u att=%u vert=%u mag=%u cal=%u",
			(unsigned) (_esekf.isInitialized() ? 1 : 0),
			(unsigned) (_esekf.hasDiverged() ? 1 : 0),
			(unsigned) (st.attitude_valid ? 1 : 0),
			(unsigned) (st.vert_pos_valid ? 1 : 0),
			(unsigned) (st.mag_aiding ? 1 : 0),
			(unsigned) (calibrator.isCalibrating() ? 1 : 0));
}

void Drone_SetStreamMode(int mode) {
	if (g_drone == nullptr) {
		telemetry.send("$ERR,NO DRONE BOUND");
		return;
	}
	g_drone->setStreamMode((DroneStreamMode) mode);
}

void Drone_ReportDiagnostics(void) {
	if (g_drone == nullptr) {
		telemetry.send("$DIAG,NO DRONE BOUND");
		return;
	}
	g_drone->reportDiagnostics();
}
