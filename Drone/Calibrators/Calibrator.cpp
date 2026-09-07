/*
 * Calibrator.cpp
 *
 *  Created on: Aug 30, 2026
 *      Author: KAVINDU
 */

#include "Calibrator.hpp"
#include "EEPROM.hpp"
#include "Telemetry.hpp"

#include <cstddef> // offsetof
#include <cstring> // memset, memcmp
#include <cmath>   // std::isfinite

// A slot too small for the record it holds cannot be written, and the only
// symptom is a NOTSAVED on every single calibration -- the gains work all
// session and silently vanish on reboot. That is what BOARDLEVELCALIBRATEDAT
// did for its whole existence, wedged into sixteen bytes for a fifty-six byte
// record. These turn that into a build error.
//
// Here rather than in EEPROM.hpp because only this file knows both halves:
// EEPROM.hpp owns the layout and has never heard of CalibrationRecord, and
// Calibrator.hpp deliberately forward-declares EEPROM to keep the HAL out.
static_assert(EEPROM::slotCapacity(EEPROMLocation::ACCLCALIBRATEDAT)
		>= sizeof(CalibrationRecord),
		"accelerometer EEPROM slot is too small for a CalibrationRecord");
static_assert(EEPROM::slotCapacity(EEPROMLocation::COMPASSCALIBRATEDAT)
		>= sizeof(CalibrationRecord),
		"compass EEPROM slot is too small for a CalibrationRecord");
static_assert(EEPROM::slotCapacity(EEPROMLocation::BOARDLEVELCALIBRATEDAT)
		>= sizeof(CalibrationRecord),
		"board-level EEPROM slot is too small for a CalibrationRecord");

// Member order here follows the declaration order in the header -- the compiler
// initialises in declaration order regardless of what is written, so a list in
// a different order reads as a lie about what happens. _levelCalibrator was
// missing from this list entirely; it worked only because its constructor
// calls reset(), which is not a property to depend on silently.
Calibrator::Calibrator() :
		_accelerometerCalibrator { }, _compassCalibrator { },
				_levelCalibrator { }, _storage { nullptr },
				_isAccelCalibrating { false }, _isCompassCalibrating { false },
				_isLevelCalibrating { false }, _isAccelCalibrated { false },
				_isCompassCalibrated { false }, _isLevelCalibrated { false },
				_lastSaveFailed { false }, _awaitingPosition { false },
				_progressThrottle { 0 }, _calibrationEpoch { 0 },
				_accelOffset { }, _accelMatrix { Matrix3f::identity() },
				_compassOffset { }, _compassMatrix { Matrix3f::identity() },
				_boardRotation { Matrix3f::identity() } {
}

void Calibrator::init(EEPROM *storage) {
	_storage = storage;

	// Establish a known idle state before loading, so init() is a clean
	// restart rather than a partial one layered on whatever came before.
	_isAccelCalibrating = false;
	_isCompassCalibrating = false;
	_isLevelCalibrating = false;
	_awaitingPosition = false;
	_progressThrottle = 0;
	_lastSaveFailed = false;
	_accelOffset = Vector3f();
	_accelMatrix = Matrix3f::identity();
	_compassOffset = Vector3f();
	_compassMatrix = Matrix3f::identity();
	_boardRotation = Matrix3f::identity();
	_accelerometerCalibrator.reset();
	_compassCalibrator.reset();
	_levelCalibrator.reset();

	_isAccelCalibrated = (loadAccelCalibrationData() == Calibrator_StatusTypeDef::OK);
	_isCompassCalibrated = (loadCompassCalibrationData() == Calibrator_StatusTypeDef::OK);
	_isLevelCalibrated = (loadLevelCalibrationData() == Calibrator_StatusTypeDef::OK);

	// Restoring from EEPROM changes the applied correction just as much as
	// running a procedure does, and init() may be called on a live system.
	noteGainsChanged();
}

void Calibrator::correctAcclData(Vector3f &acclData) {
	if (!_isAccelCalibrated) {
		return;
	}
	// _accelMatrix maps a raw reading onto the unit sphere (1 g). The rest of
	// the stack -- ESEKF, the controllers -- works in m/s^2, the same units
	// ICM42688P::readData() produces, so scale back before handing it on.
	acclData = _accelMatrix.mul(acclData - _accelOffset) * ACCEL_CAL_STANDARD_GRAVITY;
}

void Calibrator::correctCompassData(Vector3f &compassData) {
	if (!_isCompassCalibrated) {
		return;
	}
	// Already lands on a sphere of the nominal field magnitude, in the units
	// the sensor reports (Gauss), so no rescaling is needed.
	compassData = _compassMatrix.mul(compassData - _compassOffset);
}

// See the note in the header: this is a rotation of the whole board, so the
// caller applies it to the accelerometer, the gyroscope and the magnetometer
// alike. It is intentionally absent from correctAcclData() and
// correctCompassData().
void Calibrator::correctBoardFrame(Vector3f &v) const {
	if (!_isLevelCalibrated) {
		return;
	}
	v = _boardRotation.mul(v);
}

// Two procedures running at once would feed each other's samples through the
// wrong gates and both would store gains derived from the mixture, so every
// entry point refuses here rather than each caller remembering to check.
bool Calibrator::beginProcedure(const char *stage) {
	if (isCalibrating()) {
		telemetry.send("$CAL,%s,FAIL,BUSY", stage);
		return false;
	}
	_awaitingPosition = false;
	_progressThrottle = 0;
	_lastSaveFailed = false;
	return true;
}

void Calibrator::startAccelerometerCalibration() {
	if (!beginProcedure("ACCL")) {
		return;
	}
	_isAccelCalibrating = true;
	_isAccelCalibrated = false;
	// beginSixPosition() resets the calibrator itself, so no separate reset().
	_accelerometerCalibrator.beginSixPosition(CALIBRATOR_ACCEL_MOTION_THRESHOLD);

	telemetry.send("$INFO,ACCEL 6-POSITION CALIBRATION STARTED");
	promptNextPosition();
}

void Calibrator::startAccelerometerTumbleCalibration() {
	if (!beginProcedure("ACCL")) {
		return;
	}
	if (!_accelerometerCalibrator.beginTumble(ACCEL_CAL_STANDARD_GRAVITY,
			CALIBRATOR_ACCEL_STILLNESS_THRESHOLD, _sampleArena,
			CALIBRATOR_SAMPLE_ARENA)) {
		// Only reachable if the arena is mis-sized at compile time, but a
		// procedure that silently never finishes is the worst way to find out.
		telemetry.send("$CAL,ACCL,FAIL,NOBUFFER");
		return;
	}
	_isAccelCalibrating = true;
	_isAccelCalibrated = false;

	telemetry.send("$INFO,ACCEL TUMBLE CALIBRATION STARTED");
	telemetry.send("$INFO,ROTATE SLOWLY THROUGH MANY ORIENTATIONS, PAUSING IN EACH");
}

void Calibrator::startCompassCalibration() {
	if (!beginProcedure("MAG")) {
		return;
	}
	if (!_compassCalibrator.begin(CALIBRATOR_COMPASS_NOMINAL_GAUSS, _sampleArena,
			CALIBRATOR_SAMPLE_ARENA)) {
		telemetry.send("$CAL,MAG,FAIL,NOBUFFER");
		return;
	}
	_isCompassCalibrating = true;
	_isCompassCalibrated = false;

	telemetry.send("$INFO,MAG CALIBRATION STARTED");
	telemetry.send("$INFO,TUMBLE THE AIRFRAME THROUGH AS MANY ORIENTATIONS AS POSSIBLE");
}

void Calibrator::startLevelCalibration(float yaw_offset_deg) {
	// This measures what is left after bias and scale are removed, so without
	// an accelerometer calibration behind it the rotation would absorb the bias
	// as though it were a mounting error. Refuse rather than store that.
	if (!_isAccelCalibrated) {
		telemetry.send("$CAL,LEVEL,FAIL,NOACCLCAL");
		telemetry.send("$INFO,CALIBRATE THE ACCELEROMETER FIRST");
		return;
	}
	if (!beginProcedure("LEVEL")) {
		return;
	}

	_isLevelCalibrating = true;
	_isLevelCalibrated = false;

	// THE BODY FRAME IS FRD: +Z POINTS DOWN.
	//
	// So the reading to expect from a level, upright airframe is (0, 0, -1),
	// not (0, 0, +1). An accelerometer reads specific force, which at rest is
	// minus the gravity vector: with body +Z pointing down it reports -1 g on Z.
	// The same convention the IMU remap is derived against (see the banner in
	// Imu.cpp: "with the drone level and at rest the ESEKF expects body accel
	// (0, 0, -g)"), and the same one AccelPosition::Z_UP assumes -- Z_UP means
	// the body +Z axis points at the sky, i.e. the airframe is INVERTED, and
	// that is the orientation that reads +1 g on Z.
	//
	// Passing (0, 0, +1) here asked the fit to rotate the measured vertical onto
	// its own opposite. That is a 180 degree rotation, so it failed the
	// LEVEL_CAL_MAX_TILT_DEG check on every single run -- the procedure could
	// never succeed, whatever the operator did.
	_levelCalibrator.begin(Vector3f(0.0f, 0.0f, -1.0f),
			ACCEL_CAL_STANDARD_GRAVITY, CALIBRATOR_ACCEL_MOTION_THRESHOLD,
			yaw_offset_deg);

	telemetry.send("$INFO,LEVEL CALIBRATION STARTED");
	telemetry.send("$INFO,PLACE THE AIRFRAME UPRIGHT ON A LEVEL SURFACE AND HOLD STILL");
}

void Calibrator::calibrateLevel(Vector3f &acclData) {
	if (!_isLevelCalibrating) {
		return;
	}

	// Corrected here rather than by the caller so the ordering cannot be got
	// wrong: the levelling fit is only meaningful on top of a finished
	// accelerometer calibration, which startLevelCalibration() has checked for.
	Vector3f corrected = acclData;
	correctAcclData(corrected);
	_levelCalibrator.addSample(corrected);

	if (_levelCalibrator.isStalled()) {
		abortStalled("LEVEL", _levelCalibrator.getProgressPercent(),
				"AIRFRAME IS NOT SITTING STILL - USE A SOLID LEVEL SURFACE");
		return;
	}

	reportProgress("LEVEL", _levelCalibrator.getProgressPercent());

	if (!_levelCalibrator.isReadyToCalibrate()) {
		return;
	}

	finishLevelCalibration();
}

void Calibrator::finishLevelCalibration() {
	const LevelCalStatus status = _levelCalibrator.calibrate();
	if (status == LevelCalStatus::SUCCESS) {
		_boardRotation = _levelCalibrator.getRotation();
		_isLevelCalibrated = true;
		noteGainsChanged();
		_lastSaveFailed = (saveLevelCalibrationData() != Calibrator_StatusTypeDef::OK);
	}
	_isLevelCalibrating = false;

	if (status != LevelCalStatus::SUCCESS) {
		telemetry.send("$CAL,LEVEL,FAIL,%d", (int) status);
		return;
	}
	telemetry.send("$CAL,LEVEL,OK");
	telemetry.send("$LEVELTILT,%.2f", (double) _levelCalibrator.getTiltDeg());
	sendMatrix("BOARDROTATION", _boardRotation);
	telemetry.send("$CAL,LEVEL,%s", _lastSaveFailed ? "NOTSAVED" : "SAVED");
}

void Calibrator::calibrateAccelerometer(Vector3f &acclData) {
	if (!_isAccelCalibrating) {
		return;
	}

	// Between orientations the airframe is being moved, so anything arriving
	// now is motion, not a measurement. Wait for the user to confirm.
	if (_awaitingPosition) {
		return;
	}

	const AccelSampleResult result = _accelerometerCalibrator.addSample(acclData);

	// Nothing downstream can distinguish "still collecting" from "will never
	// finish", because a procedure that cannot pass its motion gate simply
	// never reports ready. Catch that here and say so, rather than leaving the
	// operator watching a frozen progress figure.
	if (_accelerometerCalibrator.isStalled()) {
		abortStalled("ACCL", _accelerometerCalibrator.getProgressPercent(),
				_accelerometerCalibrator.getMode() == AccelCalMode::SIX_POSITION
				? "HOLD THE AIRFRAME STILL - STOP MOTORS AND USE A SOLID SURFACE"
				: "PAUSE LONGER IN EACH ORIENTATION - STOP MOTORS");
		return;
	}

	if (result == AccelSampleResult::ACCEPTED_POSITION_DONE) {
		telemetry.send("$INFO,CAPTURED %s",
				positionName(_accelerometerCalibrator.getCurrentPosition()));
		if (!_accelerometerCalibrator.allPositionsComplete()) {
			promptNextPosition();
			return;
		}
	} else if (_accelerometerCalibrator.getMode() == AccelCalMode::TUMBLE) {
		reportProgress("ACCL", _accelerometerCalibrator.getProgressPercent());
	}

	// Note: this deliberately does not hand over to beginTumble() when the six
	// positions finish. beginTumble() calls reset(), which wipes the six
	// position averages, and isReadyToCalibrate() then reports on the (empty)
	// tumble buffer -- so calibrateSixPosition() was never reached and the
	// result of the whole six-position sequence was silently discarded.
	// Tumble is a separate, alternative procedure; see
	// startAccelerometerTumbleCalibration().
	if (!_accelerometerCalibrator.isReadyToCalibrate()) {
		return;
	}

	finishAccelCalibration();
}

void Calibrator::finishAccelCalibration() {
	const AccelCalStatus status = _accelerometerCalibrator.calibrate();
	if (status == AccelCalStatus::SUCCESS) {
		adoptAccelResult();
		_isAccelCalibrated = true;
		noteGainsChanged();
		// A failed write is not a failed calibration: the gains are live for
		// this session either way, they just will not survive a reboot.
		_lastSaveFailed = (saveAccelCalibrationData() != Calibrator_StatusTypeDef::OK);
	}
	_isAccelCalibrating = false;
	_awaitingPosition = false;

	// Tumble only -- six-position fits no ellipsoid and has no residual.
	if (_accelerometerCalibrator.getMode() == AccelCalMode::TUMBLE) {
		telemetry.send("$ACCLFIT,res=%.3f,max=%.2f",
				(double) _accelerometerCalibrator.getLastFitResidual(),
				(double) ACCEL_CAL_MAX_FIT_RESIDUAL);
	}

	if (status != AccelCalStatus::SUCCESS) {
		telemetry.send("$CAL,ACCL,FAIL,%d", (int) status);
		return;
	}
	telemetry.send("$CAL,ACCL,OK");
	sendVector("ACCLOFFSET", _accelOffset);
	sendMatrix("ACCLMATRIX", _accelMatrix);
	if (_accelerometerCalibrator.getMode() == AccelCalMode::SIX_POSITION) {
		telemetry.send("$ACCLMISALIGN,%.2f",
				(double) _accelerometerCalibrator.getMaxMisalignmentDeg());
	}
	telemetry.send("$CAL,ACCL,%s", _lastSaveFailed ? "NOTSAVED" : "SAVED");
}

void Calibrator::setAccelPosition(AccelPosition pos) {
	if (!_isAccelCalibrating) {
		return;
	}
	_accelerometerCalibrator.startPosition(pos);
	_awaitingPosition = false;
	telemetry.send("$INFO,RECORDING %s - HOLD STILL", positionName(pos));
}

void Calibrator::confirmReady() {
	if (!_isAccelCalibrating || !_awaitingPosition) {
		return;
	}
	setAccelPosition(nextPendingPosition());
}

AccelPosition Calibrator::nextPendingPosition() const {
	// Driven off the calibrator's own per-position state rather than a counter,
	// so an out-of-order setAccelPosition() or a redone orientation cannot
	// desynchronise the prompts from what has actually been captured.
	for (uint8_t i = 0; i < (uint8_t) AccelPosition::NUM_POSITIONS; i++) {
		const AccelPosition pos = (AccelPosition) i;
		if (!_accelerometerCalibrator.isPositionDone(pos)) {
			return pos;
		}
	}
	return AccelPosition::X_UP;
}

void Calibrator::promptNextPosition() {
	_awaitingPosition = true;
	telemetry.send("$PROMPT,%s", positionName(nextPendingPosition()));
	telemetry.send("$INFO,SEND READY WHEN IN POSITION AND STILL");
}

const char *Calibrator::positionName(AccelPosition pos) {
	switch (pos) {
	case AccelPosition::X_UP:   return "X_UP";
	case AccelPosition::X_DOWN: return "X_DOWN";
	case AccelPosition::Y_UP:   return "Y_UP";
	case AccelPosition::Y_DOWN: return "Y_DOWN";
	case AccelPosition::Z_UP:   return "Z_UP";
	case AccelPosition::Z_DOWN: return "Z_DOWN";
	default:                    return "UNKNOWN";
	}
}

void Calibrator::calibrateCompass(Vector3f &compassData) {
	if (!_isCompassCalibrating) {
		return;
	}

	const SampleResult result = _compassCalibrator.addSample(compassData);
	(void) result;

	// See calibrateAccelerometer(). For the compass a stall means the airframe
	// has stopped reaching new orientations rather than vibration, since there
	// is no stillness gate on this path.
	if (_compassCalibrator.isStalled()) {
		abortStalled("MAG", _compassCalibrator.getProgressPercent(),
				"KEEP TURNING THE AIRFRAME - ALL SIDES, INCLUDING INVERTED");
		return;
	}

	reportProgress("MAG", _compassCalibrator.getProgressPercent());

	if (!_compassCalibrator.isReadyToCalibrate()) {
		return;
	}

	finishCompassCalibration();
}

void Calibrator::finishCompassCalibration() {
	const CalStatus status = _compassCalibrator.calibrate();
	if (status == CalStatus::SUCCESS) {
		adoptCompassResult();
		_isCompassCalibrated = true;
		noteGainsChanged();
		_lastSaveFailed = (saveCompassCalibrationData() != Calibrator_StatusTypeDef::OK);
	}
	_isCompassCalibrating = false;

	// Emitted whether the fit passed or failed. On a failure the status code
	// alone says which gate rejected it but not by how much, and for
	// FAILED_POOR_FIT that is the whole question: a residual of 0.16 is a sweep
	// that nearly worked, 0.90 is an environment or a sensor carrying no usable
	// field, and they call for completely different responses. On a success the
	// same numbers are a quality score -- a residual near the limit means the
	// calibration was accepted but is not one to trust far.
	telemetry.send("$MAGFIT,res=%.3f,max=%.2f,n=%u,bins=%u,scatter=%.2f",
			(double) _compassCalibrator.getLastFitResidual(),
			(double) COMPASS_CAL_MAX_FIT_RESIDUAL,
			(unsigned) _compassCalibrator.getSampleCount(),
			(unsigned) _compassCalibrator.getFilledBinsCount(),
			(double) _compassCalibrator.getScatterRatio());

	if (status != CalStatus::SUCCESS) {
		telemetry.send("$CAL,MAG,FAIL,%d", (int) status);
		return;
	}
	telemetry.send("$CAL,MAG,OK");
	sendVector("MAGOFFSET", _compassOffset);
	sendMatrix("MAGMATRIX", _compassMatrix);
	telemetry.send("$CAL,MAG,%s", _lastSaveFailed ? "NOTSAVED" : "SAVED");
}

// Reports what is APPLIED, not what is on the device. Those are the same thing
// after a cold boot -- init() loads the gains and nothing else writes them --
// which is what makes a power-cycle followed by this call a genuine test of
// persistence. Within a session they can differ: a calibration that succeeded
// but failed to save is live in RAM and absent from EEPROM, which is why the
// storage state is reported alongside.
void Calibrator::reportCalibration() {
	telemetry.send("$CALDUMP,storage=%s", hasStorage() ? "OK" : "NONE");

	telemetry.send("$STATUS,ACCL,%s,%s", _isAccelCalibrated ? "CAL" : "UNCAL",
			_isAccelCalibrating ? "BUSY" : "IDLE");
	if (_isAccelCalibrated) {
		sendVector("ACCLOFFSET", _accelOffset);
		sendMatrix("ACCLMATRIX", _accelMatrix);
	}

	telemetry.send("$STATUS,MAG,%s,%s", _isCompassCalibrated ? "CAL" : "UNCAL",
			_isCompassCalibrating ? "BUSY" : "IDLE");
	if (_isCompassCalibrated) {
		sendVector("MAGOFFSET", _compassOffset);
		sendMatrix("MAGMATRIX", _compassMatrix);
	}

	telemetry.send("$STATUS,LEVEL,%s,%s", _isLevelCalibrated ? "CAL" : "UNCAL",
			_isLevelCalibrating ? "BUSY" : "IDLE");
	if (_isLevelCalibrated) {
		sendMatrix("BOARDROTATION", _boardRotation);
	}

	telemetry.send("$CALDUMP,END");
}

void Calibrator::sendVector(const char *key, const Vector3f &v) {
	telemetry.send("$%s,%.5f,%.5f,%.5f", key, (double) v.x, (double) v.y,
			(double) v.z);
}

void Calibrator::sendMatrix(const char *key, const Matrix3f &m) {
	for (int i = 0; i < 3; i++) {
		telemetry.send("$%s,%d,%.5f,%.5f,%.5f", key, i, (double) m.m[i][0],
				(double) m.m[i][1], (double) m.m[i][2]);
	}
}

void Calibrator::reportProgress(const char *stage, float percent) {
	// calibrate*() runs once per control loop; sending every time would swamp
	// the link and starve the sampling it is reporting on.
	if ((_progressThrottle++ % CALIBRATOR_PROGRESS_INTERVAL) != 0) {
		return;
	}
	telemetry.send("$PROG,%s,%.1f", stage, (double) percent);
}

// A stalled procedure is reported as a failure and torn down, so the caller
// ends up in the same idle state as any other failure and can simply start
// again. The progress figure it reached is included because that is what says
// how far it got before things stopped moving.
void Calibrator::abortStalled(const char *stage, float percent,
		const char *advice) {
	// stopProcedures() rather than cancelCalibration(): this is a FAILURE, and
	// emitting "CANCELLED" ahead of "FAIL,STALLED" would tell the operator their
	// own CANCEL had been received when nothing of the sort happened.
	stopProcedures();

	telemetry.send("$CAL,%s,FAIL,STALLED", stage);
	telemetry.send("$INFO,NO PROGRESS AT %.0f%% - %s", (double) percent, advice);
}

// Tear down whatever is running and drop its partial samples, silently.
// Returns true if anything was actually running.
bool Calibrator::stopProcedures() {
	const bool was_running = isCalibrating();

	if (_isAccelCalibrating) {
		_accelerometerCalibrator.reset();
		_isAccelCalibrating = false;
	}
	if (_isCompassCalibrating) {
		_compassCalibrator.reset();
		_isCompassCalibrating = false;
	}
	if (_isLevelCalibrating) {
		_levelCalibrator.reset();
		_isLevelCalibrating = false;
	}
	_awaitingPosition = false;

	return was_running;
}

void Calibrator::cancelCalibration() {
	if (stopProcedures()) {
		telemetry.send("$INFO,CALIBRATION CANCELLED");
	}
}

float Calibrator::getAccelProgressPercent() const {
	return _accelerometerCalibrator.getProgressPercent();
}

float Calibrator::getCompassProgressPercent() const {
	return _compassCalibrator.getProgressPercent();
}

float Calibrator::getLevelProgressPercent() const {
	return _levelCalibrator.getProgressPercent();
}

// ---------------------------------------------------------------------------
// Turning a finished calibration into the applied affine correction
// ---------------------------------------------------------------------------

void Calibrator::adoptAccelResult() {
	_accelOffset = _accelerometerCalibrator.getBias();

	if (_accelerometerCalibrator.getMode() == AccelCalMode::TUMBLE) {
		// correct() computes matrix * (raw - offset) / nominal_g; fold the
		// division into the matrix so both modes share one representation.
		const float nominal = _accelerometerCalibrator.getNominalRadius();
		_accelMatrix = _accelerometerCalibrator.getMatrix().scaled(1.0f / nominal);
		return;
	}

	// Six-position: correct() divides component-wise by the per-axis scale.
	// calibrateSixPosition() has already rejected a near-zero scale, so this
	// cannot divide by zero.
	const Vector3f scale = _accelerometerCalibrator.getScale();
	_accelMatrix = Matrix3f::identity();
	_accelMatrix.m[0][0] = 1.0f / scale.x;
	_accelMatrix.m[1][1] = 1.0f / scale.y;
	_accelMatrix.m[2][2] = 1.0f / scale.z;
}

void Calibrator::adoptCompassResult() {
	_compassOffset = _compassCalibrator.getOffset();
	_compassMatrix = _compassCalibrator.getSoftIronMatrix();
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

uint16_t Calibrator::recordCrc(const CalibrationRecord &rec) {
	// CRC-16/CCITT-FALSE over everything ahead of the crc field, so a blank
	// (0xFF) or half-written device is rejected instead of being flown with.
	const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&rec);
	const size_t len = offsetof(CalibrationRecord, crc);

	uint16_t crc = 0xFFFFU;
	for (size_t i = 0; i < len; i++) {
		crc ^= static_cast<uint16_t>(bytes[i]) << 8;
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
			                      : static_cast<uint16_t>(crc << 1);
		}
	}
	return crc;
}

void Calibrator::packRecord(const Vector3f &offset, const Matrix3f &matrix,
		CalibrationRecord &out) {
	memset(&out, 0, sizeof(out)); // keeps reserved/pad deterministic for the CRC

	out.magic = CALIBRATION_RECORD_MAGIC;
	out.version = CALIBRATION_RECORD_VERSION;
	out.reserved = 0;

	out.offset[0] = offset.x;
	out.offset[1] = offset.y;
	out.offset[2] = offset.z;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.matrix[i * 3 + j] = matrix.m[i][j];
		}
	}

	out.pad = 0;
	out.crc = recordCrc(out);
}

bool Calibrator::unpackRecord(const CalibrationRecord &rec, Vector3f &offset,
		Matrix3f &matrix) {
	if (rec.magic != CALIBRATION_RECORD_MAGIC) {
		return false;
	}
	if (rec.version != CALIBRATION_RECORD_VERSION) {
		return false;
	}
	if (rec.crc != recordCrc(rec)) {
		return false;
	}

	// A NaN or infinity that happens to carry a valid CRC would silently
	// poison every corrected sample, so screen the payload too.
	for (int i = 0; i < 3; i++) {
		if (!std::isfinite(rec.offset[i])) return false;
	}
	for (int i = 0; i < 9; i++) {
		if (!std::isfinite(rec.matrix[i])) return false;
	}

	offset = Vector3f(rec.offset[0], rec.offset[1], rec.offset[2]);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix.m[i][j] = rec.matrix[i * 3 + j];
		}
	}
	return true;
}

Calibrator_StatusTypeDef Calibrator::loadRecord(EEPROMLocation location,
		Vector3f &offset, Matrix3f &matrix) {
	if (_storage == nullptr) {
		return Calibrator_StatusTypeDef::ERROR;
	}

	CalibrationRecord rec;
	if (_storage->read(location, rec) != EEPROM_StatusTypeDef::OK) {
		return Calibrator_StatusTypeDef::ERROR;
	}
	if (!unpackRecord(rec, offset, matrix)) {
		return Calibrator_StatusTypeDef::ERROR;
	}
	return Calibrator_StatusTypeDef::OK;
}

Calibrator_StatusTypeDef Calibrator::saveRecord(EEPROMLocation location,
		const Vector3f &offset, const Matrix3f &matrix) {
	if (_storage == nullptr) {
		return Calibrator_StatusTypeDef::ERROR;
	}

	CalibrationRecord rec;
	packRecord(offset, matrix, rec);

	if (_storage->write(location, rec) != EEPROM_StatusTypeDef::OK) {
		return Calibrator_StatusTypeDef::ERROR;
	}

	// Read back and re-validate: an M24C02 that NAKs mid-page still reports a
	// successful transfer for the chunks that landed, so the only way to know
	// the record is intact is to read it.
	CalibrationRecord verify;
	if (_storage->read(location, verify) != EEPROM_StatusTypeDef::OK) {
		return Calibrator_StatusTypeDef::ERROR;
	}
	if (memcmp(&rec, &verify, sizeof(rec)) != 0) {
		return Calibrator_StatusTypeDef::ERROR;
	}
	return Calibrator_StatusTypeDef::OK;
}

Calibrator_StatusTypeDef Calibrator::loadAccelCalibrationData() {
	return loadRecord(EEPROMLocation::ACCLCALIBRATEDAT, _accelOffset, _accelMatrix);
}

Calibrator_StatusTypeDef Calibrator::loadCompassCalibrationData() {
	return loadRecord(EEPROMLocation::COMPASSCALIBRATEDAT, _compassOffset,
			_compassMatrix);
}

// The levelling result is a rotation with no offset, so it reuses the same
// record shape as the other two: offset stays zero and the matrix carries the
// rotation. One format, one CRC, one set of rejection rules.
Calibrator_StatusTypeDef Calibrator::loadLevelCalibrationData() {
	Vector3f unused;
	return loadRecord(EEPROMLocation::BOARDLEVELCALIBRATEDAT, unused,
			_boardRotation);
}

Calibrator_StatusTypeDef Calibrator::saveLevelCalibrationData() {
	return saveRecord(EEPROMLocation::BOARDLEVELCALIBRATEDAT, Vector3f(),
			_boardRotation);
}

Calibrator_StatusTypeDef Calibrator::saveAccelCalibrationData() {
	return saveRecord(EEPROMLocation::ACCLCALIBRATEDAT, _accelOffset, _accelMatrix);
}

Calibrator_StatusTypeDef Calibrator::saveCompassCalibrationData() {
	return saveRecord(EEPROMLocation::COMPASSCALIBRATEDAT, _compassOffset,
			_compassMatrix);
}

Calibrator_StatusTypeDef Calibrator::clearStoredCalibration() {
	_isAccelCalibrated = false;
	_isCompassCalibrated = false;
	_isLevelCalibrated = false;
	_accelOffset = Vector3f();
	_accelMatrix = Matrix3f::identity();
	_compassOffset = Vector3f();
	_compassMatrix = Matrix3f::identity();
	_boardRotation = Matrix3f::identity();
	noteGainsChanged();

	if (_storage == nullptr) {
		return Calibrator_StatusTypeDef::ERROR;
	}

	// Zeroing the magic is enough to invalidate a slot, and it is a single
	// page write rather than a full-record erase.
	CalibrationRecord blank;
	memset(&blank, 0, sizeof(blank));

	bool ok = (_storage->write(EEPROMLocation::ACCLCALIBRATEDAT, blank)
			== EEPROM_StatusTypeDef::OK);
	ok = (_storage->write(EEPROMLocation::COMPASSCALIBRATEDAT, blank)
			== EEPROM_StatusTypeDef::OK) && ok;
	ok = (_storage->write(EEPROMLocation::BOARDLEVELCALIBRATEDAT, blank)
			== EEPROM_StatusTypeDef::OK) && ok;

	return ok ? Calibrator_StatusTypeDef::OK : Calibrator_StatusTypeDef::ERROR;
}
