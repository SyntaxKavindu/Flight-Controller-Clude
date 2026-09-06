/*
 * Calibrator.hpp
 *
 *  Created on: Aug 30, 2026
 *      Author: KAVINDU
 */

#ifndef CALIBRATORS_CALIBRATOR_HPP_
#define CALIBRATORS_CALIBRATOR_HPP_

#include "AccelerometerCalibrator.hpp"
#include "CompassCalibrator.hpp"
#include "LevelCalibrator.hpp"
#include "common.hpp"

// Only a handle and a location are named here; EEPROM.hpp is included in the
// .cpp so this header does not drag the HAL in through M24C02.
class EEPROM;
enum class EEPROMLocation : uint8_t;

enum class Calibrator_StatusTypeDef{
	OK = 0,
	ERROR = 1
};

// Rough magnitude of Earth's field, in the units LIS3MDL::readData() reports
// (Gauss). It sets both the fit normalisation and the output scale of
// correctCompassData(), so it must be in sensor units -- a raw-LSB figure such
// as 500 here would rescale every corrected reading by ~1000x.
#define CALIBRATOR_COMPASS_NOMINAL_GAUSS      0.5f

// Six-position motion gate and tumble stillness gate, in m/s^2 -- the units
// ICM42688P::readData() / IMU_Data::accel are in. A few times the sensor's
// at-rest noise floor; tune to your airframe.
#define CALIBRATOR_ACCEL_MOTION_THRESHOLD     0.5f
#define CALIBRATOR_ACCEL_STILLNESS_THRESHOLD  0.2f

// Send one progress line every N samples. calibrate*() is called once per
// control loop, so reporting every sample would swamp the serial link.
#define CALIBRATOR_PROGRESS_INTERVAL          50

// On-EEPROM correction record. Both sensors persist the same shape, because
// every procedure this class runs reduces to one affine correction:
//
//     corrected = matrix * (raw - offset)
//
// For the accelerometer that yields a unit-g vector (six-position uses
// matrix = diag(1/scale), tumble uses matrix = tumble_matrix / nominal_g); for
// the compass it yields a vector of the nominal field magnitude.
//
// Layout is fixed and read back by the same MCU that wrote it, so native float
// representation and byte order are fine. Bump CALIBRATION_RECORD_VERSION if
// the meaning of any field changes -- a record with an older version is
// rejected rather than misinterpreted.
#define CALIBRATION_RECORD_MAGIC    0xCA1BU
#define CALIBRATION_RECORD_VERSION  1U

struct CalibrationRecord {
	uint16_t magic;
	uint8_t  version;
	uint8_t  reserved;
	float    offset[3];
	float    matrix[9];
	uint16_t crc;      // CRC-16/CCITT-FALSE over every byte before this field
	uint16_t pad;
};

static_assert(sizeof(CalibrationRecord) == 56,
		"CalibrationRecord must stay 56 bytes to fit an EEPROM slot");

class Calibrator {
public:
	Calibrator();

	// `storage` may be null, in which case calibration works normally but is
	// not persisted and nothing is restored at boot. Also puts the calibrator
	// back into a known idle state, so calling it again is a clean restart.
	// Progress and results are reported through the global `telemetry`.
	void init(EEPROM *storage = nullptr);
	void correctAcclData(Vector3f &acclData);
	void correctCompassData(Vector3f &compassData);

	// Board-mounting rotation, from startLevelCalibration().
	//
	// APPLY THIS TO EVERY SENSOR ON THE BOARD -- accelerometer, gyroscope and
	// magnetometer -- after each has had its own correction. It is deliberately
	// NOT folded into correctAcclData() or correctCompassData(): the rotation
	// describes the board, not one sensor, and applying it to some sensors and
	// not others would leave them disagreeing about which way the airframe
	// points, which is worse than not correcting at all. There is no gyro path
	// through this class, so silently rotating the other two would guarantee
	// that mismatch. Kept separate so the caller applies it to all three in one
	// place, and so forgetting it changes nothing rather than breaking the
	// frame.
	//
	// No-op until a levelling calibration has succeeded or been restored.
	void correctBoardFrame(Vector3f &v) const;
	Mat3f getBoardRotation() const { return _boardRotation; }

	// Six-position accelerometer calibration: call this, then drive it with
	// setAccelPosition() + calibrateAccelerometer() for each orientation.
	void startAccelerometerCalibration();
	// Alternative single-shot accelerometer procedure: tumble the airframe
	// through as many orientations as possible, pausing briefly in each.
	// Mutually exclusive with the six-position sequence above.
	void startAccelerometerTumbleCalibration();
	void startCompassCalibration();

	// Board levelling: rest the airframe on a surface known to be level and
	// upright, then drive this with calibrateLevel() until it finishes.
	// Requires a completed accelerometer calibration -- it measures what is
	// left once bias and scale are gone -- and reports an error without one.
	//
	// yaw_offset_deg: mounting rotation about the vertical, which gravity
	// cannot observe. See LevelCalibrator::begin().
	void startLevelCalibration(float yaw_offset_deg = 0.0f);

	// Feed the RAW (axis-remapped, uncorrected) accelerometer sample, exactly as
	// calibrateAccelerometer() takes it. The accelerometer correction is applied
	// inside, because the levelling fit is only meaningful on top of a finished
	// accelerometer calibration and having one caller pass corrected data and
	// another raw is the kind of mistake that produces a plausible, wrong
	// rotation rather than an error.
	void calibrateLevel(Vector3f &acclData);

	// Feed one sample per loop while a procedure is running. Ignored when
	// idle, and ignored between positions while waiting for confirmReady().
	void calibrateAccelerometer(Vector3f &acclData);
	void setAccelPosition(AccelPosition pos);

	// Six-position flow: true while the airframe should be moved to the next
	// orientation and the user has not confirmed yet. confirmReady() starts
	// recording that orientation -- this is what a "READY" command calls.
	bool isAwaitingPosition() const { return _awaitingPosition; }
	void confirmReady();

	// First orientation of the six-position sequence still outstanding.
	AccelPosition nextPendingPosition() const;
	static const char *positionName(AccelPosition pos);

	void calibrateCompass(Vector3f &compassData);

	// Abort whichever calibration is running and drop its partial samples.
	void cancelCalibration();

	bool isAcclCalibrating() const { return _isAccelCalibrating; }
	bool isCompassCalibrating() const { return _isCompassCalibrating; }
	bool isLevelCalibrating() const { return _isLevelCalibrating; }

	// True while ANY procedure is running. This is the predicate the flight
	// stack should stand down on, and the one a command handler should refuse
	// on: a calibration means an operator is deliberately moving the airframe
	// and the sensor frontends are handing back data that is uncorrected, being
	// corrected against gains that are about to change, or both.
	//
	// Spelling it out at each call site as an OR of the three flags is how the
	// levelling procedure came to be left out of every one of them -- the checks
	// were written when there were only two.
	bool isCalibrating() const {
		return _isAccelCalibrating || _isCompassCalibrating || _isLevelCalibrating;
	}

	// Bumped every time the APPLIED correction changes: a procedure succeeded,
	// a stored calibration was restored at boot, or the calibration was erased.
	//
	// This is what lets the estimator notice. A new calibration silently changes
	// what every subsequent sample means -- a different accelerometer scale, a
	// different magnetic frame, a board rotation that was not there a moment ago
	// -- and an attitude filter carrying state built on the OLD gains will fuse
	// the new samples as though the vehicle had moved. Watch this and re-seed
	// when it changes; a counter rather than a flag so a consumer that polls
	// slowly cannot miss one.
	uint32_t getCalibrationEpoch() const { return _calibrationEpoch; }
	bool isAcclCalibrated() const { return _isAccelCalibrated; }
	bool isCompassCalibrated() const { return _isCompassCalibrated; }
	bool isLevelCalibrated() const { return _isLevelCalibrated; }

	float getAccelProgressPercent() const;
	float getCompassProgressPercent() const;
	float getLevelProgressPercent() const;

	// True when the last completed calibration could not be written to EEPROM
	// (no storage handle, or the device rejected the write). The gains are
	// still applied for this session -- they just will not survive a reboot.
	bool lastSaveFailed() const { return _lastSaveFailed; }

	// Discard the stored calibration for both sensors, in RAM and on EEPROM.
	Calibrator_StatusTypeDef clearStoredCalibration();

private:
	AccelerometerCalibrator _accelerometerCalibrator;
	CompassCalibrator _compassCalibrator;
	LevelCalibrator _levelCalibrator;

	EEPROM *_storage;

	bool _isAccelCalibrating;
	bool _isCompassCalibrating;
	bool _isLevelCalibrating;

	bool _isAccelCalibrated;
	bool _isCompassCalibrated;
	bool _isLevelCalibrated;

	bool _lastSaveFailed;

	// Six-position sequencing, and a divider so progress lines do not flood
	// the link -- calibrate*() is called once per control loop.
	bool _awaitingPosition;
	uint16_t _progressThrottle;

	// See getCalibrationEpoch().
	uint32_t _calibrationEpoch;

	// The applied correction, held here rather than read back out of the
	// calibrator objects on every sample: a calibration restored from EEPROM
	// has no estimator state behind it, so the facade has to own the gains for
	// the boot path and the just-calibrated path to behave identically.
	Vector3f _accelOffset;
	Mat3f    _accelMatrix;
	Vector3f _compassOffset;
	Mat3f    _compassMatrix;
	// Rotation from measured board axes to airframe axes; identity until a
	// levelling calibration succeeds.
	Mat3f    _boardRotation;

	// Pull the finished gains out of a calibrator into the affine form above.
	void adoptAccelResult();
	void adoptCompassResult();

	// Every path that changes an applied gain goes through this, so no path can
	// change one without the estimator being told. See getCalibrationEpoch().
	void noteGainsChanged() { _calibrationEpoch++; }

	// Shared front half of every start*() entry point: refuses if a procedure is
	// already running, and puts the shared sequencing state back to a known
	// point. Returns false when the caller must not start.
	bool beginProcedure(const char *stage);

	// Tear down whatever is running, silently. True if anything was.
	bool stopProcedures();

	// Reports a procedure that has stopped making progress and tears it down,
	// so a wedged calibration ends in a failure the operator can act on
	// instead of running for ever.
	void abortStalled(const char *stage, float percent, const char *advice);
	void promptNextPosition();
	void finishAccelCalibration();
	void finishCompassCalibration();
	void finishLevelCalibration();
	void reportProgress(const char *stage, float percent);
	// Telemetry only frames and queues lines now, so the records the
	// calibration result is reported as are built here.
	void sendVector(const char *key, const Vector3f &v);
	void sendMatrix(const char *key, const Mat3f &m);

	Calibrator_StatusTypeDef loadAccelCalibrationData();
	Calibrator_StatusTypeDef loadCompassCalibrationData();
	Calibrator_StatusTypeDef loadLevelCalibrationData();
	Calibrator_StatusTypeDef saveAccelCalibrationData();
	Calibrator_StatusTypeDef saveCompassCalibrationData();
	Calibrator_StatusTypeDef saveLevelCalibrationData();

	// Serialisation helpers. Kept static and free of any I/O so the format,
	// the CRC and the rejection rules are directly testable.
	static uint16_t recordCrc(const CalibrationRecord &rec);
	static void packRecord(const Vector3f &offset, const Mat3f &matrix,
			CalibrationRecord &out);
	static bool unpackRecord(const CalibrationRecord &rec, Vector3f &offset,
			Mat3f &matrix);

	Calibrator_StatusTypeDef loadRecord(EEPROMLocation location,
			Vector3f &offset, Mat3f &matrix);
	Calibrator_StatusTypeDef saveRecord(EEPROMLocation location,
			const Vector3f &offset, const Mat3f &matrix);
};

// The one calibrator. Telemetry drives it; the sensor frontends feed and
// consult it. Defined in Globals.cpp.
extern Calibrator calibrator;

#endif /* CALIBRATORS_CALIBRATOR_HPP_ */
