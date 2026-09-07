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

// Rate at which the accelerometer procedures WANT to be fed, in Hz.
//
// Every rate-sensitive constant in AccelerometerCalibrator -- the stillness
// window, the samples-per-position count, both stall limits -- was chosen and
// characterised against a 200 Hz feed, and its header states the resulting
// times in seconds on that basis. Nothing in that class measures time, so
// feeding it faster silently rescales all four:
//
//                      at 200 Hz (intended)   at 1 kHz (this fast loop)
//   stillness window          60 ms             12 ms  <- admits motion
//   samples/position         500 ms            100 ms  <- less averaging
//   six-position stall         50 s              10 s
//   tumble stall              200 s              41 s  <- aborts a good run
//
// The last one is not theoretical: the longest no-progress gap measured in a
// SUCCESSFUL tumble was 10151 samples (see ACCEL_CAL_TUMBLE_STALL_LIMIT), and
// that same operator pause at 1 kHz is ~50000 samples -- past the 40000 limit.
//
// So the feed is decimated back to this rate rather than the constants being
// rescaled. Decimation, not averaging: dropping N-1 of every N samples leaves
// the noise statistics exactly as they were characterised, where a box average
// would shrink them by sqrt(N) and quietly loosen the stillness gate by the
// same factor. It also costs nothing -- no accumulator, no extra state per
// axis -- which rescaling the stillness window would not (it is an array).
//
// The compass path needs none of this: it is fed from the 50 Hz mid loop.
#define CALIBRATOR_ACCEL_FEED_HZ              200u

// ONE sample buffer, shared by the accelerometer tumble and the compass sweep.
//
// Both procedures collect into a caller-supplied buffer (see
// AccelerometerCalibrator::beginTumble and CompassCalibrator::begin) and both
// want 300 Vector3f, about 3.6 kB each. Giving each its own array put 7.2 kB
// permanently in .bss to serve two procedures that run for a few seconds on the
// ground, never in flight, and NEVER AT THE SAME TIME -- beginProcedure()
// refuses to start one while another is running, which is what makes sharing
// provably safe rather than merely probable.
//
// If you port this to a part where even 3.6 kB is too much: the buffer is a
// caller parameter precisely so it can live somewhere else, and the six-
// position accelerometer procedure needs no buffer at all.
#define CALIBRATOR_SAMPLE_ARENA \
	((ACCEL_CAL_TUMBLE_MAX_SAMPLES > COMPASS_CAL_MAX_SAMPLES) \
			? ACCEL_CAL_TUMBLE_MAX_SAMPLES : COMPASS_CAL_MAX_SAMPLES)

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

	// Worst-case cross-axis misalignment the six-position fit LEFT BEHIND,
	// in tenths of a degree, PLUS ONE. Zero means "not recorded".
	//
	// The +1 is what makes this backward compatible. This byte was `reserved`
	// and written as zero, so every record already on a device decodes as "not
	// recorded" rather than as a perfect 0.00 deg -- which is the one reading
	// that would be actively misleading. No version bump, and an existing
	// calibration keeps working.
	//
	// One byte is enough: the useful range is 0..25.4 deg in 0.1 deg steps, and
	// anything past a couple of degrees means "use tumble" long before the
	// resolution matters. It sits ahead of `crc` so it is covered by it.
	//
	// Absent for a tumble fit, which removes cross-axis error rather than
	// leaving it, and for the compass and levelling records, where the quantity
	// has no meaning.
	uint8_t  misalign_deci;

	float    offset[3];
	float    matrix[9];
	uint16_t crc;      // CRC-16/CCITT-FALSE over every byte before this field
	uint16_t pad;
};

// Encode/decode for the field above. -1 means "not recorded" on both sides.
inline uint8_t calibrationEncodeMisalign(float deg) {
	if (!(deg >= 0.0f)) {          // also catches NaN
		return 0;
	}
	const long v = (long) (deg * 10.0f + 0.5f) + 1;
	return (v > 255) ? (uint8_t) 255 : (uint8_t) v;
}

inline float calibrationDecodeMisalign(uint8_t stored) {
	return (stored == 0) ? -1.0f : (float) (stored - 1) * 0.1f;
}

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
	Matrix3f getBoardRotation() const { return _boardRotation; }

	// Six-position accelerometer calibration: call this, then drive it with
	// setAccelPosition() + calibrateAccelerometer() for each orientation.
	//
	// chain_level: run the board levelling immediately after the fit succeeds,
	// with no further operator input. The sequence ends on Z_DOWN -- +Z, the
	// body DOWN axis, pointing down, i.e. the airframe sitting upright reading
	// (0,0,-g) -- which is exactly the orientation levelling wants, and the
	// airframe is already still in it. Six-position only: a tumble finishes in
	// whatever orientation the operator stopped in, so it is never chained.
	//
	// This makes the surface the six-position run finished on the reference
	// that DEFINES level for the airframe. That is the whole reason it is a
	// parameter and not the default -- see the note at startLevelCalibration().
	void startAccelerometerCalibration(bool chain_level = false);
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
	//
	// Whatever surface the airframe is resting on when this runs BECOMES the
	// definition of level for every later flight: the fit stores the residual
	// attitude as a correction, so a bench that is a degree out puts a degree
	// of bias into level hover for good. Two things follow, and they are why
	// this is a deliberate operator action rather than something chained
	// automatically onto the end of every accelerometer run:
	//
	//   - the surface has to be one the operator has actually checked, not
	//     merely a flat one;
	//   - the board has to be IN the airframe. Levelling measures a MOUNTING
	//     property, so a six-position run done on a loose board on the bench
	//     has nothing meaningful to level against.
	void startLevelCalibration(float yaw_offset_deg = 0.0f);

	// Rate, in Hz, at which calibrateAccelerometer() and calibrateLevel() will
	// be called -- the caller's control-loop rate, not the sensor's ODR. Sets up
	// the decimation described at CALIBRATOR_ACCEL_FEED_HZ.
	//
	// Optional. Left unset, every sample is passed through, which is correct
	// for a caller already running at CALIBRATOR_ACCEL_FEED_HZ or slower.
	// Ignored while a procedure is running: changing the divider mid-run would
	// rescale the gates the run has already been judged against.
	void setAccelFeedRate(uint16_t loop_hz);

	// Divider currently in force. 1 means every sample is used.
	uint16_t getAccelFeedDivider() const { return _feedDivider; }

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

	// Dump the correction the flight stack is CURRENTLY APPLYING: offsets,
	// matrices and the board rotation, for all three procedures.
	//
	// These values are otherwise printed exactly once, at the moment a
	// procedure finishes, and never again -- so after a reboot there was no way
	// to see what had been restored from EEPROM, or to confirm anything had
	// been. Power-cycle and call this: whatever it reports came off the device,
	// because nothing else could have put it there.
	void reportCalibration();

	// False when no EEPROM handle was supplied to init(), in which case every
	// calibration works for the session and none of it survives a reboot.
	bool hasStorage() const { return _storage != nullptr; }

	// True when the last completed calibration could not be written to EEPROM
	// (no storage handle, or the device rejected the write). The gains are
	// still applied for this session -- they just will not survive a reboot.
	bool lastSaveFailed() const { return _lastSaveFailed; }

	// Discard the stored calibration for both sensors, in RAM and on EEPROM.
	Calibrator_StatusTypeDef clearStoredCalibration();

	// Serialisation, exposed deliberately. These are pure functions with no I/O
	// and no dependence on instance state, so the on-EEPROM format, its CRC and
	// its rejection rules can be exercised directly -- which is the only way to
	// check that a blank device, a half-written record or a NaN payload is
	// refused, since none of those can be produced through the normal path.
	// Also useful to a ground-station or config tool that needs to build or
	// validate a record without an MCU.
	static uint16_t recordCrc(const CalibrationRecord &rec);
	// misalign_deg: -1 when the quantity does not apply, which is every record
	// except a six-position accelerometer fit. unpackRecord() hands back -1 for
	// a record that predates the field, so a caller cannot tell "not recorded"
	// from "recorded as zero" by accident.
	static void packRecord(const Vector3f &offset, const Matrix3f &matrix,
			CalibrationRecord &out, float misalign_deg = -1.0f);
	static bool unpackRecord(const CalibrationRecord &rec, Vector3f &offset,
			Matrix3f &matrix, float *misalign_deg = nullptr);

private:
	AccelerometerCalibrator _accelerometerCalibrator;
	CompassCalibrator _compassCalibrator;
	LevelCalibrator _levelCalibrator;

	// The one sample buffer, lent to whichever procedure is running. See
	// CALIBRATOR_SAMPLE_ARENA for why sharing is safe.
	Vector3f _sampleArena[CALIBRATOR_SAMPLE_ARENA];

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

	// Feed decimation for the two accelerometer-path entry points. _feedPhase
	// counts calls and one in every _feedDivider is passed on. Reset at the
	// start of each procedure so a run always begins on an accepted sample.
	uint16_t _feedDivider;
	uint16_t _feedPhase;

	// Set by startAccelerometerCalibration(true); consumed once by
	// finishAccelCalibration(). Cleared on cancel so an abandoned run cannot
	// chain a levelling onto a later, unrelated one.
	bool _chainLevel;

	// See getCalibrationEpoch().
	uint32_t _calibrationEpoch;

	// The applied correction, held here rather than read back out of the
	// calibrator objects on every sample: a calibration restored from EEPROM
	// has no estimator state behind it, so the facade has to own the gains for
	// the boot path and the just-calibrated path to behave identically.
	// Cross-axis misalignment the stored accelerometer fit left behind, in
	// degrees; -1 when not recorded. Persisted, so CALDUMP can report it after
	// a reboot instead of it existing only in the one line CALIMU printed.
	float _accelMisalignDeg;

	Vector3f _accelOffset;
	Matrix3f    _accelMatrix;
	Vector3f _compassOffset;
	Matrix3f    _compassMatrix;
	// Rotation from measured board axes to airframe axes; identity until a
	// levelling calibration succeeds.
	Matrix3f    _boardRotation;

	// True once per _feedDivider calls. See CALIBRATOR_ACCEL_FEED_HZ.
	bool acceptFeedSample();

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
	void sendMatrix(const char *key, const Matrix3f &m);

	Calibrator_StatusTypeDef loadAccelCalibrationData();
	Calibrator_StatusTypeDef loadCompassCalibrationData();
	Calibrator_StatusTypeDef loadLevelCalibrationData();
	Calibrator_StatusTypeDef saveAccelCalibrationData();
	Calibrator_StatusTypeDef saveCompassCalibrationData();
	Calibrator_StatusTypeDef saveLevelCalibrationData();



	Calibrator_StatusTypeDef loadRecord(EEPROMLocation location,
			Vector3f &offset, Matrix3f &matrix, float *misalign_deg = nullptr);
	Calibrator_StatusTypeDef saveRecord(EEPROMLocation location,
			const Vector3f &offset, const Matrix3f &matrix,
			float misalign_deg = -1.0f);
};

// The one calibrator. Telemetry drives it; the sensor frontends feed and
// consult it. Defined in Globals.cpp.
extern Calibrator calibrator;

#endif /* CALIBRATORS_CALIBRATOR_HPP_ */
