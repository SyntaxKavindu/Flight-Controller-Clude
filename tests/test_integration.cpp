/*
 * The Calibrator facade against the sensor frontends and the estimator: the
 * wiring, ordering and stand-down behaviour that the engine tests cannot see.
 */
#include "Globals.hpp"
#include "ESEKF.hpp"
#include "test.hpp"

using namespace phys;

// Drive one accelerometer calibration to completion through the facade, so the
// board-frame and level tests have the prerequisite they need.
// calls_per_position is CALLS, not accepted samples: with the feed decimated
// (see CALIBRATOR_ACCEL_FEED_HZ) a position needs the divider times as many.
static void runSixPosition(const Vector3f &scale, const Vector3f &bias,
                           int calls_per_position = 120)
{
    calibrator.startAccelerometerCalibration();
    const Vector3f truth[6] = {{G,0,0},{-G,0,0},{0,G,0},{0,-G,0},{0,0,G},{0,0,-G}};
    for (int p = 0; p < 6 && calibrator.isAcclCalibrating(); p++) {
        calibrator.confirmReady();
        for (int i = 0; i < calls_per_position && calibrator.isAcclCalibrating(); i++) {
            Vector3f s(scale.x * truth[p].x + bias.x,
                       scale.y * truth[p].y + bias.y,
                       scale.z * truth[p].z + bias.z);
            calibrator.calibrateAccelerometer(s);
        }
    }
}

int main()
{
    section("Facade: procedure exclusion and epoch");
    {
        calibrator.init(nullptr);   // no EEPROM: gains live for this session only
        check(!calibrator.isCalibrating(), "starts idle");
        check(!calibrator.isAcclCalibrated(), "starts uncalibrated with no storage");

        calibrator.startCompassCalibration();
        check(calibrator.isCompassCalibrating(), "compass calibration starts");
        calibrator.startAccelerometerCalibration();
        check(!calibrator.isAcclCalibrating(), "a second procedure is refused while one runs");
        calibrator.startLevelCalibration();
        check(!calibrator.isLevelCalibrating(), "levelling is refused too");
        check(calibrator.isCalibrating(), "isCalibrating() covers all three");
        calibrator.cancelCalibration();
        check(!calibrator.isCalibrating(), "cancel clears every procedure");
    }
    {
        const uint32_t e0 = calibrator.getCalibrationEpoch();
        runSixPosition(Vector3f(1.03f, 0.98f, 1.01f), Vector3f(0.2f, -0.15f, 0.1f));
        check(calibrator.isAcclCalibrated(), "six-position completes through the facade");
        check(calibrator.getCalibrationEpoch() != e0, "epoch bumps when gains change");
        const uint32_t e1 = calibrator.getCalibrationEpoch();
        calibrator.clearStoredCalibration();
        check(calibrator.getCalibrationEpoch() != e1, "epoch bumps when calibration is erased");
        check(!calibrator.isAcclCalibrated(), "erasing clears the applied gains");
    }

    section("Facade: correction is applied, in the right order");
    {
        calibrator.init(nullptr);
        const Vector3f scale(1.03f, 0.98f, 1.01f), bias(0.2f, -0.15f, 0.1f);
        runSixPosition(scale, bias);
        // A level airframe: raw reading distorted by the sensor, corrected back.
        Vector3f raw(scale.x * 0.0f + bias.x, scale.y * 0.0f + bias.y,
                     scale.z * -G + bias.z);
        calibrator.correctAcclData(raw);
        checkNear(raw.length(), G, 0.02f, "corrected accel is restored to 1 g in m/s^2");
        checkNear(raw.z, -G, 0.05f, "... and points down the body Z axis");
    }

    section("Level calibration through the facade");
    {
        calibrator.init(nullptr);
        runSixPosition(Vector3f(1,1,1), Vector3f(0,0,0));   // ideal sensor
        check(calibrator.isAcclCalibrated(), "accelerometer calibrated first");

        // Board bolted in 3 degrees out about the roll axis.
        const float a = 3.0f * D2R, c = std::cos(a), s = std::sin(a);
        const Vector3f level(0.0f, 0.0f, -G);
        const Vector3f measured(level.x, c*level.y - s*level.z, s*level.y + c*level.z);

        calibrator.startLevelCalibration();
        check(calibrator.isLevelCalibrating(), "levelling starts once accel is calibrated");
        for (int i = 0; i < 400 && calibrator.isLevelCalibrating(); i++) {
            Vector3f v = measured;
            calibrator.calibrateLevel(v);
        }
        check(calibrator.isLevelCalibrated(), "levelling completes");

        Vector3f corrected = measured;
        calibrator.correctBoardFrame(corrected);
        const float err = std::acos(std::fmin(1.0f, corrected.dot(level) / (corrected.length()*G))) * R2D;
        checkNear(err, 0.0f, 0.02f, "board rotation removes the mounting error");
    }
    {
        calibrator.init(nullptr);
        calibrator.startLevelCalibration();
        check(!calibrator.isLevelCalibrating(),
              "levelling refuses to run without an accelerometer calibration");
    }

    section("Sensor frontends apply the board rotation");
    {
        // The rotation describes the BOARD, so it must reach the gyro too --
        // an accelerometer in one frame and a gyro in another drifts only under
        // motion, which is the hardest failure to attribute.
        calibrator.init(nullptr);
        runSixPosition(Vector3f(1,1,1), Vector3f(0,0,0));
        const float a = 5.0f * D2R, c = std::cos(a), s = std::sin(a);
        const Vector3f level(0.0f, 0.0f, -G);
        const Vector3f measured(level.x, c*level.y - s*level.z, s*level.y + c*level.z);
        calibrator.startLevelCalibration();
        for (int i = 0; i < 400 && calibrator.isLevelCalibrating(); i++) {
            Vector3f v = measured; calibrator.calibrateLevel(v);
        }
        check(calibrator.isLevelCalibrated(), "levelling completed for the frame test");

        // Same rotation must be applied to any vector handed to correctBoardFrame,
        // gyro included -- verified by construction: one function, one matrix.
        Vector3f gyro(0.1f, 0.2f, 0.3f), accel(0.1f, 0.2f, 0.3f);
        calibrator.correctBoardFrame(gyro);
        calibrator.correctBoardFrame(accel);
        check(gyro.x == accel.x && gyro.y == accel.y && gyro.z == accel.z,
              "gyro and accelerometer get an identical board rotation");
        checkNear(gyro.length(), 0.374166f, 1e-4f, "a rotation preserves magnitude");
    }

    section("EEPROM record round-trip");
    {
        // Pack/unpack must reject anything it did not write: a blank device, a
        // half-written record, or a payload that is finite-looking garbage.
        CalibrationRecord rec;
        std::memset(&rec, 0xFF, sizeof(rec));       // erased EEPROM reads as 0xFF
        Vector3f off; Matrix3f m;
        check(!Calibrator::unpackRecord(rec, off, m), "a blank (0xFF) record is rejected");
        std::memset(&rec, 0x00, sizeof(rec));
        check(!Calibrator::unpackRecord(rec, off, m), "an all-zero record is rejected");

        Vector3f o(1.5f, -2.5f, 0.25f);
        Matrix3f mm = Matrix3f::identity();
        mm.m[0][1] = 0.125f; mm.m[2][0] = -0.75f;
        Calibrator::packRecord(o, mm, rec);
        check(Calibrator::unpackRecord(rec, off, m), "a well-formed record round-trips");
        checkNear(off.x, o.x, 1e-6f, "offset survives the round trip");
        checkNear(m.m[0][1], 0.125f, 1e-6f, "matrix survives the round trip");
        // Flip one bit anywhere in the payload: the CRC must catch it.
        Calibrator::packRecord(o, mm, rec);
        rec.matrix[4] = rec.matrix[4] + 1.0f;
        check(!Calibrator::unpackRecord(rec, off, m), "a corrupted payload fails the CRC");
        Calibrator::packRecord(o, mm, rec);
        rec.version = CALIBRATION_RECORD_VERSION + 1;
        check(!Calibrator::unpackRecord(rec, off, m), "a future record version is rejected");
        Calibrator::packRecord(o, mm, rec);
        rec.offset[1] = NAN;
        check(!Calibrator::unpackRecord(rec, off, m), "a NaN payload is rejected even with a good CRC");
    }

    section("Stored cross-axis misalignment");
    {
        CalibrationRecord rec;
        Vector3f o(1.0f, 2.0f, 3.0f), off;
        Matrix3f mm = Matrix3f::identity(), m;
        float got = -99.0f;

        // Round-trips at 0.1 deg resolution across the useful range.
        static const float degs[] = {0.0f, 0.4f, 1.2f, 5.0f, 25.4f};
        for (float deg : degs) {
            Calibrator::packRecord(o, mm, rec, deg);
            got = -99.0f;
            check(Calibrator::unpackRecord(rec, off, m, &got), "record with misalignment round-trips");
            checkNear(got, deg, 0.06f, "misalignment survives storage");
        }

        // -1 in, -1 out: "not applicable" must not become "zero".
        Calibrator::packRecord(o, mm, rec, -1.0f);
        got = -99.0f;
        Calibrator::unpackRecord(rec, off, m, &got);
        checkNear(got, -1.0f, 0.001f, "a tumble fit stores 'not recorded', not 0.00");

        // THE compatibility case: a record written before this field existed
        // had that byte as zero. It must decode as absent, never as a perfect
        // 0.00 deg -- which would read as the best calibration possible.
        Calibrator::packRecord(o, mm, rec, 3.0f);
        rec.misalign_deci = 0;                 // as an older record would have it
        rec.crc = Calibrator::recordCrc(rec);  // ... with its own valid CRC
        got = -99.0f;
        check(Calibrator::unpackRecord(rec, off, m, &got), "a pre-existing record still loads");
        checkNear(got, -1.0f, 0.001f, "... and reports 'not recorded' rather than 0.00 deg");

        // The field sits ahead of crc, so corruption of it is caught.
        Calibrator::packRecord(o, mm, rec, 3.0f);
        rec.misalign_deci = 99;
        check(!Calibrator::unpackRecord(rec, off, m, &got),
              "a corrupted misalignment byte fails the CRC");

        // Out-of-range input saturates rather than wrapping to a small angle.
        Calibrator::packRecord(o, mm, rec, 900.0f);
        Calibrator::unpackRecord(rec, off, m, &got);
        check(got > 25.0f, "an absurd misalignment saturates high, it does not wrap");
    }

    section("Accelerometer feed decimation");
    {
        // Every sample-count gate in AccelerometerCalibrator and LevelCalibrator
        // was characterised against a 200 Hz feed, and neither class measures
        // time. Driven from a 1 kHz control loop they all run 5x short -- most
        // damagingly the tumble stall limit, which then aborts a run that a
        // real operator is completing normally. See CALIBRATOR_ACCEL_FEED_HZ.
        calibrator.init(nullptr);
        check(calibrator.getAccelFeedDivider() == 1,
              "undecimated by default, so an existing 200 Hz caller is unaffected");

        calibrator.setAccelFeedRate(1000);
        check(calibrator.getAccelFeedDivider() == 5, "1 kHz feed decimates by 5");
        calibrator.setAccelFeedRate(981);   // the rate this airframe measures
        check(calibrator.getAccelFeedDivider() == 5, "981 Hz rounds to 5, not 4");
        calibrator.setAccelFeedRate(500);
        check(calibrator.getAccelFeedDivider() == 3, "500 Hz rounds to nearest (166 Hz)");
        calibrator.setAccelFeedRate(200);
        check(calibrator.getAccelFeedDivider() == 1, "a caller already at 200 Hz is left alone");
        calibrator.setAccelFeedRate(50);
        check(calibrator.getAccelFeedDivider() == 1, "a slower caller is never upsampled");
        calibrator.setAccelFeedRate(0);
        check(calibrator.getAccelFeedDivider() == 1, "zero does not divide by zero");
    }
    {
        // Changing the divider mid-run would judge the rest of a procedure
        // against a different stillness window and stall timeout than the part
        // already collected.
        calibrator.init(nullptr);
        calibrator.setAccelFeedRate(1000);
        calibrator.startCompassCalibration();
        calibrator.setAccelFeedRate(200);
        check(calibrator.getAccelFeedDivider() == 5, "the divider is frozen while a run is live");
        calibrator.cancelCalibration();
        calibrator.setAccelFeedRate(200);
        check(calibrator.getAccelFeedDivider() == 1, "... and settable again once idle");
    }
    {
        // The behavioural proof: with a divider of 5 a position needs 5x the
        // calls, and lands on exactly 5x -- which is only true if the phase is
        // reset at the start of the run rather than left wherever it was.
        calibrator.init(nullptr);
        calibrator.setAccelFeedRate(1000);
        calibrator.startAccelerometerCalibration();
        calibrator.confirmReady();
        // Phase 0 is the accepted one, so accepted-count after N calls is
        // ceil(N/5) and the 100th lands on call 5*99+1 = 496. Landing exactly
        // there is what proves the phase was reset at the start of the run:
        // left wherever the previous procedure abandoned it, the first accepted
        // sample would slip by up to four calls.
        for (int i = 0; i < 495; i++) { Vector3f s(G, 0, 0); calibrator.calibrateAccelerometer(s); }
        check(!calibrator.isAwaitingPosition(),
              "495 calls at div=5 is one accepted sample short of the position");
        Vector3f s(G, 0, 0);
        calibrator.calibrateAccelerometer(s);
        check(calibrator.isAwaitingPosition(),
              "the 496th call completes it -- 100 accepted samples, exactly 5x");
        calibrator.cancelCalibration();
    }
    {
        // Decimation must not change the ANSWER, only the rate: same sensor,
        // same fit, whether or not samples are being dropped on the way in.
        const Vector3f scale(1.03f, 0.98f, 1.01f), bias(0.2f, -0.15f, 0.1f);
        Vector3f probe(bias.x, bias.y, scale.z * -G + bias.z);

        calibrator.init(nullptr);
        check(calibrator.getAccelFeedDivider() == 5,
              "init() leaves the divider alone -- it is caller wiring, not calibration state");
        calibrator.setAccelFeedRate(200);               // div = 1
        runSixPosition(scale, bias);
        Vector3f undecimated = probe;
        calibrator.correctAcclData(undecimated);

        calibrator.init(nullptr);
        calibrator.setAccelFeedRate(1000);              // div = 5
        runSixPosition(scale, bias, 600);
        check(calibrator.isAcclCalibrated(), "six-position still completes when decimated");
        Vector3f decimated = probe;
        calibrator.correctAcclData(decimated);

        checkNear(decimated.x, undecimated.x, 1e-5f, "decimation leaves the X gain identical");
        checkNear(decimated.y, undecimated.y, 1e-5f, "... and Y");
        checkNear(decimated.z, undecimated.z, 1e-5f, "... and Z");
    }
    {
        // The levelling fit is on the same feed and has the same kind of gate,
        // so it is decimated too -- LEVEL_CAL_SAMPLES is a count, not a time.
        calibrator.init(nullptr);
        calibrator.setAccelFeedRate(1000);
        runSixPosition(Vector3f(1,1,1), Vector3f(0,0,0), 600);

        const float a = 3.0f * D2R, c = std::cos(a), sn = std::sin(a);
        const Vector3f level(0.0f, 0.0f, -G);
        const Vector3f measured(level.x, c*level.y - sn*level.z, sn*level.y + c*level.z);

        calibrator.startLevelCalibration();
        for (int i = 0; i < 400 && calibrator.isLevelCalibrating(); i++) {
            Vector3f v = measured; calibrator.calibrateLevel(v);
        }
        check(!calibrator.isLevelCalibrated(),
              "400 calls at div=5 is short of LEVEL_CAL_SAMPLES -- the gate really is decimated");
        for (int i = 0; i < 1200 && calibrator.isLevelCalibrating(); i++) {
            Vector3f v = measured; calibrator.calibrateLevel(v);
        }
        check(calibrator.isLevelCalibrated(), "... and completes once enough samples arrive");

        Vector3f corrected = measured;
        calibrator.correctBoardFrame(corrected);
        const float err = std::acos(std::fmin(1.0f, corrected.dot(level) / (corrected.length()*G))) * R2D;
        checkNear(err, 0.0f, 0.02f, "the decimated levelling fit is just as accurate");
    }

    section("CALIMU,LEVEL chains levelling onto the six-position run");
    // init() deliberately does not clear the divider (see above), and the
    // decimation section left it at 5. These blocks count calls, so put it
    // back rather than quietly running them five times short.
    calibrator.setAccelFeedRate(200);
    {
        // The sequence ends on Z_DOWN -- +Z (the body DOWN axis) pointing down,
        // i.e. the airframe upright reading (0,0,-g). That is exactly what the
        // levelling fit wants, and the airframe is already still in it, so the
        // level run can start with no operator input at all.
        calibrator.init(nullptr);

        // A board bolted in 3 degrees out about roll. The six-position truth
        // vectors are what the SENSOR sees, so the mounting tilt shows up in
        // the Z_DOWN reading -- which is the whole point: that residual is
        // what levelling is there to measure.
        const float a = 3.0f * D2R, c = std::cos(a), sn = std::sin(a);
        auto tilt = [&](const Vector3f &v) {
            return Vector3f(v.x, c*v.y - sn*v.z, sn*v.y + c*v.z);
        };

        calibrator.startAccelerometerCalibration(true);
        const Vector3f truth[6] = {{G,0,0},{-G,0,0},{0,G,0},{0,-G,0},{0,0,G},{0,0,-G}};
        for (int p = 0; p < 6 && calibrator.isAcclCalibrating(); p++) {
            calibrator.confirmReady();
            for (int i = 0; i < 120 && calibrator.isAcclCalibrating(); i++) {
                Vector3f smp = truth[p];
                calibrator.calibrateAccelerometer(smp);
            }
        }
        check(calibrator.isAcclCalibrated(), "the six-position fit still completes");
        check(calibrator.isLevelCalibrating(),
              "levelling starts by itself -- no second command");
        check(calibrator.isCalibrating(), "isCalibrating() covers the chained run");

        // The operator has not moved anything: keep feeding the Z_DOWN reading.
        const Vector3f measured = tilt(Vector3f(0.0f, 0.0f, -G));
        for (int i = 0; i < 400 && calibrator.isLevelCalibrating(); i++) {
            Vector3f v = measured; calibrator.calibrateLevel(v);
        }
        check(calibrator.isLevelCalibrated(), "the chained levelling completes");

        Vector3f corrected = measured;
        calibrator.correctBoardFrame(corrected);
        const Vector3f level(0.0f, 0.0f, -G);
        const float err = std::acos(std::fmin(1.0f,
                corrected.dot(level) / (corrected.length()*G))) * R2D;
        checkNear(err, 0.0f, 0.02f, "... and removes the mounting tilt it was chained for");
    }
    {
        // Default is unchanged: a plain CALIMU must not silently promote
        // whatever surface it ran on into the definition of level.
        calibrator.init(nullptr);
        runSixPosition(Vector3f(1,1,1), Vector3f(0,0,0));
        check(calibrator.isAcclCalibrated(), "plain six-position completes");
        check(!calibrator.isLevelCalibrating(), "plain CALIMU does NOT chain");
        check(!calibrator.isCalibrating(), "... and leaves the facade idle");
    }
    {
        // A cancelled run must not leave a chain armed for whatever runs next.
        calibrator.init(nullptr);
        calibrator.startAccelerometerCalibration(true);
        calibrator.cancelCalibration();
        check(!calibrator.isCalibrating(), "the armed run cancels cleanly");
        runSixPosition(Vector3f(1,1,1), Vector3f(0,0,0));
        check(calibrator.isAcclCalibrated(), "a later plain run completes");
        check(!calibrator.isLevelCalibrating(),
              "... and does not inherit the cancelled run's chain");
    }
    {
        // A tumble ends in whatever orientation the operator stopped in, which
        // is not a level reference. The flag is never set on that path, but the
        // mode is re-checked in finishAccelCalibration() rather than trusted.
        calibrator.init(nullptr);
        calibrator.startAccelerometerTumbleCalibration();
        check(calibrator.isAcclCalibrating(), "tumble starts");
        calibrator.cancelCalibration();
        check(!calibrator.isLevelCalibrating(), "a tumble never chains levelling");
    }

    return testReport("Integration");
}
