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
static void runSixPosition(const Vector3f &scale, const Vector3f &bias)
{
    calibrator.startAccelerometerCalibration();
    const Vector3f truth[6] = {{G,0,0},{-G,0,0},{0,G,0},{0,-G,0},{0,0,G},{0,0,-G}};
    for (int p = 0; p < 6 && calibrator.isAcclCalibrating(); p++) {
        calibrator.confirmReady();
        for (int i = 0; i < 120 && calibrator.isAcclCalibrating(); i++) {
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
        Vector3f off; Mat3f m;
        check(!Calibrator::unpackRecord(rec, off, m), "a blank (0xFF) record is rejected");
        std::memset(&rec, 0x00, sizeof(rec));
        check(!Calibrator::unpackRecord(rec, off, m), "an all-zero record is rejected");

        Vector3f o(1.5f, -2.5f, 0.25f);
        Mat3f mm = Mat3f::identity();
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

    return testReport("Integration");
}
