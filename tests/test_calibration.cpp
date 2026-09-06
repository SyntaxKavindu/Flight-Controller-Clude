/*
 * The three calibration engines, driven directly (no Calibrator facade, no
 * telemetry, no EEPROM) -- which is also a standing check that they are usable
 * standalone in another project.
 */
#include "MathTypes.hpp"
#include "AccelerometerCalibrator.hpp"
#include "CompassCalibrator.hpp"
#include "LevelCalibrator.hpp"
#include "test.hpp"

using namespace phys;

// Deterministic pseudo-random in [-1,1]; no <random>, so the suite stays
// reproducible across toolchains.
static uint32_t g_seed = 12345u;
static float rnd()
{
    g_seed = g_seed * 1664525u + 1013904223u;
    return ((float)(g_seed >> 8) / 8388608.0f) - 1.0f;
}

// A distorted sensor: raw = A*(true) + bias, with A = scale + cross-axis terms.
static Vector3f distort(const Vector3f &t, const Vector3f &scale,
                        const Vector3f &bias, float cross)
{
    return Vector3f(scale.x * t.x + cross * t.y + bias.x,
                    scale.y * t.y + cross * t.z + bias.y,
                    scale.z * t.z + cross * t.x + bias.z);
}

// Points spread over a sphere, so a tumble sees real coverage.
static Vector3f spherePoint(int i, int n, float radius)
{
    const float k  = (float)i + 0.5f;
    const float ph = std::acos(1.0f - 2.0f * k / (float)n);
    const float th = 3.14159265f * (1.0f + 2.2360679775f) * k;
    return Vector3f(radius * std::cos(th) * std::sin(ph),
                    radius * std::sin(th) * std::sin(ph),
                    radius * std::cos(ph));
}

int main()
{
    section("Accelerometer: six position");
    {
        AccelerometerCalibrator c;
        const Vector3f scale(1.04f, 0.97f, 1.02f), bias(0.30f, -0.22f, 0.15f);
        c.beginSixPosition(0.5f);
        // The axis pointing UP reads +1 g -- an accelerometer reads specific force.
        const Vector3f truth[6] = {{G,0,0},{-G,0,0},{0,G,0},{0,-G,0},{0,0,G},{0,0,-G}};
        for (int p = 0; p < 6; p++) {
            c.startPosition((AccelPosition)p);
            for (int i = 0; i < 120; i++) {
                Vector3f s = distort(truth[p], scale, bias, 0.0f);
                s.x += 0.01f * rnd(); s.y += 0.01f * rnd(); s.z += 0.01f * rnd();
                c.addSample(s);
            }
        }
        check(c.allPositionsComplete(), "all six orientations captured");
        check(c.calibrate() == AccelCalStatus::SUCCESS, "six-position fit succeeds");
        checkNear(c.getBias().x, bias.x, 0.02f, "recovers X bias");
        checkNear(c.getBias().y, bias.y, 0.02f, "recovers Y bias");
        checkNear(c.getBias().z, bias.z, 0.02f, "recovers Z bias");
        checkNear(c.getScale().x / G, scale.x, 0.01f, "recovers X scale");
        checkNear(c.getScale().y / G, scale.y, 0.01f, "recovers Y scale");
        checkNear(c.getScale().z / G, scale.z, 0.01f, "recovers Z scale");
        // correct() must land a distorted reading back on the unit sphere.
        Vector3f got = c.correct(distort(Vector3f(0,0,-G), scale, bias, 0.0f));
        checkNear(got.length(), 1.0f, 0.02f, "corrected reading lands on 1 g");
    }
    {
        // Captured back-to-front: the axis that should read +1 g reads -1 g.
        // A magnitude-only check cannot see this, and accepting it flies inverted.
        AccelerometerCalibrator c;
        c.beginSixPosition(0.5f);
        const Vector3f swapped[6] = {{-G,0,0},{G,0,0},{0,-G,0},{0,G,0},{0,0,-G},{0,0,G}};
        for (int p = 0; p < 6; p++) {
            c.startPosition((AccelPosition)p);
            for (int i = 0; i < 120; i++) c.addSample(swapped[p]);
        }
        check(c.calibrate() == AccelCalStatus::FAILED_BAD_GEOMETRY,
              "orientations captured inverted are REJECTED, not silently negated");
    }
    {
        AccelerometerCalibrator c;
        c.beginSixPosition(0.5f);
        c.startPosition(AccelPosition::X_UP);
        for (int i = 0; i < 60; i++) c.addSample(Vector3f(G,0,0));
        check(c.calibrate() == AccelCalStatus::FAILED_NOT_ENOUGH_SAMPLES,
              "an incomplete sequence fails rather than fitting on partial data");
    }
    {
        // Motion must reset the running average, not pollute it.
        AccelerometerCalibrator c;
        c.beginSixPosition(0.5f);
        c.startPosition(AccelPosition::Z_DOWN);
        for (int i = 0; i < 50; i++) c.addSample(Vector3f(0,0,-G));
        AccelSampleResult r = c.addSample(Vector3f(0, 0, -G + 5.0f)); // a knock
        check(r == AccelSampleResult::REJECTED_MOTION, "a sample outside the gate is rejected");
        checkNear(c.getProgressPercent(), 0.0f, 0.01f, "the disturbed average is discarded");
    }

    section("Accelerometer: tumble (caller-supplied buffer)");
    {
        Vector3f buf[ACCEL_CAL_TUMBLE_MAX_SAMPLES];
        AccelerometerCalibrator c;
        check(!c.beginTumble(G, 0.2f, nullptr, 300), "a null buffer is refused");
        check(!c.beginTumble(G, 0.2f, buf, 10), "a buffer too small to finish is refused");
        check(c.beginTumble(G, 0.2f, buf, ACCEL_CAL_TUMBLE_MAX_SAMPLES), "a valid buffer starts");

        const Vector3f scale(1.05f, 0.96f, 1.01f), bias(0.25f, -0.30f, 0.18f);
        const int N = 400;
        for (int i = 0; i < N && !c.isReadyToCalibrate(); i++) {
            const Vector3f t = spherePoint(i % N, N, G);
            const Vector3f raw = distort(t, scale, bias, 0.02f);
            // Hold each orientation long enough to pass the stillness window.
            for (int h = 0; h < 15 && !c.isReadyToCalibrate(); h++) c.addSample(raw);
        }
        check(c.isReadyToCalibrate(), "tumble reaches coverage");
        check(c.calibrate() == AccelCalStatus::SUCCESS, "ellipsoid fit succeeds");
        checkNear(c.getBias().x, bias.x, 0.05f, "tumble recovers X bias");
        checkNear(c.getBias().z, bias.z, 0.05f, "tumble recovers Z bias");
        float worst = 0.0f;
        for (int i = 0; i < 200; i++) {
            const Vector3f t = spherePoint(i, 200, G);
            const Vector3f got = c.correct(distort(t, scale, bias, 0.02f));
            worst = std::fmax(worst, std::fabs(got.length() - 1.0f));
        }
        checkNear(worst, 0.0f, 0.02f, "corrected tumble readings land on the unit sphere");
    }
    {
        // Structureless input must be rejected, not fitted.
        Vector3f buf[ACCEL_CAL_TUMBLE_MAX_SAMPLES];
        AccelerometerCalibrator c;
        c.beginTumble(G, 5.0f, buf, ACCEL_CAL_TUMBLE_MAX_SAMPLES);
        for (int i = 0; i < 6000 && !c.isReadyToCalibrate(); i++)
            c.addSample(Vector3f(rnd()*G, rnd()*G, rnd()*G));
        const AccelCalStatus st = c.calibrate();
        check(st != AccelCalStatus::SUCCESS, "noise with no ellipsoidal structure is rejected");
    }

    section("Compass (caller-supplied buffer)");
    {
        Vector3f buf[COMPASS_CAL_MAX_SAMPLES];
        CompassCalibrator c;
        check(!c.begin(0.5f, nullptr, 300), "a null buffer is refused");
        check(!c.begin(0.5f, buf, 10), "a buffer too small to finish is refused");
        check(c.begin(0.5f, buf, COMPASS_CAL_MAX_SAMPLES), "a valid buffer starts");

        // Hard iron LARGER than the field itself -- the case that collapses a
        // fit binned about the origin.
        const Vector3f scale(1.10f, 0.92f, 1.03f), hard(0.75f, -0.60f, 0.40f);
        // Interleaved order: a real tumble revisits both hemispheres throughout,
        // so any prefix of the sweep is spread over the sphere. (Feeding the
        // spiral in index order instead walks pole to pole, and stopping early
        // on that IS a one-sided sweep -- correctly rejected, see below.)
        const int N = 1200;
        for (int i = 0; i < N && !c.isReadyToCalibrate(); i++) {
            const int idx = (int)(((long)i * 397L) % N);
            c.addSample(distort(spherePoint(idx, N, 0.5f), scale, hard, 0.03f));
        }
        check(c.isReadyToCalibrate(), "sweep reaches coverage despite heavy hard iron");
        check(c.calibrate() == CalStatus::SUCCESS, "compass fit succeeds");
        checkNear(c.getOffset().x, hard.x, 0.05f, "recovers X hard-iron offset");
        checkNear(c.getOffset().y, hard.y, 0.05f, "recovers Y hard-iron offset");
        checkNear(c.getOffset().z, hard.z, 0.05f, "recovers Z hard-iron offset");
        float worst = 0.0f;
        for (int i = 0; i < 200; i++) {
            const Vector3f got = c.correct(distort(spherePoint(i, 200, 0.5f), scale, hard, 0.03f));
            worst = std::fmax(worst, std::fabs(got.length() - 0.5f));
        }
        checkNear(worst, 0.0f, 0.03f, "corrected field lands on the nominal sphere");
    }
    {
        // A one-sided sweep has no coverage and must not complete.
        Vector3f buf[COMPASS_CAL_MAX_SAMPLES];
        CompassCalibrator c;
        c.begin(0.5f, buf, COMPASS_CAL_MAX_SAMPLES);
        // Long enough to exceed COMPASS_CAL_STALL_LIMIT, i.e. an operator who
        // keeps going for ~30 s at the mid-loop rate without ever inverting.
        for (int i = 0; i < 20000; i++) {
            Vector3f p = spherePoint((i * 397) % 300, 300, 0.5f);
            if (p.z < 0.0f) continue;             // never inverted
            c.addSample(p);
        }
        // Bin coverage alone IS satisfied by this sweep -- that is the
        // documented weakness of binning about the centroid, which a one-sided
        // cloud moves into the middle of its own cap. The scatter test is what
        // catches it, in both places it matters.
        check(c.getFilledBinsCount() >= COMPASS_CAL_MIN_BINS,
              "bin coverage alone is fooled by a one-sided sweep");
        check(!c.isReadyToCalibrate(),
              "... but the sweep is NOT reported ready, so collection continues");
        check(c.isStalled(),
              "... and it is reported stalled, which is what tells the operator to invert");
        check(c.calibrate() == CalStatus::FAILED_POOR_COVERAGE,
              "... and a forced fit on it is rejected, never silently accepted");
    }

    section("Level");
    {
        // FRD: a level, upright airframe reads (0,0,-g).
        const Vector3f level(0.0f, 0.0f, -G);
        for (int deg = 1; deg <= 10; deg += 3) {
            const float a = (float)deg * D2R, c_ = std::cos(a), s = std::sin(a);
            const Vector3f measured(level.x, c_*level.y - s*level.z, s*level.y + c_*level.z);
            LevelCalibrator lc;
            lc.begin(Vector3f(0,0,-1), G, 0.5f);
            for (int i = 0; i < LEVEL_CAL_SAMPLES + 10; i++) lc.addSample(measured);
            char msg[80];
            std::snprintf(msg, sizeof msg, "recovers a %d deg mounting tilt", deg);
            if (lc.calibrate() != LevelCalStatus::SUCCESS) { check(false, msg); continue; }
            const Vector3f corrected = lc.correct(measured);
            const float err = std::acos(std::fmin(1.0f, corrected.dot(level) / (corrected.length()*G))) * R2D;
            checkNear(err, 0.0f, 0.01f, msg);
            checkNear(lc.getTiltDeg(), (float)deg, 0.01f, "reports the tilt it measured");
        }
    }
    {
        LevelCalibrator lc;
        lc.begin(Vector3f(0,0,-1), G, 0.5f);
        // 30 deg is not a mounting error, it is a surface that is not level.
        const float a = 30.0f * D2R;
        const Vector3f tilted(0.0f, -std::sin(a)*-G, std::cos(a)*-G);
        for (int i = 0; i < LEVEL_CAL_SAMPLES + 10; i++) lc.addSample(tilted);
        check(lc.calibrate() == LevelCalStatus::FAILED_EXCESSIVE_TILT,
              "an implausibly large tilt is refused, not stored");
    }
    {
        LevelCalibrator lc;
        lc.begin(Vector3f(0,0,-1), G, 0.5f);
        for (int i = 0; i < LEVEL_CAL_SAMPLES + 10; i++) lc.addSample(Vector3f(0,0,-G*0.5f));
        check(lc.calibrate() == LevelCalStatus::FAILED_BAD_MAGNITUDE,
              "a reading that is not 1 g is refused (bad accel cal, or moving)");
    }
    {
        LevelCalibrator lc;
        lc.begin(Vector3f(0,0,-1), G, 0.5f);
        for (int i = 0; i < 50; i++) lc.addSample(Vector3f(0,0,-G));
        lc.addSample(Vector3f(0, 3.0f, -G));            // a knock
        checkNear(lc.getProgressPercent(), 0.0f, 0.01f, "motion discards the partial average");
        check(lc.calibrate() == LevelCalStatus::FAILED_NOT_ENOUGH_SAMPLES,
              "too few settled samples fails rather than fitting");
    }
    {
        // The yaw offset gravity cannot observe, supplied by the operator.
        LevelCalibrator lc;
        lc.begin(Vector3f(0,0,-1), G, 0.5f, 90.0f);
        for (int i = 0; i < LEVEL_CAL_SAMPLES + 10; i++) lc.addSample(Vector3f(0,0,-G));
        check(lc.calibrate() == LevelCalStatus::SUCCESS, "a yaw-only offset calibrates");
        const Vector3f fwd = lc.correct(Vector3f(1,0,0));
        checkNear(fwd.y, 1.0f, 0.001f, "90 deg yaw offset maps body +X onto +Y");
        checkNear(lc.correct(Vector3f(0,0,-G)).z, -G, 0.001f, "... and leaves the vertical alone");
    }

    return testReport("Calibration");
}
