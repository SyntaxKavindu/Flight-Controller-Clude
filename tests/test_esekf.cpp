/*
 * ESEKF: alignment, propagation, each fusion path, integrity reporting and the
 * numerical-fault recovery.
 */
#include "MathTypes.hpp"
#include "ESEKF.hpp"
#include "test.hpp"

using namespace phys;

static ESEKF makeLevelFilter(float declination_rad = 0.0f)
{
    ESEKF k;
    k.setMagneticDeclination(declination_rad);
    k.initialize(restAccel(0, 0), bodyField(0, 0, 0), 0.0f, Vector3f());
    return k;
}

// Hold the filter at a fixed true attitude for `seconds`, feeding consistent
// sensors at 400 Hz with the magnetometer at 80 Hz.
static void settle(ESEKF &k, float roll, float pitch, float yaw, float seconds)
{
    const int n = (int)(seconds * 400.0f);
    for (int i = 0; i < n; i++) {
        k.predict(Vector3f(0, 0, 0), restAccel(roll, pitch), 0.0025f);
        k.updateAccelerometer(restAccel(roll, pitch));
        if (i % 5 == 0) k.updateMagnetometer(bodyField(roll, pitch, yaw));
    }
}

int main()
{
    section("Alignment");
    {
        ESEKF k = makeLevelFilter();
        check(k.isInitialized(), "aligns from a level FRD sample");
        check(!k.hasDiverged(), "not diverged after alignment");
        Vector3f e = k.getEulerAngles();
        checkNear(e.x * R2D, 0.0f, 0.01f, "roll = 0 when level");
        checkNear(e.y * R2D, 0.0f, 0.01f, "pitch = 0 when level");
        checkNear(e.z * R2D, 0.0f, 0.01f, "yaw = 0 facing magnetic north");
    }
    {
        // Alignment must recover the attitude it was given, at a real angle.
        const float r = 20.0f * D2R, p = -15.0f * D2R, y = 100.0f * D2R;
        ESEKF k; k.setMagneticDeclination(0.0f);
        k.initialize(restAccel(r, p), bodyField(r, p, y), 0.0f, Vector3f());
        Vector3f e = k.getEulerAngles();
        checkNear(e.x * R2D,  20.0f, 0.1f, "alignment recovers roll");
        checkNear(e.y * R2D, -15.0f, 0.1f, "alignment recovers pitch");
        checkNear(e.z * R2D, 100.0f, 0.1f, "alignment recovers yaw (tilt-compensated)");
    }
    {
        ESEKF k;
        k.initialize(Vector3f(NAN, 0, 0), Vector3f(1, 0, 0), 0.0f, Vector3f());
        check(!k.isInitialized(), "refuses a non-finite accelerometer sample");
        ESEKF k2;
        k2.initialize(Vector3f(0, 0, 0), Vector3f(1, 0, 0), 0.0f, Vector3f());
        check(!k2.isInitialized(), "refuses a dead (zero) accelerometer");
        ESEKF k3;
        // lat/lon in DEGREES is the classic caller error; radians are required.
        k3.initialize(restAccel(0,0), bodyField(0,0,0), 0.0f, Vector3f(51.5f, -0.1f, 0));
        check(!k3.isInitialized(), "refuses lat/lon that cannot be radians");
        ESEKF k4;
        k4.initialize(restAccel(0,0), Vector3f(0,0,0), 0.0f, Vector3f());
        check(k4.isInitialized(), "aligns WITHOUT a magnetometer (roll/pitch only)");
    }

    section("Propagation");
    {
        ESEKF k = makeLevelFilter();
        // A stationary vehicle must stay stationary: gravity in, no drift out.
        for (int i = 0; i < 4000; i++) {
            k.predict(Vector3f(0, 0, 0), restAccel(0, 0), 0.0025f);
        }
        Vector3f v = k.getVelocity(), e = k.getEulerAngles();
        checkNear(v.z, 0.0f, 0.01f, "10 s at rest: no vertical velocity drift");
        checkNear(e.x * R2D, 0.0f, 0.01f, "10 s at rest: attitude holds");
        check(!k.hasDiverged(), "10 s of pure propagation stays healthy");
    }
    {
        // Integrate a known body rotation and check the attitude lands right.
        ESEKF k = makeLevelFilter();
        const float rate = 30.0f * D2R; // 30 deg/s about body X for 1 s
        for (int i = 0; i < 400; i++) {
            k.predict(Vector3f(rate, 0, 0), restAccel(0, 0), 0.0025f);
        }
        checkNear(k.getEulerAngles().x * R2D, 30.0f, 0.2f, "gyro integrates to 30 deg roll");
    }
    {
        ESEKF k = makeLevelFilter();
        Vector3f before = k.getEulerAngles();
        k.predict(Vector3f(NAN, 0, 0), restAccel(0,0), 0.0025f);
        k.predict(Vector3f(0, 0, 0), restAccel(0,0), NAN);
        k.predict(Vector3f(0, 0, 0), restAccel(0,0), -1.0f);
        k.predict(Vector3f(0, 0, 0), restAccel(0,0), 0.0f);
        Vector3f after = k.getEulerAngles();
        check(before.x == after.x && before.y == after.y && before.z == after.z,
              "non-finite / non-positive dt and gyro are rejected, state untouched");
        check(!k.hasDiverged(), "rejected inputs do not diverge the filter");
    }

    section("Accelerometer fusion");
    {
        // Start with a deliberate 10 deg roll error and let gravity pull it out.
        ESEKF k; k.setMagneticDeclination(0.0f);
        k.initialize(restAccel(10.0f * D2R, 0), bodyField(0,0,0), 0.0f, Vector3f());
        settle(k, 0, 0, 0, 20.0f);   // truth is level; the estimate started tilted
        checkNear(k.getEulerAngles().x * R2D, 0.0f, 0.5f, "gravity corrects a 10 deg tilt error");
    }
    {
        ESEKF k = makeLevelFilter();
        k.setAccelGateThreshold(1.0f);
        k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
        check(!k.updateAccelerometer(Vector3f(0, 0, -G - 5.0f)),
              "a sample far outside the motion gate is rejected");
        checkNear(k.getAccelMotion(), 5.0f, 1e-3f, "motion metric reports the rejected sample");
        check(k.updateAccelerometer(Vector3f(0, 0, -G)), "a clean sample is fused");
        checkNear(k.getLastAccelNoiseInflation(), 1.0f, 1e-3f, "clean sample: nominal noise");
        k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
        k.updateAccelerometer(Vector3f(0, 0, -G - 0.5f)); // halfway into the gate
        checkNear(k.getLastAccelNoiseInflation(), 1.0f + 9.0f * 0.25f, 0.01f,
                  "half-gate sample: noise inflated by 1 + 9*(1/2)^2");
        check(!k.updateAccelerometer(Vector3f(NAN, 0, 0)), "a NaN sample is rejected");
    }

    section("Magnetometer: heading only");
    {
        ESEKF k = makeLevelFilter();
        settle(k, 0, 0, 40.0f * D2R, 10.0f);
        Vector3f e = k.getEulerAngles();
        checkNear(e.z * R2D, 40.0f, 1.0f, "yaw converges to the true heading");
        checkNear(e.x * R2D, 0.0f, 0.3f, "roll stays level while yaw slews");
        checkNear(e.y * R2D, 0.0f, 0.3f, "pitch stays level while yaw slews");
    }
    {
        // THE regression: a magnetic disturbance must not touch roll or pitch.
        // The original 3-axis fusion moved roll by 3.6 deg on this input.
        ESEKF k = makeLevelFilter();
        settle(k, 0, 0, 0, 1.0f);
        Vector3f before = k.getEulerAngles();
        for (int i = 0; i < 400; i++) {           // no accel updates: nothing can mask a tilt
            k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
            k.updateMagnetometer(Vector3f(0.10f, 0.30f, -0.60f));
        }
        Vector3f after = k.getEulerAngles();
        checkNear((after.x - before.x) * R2D, 0.0f, 0.02f, "magnetic slam cannot move ROLL");
        checkNear((after.y - before.y) * R2D, 0.0f, 0.02f, "magnetic slam cannot move PITCH");
        check(std::fabs((after.z - before.z) * R2D) > 5.0f, "... it moves yaw, which is bounded");
    }
    {
        const float r = 25.0f * D2R, p = -18.0f * D2R, y = 70.0f * D2R;
        ESEKF k; k.setMagneticDeclination(0.0f);
        k.initialize(restAccel(r, p), bodyField(r, p, y), 0.0f, Vector3f());
        settle(k, r, p, y, 10.0f);
        Vector3f e = k.getEulerAngles();
        checkNear(e.z * R2D, 70.0f, 1.0f, "heading correct at 25 deg roll / -18 deg pitch");
        checkNear(e.x * R2D, 25.0f, 0.5f, "roll held while fusing heading");
        checkNear(e.y * R2D, -18.0f, 0.5f, "pitch held while fusing heading");
    }
    {
        ESEKF k = makeLevelFilter(-12.0f * D2R);
        checkNear(k.getEulerAngles().z * R2D, -12.0f, 0.2f,
                  "declination makes alignment yaw a TRUE heading");
    }
    {
        ESEKF k = makeLevelFilter();
        k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
        check(!k.updateMagnetometer(Vector3f(0, 0, 0)), "zero field refused");
        check(!k.updateMagnetometer(Vector3f(NAN, 0, 0)), "NaN field refused");
        check(!k.updateMagnetometer(Vector3f(0, 0, 0.5f)), "vertical field carries no heading");
    }

    section("Barometer fusion");
    {
        ESEKF k; k.setMagneticDeclination(0.0f);
        k.initialize(restAccel(0,0), bodyField(0,0,0), 100.0f, Vector3f());
        checkNear(k.getAltitude(), 0.0f, 0.01f, "altitude is relative to the alignment point");
        for (int i = 0; i < 4000; i++) {
            k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
            k.updateAccelerometer(restAccel(0,0));
            if (i % 8 == 0) k.updateBarometer(125.0f);   // climbed 25 m
        }
        checkNear(k.getAltitude(), 25.0f, 0.5f, "tracks a 25 m climb above the datum");
        check(k.getStatus().vert_pos_valid, "height aiding reported valid while fusing");
    }
    {
        ESEKF k; k.setMagneticDeclination(0.0f);
        k.initialize(restAccel(0,0), bodyField(0,0,0), 100.0f, Vector3f());
        for (int i = 0; i < 400; i++) { k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
                                        k.updateBarometer(100.0f); }
        // A huge step is gated out at first, then force-fused once the source
        // times out -- the lockout escape. Without it the baro is lost for ever.
        int fused = 0;
        for (int i = 0; i < 4000; i++) {
            k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
            if (k.updateBarometer(600.0f)) fused++;
        }
        check(fused > 0, "a gated-out barometer recovers via the timeout escape");
        check(!k.hasDiverged(), "the recovery does not diverge the filter");
    }

    section("Integrity reporting");
    {
        ESEKF k = makeLevelFilter();
        settle(k, 0, 0, 0, 1.0f);
        ESEKFStatus st = k.getStatus();
        check(st.attitude_valid, "attitude reported valid");
        check(st.mag_aiding, "mag aiding reported while fusing");
        check(st.dead_reckoning, "dead reckoning reported with no GPS -- position is inertial");
        check(!st.horiz_pos_valid, "horizontal position NOT claimed without GPS");
        // Stop feeding the magnetometer; aiding must time out and say so.
        for (int i = 0; i < 4000; i++) k.predict(Vector3f(0,0,0), restAccel(0,0), 0.0025f);
        check(!k.getStatus().mag_aiding, "mag aiding drops out after the timeout");
    }

    section("Numerical fault recovery");
    {
        ESEKF k = makeLevelFilter();
        settle(k, 0, 0, 25.0f * D2R, 5.0f);
        Vector3f before = k.getEulerAngles();
        check(k.getFaultCount() == 0, "no faults during normal operation");

        // Drive P to the edge of float range with a legal value; the next
        // F P F^T then overflows. Every public setter screens its input, so
        // this is how a fault actually arises rather than being injected.
        float P[ESEKF_STATE_DIM][ESEKF_STATE_DIM];
        k.getCovariance(P);
        for (int i = 0; i < ESEKF_STATE_DIM; i++)
            for (int j = 0; j < ESEKF_STATE_DIM; j++) P[i][j] = (i == j) ? 3.0e38f : 0.0f;
        k.setCovariance(P);
        k.predict(Vector3f(0.5f, 0.5f, 0.5f), restAccel(0,0), 0.0025f);

        check(k.getFaultCount() >= 1, "the fault is counted");
        check(!k.hasDiverged(), "the filter repairs instead of latching");
        Vector3f after = k.getEulerAngles();
        checkNear((after.z - before.z) * R2D, 0.0f, 1.0f, "yaw survives the repair");
        checkNear(after.x * R2D, before.x * R2D, 1.0f, "roll survives the repair");
        check(k.isInitialized(), "still initialized after the repair");
        settle(k, 0, 0, 25.0f * D2R, 2.0f);
        checkNear(k.getEulerAngles().z * R2D, 25.0f, 1.0f, "converges again after the repair");
    }
    {
        // A saturated accelerometer must not align: the DIRECTION is plausible,
        // so only a magnitude check can catch it.
        ESEKF k;
        k.initialize(Vector3f(1e30f, 1e30f, 1e30f), Vector3f(1, 0, 0), 0.0f, Vector3f());
        check(!k.isInitialized(), "a saturated accelerometer is refused at alignment");
        ESEKF k2;
        k2.initialize(Vector3f(0.5f, 0.0f, -0.5f), Vector3f(1, 0, 0), 0.0f, Vector3f());
        check(!k2.isInitialized(), "a reading nowhere near 1 g is refused");
        ESEKF k3;
        // ... but a poorly calibrated part still 20% off must be accepted.
        k3.initialize(Vector3f(0.0f, 0.0f, -G * 1.2f), phys::bodyField(0,0,0), 0.0f, Vector3f());
        check(k3.isInitialized(), "a 20% scale error still aligns (the band is loose)");
    }

    section("Setter validation");
    {
        ESEKF k = makeLevelFilter();
        const float good = k.getAccelGateThreshold();
        k.setAccelGateThreshold(NAN);
        checkNear(k.getAccelGateThreshold(), good, 1e-6f, "NaN gate threshold rejected");
        const float baro = k.getBaroNoise();
        k.setBaroNoise(-1.0f);
        checkNear(k.getBaroNoise(), baro, 1e-6f, "negative baro variance rejected");
        Vector3f g0 = k.getGravity();
        k.setGravity(Vector3f(NAN, 0, 0));
        checkNear(k.getGravity().z, g0.z, 1e-6f, "NaN gravity rejected");
        float Rbad[3][3] = {{NAN,0,0},{0,1,0},{0,0,1}};
        float Rwas[3][3]; k.getAccelNoise(Rwas);
        k.setAccelNoise(Rbad);
        float Rnow[3][3]; k.getAccelNoise(Rnow);
        checkNear(Rnow[0][0], Rwas[0][0], 1e-9f, "NaN measurement noise rejected");
    }

    return testReport("ESEKF");
}
