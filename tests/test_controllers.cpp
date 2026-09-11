/*
 * The control cascade's implemented half: PID, Position, Velocity, Attitude.
 *
 * Written against the PHYSICS rather than against the implementation wherever
 * possible -- a test that restates the code cannot catch the code being wrong.
 * So the hover checks compute what a multirotor must do from g and the lean
 * angle, and compare; they do not re-derive it the way Velocity.cpp does.
 */
#include "Attitude.hpp"
#include "PID.hpp"
#include "Position.hpp"
#include "Velocity.hpp"
#include "test.hpp"

using namespace phys;

namespace {

bool finiteVec(const Vector3f &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Run a PID to steady state against a fixed target and measurement.
float settle(PID &pid, float target, float meas, int n, float dt)
{
    float out = 0.0f;
    for (int i = 0; i < n; i++) out = pid.update(target, meas, dt);
    return out;
}

// ---- Attitude helpers ----
// An attitude from ZYX Euler angles, computed HERE rather than borrowed from
// the class under test: a test that builds its inputs with the code it is
// checking cannot catch that code being wrong.
Quaternionf quat(float roll, float pitch, float yaw)
{
    float R[3][3];
    dcm(roll, pitch, yaw, R);
    // Shepperd's method, on the largest diagonal term, so the square root is
    // never taken of something near zero.
    const float t = R[0][0] + R[1][1] + R[2][2];
    float w, x, y, z;
    if (t > 0.0f) {
        const float r = std::sqrt(1.0f + t) * 2.0f;
        w = 0.25f * r; x = (R[2][1] - R[1][2]) / r;
        y = (R[0][2] - R[2][0]) / r; z = (R[1][0] - R[0][1]) / r;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const float r = std::sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        w = (R[2][1] - R[1][2]) / r; x = 0.25f * r;
        y = (R[0][1] + R[1][0]) / r; z = (R[0][2] + R[2][0]) / r;
    } else if (R[1][1] > R[2][2]) {
        const float r = std::sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        w = (R[0][2] - R[2][0]) / r; x = (R[0][1] + R[1][0]) / r;
        y = 0.25f * r; z = (R[1][2] + R[2][1]) / r;
    } else {
        const float r = std::sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        w = (R[1][0] - R[0][1]) / r; x = (R[0][2] + R[2][0]) / r;
        y = (R[1][2] + R[2][1]) / r; z = 0.25f * r;
    }
    return Quaternionf(w, x, y, z);
}

// Advance an attitude by a body-frame angular rate for one step. Body axes, so
// the increment composes on the RIGHT. This is the "perfect rate controller"
// the closed-loop checks fly against: whatever Attitude asks for, it gets.
Quaternionf spin(const Quaternionf &q, const Vector3f &w, float dt)
{
    Quaternionf dq;
    const float rate = w.length();
    if (rate * dt > 1e-9f) {
        const Vector3f axis = w / rate;
        const float h = rate * dt * 0.5f, s = std::sin(h);
        dq = Quaternionf(std::cos(h), axis.x * s, axis.y * s, axis.z * s);
    }
    const Quaternionf r = q * dq;
    const float n = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
    return Quaternionf(r.w/n, r.x/n, r.y/n, r.z/n);
}

// Angle between two attitudes, rad -- the size of the rotation from one to the
// other, however it is parameterised.
float angleBetween(const Quaternionf &a, const Quaternionf &b)
{
    const Quaternionf d = a.conjugate() * b;
    // atan2 of (vector part, scalar part), NOT acos of the scalar part: acos
    // has infinite slope at 1, so in float it reports about 0.06 degrees for
    // two attitudes that are bit-identical. Every use of this is a comparison
    // near zero, which is precisely where that form is worthless.
    const float v = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
    return 2.0f * std::atan2(v, std::fabs(d.w));
}

} // namespace

int main()
{
    const float dt = 0.0025f;   // 400 Hz, a plausible controller rate

    // =====================================================================
    section("PID: the terms do what they say");
    // =====================================================================
    {
        PID p; p.setGains(2.0f, 0.0f, 0.0f, 0.0f);
        const float out = p.update(1.0f, 0.0f, dt);
        checkNear(out, 2.0f, 1e-6f, "P alone is kp * error");
        checkNear(p.getI(), 0.0f, 1e-9f, "no I gain means no integrator");
    }
    {
        // I accumulates at ki * error per second, and stops at iMax.
        PID p(0.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f);
        for (int i = 0; i < 400; i++) p.update(1.0f, 0.0f, dt);   // 1 s of error 1
        checkNear(p.getI(), 1.0f, 0.02f, "I integrates error * ki * time");
        for (int i = 0; i < 400 * 60; i++) p.update(1.0f, 0.0f, dt);
        checkNear(p.getI(), 10.0f, 1e-4f, "and is clamped at iMax");
    }
    {
        PID p; p.setGains(0.0f, 0.0f, 0.0f, 3.0f);
        const float out = p.update(2.0f, 0.0f, dt);
        checkNear(out, 6.0f, 1e-6f, "FF rides the TARGET, not the error");
    }
    {
        PID p(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        p.setOutputLimit(0.5f);
        const float out = p.update(10.0f, 0.0f, dt);
        checkNear(out, 0.5f, 1e-6f, "the output limit clamps");
        check(p.isSaturated(), "... and reports saturation");
        p.update(0.1f, 0.0f, dt);
        check(!p.isSaturated(), "... and clears it when back in range");
    }

    // =====================================================================
    section("PID: derivative kick");
    // =====================================================================
    {
        // THE reason update() differentiates the measurement. A step in the
        // TARGET must not produce a D spike; a step in the MEASUREMENT must.
        PID p(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 50; i++) p.update(0.0f, 0.0f, dt);    // settled
        const float d_on_target_step = p.update(100.0f, 0.0f, dt);
        checkNear(d_on_target_step, 0.0f, 1e-6f,
                  "a setpoint step produces NO derivative kick");

        PID q(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 50; i++) q.update(0.0f, 0.0f, dt);
        const float d_on_meas_step = q.update(0.0f, 1.0f, dt);
        check(d_on_meas_step < -100.0f,
              "a measurement step DOES move D, and opposes it");
    }
    {
        // updateError() has only the error, so it must differentiate that.
        PID p(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 50; i++) p.updateError(0.0f, dt);
        const float out = p.updateError(1.0f, dt);
        check(out > 100.0f, "updateError differentiates the error itself");
    }
    {
        // First sample after reset() must seed, not read as a step from zero.
        PID p(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        const float first = p.update(0.0f, 500.0f, dt);
        checkNear(first, 0.0f, 1e-6f, "no false derivative on the first sample");
        p.reset();
        const float after_reset = p.update(0.0f, -500.0f, dt);
        checkNear(after_reset, 0.0f, 1e-6f, "... nor on the first after reset()");
    }

    // =====================================================================
    section("PID: anti-windup");
    // =====================================================================
    {
        PID p(0.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f);
        settle(p, 1.0f, 0.0f, 400, dt);
        const float before = p.getI();
        for (int i = 0; i < 400; i++) p.update(1.0f, 0.0f, dt, true);  // limited
        checkNear(p.getI(), before, 1e-6f, "a limited actuator freezes the integrator");

        // ... but it must still be able to come back off the stop.
        for (int i = 0; i < 200; i++) p.update(-1.0f, 0.0f, dt, true);
        check(p.getI() < before - 0.1f, "an opposing error still shrinks it");
    }
    {
        // Setting ki to zero must not leave a stale integrator contributing.
        PID p(0.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f);
        settle(p, 1.0f, 0.0f, 400, dt);
        check(p.getI() > 0.5f, "integrator wound up");
        p.setGains(0.0f, 0.0f, 0.0f, 0.0f);
        p.update(1.0f, 0.0f, dt);
        checkNear(p.getI(), 0.0f, 1e-9f, "dropping ki drops the accumulated I");
    }

    // =====================================================================
    section("PID: bad input cannot poison the loop");
    // =====================================================================
    {
        PID p(1.0f, 1.0f, 1.0f, 0.0f, 10.0f, 0.0f);
        const float good = settle(p, 1.0f, 0.0f, 100, dt);
        check(std::isfinite(good), "a healthy loop produces a finite output");

        const float nan = std::nanf("");
        const float inf = 1.0f / 0.0f;
        checkNear(p.update(nan, 0.0f, dt), good, 1e-4f, "a NaN target is refused");
        checkNear(p.update(1.0f, nan, dt), good, 1e-4f, "a NaN measurement is refused");
        checkNear(p.update(1.0f, 0.0f, nan), good, 1e-4f, "a NaN dt is refused");
        checkNear(p.update(inf, 0.0f, dt), good, 1e-4f, "an infinite target is refused");
        checkNear(p.update(1.0f, 0.0f, 0.0f), good, 1e-4f, "dt == 0 is refused");
        checkNear(p.update(1.0f, 0.0f, -dt), good, 1e-4f, "a negative dt is refused");
        check(std::isfinite(p.getI()), "the integrator is still finite after all of it");
        check(std::isfinite(p.update(1.0f, 0.0f, dt)), "and the loop still works");
    }
    {
        // A very small dt. Nothing rejects it, and the derivative divides by it.
        PID p(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 50; i++) p.update(0.0f, 0.0f, dt);
        const float out = p.update(0.0f, 0.001f, 1e-7f);
        check(std::isfinite(out), "a tiny dt does not produce a non-finite output");
        std::printf("        (dt=1e-7 with a 1 mm step gave D = %.1f)\n", (double)out);
    }

    // =====================================================================
    section("Position: the square-root controller");
    // =====================================================================
    {
        Position pos;
        pos.reset(Vector3f(0, 0, 0));
        // Continuity at the linear/sqrt join. linear_dist = accel / p^2.
        // With the defaults that is 2.5 / 1 = 2.5 m.
        const float L = POSITION_ACCEL_NE / (POSITION_P_NE * POSITION_P_NE);
        const float eps = 1e-4f;

        // AT the join the linear branch runs, and must give exactly L*p. The
        // sqrt branch just above it must converge to the same value -- the two
        // may only differ by one step of the shared slope, which is p.
        const float at_join = Position::sqrtController(L, POSITION_P_NE, POSITION_ACCEL_NE, dt);
        const float above   = Position::sqrtController(L + eps, POSITION_P_NE, POSITION_ACCEL_NE, dt);
        const float below   = Position::sqrtController(L - eps, POSITION_P_NE, POSITION_ACCEL_NE, dt);
        checkNear(at_join, L * POSITION_P_NE, 1e-4f, "the join value is exactly L * p");
        checkNear(above - at_join, eps * POSITION_P_NE, 1e-4f,
                  "the sqrt branch meets it -- no step across the join");

        // And the slopes match, which is what stops the output jumping every
        // time the vehicle crosses the boundary. Both must equal p.
        const float slope_below = (at_join - below) / eps;
        const float slope_above = (above - at_join) / eps;
        checkNear(slope_below, POSITION_P_NE, 0.02f, "slope below the join is p");
        checkNear(slope_above, POSITION_P_NE, 0.02f, "and the slope above matches it");
    }
    {
        Position pos;
        pos.reset(Vector3f(0, 0, 0));
        // A demand must never exceed what lands on the target next cycle,
        // or the approach oscillates across it for ever.
        float n, e;
        pos.sqrtControllerNE(0.001f, 0.0f, POSITION_P_NE, POSITION_ACCEL_NE, dt, n, e);
        check(n <= 0.001f / dt + 1e-3f, "never asks for more than error/dt");
    }
    {
        // A diagonal target must be approached in a STRAIGHT line: shaping the
        // axes independently would bow the path.
        Position pos;
        pos.reset(Vector3f(0, 0, 0));
        float n, e;
        pos.sqrtControllerNE(3.0f, 4.0f, POSITION_P_NE, POSITION_ACCEL_NE, dt, n, e);
        checkNear(n / e, 3.0f / 4.0f, 1e-4f, "the velocity points along the error");
    }

    // =====================================================================
    section("Position: limits and runaway");
    // =====================================================================
    {
        Position pos;
        pos.reset(Vector3f(0, 0, 0));
        // Sticks hard north for 10 s while the aircraft never moves: the
        // classic target-runaway case (asking for more than the wind allows).
        for (int i = 0; i < 4000; i++) {
            pos.inputVelocity(Vector3f(POSITION_SPEED_NE, 0, 0), dt);
            pos.update(Vector3f(0, 0, 0), dt);
        }
        const Vector3f err = pos.getPositionError();
        check(std::fabs(err.x) <= POSITION_MAX_ERROR_NE + 0.5f,
              "a target the aircraft cannot follow is capped, not run away with");
        check(pos.getLimits().pos_error, "and the cap is reported");
        const Vector3f vt = pos.getVelocityTarget();
        check(vt.length() <= POSITION_SPEED_NE + 1e-3f, "the velocity target respects the speed limit");
    }
    {
        // Climb and descent limits are deliberately different.
        Position pos;
        pos.reset(Vector3f(0, 0, 0));
        for (int i = 0; i < 2000; i++) {
            pos.inputVelocity(Vector3f(0, 0, -10.0f), dt);   // NED: up
            pos.update(Vector3f(0, 0, 0), dt);
        }
        checkNear(pos.getVelocityTarget().z, -POSITION_SPEED_UP, 1e-3f, "climb is capped at SPEED_UP");
        pos.reset(Vector3f(0, 0, 0));
        for (int i = 0; i < 2000; i++) {
            pos.inputVelocity(Vector3f(0, 0, 10.0f), dt);    // NED: down
            pos.update(Vector3f(0, 0, 0), dt);
        }
        checkNear(pos.getVelocityTarget().z, POSITION_SPEED_DOWN, 1e-3f, "descent is capped at SPEED_DOWN");
        check(POSITION_SPEED_DOWN < POSITION_SPEED_UP, "and descent is the slower of the two");
    }
    {
        Position pos;
        const float nan = std::nanf("");
        pos.update(Vector3f(0, 0, 0), dt);
        check(pos.getVelocityTarget().length() == 0.0f, "an un-reset controller stays silent");
        pos.reset(Vector3f(nan, 0, 0));
        check(!pos.isActive(), "reset() refuses a non-finite position");
        pos.reset(Vector3f(0, 0, 0));
        pos.inputVelocity(Vector3f(nan, nan, nan), dt);
        pos.update(Vector3f(0, 0, 0), dt);
        check(finiteVec(pos.getVelocityTarget()), "a NaN stick cannot make a NaN demand");
    }

    // =====================================================================
    section("Velocity: hovering");
    // =====================================================================
    {
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(0, 0, 0));
        for (int i = 0; i < 400; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);

        const Vector3f tv = vel.getThrustVector();
        checkNear(tv.z, -1.0f, 1e-4f, "level hover points thrust straight up");
        checkNear(vel.getTiltTarget() * R2D, 0.0f, 0.05f, "tilt is zero");
        checkNear(vel.getThrottle(), VELOCITY_HOVER_THROTTLE, 1e-3f,
                  "throttle sits exactly at the hover value");
        checkNear(vel.getRollTarget(), 0.0f, 1e-4f, "roll is zero");
        checkNear(vel.getPitchTarget(), 0.0f, 1e-4f, "pitch is zero");
    }

    // =====================================================================
    section("Velocity: tilt, and the throttle boost that comes with it");
    // =====================================================================
    {
        // Hold a steady horizontal error until the loop settles, then check
        // the geometry from first principles: a tilt of theta buys
        // g*tan(theta) horizontally, and needs g/cos(theta) of thrust.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(1.0f, 0, 0));       // 1 m/s north
        for (int i = 0; i < 2000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);

        const float tilt = vel.getTiltTarget();
        const float a_h = VELOCITY_GRAVITY_MSS * std::tan(tilt);
        const float expect_throttle =
                VELOCITY_HOVER_THROTTLE * (1.0f / std::cos(tilt));
        checkNear(vel.getThrottle(), expect_throttle, 2e-3f,
                  "throttle is hover / cos(tilt) -- the boost falls out of the vector");
        check(a_h > 0.0f, "and the aircraft is leaning the way it wants to go");
        check(vel.getPitchTarget() < 0.0f,
              "north demand pitches the NOSE DOWN");
        checkNear(vel.getRollTarget(), 0.0f, 1e-3f, "with no roll");
    }
    {
        // East demand must roll RIGHT, at zero yaw, and not pitch.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(0, 1.0f, 0));
        for (int i = 0; i < 2000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        check(vel.getRollTarget() > 0.0f, "east demand rolls RIGHT");
        checkNear(vel.getPitchTarget(), 0.0f, 1e-3f, "with no pitch");
    }
    {
        // Heading must rotate the demand into the body frame. Facing east, a
        // NORTH demand is a LEFT roll and no pitch.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(1.0f, 0, 0));
        for (int i = 0; i < 2000; i++) vel.update(Vector3f(0, 0, 0), 90.0f * D2R, dt);
        check(vel.getRollTarget() < 0.0f, "facing east, a north demand rolls LEFT");
        checkNear(vel.getPitchTarget(), 0.0f, 2e-3f, "and does not pitch");
    }

    // =====================================================================
    section("Velocity: the limits that keep it flyable");
    // =====================================================================
    {
        // A huge demand must be held at the lean limit, not beyond it.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(50.0f, 50.0f, 0));
        for (int i = 0; i < 4000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        check(vel.getTiltTarget() <= VELOCITY_MAX_LEAN_ANGLE + 1e-3f,
              "tilt never exceeds the lean limit");
        check(vel.getLimits().accel_ne, "and the limit is reported for anti-windup");
    }
    {
        // The thrust vector must ALWAYS point skyward. If it ever reached
        // horizontal the normalisation divides by ~0 and the attitude target
        // becomes garbage -- this is the one that would drop the aircraft.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVerticalAccelLimits(2.5f, 1000.0f);   // absurd descent demand
        bool always_up = true, always_finite = true;
        for (int i = 0; i < 4000; i++) {
            vel.setVelocityTarget(Vector3f(50.0f, 50.0f, 100.0f));  // dive, hard
            vel.update(Vector3f(0, 0, 0), 0.0f, dt);
            const Vector3f tv = vel.getThrustVector();
            if (!(tv.z < -0.1f)) always_up = false;
            if (!finiteVec(tv) || !std::isfinite(vel.getThrottle())) always_finite = false;
        }
        check(always_up, "the thrust vector never approaches horizontal");
        check(always_finite, "and nothing goes non-finite under an absurd demand");
        check(vel.getAccelDownLimit() < VELOCITY_GRAVITY_MSS,
              "the descent accel limit is capped below g");
    }
    {
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        const float nan = std::nanf("");
        vel.setVelocityTarget(Vector3f(0, 0, 0));
        for (int i = 0; i < 100; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        const float t = vel.getThrottle();
        vel.update(Vector3f(nan, 0, 0), 0.0f, dt);
        checkNear(vel.getThrottle(), t, 1e-6f, "a NaN velocity estimate is refused");
        vel.update(Vector3f(0, 0, 0), nan, dt);
        checkNear(vel.getThrottle(), t, 1e-6f, "a NaN yaw is refused");
        vel.update(Vector3f(0, 0, 0), 0.0f, 0.0f);
        checkNear(vel.getThrottle(), t, 1e-6f, "dt == 0 is refused");
        check(finiteVec(vel.getThrustVector()), "the thrust vector stays finite");
    }

    // =====================================================================
    section("Velocity: how wrong may the hover throttle be?");
    // =====================================================================
    {
        // The vertical loop trims throttle only through a_d, and a_d is capped
        // at the accel limits. So the reachable steady throttle is
        //     hover * (g -/+ accel) / g
        // and a hover throttle outside that band can never be corrected.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setHoverThrottle(0.5f);
        vel.setVelocityTarget(Vector3f(0, 0, 0));

        // Simulate an airframe whose TRUE hover is 0.30: it climbs whenever
        // throttle exceeds that. Feed back a vertical velocity consistent with
        // the excess and see whether the loop can ever settle.
        float vz = 0.0f;
        for (int i = 0; i < 20000; i++) {
            vel.update(Vector3f(0, 0, vz), 0.0f, dt);
            const float accel = (vel.getThrottle() - 0.30f) * 20.0f;  // crude thrust curve
            vz -= accel * dt;                                          // NED: up is -z
            vz = clampf(vz, -20.0f, 20.0f);
        }
        std::printf("        (settled throttle %.3f, climb rate %.2f m/s, saturated %.1f s)\n",
                    (double)vel.getThrottle(), (double)-vz,
                    (double)vel.getVerticalSaturationTime());
        check(std::fabs(vz) > 0.5f,
              "a hover throttle set 67% above the truth cannot be trimmed out");
        check(vel.getLimits().accel_up || vel.getLimits().accel_down,
              "and the loop reports the accel limit it is pinned against");

        // The failure is unchanged -- the physics does not care -- but it is
        // no longer SILENT. Sustained saturation is the only symptom this has.
        check(vel.getVerticalSaturationTime() > 5.0f,
              "and sustained saturation makes the untrimmable case detectable");

        // The band that could have been checked on the bench beforehand.
        float lo = 0.0f, hi = 0.0f;
        vel.getThrottleRange(lo, hi);
        std::printf("        (reachable steady throttle %.3f .. %.3f)\n",
                    (double)lo, (double)hi);
        check(0.30f < lo, "0.30 really is outside the reachable band");
        checkNear(lo, 0.5f * (G - VELOCITY_ACCEL_DOWN) / G, 1e-4f,
                  "the lower bound is hover * (g - accel_down) / g");
        checkNear(hi, 0.5f * (G + VELOCITY_ACCEL_UP) / G, 1e-4f,
                  "the upper bound is hover * (g + accel_up) / g");
    }

    // =====================================================================
    section("The three defences added after the first review");
    // =====================================================================
    {
        // 1. A frozen loop is no longer indistinguishable from a healthy one.
        //    Holding the last output on bad input is still right; what was
        //    missing was any evidence it had happened.
        PID p(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        const float last = p.update(1.0f, 0.0f, dt);
        check(p.getRejectedCount() == 0, "a healthy loop rejects nothing");

        const float nan = std::nanf("");
        for (int i = 0; i < 1000; i++) p.update(nan, nan, dt);
        checkNear(p.getOutput(), last, 1e-6f, "a dead input still freezes the output");
        check(p.getRejectedCount() == 1000, "... and every refused sample is now counted");

        p.updateError(nan, dt);
        check(p.getRejectedCount() == 1001, "updateError() counts them too");
        p.update(1.0f, 0.0f, 0.0f);
        check(p.getRejectedCount() == 1002, "and so does a zero dt");

        p.reset();
        check(p.getRejectedCount() == 0,
              "reset() clears it -- a fault from a previous arm is not current");
    }
    {
        // 2. Velocity now idles at zero thrust, matching Position. A caller
        //    that forgets reset() must read "no thrust", not a number that
        //    flies.
        Velocity vel;
        check(!vel.isActive(), "a fresh Velocity is inactive");
        checkNear(vel.getThrottle(), 0.0f, 1e-6f, "and idles at ZERO throttle");
        Position pos;
        checkNear(pos.getVelocityTarget().length(), 0.0f, 1e-6f,
                  "matching Position, which always did");

        vel.reset(Vector3f(0, 0, 0));
        checkNear(vel.getThrottle(), VELOCITY_HOVER_THROTTLE, 1e-6f,
                  "engaging it produces the hover value, as before");
    }
    {
        // 3. The integrator clamp covers BOTH vertical limits, so steady-state
        //    descent trim is no longer bounded by the climb limit.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVerticalAccelLimits(1.0f, 5.0f);
        checkNear(vel.getVerticalPID().getIMax(), 5.0f, 1e-6f,
                  "iMax follows the LARGER of up and down");
        vel.setVerticalAccelLimits(4.0f, 1.0f);
        checkNear(vel.getVerticalPID().getIMax(), 4.0f, 1e-6f, "... either way round");

        // Peak descent still reaches the down limit -- P carries that, and did
        // before. This is the part that was already fine.
        vel.setVerticalAccelLimits(1.0f, 5.0f);
        vel.setVelocityTarget(Vector3f(0, 0, 5.0f));
        for (int i = 0; i < 8000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        checkNear(vel.getAccelTarget().z, 5.0f, 1e-3f, "peak descent still reaches its limit");
    }
    {
        // The saturation timer must be CONTINUOUS, not cumulative: a moment of
        // saturation is normal and must not accumulate into a false report.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(0, 0, -10.0f));          // hard climb
        for (int i = 0; i < 400; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        check(vel.getVerticalSaturationTime() > 0.5f, "a hard climb does saturate");

        vel.setVelocityTarget(Vector3f(0, 0, 0));               // demand removed
        for (int i = 0; i < 2000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);
        checkNear(vel.getVerticalSaturationTime(), 0.0f, 1e-6f,
                  "and one unsaturated cycle clears the clock");
    }

    // =====================================================================
    section("Attitude: the rotation maths");
    // =====================================================================
    {
        // The thrust-vector solve must agree with Velocity's, everywhere.
        // Two classes answer the same question -- "which attitude puts the
        // thrust axis there, at this heading" -- and they are deliberately
        // not shared, so this is the check that keeps them from drifting
        // apart. If it ever fails, the roll and pitch on the telemetry line
        // and the attitude actually being flown have diverged.
        float worst_rp = 0.0f, worst_thrust = 0.0f, worst_heading = 0.0f;
        const float yaws[] = { 0.0f, 30.0f, 90.0f, -120.0f, 179.0f };
        for (int k = 0; k < 5; k++) {
            const float yaw = yaws[k] * D2R;
            for (int i = -6; i <= 6; i++) {
                for (int j = -6; j <= 6; j++) {
                    const float tn = i * 0.08f, te = j * 0.08f;
                    if (tn*tn + te*te >= 0.98f) continue;
                    const Vector3f t(tn, te, -std::sqrt(1.0f - tn*tn - te*te));

                    float roll_v, pitch_v;
                    Velocity::computeAttitudeTarget(t, yaw, roll_v, pitch_v);
                    const Quaternionf q = Attitude::attitudeFromThrustVector(t, yaw);

                    const float d_rp = angleBetween(q, quat(roll_v, pitch_v, yaw));
                    if (d_rp > worst_rp) worst_rp = d_rp;

                    // The thrust axis must come back out of the quaternion.
                    const Vector3f back = q.rotate(Vector3f(0, 0, -1));
                    const float d_t = (back - t).length();
                    if (d_t > worst_thrust) worst_thrust = d_t;

                    // ... and so must the heading that was asked for.
                    float dh = Attitude::headingFromAttitude(q) - yaw;
                    while (dh >  3.14159265f) dh -= 6.28318531f;
                    while (dh < -3.14159265f) dh += 6.28318531f;
                    if (std::fabs(dh) > worst_heading) worst_heading = std::fabs(dh);
                }
            }
        }
        checkNear(worst_rp * R2D, 0.0f, 1e-2f,
                  "thrust-vector attitude matches Velocity's solver, over the whole envelope (deg)");
        checkNear(worst_thrust, 0.0f, 1e-5f, "the thrust axis comes back out exactly");
        checkNear(worst_heading * R2D, 0.0f, 1e-2f, "and so does the heading (deg)");
    }
    {
        // decompose() must be an exact factorisation, not an approximation:
        // q_err == q_tilt * q_yaw, with the tilt part carrying no rotation
        // about the thrust axis at all.
        float worst = 0.0f, worst_z = 0.0f;
        for (int a = 0; a < 60; a++) {
            const Quaternionf qe = quat((a * 37 % 71) / 71.0f * 2.6f - 1.3f,
                                        (a * 53 % 67) / 67.0f * 2.6f - 1.3f,
                                        (a * 29 % 61) / 61.0f * 6.0f - 3.0f);
            Vector3f tilt; float yaw;
            Attitude::decompose(qe, tilt, yaw);
            if (std::fabs(tilt.z) > worst_z) worst_z = std::fabs(tilt.z);

            const float ang = tilt.length();
            Quaternionf q_tilt;
            if (ang > 1e-9f) {
                const Vector3f ax = tilt / ang;
                const float h = ang * 0.5f, s = std::sin(h);
                q_tilt = Quaternionf(std::cos(h), ax.x*s, ax.y*s, ax.z*s);
            }
            const Quaternionf q_yaw(std::cos(yaw*0.5f), 0, 0, std::sin(yaw*0.5f));
            const float d = angleBetween(qe, q_tilt * q_yaw);
            if (d > worst) worst = d;
        }
        checkNear(worst * R2D, 0.0f, 1e-2f,
                  "decompose() factors the error exactly: q_err == tilt * heading (deg)");
        checkNear(worst_z, 0.0f, 1e-6f,
                  "and the tilt part turns nothing about the thrust axis");
    }
    {
        // A rotation and its negation are the same attitude. The correction
        // must be the short way round in both cases, or the aircraft takes
        // the scenic route -- a full turn to fix nothing.
        const Quaternionf q = quat(0, 0, 350.0f * D2R);
        const Vector3f v = Attitude::rotationVector(q);
        checkNear(v.z * R2D, -10.0f, 0.01f, "350 degrees of yaw reads as -10, not +350");

        const Quaternionf neg(-q.w, -q.x, -q.y, -q.z);
        const Vector3f vn = Attitude::rotationVector(neg);
        checkNear(vn.z * R2D, -10.0f, 0.01f, "and -q, the same attitude, gives the same answer");

        checkNear(Attitude::rotationVector(Quaternionf()).length(), 0.0f, 1e-9f,
                  "the identity is no rotation at all");
    }
    {
        // v^2 = 2*a*d, which is the whole argument for the cap.
        checkNear(Attitude::stoppingRate(0.5f, 8.0f), std::sqrt(2.0f*8.0f*0.5f), 1e-5f,
                  "stopping rate is sqrt(2*a*angle)");
        checkNear(Attitude::stoppingRate(-0.5f, 8.0f), std::sqrt(2.0f*8.0f*0.5f), 1e-5f,
                  "... on the magnitude, so the sign of the error does not matter");
        checkNear(Attitude::stoppingRate(0.0f, 8.0f), 0.0f, 1e-9f,
                  "on the target, no rate at all can be stopped from later");
    }

    // =====================================================================
    section("Attitude: the error is a rotation, not three subtractions");
    // =====================================================================
    {
        // Each axis on its own: the demand must be P * error, on that axis
        // and no other. Cross-coupling here would show up in the air as a
        // roll input producing yaw.
        struct { float r, p, y; const char *what; } cases[] = {
            { 10.0f, 0.0f, 0.0f, "roll" },
            { 0.0f, 10.0f, 0.0f, "pitch" },
            { 0.0f, 0.0f, 10.0f, "yaw" },
        };
        for (int i = 0; i < 3; i++) {
            Attitude att;
            att.reset(quat(0, 0, 0));
            att.setEuler(cases[i].r * D2R, cases[i].p * D2R, cases[i].y * D2R);
            att.update(quat(0, 0, 0), dt);
            const Vector3f w = att.getRateTarget();
            const float on = (i == 0) ? w.x : ((i == 1) ? w.y : w.z);
            const float off1 = (i == 0) ? w.y : w.x;
            const float off2 = (i == 2) ? w.y : w.z;
            checkNear(on, ATTITUDE_P_ROLL * 10.0f * D2R, 1e-4f, cases[i].what);
            checkNear(std::fabs(off1) + std::fabs(off2), 0.0f, 1e-5f,
                      "... and nothing on the other two axes");
        }
    }
    {
        // The wrap. Subtracting Euler angles here gives 358 degrees and the
        // aircraft spins the long way round to correct two.
        Attitude att;
        att.reset(quat(0, 0, -179.0f * D2R));
        att.setEuler(0, 0, 179.0f * D2R);
        att.update(quat(0, 0, -179.0f * D2R), dt);
        checkNear(att.getYawError() * R2D, -2.0f, 0.01f,
                  "-179 to +179 is 2 degrees the short way, not 358 the long way");
        check(att.getRateTarget().z < 0.0f, "and the aircraft turns the short way");
    }
    {
        // Gimbal lock, where three Euler differences stop being defined at
        // all: pitched exactly 90 degrees nose up, asked to be level.
        Attitude att;
        att.reset(quat(0, 90.0f * D2R, 0));
        att.setEuler(0, 0, 0);
        att.update(quat(0, 90.0f * D2R, 0), dt);

        const Vector3f w = att.getRateTarget();
        checkNear(att.getTiltError() * R2D, 90.0f, 0.05f,
                  "straight up with a level target is 90 degrees of tilt error");
        check(std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z),
              "the demand is finite where Euler subtraction is undefined");
        check(w.y < 0.0f, "and it is a nose-DOWN pitch rate, which is the way out");
        checkNear(std::fabs(w.x), 0.0f, 1e-4f, "with nothing on roll");
    }

    // =====================================================================
    section("Attitude: tilt is worth more than heading");
    // =====================================================================
    {
        // Small tilt error: heading is corrected at full gain.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(5.0f * D2R, 0, 20.0f * D2R);
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getYawPriority(), 1.0f, 1e-6f, "a small tilt error costs heading nothing");
        checkNear(att.getRateTarget().z, ATTITUDE_P_YAW * 20.0f * D2R, 2e-3f,
                  "so yaw gets the full P * error");
        check(!att.getLimits().yaw_priority, "and nothing is reported");
    }
    {
        // Past twice the priority angle the aircraft is in trouble, and every
        // bit of motor thrust belongs to getting the thrust axis back.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(70.0f * D2R, 0, 30.0f * D2R);
        att.update(quat(0, 0, 0), dt);
        check(att.getTiltError() > 2.0f * ATTITUDE_YAW_PRIORITY_ANGLE,
              "70 degrees of tilt error is past twice the priority angle");
        checkNear(att.getYawPriority(), 0.0f, 1e-6f, "so heading is given up entirely");
        checkNear(att.getRateTarget().z, 0.0f, 1e-6f, "and the yaw demand is exactly zero");
        check(att.getLimits().yaw_priority, "reported, so a mode knows it is not holding heading");
        check(std::fabs(att.getYawError()) > 0.0f,
              "but the error itself is still told truthfully -- the fade is not a lie about the angle");
        check(std::fabs(att.getRateTarget().x) > 0.1f, "meanwhile the tilt correction is untouched");
    }
    {
        // Halfway through the fade, half the authority. A hard switch would
        // step the yaw demand as the tilt error wandered over the threshold.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(45.0f * D2R, 0, 20.0f * D2R);
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getTiltError() * R2D, 45.0f, 0.05f, "45 degrees of tilt error");
        checkNear(att.getYawPriority(), 0.5f, 1e-3f, "is halfway through the fade");
    }

    // =====================================================================
    section("Attitude: the limits, and the one the feed-forward must escape");
    // =====================================================================
    {
        // A huge error must be held at the rate limit, not beyond it.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(0, 0, 170.0f * D2R);
        att.update(quat(0, 0, 0), dt);
        check(std::fabs(att.getRateTarget().z) <= ATTITUDE_RATE_YAW_MAX + 1e-4f,
              "the yaw demand never exceeds the rate limit");
        check(att.getLimits().rate_yaw || att.getLimits().accel_yaw,
              "and the clip is reported for the rate loop's anti-windup");
    }
    {
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(0, 150.0f * D2R, 0);
        att.update(quat(0, 0, 0), dt);
        const Vector3f w = att.getRateTarget();
        checkNear(std::sqrt(w.x*w.x + w.y*w.y), ATTITUDE_RATE_RP_MAX, 1e-3f,
                  "a large tilt error is held at the roll/pitch rate limit");
        check(att.getLimits().rate_rp, "reported");
    }
    {
        // Roll and pitch are limited as a VECTOR. A 45 degree correction with
        // both axes over the limit must stay on the diagonal -- clamping the
        // two separately would swing it toward whichever axis was still
        // under its cap, and the aircraft would recover in a direction nobody
        // asked for.
        // Measure the uncapped demand first, then cap it: the claim is that
        // capping changes the LENGTH and nothing else, and checking it this
        // way does not depend on predicting what the geometry produces.
        Attitude loose;
        loose.reset(quat(0, 0, 0));
        loose.setRateLimits(100.0f, 100.0f);
        loose.setEuler(40.0f * D2R, 40.0f * D2R, 0);
        loose.update(quat(0, 0, 0), dt);
        const Vector3f u = loose.getRateTarget();

        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setRateLimits(1.0f, ATTITUDE_RATE_YAW_MAX);
        att.setEuler(40.0f * D2R, 40.0f * D2R, 0);
        att.update(quat(0, 0, 0), dt);
        const Vector3f w = att.getRateTarget();

        check(std::sqrt(u.x*u.x + u.y*u.y) > 1.0f, "the demand really does need capping");
        checkNear(std::sqrt(w.x*w.x + w.y*w.y), 1.0f, 1e-4f, "the pair is capped by length");
        checkNear(w.x / w.y, u.x / u.y, 1e-4f,
                  "and the direction of the correction survives the cap untouched");
    }
    {
        // THE one that matters. A target turning steadily, tracked perfectly,
        // has zero error -- so the stopping cap is also zero. If the cap were
        // applied to the output rather than to the correction, the demand
        // would be clamped to nothing and the aircraft would stop dead and
        // fall behind its own target until enough error built up to earn the
        // rate back. On a commanded yaw that is a stutter.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setTargetAngularVelocity(Vector3f(0, 0, 0.8f));
        att.update(quat(0, 0, 0), dt);

        checkNear(att.getTiltError(), 0.0f, 1e-6f, "no error at all");
        checkNear(Attitude::stoppingRate(0.0f, ATTITUDE_ACCEL_YAW_MAX), 0.0f, 1e-9f,
                  "so the stopping cap is zero");
        checkNear(att.getRateTarget().z, 0.8f, 1e-6f,
                  "and the commanded rate still comes out in full");
        check(!att.getLimits().accel_yaw && !att.getLimits().accel_rp,
              "with no saturation reported, because none happened");
    }

    // =====================================================================
    section("Attitude: feed-forward");
    // =====================================================================
    {
        // A supplied rate is good for exactly one cycle. A stale feed-forward
        // is a rate command nobody asked for, so forgetting to set it must
        // decay to the estimate rather than to a runaway.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setTargetAngularVelocity(Vector3f(0, 0, 0.5f));
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getRateTarget().z, 0.5f, 1e-6f, "a supplied rate is fed forward");
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getRateTarget().z, 0.0f, 1e-6f, "and is consumed -- it does not stand");
    }
    {
        // Measured, not supplied: the target turns and the class works out
        // how fast by itself. The aircraft is held exactly on the target, so
        // the error is zero throughout and every bit of the output is
        // feed-forward.
        Attitude att;
        att.reset(quat(0, 0, 0));
        const float rate = 0.6f;
        float yaw = 0.0f;
        for (int i = 0; i < 2000; i++) {
            yaw += rate * dt;
            att.setEuler(0, 0, yaw);
            att.update(quat(0, 0, yaw), dt);
        }
        checkNear(att.getTiltError(), 0.0f, 1e-4f, "tracking perfectly, there is no error");
        checkNear(att.getFeedForward().z, rate, 1e-3f,
                  "so the class has measured the target's own rate");
        checkNear(att.getRateTarget().z, rate, 1e-3f, "and asks for exactly it");
    }
    {
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setFeedForwardEnabled(false);
        att.setTargetAngularVelocity(Vector3f(0, 0, 0.5f));
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getRateTarget().z, 0.0f, 1e-9f, "turned off, nothing is fed forward");
    }

    // =====================================================================
    section("Attitude: flying it");
    // =====================================================================
    {
        // Closed loop against a perfect rate controller: whatever the class
        // asks for, the aircraft does. From 30 degrees out it must converge,
        // and it must not sail past -- a first-order loop that overshoots is
        // one whose gain and limits disagree.
        const Quaternionf target = quat(20.0f*D2R, -15.0f*D2R, 40.0f*D2R);
        Quaternionf q = quat(0, 0, 0);
        Attitude att;
        att.reset(q);
        att.setAttitudeTarget(target);

        float worst_past = 0.0f;
        const float start = angleBetween(q, target);
        for (int i = 0; i < 1600; i++) {       // 4 s
            att.update(q, dt);
            q = spin(q, att.getRateTarget(), dt);
            const Quaternionf d = target.conjugate() * q;
            // Overshoot shows up as the error changing sign on an axis.
            const Vector3f v = Attitude::rotationVector(d);
            const float past = v.length();
            if (i > 800 && past > worst_past) worst_past = past;
        }
        checkNear(start * R2D, 49.06f, 0.05f, "starting 49 degrees out");
        checkNear(angleBetween(q, target) * R2D, 0.0f, 0.05f, "it arrives on the target");
        checkNear(worst_past * R2D, 0.0f, 0.1f, "and does not overshoot it");
        checkNear(att.getRateTarget().length(), 0.0f, 1e-3f,
                  "asking for no rotation once it is there");
    }
    {
        // Tracking a target that keeps moving. This is what the feed-forward
        // is for: without it a P controller must fall behind by rate / P to
        // produce any output at all, and at 0.6 rad/s that is a nose dragging
        // 7.6 degrees behind the stick.
        const float rate = 0.6f;
        float lag_with = 0.0f, lag_without = 0.0f;
        for (int pass = 0; pass < 2; pass++) {
            Attitude att;
            Quaternionf q = quat(0, 0, 0);
            att.reset(q);
            att.setFeedForwardEnabled(pass == 0);
            float yaw = 0.0f;
            for (int i = 0; i < 3200; i++) {   // 8 s
                yaw += rate * dt;
                att.setEuler(0, 0, yaw);
                att.update(q, dt);
                q = spin(q, att.getRateTarget(), dt);
            }
            (pass == 0 ? lag_with : lag_without) = std::fabs(att.getYawError());
        }
        checkNear(lag_without * R2D, rate / ATTITUDE_P_YAW * R2D, 0.5f,
                  "without feed-forward the nose lags by rate / P (deg)");
        check(lag_with < lag_without * 0.05f,
              "with it, the lag all but disappears");
        checkNear(lag_with * R2D, 0.0f, 0.3f, "-- to under a third of a degree (deg)");
    }
    {
        // The handoff the cascade actually uses: a thrust vector and a
        // heading from the velocity controller, flown to.
        Velocity vel;
        vel.reset(Vector3f(0, 0, 0));
        vel.setVelocityTarget(Vector3f(1.5f, 0, 0));
        for (int i = 0; i < 2000; i++) vel.update(Vector3f(0, 0, 0), 0.0f, dt);

        Quaternionf q = quat(0, 0, 0);
        Attitude att;
        att.reset(q);
        for (int i = 0; i < 2000; i++) {
            att.setThrustVectorHeading(vel.getThrustVector(), 0.0f);
            att.update(q, dt);
            q = spin(q, att.getRateTarget(), dt);
        }
        const Vector3f flown = q.rotate(Vector3f(0, 0, -1));
        checkNear((flown - vel.getThrustVector()).length(), 0.0f, 1e-3f,
                  "the aircraft ends up pointing its thrust where Velocity asked");
        checkNear(Attitude::headingFromAttitude(q) * R2D, 0.0f, 0.1f,
                  "at the heading it asked for");
        checkNear(att.getTiltError(), 0.0f, 1e-3f, "with no tilt error left");
    }
    {
        // A yaw stick: the heading is integrated from the TARGET, not from
        // the measured attitude. Seeding it from the aircraft each cycle
        // would let the tracking error leak into the target and the nose
        // would creep.
        Attitude att;
        Quaternionf q = quat(0, 0, 0);
        att.reset(q);
        const float stick = 0.5f;            // rad/s
        for (int i = 0; i < 1200; i++) {     // 3 s
            att.setThrustVectorYawRate(Vector3f(0, 0, -1), stick, dt);
            att.update(q, dt);
            q = spin(q, att.getRateTarget(), dt);
        }
        checkNear(att.getHeadingTarget() * R2D, stick * 3.0f * R2D, 0.2f,
                  "the target heading is the stick integrated, exactly (deg)");
        checkNear(Attitude::headingFromAttitude(q) * R2D, stick * 3.0f * R2D, 0.5f,
                  "and the aircraft is on it (deg)");
        att.setThrustVectorYawRate(Vector3f(0, 0, -1), 0.0f, dt);
        const float held = att.getHeadingTarget();
        for (int i = 0; i < 400; i++) {
            att.setThrustVectorYawRate(Vector3f(0, 0, -1), 0.0f, dt);
            att.update(q, dt);
            q = spin(q, att.getRateTarget(), dt);
        }
        checkNear(att.getHeadingTarget(), held, 1e-6f,
                  "releasing the stick holds the heading it reached");
    }

    {
        // Hand-picked cases only ever cover what was thought of. This flies
        // the loop from a spread of attitudes to a spread of targets --
        // inversions, near-vertical, and the half turns where the shortest
        // rotation is ambiguous -- and asserts the three things that must hold
        // everywhere: the demand stays finite, it stays inside the limits, and
        // the aircraft arrives.
        //
        // Deterministic on purpose: a fixed LCG rather than rand(), so a
        // failure is the same failure on every machine and can be chased.
        uint32_t seed = 12345u;
        auto nextf = [&seed]() {
            seed = seed * 1664525u + 1013904223u;
            return (float)((seed >> 8) & 0xFFFFFFu) / (float)0x1000000u;
        };
        auto randomAttitude = [&nextf]() {
            // Shoemake: uniform over the rotation group, so this reaches the
            // upside-down cases that a sweep of Euler angles under-samples.
            const float u1 = nextf(), u2 = nextf(), u3 = nextf();
            const float s1 = std::sqrt(1.0f - u1), s2 = std::sqrt(u1);
            return Quaternionf(s1 * std::sin(6.2831853f * u2),
                               s1 * std::cos(6.2831853f * u2),
                               s2 * std::sin(6.2831853f * u3),
                               s2 * std::cos(6.2831853f * u3));
        };

        int nonfinite = 0, over_limit = 0, stranded = 0;
        float worst = 0.0f;
        for (int trial = 0; trial < 200; trial++) {
            Quaternionf q = randomAttitude();
            const Quaternionf target = randomAttitude();
            Attitude att;
            att.reset(q);
            att.setAttitudeTarget(target);
            for (int i = 0; i < 4000; i++) {        // 10 s
                att.update(q, dt);
                const Vector3f w = att.getRateTarget();
                if (!finiteVec(w)) { nonfinite++; break; }
                if (std::sqrt(w.x*w.x + w.y*w.y) > ATTITUDE_RATE_RP_MAX + 1e-3f
                        || std::fabs(w.z) > ATTITUDE_RATE_YAW_MAX + 1e-3f) {
                    over_limit++;
                }
                q = spin(q, w, dt);
            }
            const float left = angleBetween(q, target);
            if (left > worst) worst = left;
            if (left > 0.01f) stranded++;
        }
        check(nonfinite == 0, "200 random attitude pairs: the demand is always finite");
        check(over_limit == 0, "... always inside the rate limits");
        check(stranded == 0, "... and always arrives, inversions included");
        checkNear(worst * R2D, 0.0f, 0.1f, "worst error left after 10 s, over all 200 (deg)");
    }

    // =====================================================================
    section("Attitude: what it does when it is not flying, or is lied to");
    // =====================================================================
    {
        Attitude att;
        check(!att.isActive(), "a fresh Attitude is inactive");
        checkNear(att.getRateTarget().length(), 0.0f, 1e-9f, "and asks for no rotation");
        att.setEuler(30.0f * D2R, 0, 0);
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getRateTarget().length(), 0.0f, 1e-9f,
                  "update() before reset() does nothing -- no target is flown by accident");

        att.reset(quat(0, 0, 0));
        checkNear(att.getRateTarget().length(), 0.0f, 1e-9f,
                  "and engaging it holds where it is, rather than where the last target was");
        checkNear(att.getTiltError(), 0.0f, 1e-9f, "with no error to correct");
    }
    {
        // A dead estimator must not become a rate demand -- and the freeze
        // must be visible, or a stuck loop reads exactly like a healthy one
        // holding still.
        Attitude att;
        att.reset(quat(0, 0, 0));
        att.setEuler(10.0f * D2R, 0, 0);
        att.update(quat(0, 0, 0), dt);
        const Vector3f last = att.getRateTarget();
        check(att.getRejectedCount() == 0, "a healthy loop rejects nothing");

        const float nan = std::nanf("");
        for (int i = 0; i < 500; i++) att.update(Quaternionf(nan, nan, nan, nan), dt);
        checkNear((att.getRateTarget() - last).length(), 0.0f, 1e-9f,
                  "a dead attitude estimate freezes the output");
        check(att.getRejectedCount() == 500, "... and every refused sample is counted");

        att.update(Quaternionf(0, 0, 0, 0), dt);
        check(att.getRejectedCount() == 501,
              "a zero-length quaternion is refused too -- it carries no attitude");
        att.update(quat(0, 0, 0), 0.0f);
        check(att.getRejectedCount() == 502, "and so does a zero dt");

        // The gap must not be differentiated into a rate that never happened.
        att.setEuler(80.0f * D2R, 0, 0);
        att.update(quat(0, 0, 0), dt);
        checkNear(att.getFeedForward().length(), 0.0f, 1e-6f,
                  "and the target is not differenced across the gap");

        att.reset(quat(0, 0, 0));
        check(att.getRejectedCount() == 0,
              "reset() clears it -- a fault from a previous arm is not current");
    }
    {
        // Setters refuse what they cannot use, rather than accepting it and
        // flying it. A negative angle gain drives the aircraft AWAY from the
        // target, which is the one failure that cannot be flown out of.
        Attitude att;
        att.setGains(-1.0f, 1.0f, 1.0f);
        checkNear(att.getRollPID().getKp(), ATTITUDE_P_ROLL, 1e-6f, "a negative gain is refused");
        att.setGains(std::nanf(""), 1.0f, 1.0f);
        checkNear(att.getRollPID().getKp(), ATTITUDE_P_ROLL, 1e-6f, "and so is a NaN one");
        att.setRateLimits(0.0f, 0.0f);
        checkNear(att.getRateLimitRP(), ATTITUDE_RATE_RP_MAX, 1e-6f,
                  "a zero rate limit is refused -- it would forbid all correction");
        att.setGains(3.0f, 3.0f, 3.0f);
        checkNear(att.getRollPID().getKp(), 3.0f, 1e-6f, "a sane one is taken");

        // I and D are off, and stay off unless someone means it.
        checkNear(att.getRollPID().getKi(), 0.0f, 1e-9f, "the angle loop has no integrator");
        checkNear(att.getRollPID().getKd(), 0.0f, 1e-9f, "and no derivative");
    }

    return testReport("Controllers");
}
