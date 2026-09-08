/*
 * The control cascade's implemented half: PID, Position, Velocity.
 *
 * Written against the PHYSICS rather than against the implementation wherever
 * possible -- a test that restates the code cannot catch the code being wrong.
 * So the hover checks compute what a multirotor must do from g and the lean
 * angle, and compare; they do not re-derive it the way Velocity.cpp does.
 */
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

    return testReport("Controllers");
}
