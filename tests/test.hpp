/*
 * Minimal check harness. No framework: these tests must build with nothing but
 * a C++17 compiler, so that "does the core still work" is one command on any
 * machine rather than a dependency install.
 */
#ifndef TESTS_TEST_HPP_
#define TESTS_TEST_HPP_

#include <cstdio>
#include <cmath>

inline int &testFailures() { static int n = 0; return n; }
inline int &testCount()    { static int n = 0; return n; }

inline void check(bool ok, const char *what)
{
    testCount()++;
    if (!ok) { testFailures()++; std::printf("  FAIL  %s\n", what); }
    else     { std::printf("  ok    %s\n", what); }
}

inline void checkNear(float got, float want, float tol, const char *what)
{
    const bool ok = std::isfinite(got) && std::fabs(got - want) <= tol;
    testCount()++;
    if (!ok) {
        testFailures()++;
        std::printf("  FAIL  %s (got %.5f, want %.5f +/- %.5f)\n", what,
                    (double)got, (double)want, (double)tol);
    } else {
        std::printf("  ok    %s (%.5f)\n", what, (double)got);
    }
}

inline void section(const char *name) { std::printf("\n== %s ==\n", name); }

inline int testReport(const char *suite)
{
    std::printf("\n%s: %d checks, %d failed -- %s\n\n", suite, testCount(),
                testFailures(), testFailures() ? "FAILURES" : "ALL PASS");
    return testFailures() != 0;
}

// ---- Shared physical helpers, in the conventions this stack uses ----
// Body frame FRD (X forward, Y right, Z down). NED earth frame, down positive.
namespace phys {

constexpr float G    = 9.80665f;
constexpr float D2R  = 0.0174532925199433f;
constexpr float R2D  = 57.2957795130823f;

// R_b^n for ZYX (yaw-pitch-roll) Euler angles, radians.
inline void dcm(float roll, float pitch, float yaw, float R[3][3])
{
    const float cr = std::cos(roll),  sr = std::sin(roll);
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const float cy = std::cos(yaw),   sy = std::sin(yaw);
    R[0][0] = cy*cp; R[0][1] = cy*sp*sr - sy*cr; R[0][2] = cy*sp*cr + sy*sr;
    R[1][0] = sy*cp; R[1][1] = sy*sp*sr + cy*cr; R[1][2] = sy*sp*cr - cy*sr;
    R[2][0] = -sp;   R[2][1] = cp*sr;            R[2][2] = cp*cr;
}

// Rotate an NED vector into the body frame: v_body = R^T * v_ned.
inline Vector3f toBody(float roll, float pitch, float yaw, const Vector3f &v)
{
    float R[3][3]; dcm(roll, pitch, yaw, R);
    return Vector3f(R[0][0]*v.x + R[1][0]*v.y + R[2][0]*v.z,
                    R[0][1]*v.x + R[1][1]*v.y + R[2][1]*v.z,
                    R[0][2]*v.x + R[1][2]*v.y + R[2][2]*v.z);
}

// Specific force an accelerometer reads at rest: f = -g_ned, in body axes.
inline Vector3f restAccel(float roll, float pitch)
{
    return toBody(roll, pitch, 0.0f, Vector3f(0.0f, 0.0f, -G));
}

// A representative Earth field: ~0.48 Gauss, 60 deg inclination, declination 0.
inline Vector3f earthFieldNED() { return Vector3f(0.24f, 0.0f, 0.415f); }

inline Vector3f bodyField(float roll, float pitch, float yaw)
{
    return toBody(roll, pitch, yaw, earthFieldNED());
}

} // namespace phys

#endif /* TESTS_TEST_HPP_ */
