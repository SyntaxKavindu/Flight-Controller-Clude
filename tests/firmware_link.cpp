/*
 * Not a test -- a link check. It builds the entire flight stack, Drone.cpp
 * included, exactly as main.cpp would, and runs a few loop() iterations.
 *
 * The suites either side of it exercise the maths in isolation, which is where
 * the bugs are; this catches the other kind: a change that is correct on its own
 * and does not build, or does not survive one pass, in the real assembly.
 */
#include "Drone.hpp"

// No object is declared here any more: Drone.cpp defines the one `drone`, and
// a second would be a compile error rather than a silently divergent estimator.
int main(void)
{
    drone.init();
    for (int i = 0; i < 16; i++) {
        drone.loop();
    }
    drone.reportDiagnostics();
    return 0;
}
