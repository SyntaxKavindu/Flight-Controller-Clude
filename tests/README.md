# Host tests for the estimator and calibration core

These build and run the flight stack's core on a workstation, with no hardware
and no dependencies beyond a C++17 compiler.

```
cd tests && make
```

| Target | What it covers |
|---|---|
| `make esekf` | Alignment, propagation, every fusion path, integrity flags, fault recovery, setter validation |
| `make cal`   | The three calibration engines, driven directly |
| `make integ` | The `Calibrator` facade, the sensor frontends, the EEPROM record |
| `make indicator` | The status panel: pattern rendering, lamp test, phase handling, GPIO write suppression |
| `make firmware` | Links the whole stack including `Drone.cpp` and runs `loop()` |

`make cal` links **only** `AccelerometerCalibrator`, `CompassCalibrator` and
`LevelCalibrator` — no facade, no HAL, no `stub/`. If that link ever starts
needing `stub/`, a board dependency has crept back into a class that is meant to
be portable.

## What these do and do not prove

They cover the maths and the state machines: the parts that are pure functions
of their inputs, where a regression is silent and a test is the only thing that
finds it. Several checks exist because the behaviour they pin was previously
wrong — the magnetometer disturbance test and the FRD level-calibration test in
particular.

They do **not** cover timing, SPI/I2C behaviour, USB throughput, sensor noise,
vibration, or anything about the real parts. `stub/` always succeeds and its
clock only advances where a test moves `g_stub_tick` by hand (the indicator
suite is the only one that does); a fake ICM-42688-P would only test the fake. Those belong
on the bench.

## Conventions the tests assume

- Body frame **FRD**: X forward, Y right, Z **down**.
- A level, upright airframe reads `(0, 0, -g)` — an accelerometer measures
  specific force, so the axis pointing *up* reads `+1 g`.
- Earth frame **NED**, down positive; gravity is `(0, 0, +9.80665)`.
- `AccelPosition::Z_UP` means body +Z points at the sky, i.e. **inverted**.

`tests/test.hpp` has helpers (`phys::restAccel`, `phys::bodyField`) that build
samples in these conventions; use them rather than hand-rolling signs.

## Adding a test

`check(cond, "what")` and `checkNear(got, want, tol, "what")`, grouped with
`section("name")`, then `return testReport("Suite")`. Any non-zero exit fails
the `make`.
