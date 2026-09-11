# Calibration check — plot raw against corrected

A calibration is `corrected = M * (raw - offset)`, which maps an ellipsoid onto
a sphere. So the check is: turn the airframe through every orientation you can,
and see whether the corrected cloud is a sphere of the right radius.

```
pip install pyserial matplotlib numpy
python3 tools/stream_plot.py --port /dev/ttyACM0 --sensor mag
```

`--sensor accl` for the accelerometer. `Ctrl-C` or closing the window sends
`STREAM,OFF`.

## Reading the screen

**The three scatter panels.** Raw is red, corrected blue, and the green dashed
circle is the expected radius.

Corrected points fill a **disc, not a ring** — a 2-D projection of a hollow
sphere is a filled disc, because points on the far side project inside the
outline. It is the **edge** you compare with the green circle, not the middle.

What you want:

| | raw (red) | corrected (blue) |
|---|---|---|
| centre | offset from origin | on the origin |
| shape | squashed | round |
| edge | anywhere | on the green circle |

The three panels must agree with each other. A correct calibration makes the
projection you happen to be looking at irrelevant, so XY, XZ and YZ should show
the same radius and the same centre. One panel disagreeing means one axis is
wrong — which is the failure a single 3-D view hides.

**The magnitude panel is where you actually judge it.** The eye is bad at
spotting a 5% radius error in a scatter plot and very good at spotting a sloped
line. `|corrected|` should sit flat on the reference.

## The numbers

```
n=1500  bad=0  coverage= 100%   |cor| mean 0.4000  spread 0.0%  sigma 0.00%   [GOOD]
```

- **coverage** — fraction of orientation bins visited. **Get this above 70%
  before believing anything else on the screen.** A cloud covering one side of
  the sphere can be fitted beautifully and still be wrong everywhere you did not
  go, and that is the failure mode that reaches a first flight.
- **spread** — `(max − min) / radius`. The figure of merit. Under 5% is good.
- **sigma** — standard deviation, same units. Less sensitive to a single outlier
  than spread, so quote both.
- **bad** — malformed lines. Non-zero means the link is dropping bytes; check
  `$ERR,RX OVERFLOW`.

## What the radius is compared against

**Accelerometer:** exactly `g` (9.80665 m/s²), known. An absolute scale error
shows up directly.

**Magnetometer:** defaults to the running mean of `|corrected|`, because the
local field strength is not known to this script. That makes *spread* and
*sigma* meaningful but **cannot detect a systematic scale error** — the circle
follows the data. To check the absolute value, look your location up in a World
Magnetic Model and pass it:

```
python3 tools/stream_plot.py --port /dev/ttyACM0 --sensor mag --radius 0.41
```

## Collecting the data

Turn the airframe slowly through as many orientations as you can — the same
motion a `CALMAG` wants. Slowly matters: the raw and corrected values in one
line are the same sample, but consecutive lines are 40 ms apart, and whipping
the airframe around just adds noise.

For the accelerometer, rest it on each face and edge rather than waving it:
`--sensor accl` measures specific force, so any movement adds acceleration that
is not gravity and inflates the spread for reasons that have nothing to do with
the calibration.

`--save capture.csv` appends every sample if you want to compare two runs later.

## If nothing appears

1. `DIAG` should answer, and report `$DIAG,STREAM mode=1` or `mode=2`.
2. `$HB` carries `rx=` and `cmd=`. `rx=0` means nothing is reaching the
   firmware at all — see `Telemetry::getRxByteCount()`.
3. The port is busy if another terminal has it open. Close that first.
