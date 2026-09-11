#!/usr/bin/env python3
"""
Live calibration check: plot raw against corrected sensor samples.

    python3 tools/stream_plot.py --port /dev/ttyACM0 --sensor mag

Sends STREAM,MAG (or STREAM,ACCL), reads the "$STREAM,..." lines the firmware
emits, and plots both clouds as you turn the airframe.

What you are looking for
-----------------------
A correction of the form  corrected = M * (raw - offset)  maps an ellipsoid
onto a sphere. So turn the airframe through as many orientations as you can and:

  RAW        an ellipsoid, off-centre. The offset is hard iron (mag) or sensor
             bias (accel); the squash is soft iron or per-axis scale error.
  CORRECTED  a sphere centred on the origin, of a known radius -- the local
             field strength for the magnetometer, exactly g for the
             accelerometer.

In each of the three 2-D projections that reads as a circle. The circles in all
three panels must have the SAME radius and the SAME centre: a correct
calibration makes the projection you are looking at irrelevant.

The magnitude panel is where you actually judge it, though. The eye is bad at
spotting a 5% radius error in a scatter plot and very good at spotting a sloped
line. |corrected| should sit flat on the reference, and the spread printed
underneath is the number to quote.

Coverage matters as much as fit: a cloud that only covers one side of the sphere
can be fitted well and still be wrong everywhere you did not visit. The
coverage figure is the fraction of orientation bins seen -- get it above ~70%
before believing anything else on the screen.

Needs: pyserial, matplotlib, numpy.
"""

import argparse
import collections
import math
import sys
import threading

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")
try:
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    sys.exit("matplotlib/numpy missing:  pip install matplotlib numpy")


# --------------------------------------------------------------------------
# Serial reader
# --------------------------------------------------------------------------
class Reader(threading.Thread):
    """Drains the port in its own thread.

    Not an optimisation. A USB CDC read that blocks inside the animation
    callback stalls the GUI event loop, and the window stops repainting and
    reports "not responding" while the data is arriving perfectly well.
    """

    def __init__(self, port, baud, maxlen):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.raw = collections.deque(maxlen=maxlen)
        self.cor = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.lines = 0
        self.bad = 0
        self.running = True
        self.sensor_char = None

    def send(self, cmd):
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode("ascii", "replace").strip()
            except Exception:
                continue
            if not line.startswith("$STREAM,"):
                continue
            parts = line.split(",")
            # "$STREAM,MODE,n" is the acknowledgement, not a sample.
            if len(parts) == 3 and parts[1] == "MODE":
                continue
            if len(parts) != 8:
                self.bad += 1
                continue
            try:
                v = [float(x) for x in parts[2:8]]
            except ValueError:
                self.bad += 1
                continue
            if not all(math.isfinite(x) for x in v):
                self.bad += 1
                continue
            self.sensor_char = parts[1]
            with self.lock:
                self.raw.append(v[0:3])
                self.cor.append(v[3:6])
                self.lines += 1

    def snapshot(self):
        with self.lock:
            r = np.array(self.raw, dtype=float) if self.raw else np.empty((0, 3))
            c = np.array(self.cor, dtype=float) if self.cor else np.empty((0, 3))
            return r, c, self.lines, self.bad

    def close(self):
        self.running = False
        try:
            self.send("STREAM,OFF")
            self.ser.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# Coverage: how much of the sphere has actually been visited
# --------------------------------------------------------------------------
def coverage(points, nbins=64):
    """Fraction of direction bins occupied, by the same scheme the firmware
    calibrators use -- azimuth x elevation on the unit sphere."""
    if len(points) < 10:
        return 0.0
    n = np.linalg.norm(points, axis=1)
    ok = n > 1e-9
    if not np.any(ok):
        return 0.0
    u = points[ok] / n[ok, None]
    az = np.floor((np.arctan2(u[:, 1], u[:, 0]) + math.pi) / (2 * math.pi) * 8)
    el = np.floor((np.arcsin(np.clip(u[:, 2], -1, 1)) + math.pi / 2) / math.pi * 8)
    seen = set(zip(az.astype(int), el.astype(int)))
    return len(seen) / float(nbins)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored by USB CDC, which runs at bus speed regardless")
    ap.add_argument("--sensor", choices=["mag", "accl"], default="mag")
    ap.add_argument("--points", type=int, default=4000,
                    help="ring length; older samples scroll off")
    ap.add_argument("--radius", type=float, default=None,
                    help="expected |corrected|. Defaults to g for accl, and to "
                         "the running mean for mag, whose field strength is "
                         "local and not known here")
    ap.add_argument("--save", metavar="CSV",
                    help="also append every sample to this file")
    args = ap.parse_args()

    rd = Reader(args.port, args.baud, args.points)
    rd.start()
    rd.send("STREAM,OFF")          # a stream from a previous run would double up
    rd.send("STREAM," + ("MAG" if args.sensor == "mag" else "ACCL"))

    csv = open(args.save, "a") if args.save else None
    if csv:
        csv.write("# raw_x,raw_y,raw_z,cor_x,cor_y,cor_z\n")
    written = 0

    unit = "Gauss" if args.sensor == "mag" else "m/s^2"
    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title("Calibration check -- %s" % args.sensor)
    gs = fig.add_gridspec(2, 3, height_ratios=[3, 2], hspace=0.30, wspace=0.28)

    PAIRS = [(0, 1, "X", "Y"), (0, 2, "X", "Z"), (1, 2, "Y", "Z")]
    axes, raw_sc, cor_sc, circles = [], [], [], []
    for i, (_, _, la, lb) in enumerate(PAIRS):
        ax = fig.add_subplot(gs[0, i])
        # "box", not "datalim". With datalim, matplotlib silently overrides the
        # set_xlim/set_ylim below to satisfy the aspect ratio -- and the three
        # panels then end up on DIFFERENT scales, which destroys the one thing
        # this layout exists for: comparing the radius across projections.
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.25, linewidth=0.5)
        ax.axhline(0, color="0.6", linewidth=0.8)
        ax.axvline(0, color="0.6", linewidth=0.8)
        ax.set_xlabel("%s (%s)" % (la, unit))
        ax.set_ylabel("%s (%s)" % (lb, unit))
        ax.set_title("%s%s" % (la, lb))
        raw_sc.append(ax.scatter([], [], s=3, c="#c44e52", alpha=0.35, label="raw"))
        cor_sc.append(ax.scatter([], [], s=3, c="#4c72b0", alpha=0.55, label="corrected"))
        circles.append(ax.add_artist(plt.Circle((0, 0), 1.0, fill=False,
                                                color="#55a868", linewidth=1.4,
                                                linestyle="--", zorder=5)))
        axes.append(ax)
    axes[0].legend(loc="upper left", fontsize=8, framealpha=0.9)

    ax_mag = fig.add_subplot(gs[1, :])
    ax_mag.grid(alpha=0.25, linewidth=0.5)
    ax_mag.set_xlabel("sample")
    ax_mag.set_ylabel("|vector| (%s)" % unit)
    ax_mag.set_title("Magnitude -- this is the one that decides it")
    (ln_raw,) = ax_mag.plot([], [], linewidth=0.8, color="#c44e52", label="|raw|")
    (ln_cor,) = ax_mag.plot([], [], linewidth=1.0, color="#4c72b0", label="|corrected|")
    ref = ax_mag.axhline(1.0, color="#55a868", linewidth=1.2, linestyle="--",
                         label="expected")
    ax_mag.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.9)
    txt = ax_mag.text(0.01, 0.04, "", transform=ax_mag.transAxes, fontsize=9,
                      family="monospace", va="bottom")

    def update(_frame):
        nonlocal written
        raw, cor, nlines, nbad = rd.snapshot()
        if len(cor) < 2:
            txt.set_text("waiting for data on %s ...\n"
                         "if nothing arrives: check the port, and that the "
                         "firmware ACKed STREAM" % args.port)
            return

        if csv and nlines > written:
            for i in range(len(cor) - min(len(cor), nlines - written), len(cor)):
                csv.write("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (*raw[i], *cor[i]))
            written = nlines
            csv.flush()

        n_cor = np.linalg.norm(cor, axis=1)
        n_raw = np.linalg.norm(raw, axis=1)
        expected = args.radius if args.radius is not None else (
            9.80665 if args.sensor == "accl" else float(np.mean(n_cor)))

        for i, (a, b, _, _) in enumerate(PAIRS):
            raw_sc[i].set_offsets(np.c_[raw[:, a], raw[:, b]])
            cor_sc[i].set_offsets(np.c_[cor[:, a], cor[:, b]])
            circles[i].set_radius(expected)
            lim = max(float(np.max(np.abs(raw))), expected) * 1.15
            axes[i].set_xlim(-lim, lim)
            axes[i].set_ylim(-lim, lim)

        x = np.arange(len(n_cor))
        ln_raw.set_data(x, n_raw)
        ln_cor.set_data(x, n_cor)
        ref.set_ydata([expected, expected])
        ax_mag.set_xlim(0, max(50, len(n_cor)))
        lo = min(float(np.min(n_cor)), float(np.min(n_raw)), expected)
        hi = max(float(np.max(n_cor)), float(np.max(n_raw)), expected)
        pad = max((hi - lo) * 0.12, expected * 0.02)
        ax_mag.set_ylim(lo - pad, hi + pad)

        # The numbers to quote. Spread as a percentage of the radius is the
        # figure of merit: it is what "the sphere is a sphere" means, and it is
        # comparable between the two sensors and between airframes.
        spread = (float(np.max(n_cor)) - float(np.min(n_cor))) / expected * 100.0
        sigma = float(np.std(n_cor)) / expected * 100.0
        cov = coverage(cor) * 100.0
        verdict = ("GOOD" if spread < 5.0 and cov > 70.0 else
                   "TURN IT MORE" if cov <= 70.0 else "POOR")
        txt.set_text(
            "n=%-6d  bad=%-3d  coverage=%4.0f%%   |cor| mean %.4f  "
            "spread %.1f%%  sigma %.2f%%   [%s]\n"
            "|raw|  mean %.4f  spread %.1f%%          expected %.4f %s"
            % (nlines, nbad, cov, float(np.mean(n_cor)), spread, sigma, verdict,
               float(np.mean(n_raw)),
               (float(np.max(n_raw)) - float(np.min(n_raw))) / expected * 100.0,
               expected, unit))

    # cache_frame_data=False: this is an endless live stream, and the default
    # would retain every frame's data for replay until the process runs out of
    # memory.
    anim = FuncAnimation(fig, update, interval=120, cache_frame_data=False)
    fig._keep_anim = anim   # FuncAnimation dies if only a local refers to it

    try:
        plt.show()
    finally:
        rd.close()
        if csv:
            csv.close()
        print("stream stopped")


if __name__ == "__main__":
    main()
