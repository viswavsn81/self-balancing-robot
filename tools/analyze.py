#!/usr/bin/env python3
"""Analyze a trial log produced by log_trial.py.

Computes, per CLAUDE.md Phase 0: oscillation frequency, peak-to-peak angle,
settling time after the largest disturbance, drift rate, and motor
saturation percentage — plus loop-timing health. Stdlib only.

Usage:
    python3 tools/analyze.py logs/trial_003.csv [more.csv ...]
"""

import math
import sys
from pathlib import Path

FIELDS = ["time_ms", "raw_angle", "kalman_angle", "gyro_rate",
          "p_term", "i_term", "d_term", "motor_out", "loop_dt_us"]
OPTIONAL_FIELDS = ["gyro_y", "gyro_z"]   # appended by fw2.1+ (gyro_x=pitch is column 4)

SETTLE_BAND_DEG = 2.0    # settled = |angle - mean| stays inside this band
SETTLE_HOLD_S = 1.0      # ...for this long
SATURATION_PWM = 250     # |motor_out| at/above this counts as saturated


def load(path: Path):
    header_lines, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                if line:
                    header_lines.append(line)
                continue
            if line.startswith("time_ms"):
                continue
            parts = line.split(",")
            if len(parts) not in (len(FIELDS),
                                  len(FIELDS) + len(OPTIONAL_FIELDS)):
                continue  # torn line (logger started mid-row)
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue
    return header_lines, rows


def linear_fit_slope(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom == 0:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom


def analyze(path: Path) -> None:
    header, rows = load(path)
    print(f"\n=== {path} ===")
    for line in header:
        if line.startswith(("# Kp=", "# note:", "# state=")):
            print(line)
    if len(rows) < 20:
        print(f"only {len(rows)} data rows — not enough to analyze")
        return

    t = [r[0] / 1000.0 for r in rows]           # seconds
    angle = [r[2] for r in rows]                # kalman angle
    motor = [r[7] for r in rows]
    dt_us = [r[8] for r in rows]

    duration = t[-1] - t[0]
    n = len(rows)
    mean_angle = sum(angle) / n

    # --- loop timing health -------------------------------------------------
    mean_dt = sum(dt_us) / n
    max_dt = max(dt_us)
    overruns = sum(1 for d in dt_us if d > 1.5 * 5000)
    print(f"duration {duration:.1f} s, {n} rows "
          f"({n / duration:.0f} rows/s)")
    print(f"loop dt: mean {mean_dt:.0f} us, max {max_dt:.0f} us, "
          f"overruns(>7.5ms) {100 * overruns / n:.1f}%")

    # --- oscillation frequency (zero crossings of de-meaned angle) ----------
    centered = [a - mean_angle for a in angle]
    crossings = sum(
        1 for a, b in zip(centered, centered[1:]) if a * b < 0
    )
    osc_freq = crossings / (2.0 * duration) if duration > 0 else 0.0
    print(f"oscillation: {osc_freq:.2f} Hz ({crossings} zero-crossings)")

    # --- amplitude ----------------------------------------------------------
    p2p = max(angle) - min(angle)
    srt = sorted(angle)
    p2p_robust = srt[int(0.99 * (n - 1))] - srt[int(0.01 * (n - 1))]
    print(f"angle: mean {mean_angle:+.2f} deg, peak-to-peak {p2p:.2f} deg "
          f"(1-99% span {p2p_robust:.2f} deg)")

    # --- settling after largest excursion -----------------------------------
    i_peak = max(range(n), key=lambda i: abs(centered[i]))
    settle_time = None
    hold_start = None
    for i in range(i_peak, n):
        if abs(centered[i]) <= SETTLE_BAND_DEG:
            if hold_start is None:
                hold_start = i
            if t[i] - t[hold_start] >= SETTLE_HOLD_S:
                settle_time = t[hold_start] - t[i_peak]
                break
        else:
            hold_start = None
    peak_desc = (f"largest excursion {centered[i_peak]:+.1f} deg "
                 f"at t={t[i_peak]:.1f}s")
    if settle_time is not None:
        print(f"settling: {settle_time:.2f} s to ±{SETTLE_BAND_DEG}° "
              f"({peak_desc})")
    else:
        print(f"settling: NEVER settled to ±{SETTLE_BAND_DEG}° ({peak_desc})")

    # --- drift --------------------------------------------------------------
    drift = linear_fit_slope(t, angle)
    mean_motor = sum(motor) / n
    print(f"drift: {drift:+.3f} deg/s angle trend, "
          f"mean motor {mean_motor:+.1f} PWM "
          f"(nonzero mean = steady lean/creep -> adjust trim or Ki)")

    # --- saturation ---------------------------------------------------------
    sat = sum(1 for m in motor if abs(m) >= SATURATION_PWM)
    print(f"saturation: {100 * sat / n:.1f}% of samples at |PWM|>="
          f"{SATURATION_PWM}")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for arg in sys.argv[1:]:
        analyze(Path(arg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
