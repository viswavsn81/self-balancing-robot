#!/usr/bin/env python3
"""Run one numbered Phase 2 balance trial end-to-end.

Automates everything except the human's hands: sets gains, waits for
hum-guided arming (operator stands the robot up, releases on wheel
engage), logs until the fall/window end, analyzes, appends to
logs/tuning_journal.md, and proposes the next single gain change per the
CLAUDE.md one-change-per-trial protocol.

Usage:
    python3 tools/balance_trial.py --kp 15 --ki 0.5 --kd 0.2 [--trim 6.0]
                                   [--seconds 90] [--note "..."]
    python3 tools/balance_trial.py --repeat        # re-run last gains

The operator needs seconds per trial: stand it up when told, release on
hum->engage, tip past 45 deg when it's clearly done (or let it fall into
your hands). Everything else is automatic; the script disarms on exit.
"""

import argparse
import re
import statistics as st
import sys
import time
from pathlib import Path

import serial

REPO = Path(__file__).resolve().parent.parent
LOGS = REPO / "logs"
JOURNAL = LOGS / "tuning_journal.md"
PORT, BAUD = "/dev/ttyUSB0", 250000


def next_trial_path():
    nums = [int(m.group(1)) for p in LOGS.glob("trial_*.csv")
            if (m := re.match(r"trial_(\d+)\.csv$", p.name))]
    n = max(nums, default=0) + 1
    return LOGS / f"trial_{n:03d}.csv", n


def parse_rows(path):
    rows = []
    for line in open(path):
        if line[:1].isdigit():
            p = line.strip().split(",")
            if len(p) == 17:
                try:
                    rows.append([float(x) for x in p])
                except ValueError:
                    pass
    return rows


def smooth(x, w=8):
    return [sum(x[max(0, i - w):i + w + 1]) / len(x[max(0, i - w):i + w + 1])
            for i in range(len(x))]


def analyze(rows, trim):
    t = [r[0] / 1000 for r in rows]
    kal = [r[2] for r in rows]
    out = [r[7] for r in rows]
    vbat = [r[16] for r in rows]
    armed = [i for i in range(len(rows)) if out[i] != 0]
    if not armed:
        return {"armed_s": 0.0, "verdict": "never-armed"}
    runs, s, prev = [], armed[0], armed[0]
    for i in armed[1:]:
        if i - prev > 10:
            runs.append((s, prev))
            s = i
        prev = i
    runs.append((s, prev))
    a, b = max(runs, key=lambda r: r[1] - r[0])
    seg = kal[a:b + 1]
    dur = t[b] - t[a]
    slow = smooth(seg)
    ripple = st.pstdev([x - y for x, y in zip(seg, slow)])
    sat = sum(1 for o in out[a:b + 1] if abs(o) >= 250) / (b - a + 1)
    drift = (slow[-1] - slow[0]) / dur if dur > 1 else 0.0
    window_limited = b >= len(rows) - 30
    m = {"armed_s": round(dur, 1), "ripple": round(ripple, 2),
         "sat_pct": round(100 * sat, 1),
         "wander": (round(min(slow) - trim, 1), round(max(slow) - trim, 1)),
         "drift_dps": round(drift, 3),
         "vbat": round(st.mean(vbat), 2),
         "end": "window" if window_limited else "fall",
         "n_runs": len(runs)}
    # verdict per protocol
    if dur < 5:
        m["verdict"] = "falls-fast"
    elif not window_limited and max(abs(min(slow) - trim),
                                    abs(max(slow) - trim)) > 15:
        m["verdict"] = "drifts-out"
    elif ripple > 3.0:
        m["verdict"] = "chatter-high"
    else:
        m["verdict"] = "good"
    return m


def propose(m, kp, ki, kd):
    """One change per trial, per CLAUDE.md Phase 2 protocol + hard-won
    constraints (Kd>=0.5 destabilizes; Ki=1.5 was too hot)."""
    v = m.get("verdict")
    if v == "never-armed":
        return None, "no data — repeat the trial"
    if v == "falls-fast":
        if kd > 0.25:
            return (kp, ki, 0.2), "Kd back to 0.2 (high Kd destabilizes)"
        return (kp + 3, ki, kd), "raise Kp (falls limp = too little P)"
    if v == "drifts-out":
        if ki < 0.9:
            return (kp, min(1.0, ki + 0.3), kd), "raise Ki toward 1.0 (drift)"
        return (kp, ki, kd), "Ki at cap — check trim instead of gains"
    if v == "chatter-high":
        if kd > 0.05:
            return (kp, ki, max(0.05, kd - 0.1)), "lower Kd (chatter)"
        return (kp - 2, ki, kd), "lower Kp (chatter with low Kd)"
    return (kp, ki, kd), "good — repeat to confirm, then stop tuning"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=15.0)
    ap.add_argument("--ki", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--trim", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    path, n = next_trial_path()
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = PORT, BAUD, 0.3
    ser.dtr = ser.rts = False
    ser.open()
    time.sleep(4.5)
    ser.reset_input_buffer()
    for c in (f"p {args.kp}", f"i {args.ki}", f"d {args.kd}",
              f"o {args.trim}"):
        ser.write((c + "\n").encode())
        time.sleep(0.2)
    ser.write(b"s\n")
    time.sleep(0.2)
    ser.write(b"a\n")

    print(f"=== trial {n:03d}: Kp={args.kp} Ki={args.ki} Kd={args.kd} "
          f"trim={args.trim} ===")
    print(">>> STAND THE ROBOT UP now; release when the wheels engage. <<<")

    t0 = time.monotonic()
    armed_seen = False
    with open(path, "w") as f:
        f.write(f"# trial: {path.name}\n# note: balance_trial {args.note}\n"
                f"# gains: Kp={args.kp} Ki={args.ki} Kd={args.kd} "
                f"trim={args.trim}\n")
        while time.monotonic() - t0 < args.seconds + 120:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            f.write(line + "\n")
            if line.startswith("#"):
                print(f"  {line}")
                if "ARMED" in line and "DISARMED" not in line:
                    armed_seen = True
                    t0 = time.monotonic()          # clock from arm
                if line.startswith("# DISARMED") and armed_seen:
                    break
            if armed_seen and time.monotonic() - t0 > args.seconds:
                break
    ser.write(b"x\ns\n")
    time.sleep(0.3)
    ser.close()

    rows = parse_rows(path)
    m = analyze(rows, args.trim)
    print("metrics:", m)
    nxt, why = propose(m, args.kp, args.ki, args.kd)
    print(f"proposal: {nxt}  ({why})")

    with open(JOURNAL, "a") as j:
        j.write(f"| {n:03d} | {args.kp}/{args.ki}/{args.kd} | {args.trim} | "
                f"{m.get('armed_s')} | {m.get('ripple','-')} | "
                f"{m.get('sat_pct','-')} | {m.get('verdict')} | "
                f"{why} | {args.note} |\n")
    print(f"saved {path.name}; journal updated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
