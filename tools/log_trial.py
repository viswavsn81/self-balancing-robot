#!/usr/bin/env python3
"""Record a robot trial to logs/trial_NNN.csv.

Opens the serial console, captures the firmware's status (gains, trim,
deadband) into the file header, starts the telemetry stream, and records
until Ctrl-C or --seconds elapses. While logging, anything you type is
forwarded to the robot, so this doubles as the tuning console:

    a       arm        x  EMERGENCY STOP
    p 30    set Kp     s  toggle stream    h  help

By default an 'x' (emergency stop / disarm) is sent when the logger exits,
so the robot is never left armed without a console attached. Use
--no-estop to opt out (not recommended).

Usage:
    python3 tools/log_trial.py [--port /dev/ttyUSB0] [--seconds 20]
                               [--note "Kp sweep step 3"]
"""

import argparse
import re
import select
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

REPO = Path(__file__).resolve().parent.parent
LOGS = REPO / "logs"
BAUD_DEFAULT = 115200


def next_trial_path() -> Path:
    LOGS.mkdir(exist_ok=True)
    nums = [
        int(m.group(1))
        for p in LOGS.glob("trial_*.csv")
        if (m := re.match(r"trial_(\d+)\.csv$", p.name))
    ]
    return LOGS / f"trial_{max(nums, default=0) + 1:03d}.csv"


def read_status(ser: serial.Serial, timeout: float = 2.0) -> list[str]:
    """Send '?' and collect the '#' status lines the firmware replies with."""
    ser.reset_input_buffer()
    ser.write(b"?\n")
    lines, deadline = [], time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if raw.startswith("#"):
            lines.append(raw)
            if raw.startswith("# Kp="):  # gains line is the last status line
                break
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this many seconds (default: Ctrl-C)")
    ap.add_argument("--note", default="", help="free-text note for the header")
    ap.add_argument("--no-estop", action="store_true",
                    help="do NOT send 'x' (disarm) when the logger exits")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    time.sleep(0.3)  # some clones reset on port open; give the banner a moment

    status = read_status(ser)
    path = next_trial_path()
    stop = threading.Event()

    with open(path, "w") as f:
        f.write(f"# trial: {path.name}\n")
        f.write(f"# date: {datetime.now().isoformat(timespec='seconds')}\n")
        if args.note:
            f.write(f"# note: {args.note}\n")
        for line in status:
            f.write(line + "\n")
        f.flush()

        def reader() -> None:
            streaming_started = False
            while not stop.is_set():
                raw = ser.readline().decode("ascii", errors="replace").strip()
                if not raw:
                    continue
                f.write(raw + "\n")
                if raw.startswith("#") or not streaming_started:
                    print(raw)  # echo console chatter and the CSV header
                    if raw.startswith("time_ms,"):
                        streaming_started = True

        t = threading.Thread(target=reader, daemon=True)
        t.start()

        # Start the stream unless status says it is already on.
        if not any("stream=1" in s for s in status):
            ser.write(b"s\n")

        print(f"logging to {path} — Ctrl-C to stop; input is forwarded to robot")
        deadline = time.monotonic() + args.seconds if args.seconds else None
        try:
            while deadline is None or time.monotonic() < deadline:
                # forward operator commands typed on stdin
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if r:
                    cmd = sys.stdin.readline()
                    if not cmd:
                        break
                    ser.write(cmd.encode("ascii", errors="ignore"))
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            t.join(timeout=1.0)
            if not args.no_estop:
                ser.write(b"x\n")   # never leave the robot armed w/o console
                print("\nsent 'x' (disarm)")
            ser.write(b"s\n")       # stream off (toggle)
            time.sleep(0.2)
            ser.close()

    print(f"saved {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
