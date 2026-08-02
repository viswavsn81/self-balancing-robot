#!/usr/bin/env python3
"""Cart-pole pre-screen for balance gains (CLAUDE.md Phase 2 option).

Model: inverted pendulum on a wheeled cart, with the real robot's
parameters where measured and documented guesses where not:
  mass 0.389 kg (measured), CoM height 0.06 m (GUESS: IMU sits at 0.05 m,
  CoM a shade above), wheel r 0.0315 m (measured), PWM deadband 18
  (bench test 4: breakaway 20/18 at 8.2 V), motor first-order lag 50 ms
  (GUESS), max wheel force at PWM 255 calibrated so the KNOWN-GOOD gains
  (Kp15 Ki0.5 Kd0.2 -> stable) and KNOWN-BAD (Kp25 Ki5 -> unstable,
  Kd0.5 -> unstable chatter) reproduce. This is a RANKING tool for
  pre-screening gain sets, not ground truth; the floor decides.

Controller mirrors firmware: 200 Hz, P on angle error, clamped I,
D by differentiating a lagged (kalman-like) angle estimate, output
+/-255 with deadband mapping.

Usage: python3 tools/simulate.py [--grid]

STATUS (2026-08-02, 2nd pass): STILL UNVALIDATED. Even with the
MEASURED motor model (sysid_002: speed-source, k=0.137 cm/s/count,
tau=0.13 s), the known-good gains die in-model. Diagnosis: this
cart-pole abstraction couples motor to body only through cart
acceleration; the real robot's dominant control path at small angles is
the wheel motors' direct REACTION TORQUE on the body, which is absent
here (measured top speed is only 33 cm/s, yet real logs show recoveries
from 21 deg — impossible through cart acceleration alone). A future
model must add the torque-reaction term. Until then: floor evidence
outranks anything this prints.
"""

import argparse
import math
import random

# --- plant parameters ---
M_BODY = 0.389        # kg
L = 0.06              # m, CoM height above axle (guess; IMU at 0.05)
R_WHEEL = 0.0315      # m
G = 9.81
J = M_BODY * L * L    # point-mass pendulum inertia about axle
MOTOR_TAU = 0.13      # s, MEASURED (sysid_002, median tau both wheels)
V_PER_OUT = 0.00137   # m/s per control count, MEASURED (sysid_002 fit,
                      # k=0.137 cm/s/count mean incl. firmware deadband map)
DEADBAND = 18         # PWM counts (bench test 4)
DT = 0.005            # 200 Hz
SENSOR_LAG = 0.030    # s, kalman-ish angle lag
NOISE_DEG = 0.12      # raw angle noise std (trial logs)
# Gearbox friction is dominant on this robot: it free-stands ~2 deg off
# balance on stiction alone (friction-cone measurement, trials 029/030).
F_COUL = 0.13         # N, Coulomb friction ~ m*g*tan(2 deg)
B_VISC = 0.4          # N per m/s (guess)


def simulate(kp, ki, kd, seconds=12.0, seed=1, tilt0=3.0, push_n=1.0):
    rng = random.Random(seed)
    th = math.radians(tilt0)   # pendulum angle from vertical
    om = 0.0                   # angular rate
    v = 0.0                    # cart velocity
    f = 0.0                    # actual motor force (lagged)
    integ = 0.0
    est = th                   # lagged sensor estimate
    prev_est = th
    sat_t = 0.0
    ripple = []
    t = 0.0
    push_at = seconds * 0.5
    while t < seconds:
        # sensor: first-order lag + noise
        meas = th + math.radians(rng.gauss(0, NOISE_DEG))
        est += (DT / (SENSOR_LAG + DT)) * (meas - est)

        # Error sign includes the motor-direction inversion the firmware
        # keeps in MOTOR_SIGN_*: positive tilt must drive the cart toward
        # the fall (+a_cart reduces theta in this plant convention).
        err = math.degrees(est)           # setpoint 0
        integ += ki * err * DT
        integ = max(-255, min(255, integ))
        d = -kd * (math.degrees(est) - math.degrees(prev_est)) / DT
        prev_est = est
        out = max(-255, min(255, kp * err + integ + d))
        if abs(out) >= 250:
            sat_t += DT

        # MEASURED plant (sysid_002): geared motor = wheel-speed source
        # with first-order lag; cart velocity tracks commanded speed.
        v_cmd = V_PER_OUT * out
        v_new = v + (DT / (MOTOR_TAU + DT)) * (v_cmd - v)
        a_cart = (v_new - v) / DT
        v = v_new
        al = (G * math.sin(th) - a_cart * math.cos(th)) / L
        om += al * DT
        th += om * DT
        ripple.append(math.degrees(th))

        # disturbance push (impulse on angle rate)
        if push_at and t >= push_at:
            om += push_n * 0.05 / (M_BODY * L)   # impulse ~50 mN.s at CoM
            push_at = None

        if abs(th) > math.radians(40):
            return {"alive": t, "ok": False, "sat": sat_t,
                    "ripple": _std(ripple[-400:])}
        t += DT
    return {"alive": seconds, "ok": True, "sat": sat_t,
            "ripple": _std(ripple[-400:])}


def _std(xs):
    m = sum(xs) / len(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


def score(kp, ki, kd):
    """Aggregate across seeds/conditions; higher is better."""
    total = 0.0
    for seed in (1, 2, 3):
        for tilt0, push in ((2.0, 0.5), (4.0, 1.0), (3.0, 2.0)):
            r = simulate(kp, ki, kd, seed=seed, tilt0=tilt0, push_n=push)
            total += (r["alive"] if not r["ok"] else 12.0 + 4.0) \
                     - 2.0 * r["ripple"] - 2.0 * r["sat"]
    return total / 9.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", action="store_true")
    args = ap.parse_args()

    anchors = [(15, 0.5, 0.2, "known-good floor gains"),
               (25, 5.0, 0.5, "old repo gains (known bad)"),
               (15, 0.5, 0.5, "high Kd (known chatter-unstable)")]
    print("=== sanity anchors (sim must roughly agree with reality) ===")
    for kp, ki, kd, note in anchors:
        s = score(kp, ki, kd)
        print(f"  Kp={kp:5.1f} Ki={ki:4.1f} Kd={kd:4.2f}  score {s:7.2f}  ({note})")

    if not args.grid:
        return
    print("=== grid search ===")
    results = []
    for kp in (10, 12, 15, 18, 22, 26):
        for kd in (0.0, 0.1, 0.2, 0.3, 0.45):
            for ki in (0.0, 0.3, 0.5, 1.0):
                results.append((score(kp, ki, kd), kp, ki, kd))
    results.sort(reverse=True)
    print("top 8:")
    for s, kp, ki, kd in results[:8]:
        print(f"  score {s:7.2f}  Kp={kp} Ki={ki} Kd={kd}")
    print("bottom 3 (for contrast):")
    for s, kp, ki, kd in results[-3:]:
        print(f"  score {s:7.2f}  Kp={kp} Ki={ki} Kd={kd}")


if __name__ == "__main__":
    main()
