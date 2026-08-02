# CLAUDE.md — Self-Balancing Robot (viswavsn81/self-balancing-robot)

## Mission
Take this Arduino self-balancing robot from "balances in place (barely)" to
"balances robustly AND drives from point A to point B without falling over."
You (the agent) own the software, the control design, and the tuning
methodology. The human (Vish) is your hands: he flashes boards when needed,
places the robot on the floor, releases it, and reports/collects what happened.

## What this codebase is
- `self_balancing_robot.ino` — main sketch. Reads MPU6050 accel + gyro over
  I2C, fuses into a tilt angle with `kalman.h`, runs one PID on angle
  (Kp=25, Ki=5, Kd=0.5), clamps output to ±255, sends the SAME command to both
  wheels via `MotorDriver::drive()`. Prints `angle,drive` at 115200 baud.
- `MPU6050.cpp/.h` — raw IMU register reads. Scale: accel 8192 LSB/g (±4g),
  gyro 65.5 LSB/°/s (±500 °/s).
- `kalman.h` — 1-D Kalman filter fusing accel angle + gyro rate.
- `PIDController.cpp/.h` — custom PID class.
- `MotorDriver.cpp/.h` — TB6612FNG-style driver. Pins: AIN1=7, PWMA=5,
  BIN1=8, PWMB=6, STBY=3. Note `drive()` negates both speeds (motors are
  mounted mirrored/inverted). Only ONE direction pin per motor is wired.
- `*.inox` files — older experimental variants. Read them for history; do not
  build them.
- `datasheets/` — component datasheets.

Hardware: CONFIRMED — see "Confirmed hardware" below. Arduino Uno-class AVR
board, TB6612FNG motor driver, MPU6050 IMU, 2 DC gear motors, NO wheel
encoders (final decision: encoderless — see Phase 3).

## Ground rules
1. **Never guess-and-flash blindly.** Every firmware iteration must log
   telemetry (timestamp, raw angle, filtered angle, gyro rate, PID terms,
   motor output) as CSV over serial, and every tuning decision must be
   justified from the previous run's log.
2. **Safety first in firmware:** add a tilt cutoff (kill motors if |angle| >
   ~35–40°), a startup arming delay (motors stay off until the robot is held
   near upright for ~2 s), and keep `motors.stop()` reachable from a serial
   command (`x` = emergency stop). Vish will be picking this thing up a lot.
3. **One change at a time** during tuning. Change one gain, run one trial,
   analyze, repeat.
4. **Ask before assuming hardware.** Board type, driver wiring, battery
   voltage, wheel diameter, presence of encoders, robot mass/height — ask
   early, record answers in this file under "Confirmed hardware."
5. Use `arduino-cli` for everything: `arduino-cli compile`, `arduino-cli
   upload -p <port>`, and `arduino-cli monitor -p <port> -c baudrate=115200`
   (or a small Python `pyserial` logger script you write) to capture runs to
   CSV files in a `logs/` directory. If `arduino-cli` isn't installed, help
   Vish install it first.

## POWER PROTOCOL (dictated by Vish — never violate)
The motor battery is switched OFF by default and stays off. The original
firmware drove the motors immediately at boot (they even creep on USB power
alone) — that is unsafe and must never be reintroduced.

1. Motors are **DISARMED by default at boot**: PWM zero, TB6612 STBY low
   (driver in standby), no motion until an explicit serial arm command.
   Emergency stop `x` always returns to this disarmed state.
2. All flashing, serial console work, and calibration that doesn't need
   motor torque happens on **USB power only** (battery switch off).
3. Any step that genuinely needs motor power (motor direction test, deadband
   ramp, balance trials) is a **battery-on step**: STOP and explicitly tell
   Vish to switch the battery on, what you're about to do, what to watch
   for, and that `x` (or unplugging USB) aborts. Wait for his confirmation
   before commanding any motion. When done, tell him to switch the battery
   off again.
4. Never leave the firmware in a state where reconnecting battery power
   causes spontaneous motion.

## Phase 0 — Repo audit and infrastructure (no robot needed)
- Read every source file. Fix latent bugs before tuning. Known suspects:
  - The main loop has NO fixed timing — dt jitters with Serial printing.
    Enforce a fixed control period (5–10 ms, e.g. run control only when
    `micros() - last >= 5000`), and keep Serial output rate-limited
    (e.g. every 4th cycle) so printing doesn't starve the loop.
  - `analogWrite` PWM on pins 5/6 (Timer0 on AVR) is ~980 Hz — audible and
    coarse but workable; don't touch Timer0 (it drives `millis()`).
  - Single direction pin per motor on a TB6612 means the other IN pin is
    strapped — confirm with Vish that both directions actually work on both
    motors (write a serial-driven motor test sketch: command each motor
    ±speed independently).
  - PID integral: verify `PIDController` has anti-windup (clamp the integral
    when output saturates). If not, add it — this is the #1 cause of
    violent oscillation after a disturbance.
- Build a **serial tuning console** into the firmware: single-key commands to
  adjust Kp/Ki/Kd (and later the outer-loop gains) live, print current gains,
  zero the integral, set the angle trim, start/stop a logged trial. This lets
  you tune WITHOUT reflashing every iteration.
- Write `tools/log_trial.py` (pyserial) that records a trial to
  `logs/trial_NNN.csv` with the gain settings in a header row, and
  `tools/analyze.py` that computes: oscillation frequency, peak-to-peak angle,
  settling time after release, drift rate, and saturation percentage. You will
  read these numbers instead of watching the robot.

## Phase 1 — Calibration (robot on, wheels off the ground / held)
1. **Gyro bias**: average 500 gyro samples at rest at startup; subtract.
2. **Balance-point trim**: the mechanical center of mass is NOT at exactly 0°.
   Have Vish balance the robot by hand at its natural balance point; log the
   angle for 5 s; the mean becomes `angleTrim`, applied to the setpoint.
   Persist it in EEPROM.
3. **Motor deadband**: ramp PWM slowly from 0 for each motor (wheels free);
   have Vish report (or detect via IMU vibration) the PWM value where each
   wheel starts turning. Map PID output so output=1 → deadband PWM. Without
   this, small corrections do nothing and the robot limit-cycles.
4. **Direction sanity check**: with the robot held, tilt it forward — wheels
   must drive forward (toward the fall). If they drive backward, flip signs
   in ONE place and document it.

### Phase 2 — COMPLETE (2026-08-01, sessions 1-2)
**Final gains: Kp=15, Ki=0.5, Kd=0.2 | trim=6.0 | deadband 23/19
(fresh charge; 26/23 when sagged). All in EEPROM.**
Justifying trials: 044 (46.5 s continuous balance, ended by logger
window not a fall; wander +4.6..+8.3°, 0% saturation, mean output −5,
i-term ~0) and 045 (43.5 s incl. two disturbance recoveries in 1.08 s /
0.94 s, cleanly damped). Remaining known limitation: runaway drift —
an angle-only balancer happily balances at constant velocity, so drift
builds into speed until it falls (trial 045 run 2). That is the Phase 3
velocity loop's job; do NOT try to fix it with more angle-loop tuning.
Trim history: hand-capture 2.22 wrong (grip bias), friction-cone 8.61
wrong (USB cable kickstand), floor-trial bisection → 6.0 correct.
Chatter: ~10 Hz, ripple ~2°, cosmetic; if it bothers Phase 3, the fix
is a gyro-rate D term in firmware, NOT higher Kd (Kd≥0.5 destabilizes
via kalman-derivative phase lag — measured, trials 019 vs 020).
Arming technique: Vish holds the robot ~4-5° back of his felt vertical
(his feel reads −1..+2° on the sensor; gate is trim±5°).
IMU: intermittent wiring fixed & shake-test validated (logs 040-043);
run the shake test after ANY wiring work.

### Phase 2 session 1 notes (superseded, kept for history)
Working gains so far: **Kp=15, Ki=0.5, Kd=0.2** → 30-37 s balancing
stretches, ripple ~1.1° once deadband recalibrated for sag (26/23).
OPEN ISSUE — trim: hand-captured 2.22 is too far forward (constant
forward sprint); friction-cone estimate 8.61 is too far backward
(instant backward flip; measurement was biased by the USB cable acting
as a kickstand — distrust free-standing rest angles unless the cable is
slack and vertical). Bracketed to (4.0, 8.61); **next action: trial at
trim 6.0**, then bisect further. Battery went to charge after ~10
trials; deadband will need re-checking on the fresh charge (probably
back near 23/23). Kd=0.5 destabilizes (kalman-derivative phase at the
10 Hz chatter); if more damping is needed, switch the D term to raw
gyro rate (gx) in firmware rather than raising Kd.
HARDWARE WATCH: MPU6050 froze 4× this session (I2C alive, outputs
static), always near handling/motor transients — suspect a marginal
wire/solder joint to the IMU; ask Vish to reseat/inspect before next
session. Firmware watchdog (0.5 s bit-identical raws) catches it and
disarms; DEVICE_RESET at init clears it.

## Phase 2 — Balance tuning (the "servo tuning" the PID way)
Tuning protocol — follow it mechanically, one trial per step, ~10–20 s each,
robot released upright on a hard floor with Vish spotting it:

1. Set Ki=0, Kd=0. Raise Kp from a low value until the robot oscillates
   steadily about upright (it will NOT balance yet — you want sustained
   back-and-forth, not falling limp and not violent). If it falls limp, Kp is
   too low; if it slams to the clamp instantly, too high.
2. Add Kd to damp the oscillation until it catches itself with 1–2
   overshoots. Kd fights fast angle changes; too much Kd = jittery buzzing
   from gyro noise (visible as high-frequency content in the log).
3. Add small Ki last to remove steady lean/drift (robot slowly creeping
   across the floor = angle offset the P+D can't fix). Too much Ki = slow
   large oscillation that grows.
4. Confirm with disturbance tests: Vish gives a gentle push front and back;
   analyze recovery from the log (settling < ~1.5 s, no growing oscillation).
5. Record the final gains and the trial numbers that justify them here.

Expect the repo's current gains (25 / 5 / 0.5) to be wrong for the real
robot — treat them only as a starting order of magnitude. If trials are too
tedious, you may optionally write a quick inverted-pendulum simulation in
Python (cart-pole with the robot's approximate mass/height, which you should
ask Vish to measure) to pre-screen gain ranges — but the log-driven physical
protocol above is the source of truth.

## Phase 3 — Motion: point A to point B
A balancing robot moves by *leaning*: to go forward, the controller
deliberately tilts the setpoint forward and the balance loop chases the fall.
Implement a **cascaded controller**:

**DECISION (final, 2026-08-01): ENCODERLESS.** No wheel encoders will be
added. All velocity/position feedback is estimated, so missions must be
designed with generous tolerances (±10–20 cm on distance, ±10–15° on
heading is acceptable).

- **Inner loop (already built)**: angle PID at 100–200 Hz → motor PWM.
- **Middle loop (velocity)**: velocity PI at ~20 Hz → outputs a tilt
  setpoint, clamped to a small range (start ±3°, never more than ~6°).
  Velocity feedback is **estimated from commanded PWM**: after deadband
  removal, steady-state wheel speed ≈ k · (PWM − deadband), with k
  calibrated once in a bench trial (timed run over a known distance).
  Low-pass filter the commanded PWM (motor + robot dynamics ≈ a few hundred
  ms time constant) to get the velocity estimate. This is crude — expect it
  to be scaled wrong under battery sag; recalibrate k when behavior degrades.
- **Outer loop (position)**: **time/model-based dead reckoning** — integrate
  the estimated velocity to get distance traveled. Distance scale comes from
  wheel circumference ≈ 19.8 cm (6.3 cm dia). Distance-to-target → velocity
  setpoint with a trapezoidal profile: gentle acceleration, cruise, and
  start braking early (a balancing robot must lean BACKWARD to stop —
  budget stopping distance). Accept accumulated error; do NOT chase
  centimeter precision.
- **Station-keeping (anti-drift)**: while "holding position" a
  velocity-estimate-only robot slowly creeps. Add an **anti-drift trim
  integral**: a very slow integrator on the commanded-velocity estimate (or
  on average motor output) that nudges the angle trim so that the average
  drive → 0. This doubles as an online balance-point re-estimator and
  compensates battery sag. Keep its rate LOW (time constant tens of
  seconds) so it never fights the balance loop.
- **Turning**: differential term: `left = drive + turn`,
  `right = drive - turn`, with `turn` from a slow heading controller using
  the MPU6050 **z-gyro integrated yaw** (drifts a few °/min once bias is
  calibrated at rest — fine for short moves; re-zero heading at the start
  of each turn command).
- **Mission**: serial commands `g 100` = "go ~100 cm forward, then stop and
  hold balance," `t 90` = turn ~90°. "Point A to B" = a sequence of these.
  Keep max speed LOW (~0.2 m/s equivalent). Missions must tolerate the
  estimator's error: prefer short segments, re-settle between segments, and
  treat arrival within ±15 cm / ±15° as success.

Tune the loops **inside-out**: never touch inner-loop gains while tuning the
velocity loop; never touch velocity gains while tuning position. Each outer
loop gets the same one-change-per-trial, log-driven protocol as Phase 2.

## Phase 4 — Robustness
- Ramp velocity setpoints (slew-rate limit) so the balance loop is never
  asked for a step change.
- Handle battery sag: log battery voltage if a divider is available (ask), or
  at least re-check deadband when behavior degrades.
- Fall detection → motors off → require re-arm.
- Update README.md with wiring, calibration procedure, tuning results, and
  how to run a mission.

## Confirmed hardware (confirmed by Vish, 2026-08-01)
- **Board**: Elegoo Uno R3 (Arduino Uno clone, FQBN `arduino:avr:uno`,
  ATmega328P — 2 KB RAM, 32 KB flash). Uses a **CH340** USB-serial chip, so
  the port appears as `/dev/ttyUSB*` (Linux), `/dev/cu.usbserial*` or
  `/dev/cu.wchusbserial*` (macOS), or a COM port (Windows). It will NOT
  identify itself as an official Arduino in `arduino-cli board list` — match
  by **port**, not board name. If no port appears at all, suspect a missing
  CH340 driver and help Vish install it.
- **Battery**: Elegoo Smart Car pack — 2-cell Li-ion/LiPo, 7.4 V nominal,
  ~8.4 V full, sagging toward ~7 V when low. Assume meaningful voltage sag
  across a session: plan periodic deadband recalibration. (Open question to
  Vish: is a voltage divider to an analog pin feasible for battery
  compensation?)
- **Wheel diameter**: 6.3 cm → circumference ≈ 19.8 cm. This is the distance
  scale factor for dead reckoning.
- **Encoders**: NONE, and the decision is final — we are going encoderless
  (see Phase 3).
- Motor driver second IN pins: single direction pin per motor (see
  `datasheets/sn74lvc2g14.pdf` — likely hardware-inverted); both-direction
  operation to be verified in the Phase 1 bench motor test.
- Serial port on Vish's machine: `/dev/ttyUSB0` (Linux, dialout OK).
- Robot mass: 389 g. IMU height: 5 cm above the floor.
- **IMU conventions (verified empirically, trials 014/015)**: X axis runs
  along the wheel axle; +Y = taped FRONT. Pitch (fall) rate = **+gx**;
  forward tilt = angle DECREASING. Gyro sensitivity is NON-datasheet:
  49.06 LSB/dps at ±500 dps config (clone chip underreads 1.335×; scale
  measured by integral-vs-accel-angle method). Balance trim ≈ +2.22°,
  motor deadband ≈ PWM 24 both wheels at full charge (both in EEPROM).
- **Workflow (2026-08-01)**: the AGENT runs `tools/log_trial.py` and sends
  all serial commands itself. Vish does ONLY physical actions; tell him
  exactly what to do with his hands and wait for his "done" before sending
  any command that depends on it.

## Definition of done
The robot, started upright, executes `g 150` (1.5 m forward) then `t 180`
then `g 150` back to start, without falling, on at least 3 consecutive
attempts, with logs saved in `logs/` proving each run.
