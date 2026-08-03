# PLAN — Jitter Elimination (A) and Visual SLAM / ROS2 Point-A→B (B)

Status: PROPOSAL for review. No implementation started.
Evidence sources: logs/trial_044..074, sysid_002.json, CLAUDE.md history.

---

## Objective A — Kill the jitter

### A.0 Quantitative diagnosis (from existing logs)

Measured on trial 051 (90 s station-keep, 50 Hz telemetry, pre-ruler) and
trial 073 (93.6 s record, 25 Hz, post-ruler):

| # | Source | Evidence | Verdict |
|---|--------|----------|---------|
| 1 | **Deadband bang-bang limit cycle** | Motor output reverses sign **24×/s**; every reversal jumps instantaneously through the deadband map (0 → ±19 counts ≈ 7.5 % of full torque, applied as a step); motor spectrum peaks ~15 Hz; only 6 % of samples sit in the near-zero band | **PRIMARY DRIVER** |
| 2 | **Gear backlash** | Robot free-stands ±2° on gear friction (trials 029/030) ⇒ each torque reversal must traverse a ~±2° mechanical dead-zone before torque re-engages — matching the observed ±2–3° ripple amplitude; sustains and shapes source 1 | **PRIMARY (mechanical half of the same cycle)** |
| 3 | **D-term noise amplification** | D-term output std = **0.85×** the P-term std — the kalman-angle derivative at 200 Hz mostly re-amplifies the 10–15 Hz chatter; corroborated by the measured Kd≥0.5 instability and the raw-gyro-D failure (trials 019/057) | SECONDARY, coupled to 1 |
| 4 | Kalman tuning (Q/R) | At rest the filter is excellent (raw 0.124° → kalman 0.017° std). Its lag at ~10 Hz is currently *load-bearing* (it is the accidental D-filter). Mistuned? No. Improvable jointly with a proper D low-pass? Yes, modestly | MINOR |
| 5 | Structural vibration into IMU | Post-ruler dominant frequency ≈ 9.8 Hz vs ~10–15 Hz pre-ruler; no new resonance appeared; ruler adds inertia that *lowered* gyro-rate std (124 → 109 dps). Camera/mount vibes unmeasured but not visible as spectral lines | MINOR (verify once, cheap) |
| 6 | Loop timing / PWM quantization | Loop dt = 5002 ± 6 µs, 0 % overruns across all logged trials; 8-bit PWM granularity ≈ 0.4 % torque — dwarfed by the 7.5 % deadband step | NON-ISSUE (except as the deadband step, covered in 1) |

Summary picture: the robot chatters because **every zero-crossing of the
control output delivers a torque hammer-blow through a ±2° mechanical
dead-zone, ~12× per second, and the D term echoes it**. Everything else is
noise floor.

### A.1 Fix sequence (one change per trial, autonomous session)

Ordered by expected-improvement / cost. Fixes 1–4 are firmware (ONE manual
flash for the batch, then all knobs console-tunable); fix 6 is hands.

| Step | Fix | Mechanism | Expected | Cost |
|------|-----|-----------|----------|------|
| F1 | **Soft deadband shaping** — replace the hard 0→19 jump with a smooth blend (e.g. comp ∝ out/(|out|+k), console-tunable k) | Removes the torque step at sign reversal | Ripple std 2.6° → ~1.5–1.8°; the single biggest win | firmware + knob |
| F2 | **Output slew-rate limit** (~1500–2500 counts/s, tunable) | Turns residual reversals into ramps; backlash traversed gently | further −20–30 % ripple; slight response cost (bounded so recovery authority stays) | firmware + knob |
| F3 | **Proper D-term low-pass** (1st-order, fc ~15 Hz, tunable) on the derivative | Recovers real damping without echoing chatter; may allow Kd 0.3–0.4 (blocked today) | Damping ↑, residual wobble ↓; interacts with F1/F2 — tune after them | firmware + knob |
| F4 | **PWM dither** (small ~100 Hz alternating offset ≈ half the deadband) | Classic anti-stiction: keeps gears micro-engaged so backlash never fully opens | Uncertain on these gearboxes: −0.3–0.5° if it works, slight hum if not — cheap A/B test | firmware + knob |
| F5 | Kalman Q/R console knobs + small sweep | Joint optimum with F3 (filter no longer needs to be the D-filter) | ≤0.2° | firmware + knob |
| F6 | **Mechanical (Vish, ~15 min)**: re-tape ruler tight to chassis, foam pad under IMU, check wheel/hub wobble by hand, snug the camera mount | Removes compliance/resonance unknowns; cheap insurance | 0–0.5°, mostly de-risking | hands |

Experiment protocol: fresh battery; self-erect (S) or C-assisted start;
90 s station-keep per configuration; metric = angle ripple std + motor
sign-flips/s + audible calm; one knob per trial; journal in
logs/tuning_journal.md. Full sequence ≈ 10–14 trials ≈ one autonomous
evening including cooldowns.

### A.2 Honest floor estimate

- **Achievable with F1–F5:** ripple std ~**0.8–1.2°** (from 2.6–3.2°), sign
  flips <5/s, visibly and audibly calm. Confidence: moderate-high — the
  primary sources are control-shaped, not fundamental.
- **The truly physical residual:** the ±2° backlash dead-zone of these
  yellow TT gearboxes cannot be controlled away, only traversed gently —
  a slow ±0.5–1° low-frequency (~1–3 Hz) sway through the lash is the
  expected end state. Sub-0.5° stillness is **not achievable** on this
  drivetrain; that would need metal gearboxes/backlash-free drive
  (hardware swap, out of scope).
- The 8-bit PWM floor (~0.4 % torque) sits well below the backlash floor
  and will not be the binding constraint.

---

## Objective B — Visual SLAM + ROS2 for point A→B

### B.0 Constraints acknowledged

- **Monocular** forward ESP32-S3 camera, VGA 25 fps MJPEG over WiFi
  (~50–200 ms latency, no hardware timestamps), rolling shutter.
- All heavy compute on the laptop (ROS2); robot stays a WiFi
  sensor/actuator endpoint. Fast loops (balance, velocity) stay onboard —
  WiFi latency is fatal for balancing but fine for waypoint goals.
- Odometry: encoderless v_est (calibration still coarse) ⇒ scale/drift
  limited. Yaw: gyro-integrated, and this clone's Z-gyro scale is
  **unstable ±34 %** (logs 046–050) — yaw cannot be trusted over turns
  until the GY-521 is swapped.

### B.1 Scale ambiguity (monocular) — addressed three ways, in order

1. **Fiducials (primary):** AprilTags of known printed size give metric
   range+bearing+ID per observation — scale is solved *exactly* where it
   matters (landmarks), no SLAM bootstrap needed.
2. **Odometry fusion:** v_est integrates into the EKF between sightings.
3. **Known camera height/tilt** (measured once; tilt is *live* from the
   balance telemetry — the camera pitches with every wobble and its
   extrinsics must be published dynamically from the tilt angle).

### B.2 SLAM options — honest evaluation

| Option | Fit | Verdict |
|--------|-----|---------|
| **AprilTag landmark localization** (tags at known/estimated poses + EKF) | Metric, robust to blur at VGA, trivial compute, degrades gracefully | **RECOMMENDED PATH — gets demonstrable A→B soonest** |
| ORB-SLAM3 mono | Works on paper; fragile here: scale drift, rolling shutter, balance jitter blurs ORB features (at 124 dps ripple and ~15 ms exposure ⇒ ~2° smear), WiFi frame jitter, no timestamps | Follow-on experiment, NOT the prerequisite |
| ORB-SLAM3 mono-inertial | Needs tightly-timestamped ≥100 Hz IMU; our IMU stream is 25 Hz over two hops with no sync | Not realistic on this link |
| RTAB-Map | Wants RGB-D/stereo for metric mapping; mono support is weak | Not a fit |

**Recommendation:** AprilTag-first. Full SLAM is a follow-on *experiment*
once A→B works and jitter is reduced — that ordering is honest: tags
tolerate today's jitter; ORB tracking largely does not.

### B.3 Phases and deliverables

- **B-0 — Foundations** (laptop-only + one print)
  ROS2 installed (native or Docker fallback); camera intrinsics calibrated
  (printed checkerboard); `robot_bridge` ROS2 node wrapping the EXISTING
  ESP32 endpoints (no firmware change): `/image_raw`+`/camera_info`,
  `/robot/telemetry`, `/odom` (v_est+yaw), `/robot/cmd` passthrough +
  `/cmd_vel` mapper. TF tree: `map → odom → base_link → camera_link`
  with camera pitch published live from telemetry.
  *Deliverable:* rqt shows live image + odom; teleop A→B by hand via
  `/cmd_vel`.
- **B-1 — Single-tag homing**
  pupil-apriltags detector node; one tag on a wall; range+bearing → drive
  to a pose 50 cm in front of the tag and stop (visual servo goal via the
  onboard velocity mode).
  *Deliverable:* repeatable “dock at the tag” from anywhere it’s visible.
  This alone demonstrates metric A→B without any map.
- **B-2 — Tag-field localization + waypoint A→B**
  4–6 tags around the garage at surveyed poses; EKF
  (`robot_localization`) fusing tag observations + odom (+yaw with low
  trust until the IMU swap); waypoint follower node.
  *Deliverable:* the CLAUDE.md definition-of-done, executed via ROS2: A→B→A
  with logged pose error, tolerant tolerances (±15 cm / ±15°).
- **B-3 — Mapping comfort layer**
  Auto-survey of tag poses (graph optimization instead of hand
  measurement); Nav2 with a simple costmap if obstacle margins matter.
- **B-4 — Full SLAM experiment (stretch)**
  ORB-SLAM3 mono fed by the (post-Objective-A) calmer camera; evaluate
  tracking survival vs jitter; fuse only if it earns it.

### B.4 Jitter coupling — does A gate B?

- Tags at VGA: detectable through today's ±2.6° @ 10 Hz wobble (blur ~1–2°
  per exposure; tag edges are coarse features). **B-0..B-2 do NOT wait for
  A.**
- ORB features: smear badly at 124 dps rate spikes. **B-4 gates on
  Objective A reaching ripple std ≲1.5° and rate std ≲60 dps.**
- Recommended interleave: A-firmware batch flash → run A trials
  autonomously while B-0 foundations are built on the laptop (they don't
  touch the robot).

### B.5 What each phase needs from Vish physically

| Phase | Hands-time | Items |
|-------|-----------|-------|
| A (F1–F5) | 5 min | one Uno flash cycle (S1 dance) for the knob batch |
| A (F6) | ~15 min | re-tape ruler, foam under IMU, wobble check, camera snug |
| B-0 | ~10 min | print checkerboard (A4), hold it in front of the camera for ~20 calibration frames; measure camera height at balance |
| B-1 | ~5 min | print 1 AprilTag (36h11, ~15 cm), tape to wall, measure its center height |
| B-2 | ~20 min | print+place 4–6 tags, rough survey (tape measure to each tag), battery charges between sessions |
| Any time | ~10 min | **GY-521 swap** (owed since the yaw instability finding) — unlocks trustworthy turns; B-2 works degraded without it, better with it |

### B.6 Risks and honest unknowns

1. **WiFi latency/loss** for visual goals — mitigated by keeping all fast
   control onboard; goals are slow. Residual risk: stream stalls during
   trials (seen once); bridge node must tolerate and re-connect.
2. **No frame timestamps** from the ESP32 — EKF will assume ~100 ms
   camera latency (calibratable by waving a tag); acceptable at ≤0.2 m/s.
   A firmware timestamp header is a later nicety.
3. **Yaw scale instability** until the IMU swap — B-2 leans on tags for
   heading; between-tag dead reckoning will be sloppy. Known, bounded.
4. **v_est calibration** is still the no-load fit (k=0.14) — the on-floor
   tape-measure calibration (planned, never completed) should ride along
   with B-1's first driving sessions.
5. **Battery endurance**: heavy sessions sag the pack in ~30–40 min and
   swing-up sits on a voltage cliff — plan sessions around fresh charges.
6. **Backlash floor** (A): if the ±0.8–1.2° estimate proves optimistic,
   the honest fallback is "calm enough for tags/driving" rather than
   "visually still" — B does not depend on beating the floor.
7. **ROS2 install surface** on this laptop is unverified (distro/deps) —
   Docker fallback specified in B-0 to bound the yak-shaving.

### Proposed order of work (for discussion)

1. A-firmware batch (F1–F5 knobs) → flash (5 min hands) → autonomous A
   tuning evening → report with before/after spectra.
2. In parallel (laptop-only): B-0 foundations.
3. B-1 single-tag homing (first print).
4. F6 mechanical pass + GY-521 swap in one hands session.
5. B-2 tag-field A→B → the definition-of-done, ROS2 edition.
6. B-3/B-4 as appetite allows.
