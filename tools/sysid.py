#!/usr/bin/env python3
"""Motor system identification via webcam blue-marker tracking.

STATUS 2026-08-02: methodology debugged (fixed ROIs + radius-gated orbit
tracking — auto-ROI grabs background blue, and stationary markers alias
to phantom rotation without the radius gate), but NO VALID DATA yet:
mid-session both motors stopped responding to commands entirely (PWM 255
included) with firmware/battery healthy — suspected dislodged STBY/VM/GND
wire at the TB6612 after the robot vibration-walked across the bench.
Rerun after the wiring fix. Adjust self.roi to the current camera view
before each run (verify with a saved frame!).

Per wheel: reference step (start+end, sag drift), rest-to-PWM steps.
Marker centroid orbits the wheel hub; angle-unwrap gives rotation rate.
Outputs logs/sysid_001.json with raw tracks and fitted parameters.
"""
import json, math, sys, time
import cv2
import numpy as np
import serial

PORT, BAUD = "/dev/ttyUSB0", 250000
HSV_LO, HSV_HI = (95, 80, 60), (130, 255, 255)
STEPS = [25, 35, 50, 70, 95, 120]
REF_PWM = 60
CIRC_CM = 19.8


def mask_of(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, HSV_LO, HSV_HI)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def auto_rois(cap):
    """Find the two biggest blue blobs (the wheel markers) -> ROIs."""
    acc = None
    for _ in range(10):
        ok, img = cap.read()
        if not ok: continue
        m = mask_of(img)
        acc = m if acc is None else cv2.bitwise_or(acc, m)
    cnts, _ = cv2.findContours(acc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 100: continue
        x, y, w, h = cv2.boundingRect(c)
        cands.append((a, x + w / 2, y + h / 2, (x, y, w, h)))
    # wheel markers: similar height, separated 80-380 px horizontally
    best = None
    for i in range(len(cands)):
        for j in range(i + 1, len(cands)):
            (a1, cx1, cy1, b1), (a2, cx2, cy2, b2) = cands[i], cands[j]
            if abs(cy1 - cy2) < 70 and 80 < abs(cx1 - cx2) < 380:
                if best is None or a1 + a2 > best[0]:
                    best = (a1 + a2, (cx1, b1), (cx2, b2))
    assert best, f"no wheel-like blob pair among {len(cands)} blobs"
    rois = {}
    for cx, (x, y, w, h) in (best[1], best[2]):
        pad = int(2.2 * max(w, h))
        roi = (max(0, x - pad), min(639, x + w + pad),
               max(0, y - pad), min(479, y + h + pad))
        rois["frameL" if cx == min(best[1][0], best[2][0]) else "frameR"] = roi
    assert len(rois) == 2
    return rois


class Rig:
    def __init__(self):
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = PORT, BAUD, 0.15
        self.ser.dtr = self.ser.rts = False
        self.ser.open()
        time.sleep(4.5)
        self.ser.reset_input_buffer()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(5): self.cap.read()
        # Fixed ROIs from sysid_setup.jpg — auto-detect locked onto
        # background blue at the frame top and produced garbage.
        self.roi = {'frameL': (45, 225, 215, 410),
                    'frameR': (265, 450, 200, 400)}
        print("ROIs:", self.roi)

    def cmd(self, c):
        self.ser.write((c + "\n").encode())

    def track(self, seconds, roi_key, refresh=None):
        x0, x1, y0, y1 = self.roi[roi_key]
        t0 = time.monotonic()
        last = t0
        out = []
        while time.monotonic() - t0 < seconds:
            if refresh and time.monotonic() - last > 1.5:
                self.cmd(refresh); last = time.monotonic()
            ok, img = self.cap.read()
            ts = time.monotonic() - t0
            if not ok: continue
            m = mask_of(img)[y0:y1, x0:x1]
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            best, area = None, 50
            for c in cnts:
                a = cv2.contourArea(c)
                if a > area:
                    area = a
                    mm = cv2.moments(c)
                    best = (mm["m10"] / mm["m00"], mm["m01"] / mm["m00"])
            out.append((ts, best))
        return out


def rates(track):
    """Angle-unwrapped rotation rate (rev/s) samples from a centroid track.
    Radius gate: a stationary marker's centroid coincides with the orbit
    center, making angles pure noise — require a real orbit radius."""
    pts = [(t, p) for t, p in track if p is not None]
    if len(pts) < 8:
        return [], 0.0
    cx = np.mean([p[0] for _, p in pts])
    cy = np.mean([p[1] for _, p in pts])
    radii = [math.hypot(p[0] - cx, p[1] - cy) for _, p in pts]
    med_r = float(np.median(radii))
    if med_r < 15.0:
        return [], med_r          # not rotating
    out = []
    prev_t, prev_a = None, None
    for t, p in pts:
        a = math.atan2(p[1] - cy, p[0] - cx)
        if prev_a is not None:
            da = a - prev_a
            while da > math.pi: da -= 2 * math.pi
            while da < -math.pi: da += 2 * math.pi
            dt = t - prev_t
            if 0.01 < dt < 0.2:
                v = abs(da) / (2 * math.pi) / dt
                if v < 4.0:   # TT gearmotor tops out ~3.3 rev/s
                    out.append((t, v))
        prev_t, prev_a = t, a
    return out, med_r


def step_metrics(track):
    r, med_r = rates(track)
    if len(r) < 8:
        return {"ss_revs": 0.0, "tau_s": None, "n": len(r),
                "orbit_px": round(med_r, 1)}
    late = [v for t, v in r if t > 1.6]
    if not late:
        return {"ss_revs": 0.0, "tau_s": None, "n": len(r),
                "orbit_px": round(med_r, 1)}
    ss = float(np.median(late))
    tau = None
    if ss > 0.1:
        thr = 0.632 * ss
        for t, v in r:
            if v >= thr:
                tau = round(t, 3)
                break
    return {"ss_revs": round(ss, 3), "ss_cms": round(ss * CIRC_CM, 1),
            "tau_s": tau, "n": len(r), "orbit_px": round(med_r, 1)}


def main():
    rig = Rig()
    rig.cmd("m"); time.sleep(0.5)

    # mapping re-verify: firmware left pulse
    rig.cmd("l 45")
    trL = {k: rig.track(1.6, k, refresh="l 45") for k in ("frameL",)}
    # can't track both ROIs in one pass with this helper; do a quick check:
    rig.cmd("l 0"); time.sleep(0.8)
    mvL = step_metrics(trL["frameL"])
    rig.cmd("l 45")
    trR = rig.track(1.6, "frameR", refresh="l 45")
    rig.cmd("l 0"); time.sleep(0.8)
    mvR = step_metrics(trR)
    fw_l_roi = "frameR" if mvR["ss_revs"] > mvL["ss_revs"] else "frameL"
    print(f"mapping check: frameL {mvL['ss_revs']} rev/s, "
          f"frameR {mvR['ss_revs']} rev/s -> fw 'l' = {fw_l_roi}")

    mapping = {"l": fw_l_roi,
               "r": "frameL" if fw_l_roi == "frameR" else "frameR"}
    results = {"mapping": mapping, "wheels": {}}

    for fw in ("l", "r"):
        roi = mapping[fw]
        w = {"steps": {}, "ref": {}}
        for label, pwm in [("ref_start", REF_PWM)] + \
                          [(str(p), p) for p in STEPS] + \
                          [("ref_end", REF_PWM)]:
            time.sleep(1.0)                        # rest, wheel stops
            rig.cmd(f"{fw} {pwm}")
            tr = rig.track(3.2, roi, refresh=f"{fw} {pwm}")
            rig.cmd(f"{fw} 0")
            m = step_metrics(tr)
            (w["ref"] if label.startswith("ref") else w["steps"])[label] = m
            print(f"  {fw} pwm={pwm:3d} [{label}]: ss={m['ss_revs']} rev/s "
                  f"({m.get('ss_cms','-')} cm/s) tau={m['tau_s']}", flush=True)
        results["wheels"][fw] = w

    rig.cmd("x"); time.sleep(0.3)
    with open("/home/pyru/self_balancing_robot/logs/sysid_001.json", "w") as f:
        json.dump(results, f, indent=1)
    print("saved logs/sysid_001.json")
    rig.ser.close(); rig.cap.release()


if __name__ == "__main__":
    main()
