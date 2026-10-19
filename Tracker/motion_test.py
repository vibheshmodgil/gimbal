"""Gimbal motion + camera test, measured from the video alone.

    python Tracker/motion_test.py [--gimbal 192.168.1.8] [--quick]

Moves the head through a fixed sequence (pan +-20, tilt +-15 at most, inside the gimbal's limits)
and measures with phase correlation how far the PICTURE really moved:
  scale + direction  -> true camera FOV and pan/tilt signs for the tracker
  step accuracy      -> commanded vs measured angle, 0.5..20 deg
  backlash           -> same target reached from the left vs from the right
  resolution         -> smallest command that moves the picture
  dynamics           -> command -> first visible motion (latency), 50 % point, settle time, overshoot
  jitter / drift     -> shake while still, error coming back to home
Writes Tracker/calibration.json (read by server.py) and docs/MOTION_TEST_<date>.md.

Needs a static, textured, lit scene (nobody walking through) and the tracker server STOPPED
(the gimbal serves only 2 video viewers and only one program should move it).
"""
import argparse, datetime, json, os, statistics as stt, threading, time, urllib.request
from collections import deque

import cv2, numpy as np, websocket

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
med = stt.median
MIN_RESP = 0.1   # phase-correlation peak below this = the two pictures don't overlap enough to trust


class Video(threading.Thread):
    """Every frame as (arrival time, grey float32), newest last."""
    def __init__(self, ip):
        super().__init__(daemon=True)
        self.url, self.frames, self.err = f"http://{ip}:82/stream", deque(maxlen=400), None
    def run(self):
        try:
            r, buf = urllib.request.urlopen(self.url, timeout=5), b""
            while True:
                buf += r.read(8192)
                while (a := buf.find(b"\xff\xd8")) >= 0 and (b := buf.find(b"\xff\xd9", a + 2)) >= 0:
                    img = cv2.imdecode(np.frombuffer(buf[a:b + 2], np.uint8), cv2.IMREAD_GRAYSCALE)
                    buf = buf[b + 2:]
                    if img is not None: self.frames.append((time.time(), np.float32(img)))
        except Exception as e:
            self.err = str(e)
    def since(self, t): return [f for f in list(self.frames) if f[0] > t]


class Gimbal:
    def __init__(self, ip):
        self.ws = websocket.create_connection(f"ws://{ip}:81/", timeout=5)
        self.status, self.lock = {}, threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        t0 = time.time()
        while "pan" not in self.status and time.time() - t0 < 5: time.sleep(0.05)
    def _rx(self):
        while True:
            try: m = self.ws.recv()
            except Exception: return
            if m.startswith('{"t":"s"'): self.status = json.loads(m)
    def send(self, c):
        with self.lock: self.ws.send(c)
        return time.time()


class Test:
    def __init__(self, ip, quick):
        self.v, self.g, self.quick, self.log = Video(ip), Gimbal(ip), quick, []
        self.v.start()
        self.g.send("CAMP:FAST"); self.g.send("SPD:120"); self.g.send("L:0")
        time.sleep(2.5)
        if self.v.err or not self.v.frames: raise SystemExit(f"no video ({self.v.err}); stop the tracker server / close gimbal UI tabs")
        h, w = self.v.frames[-1][1].shape
        self.w, self.h, self.win = w, h, cv2.createHanningWindow((w, h), cv2.CV_32F)
        st = self.g.status
        self.lp, self.lt = st.get("lp", [-90, 90]), st.get("lt", [-45, 45])

    def shift(self, a, b):
        (dx, dy), resp = cv2.phaseCorrelate(a, b, self.win)
        return dx, dy, resp

    def still(self, timeout=4.0):
        """Mean of 3 frames once the picture has stopped moving (noise down, blur gone)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            fr = self.v.since(time.time() - 0.6)
            if len(fr) >= 4 and all(abs(self.shift(a[1], b[1])[0]) < 0.4 and abs(self.shift(a[1], b[1])[1]) < 0.4 for a, b in zip(fr, fr[1:])):
                return np.mean([f[1] for f in fr[-3:]], axis=0)
            time.sleep(0.1)
        return np.mean([f[1] for f in self.v.since(time.time() - 0.3)] or [self.v.frames[-1][1]], axis=0)

    def go(self, p, t, wait=None):
        p, t = float(np.clip(p, self.lp[0] + 2, self.lp[1] - 2)), float(np.clip(t, self.lt[0] + 2, self.lt[1] - 2))
        self.g.send(f"PT:{p:.2f},{t:.2f}")
        time.sleep(wait if wait is not None else 0.35 + 25 / 120)
        return self.still()

    # ---------------------------------------------------------------- measurements
    def jitter(self):
        self.go(0, 0, 1.0)
        fr = self.v.since(time.time() - 0.1); time.sleep(3); fr = self.v.since(time.time() - 3)
        ref = fr[0][1]
        s = [self.shift(ref, f[1]) for f in fr[1:]]
        return dict(frames=len(fr), fps=round((len(fr) - 1) / (fr[-1][0] - fr[0][0]), 1),
                    jitter_px=round(float(np.median([np.hypot(x, y) for x, y, _ in s])), 3),   # median: one passer-by can't fake shake
                    texture=round(float(med([r for _, _, r in s])), 3))

    def scale(self, axis, amp=5.0):
        """px per degree and direction from +-amp around 0, both approached from 0."""
        P = lambda a: (a, 0) if axis == "pan" else (0, a)
        ref = self.go(0, 0)
        plus = self.shift(ref, self.go(*P(amp))); self.go(0, 0)
        minus = self.shift(ref, self.go(*P(-amp))); self.go(0, 0)
        k = 0 if axis == "pan" else 1
        ppd = (plus[k] - minus[k]) / (2 * amp)          # signed px/deg; scene moves opposite to the camera
        cross = (plus[1 - k] - minus[1 - k]) / (2 * amp)
        return dict(px_per_deg=round(ppd, 3), cross_px_per_deg=round(cross, 3), resp=round(min(plus[2], minus[2]), 3))

    def steps(self, axis, ppd, sizes):
        k = 0 if axis == "pan" else 1
        P = lambda a: (a, 0) if axis == "pan" else (0, a)
        out = []
        for s in sizes:
            for sign in (1, -1):
                ref = self.go(0, 0)
                dx = self.shift(ref, self.go(*P(sign * s)))
                meas = dx[k] / ppd
                if dx[2] < MIN_RESP:   # too little overlap between the two pictures: no measurement, not a gimbal error
                    out.append(dict(cmd=sign * s, meas=None, err=None, resp=round(dx[2], 3))); continue
                out.append(dict(cmd=sign * s, meas=round(meas, 2), err=round(meas - sign * s, 2), resp=round(dx[2], 3)))
        self.go(0, 0)
        return out

    def backlash(self, axis, ppd, reps):
        """0 deg reached from -10 and from +10; difference = dead play in the gears/linkage."""
        k = 0 if axis == "pan" else 1
        P = lambda a: (a, 0) if axis == "pan" else (0, a)
        self.go(*P(-10)); ref = self.go(0, 0)
        frm = {"left": [], "right": []}
        for _ in range(reps):
            self.go(*P(10)); frm["right"].append(self.shift(ref, self.go(0, 0))[k] / ppd)
            self.go(*P(-10)); frm["left"].append(self.shift(ref, self.go(0, 0))[k] / ppd)
        return dict(backlash_deg=round(abs(med(frm["right"]) - med(frm["left"])), 2),
                    repeat_sd_deg=round(max(stt.pstdev(frm["right"]), stt.pstdev(frm["left"])), 3),
                    right=[round(x, 2) for x in frm["right"]], left=[round(x, 2) for x in frm["left"]])

    def resolution(self, axis, ppd):
        k = 0 if axis == "pan" else 1
        P = lambda a: (a, 0) if axis == "pan" else (0, a)
        out = []
        for s in (0.25, 0.5, 1.0, 2.0):
            ref = self.go(*P(-3)); ref = self.go(0, 0)      # approach from the same side every time
            out.append(dict(cmd=s, meas=round(self.shift(ref, self.go(*P(s)))[k] / ppd, 2)))
        self.go(0, 0)
        return out

    def dynamics(self, axis, ppd, amp, speed, reps):
        k = 0 if axis == "pan" else 1
        P = lambda a: (a, 0) if axis == "pan" else (0, a)
        self.g.send(f"SPD:{speed}")
        res = []
        for i in range(reps):
            ref = self.go(0, 0, 0.8)
            t0 = self.g.send("PT:%.2f,%.2f" % P(amp)); time.sleep(1.6)
            tr = [(t - t0, sh[k] / ppd) for t, f in self.v.since(t0) if (sh := self.shift(ref, f))[2] >= MIN_RESP / 3]   # blur lowers the peak
            final = med([d for _, d in tr[-5:]])
            if abs(final) < 0.3 * amp: continue                 # didn't move / bad match
            onset = next((t for t, d in tr if abs(d) > 0.6), None)
            t50 = next((t for t, d in tr if abs(d) >= 0.5 * abs(final)), None)
            settle = next((tr[j][0] for j in range(len(tr)) if all(abs(d - final) < 0.4 for _, d in tr[j:])), None)
            peak = max((d * np.sign(final) for _, d in tr), default=0)
            res.append(dict(onset_s=onset, t50_s=t50, settle_s=settle, final_deg=round(final, 2),
                            overshoot_pct=round(100 * max(0.0, peak - abs(final)) / abs(final), 1),
                            fps=round(len(tr) / 1.6, 1)))
        self.g.send("SPD:120"); self.go(0, 0)
        if not res: return dict(ok=False)
        pick = lambda key: round(med([r[key] for r in res if r[key] is not None]), 3) if any(r[key] is not None for r in res) else None
        return dict(ok=True, speed=speed, amp=amp, onset_s=pick("onset_s"), t50_s=pick("t50_s"), settle_s=pick("settle_s"),
                    overshoot_pct=pick("overshoot_pct"), final_deg=pick("final_deg"), runs=len(res), fps=pick("fps"))

    def run(self):
        R = dict(when=datetime.datetime.now().isoformat(timespec="seconds"), gimbal=dict(
            cal=self.g.status.get("cal"), lp=self.lp, lt=self.lt, fw_pan=self.g.status.get("pan"), image=[self.w, self.h]))
        reps = 2 if self.quick else 4
        def step(name, f):
            print(f"  {name}...", flush=True)
            R[name] = f()
            return R[name]
        try:
            ref0 = self.go(0, 0, 1.0)
            step("still", self.jitter)
            for ax in ("pan", "tilt"):
                sc = step(f"{ax}_scale", lambda: self.scale(ax))
                ppd = sc["px_per_deg"]
                if abs(ppd) < 0.5:
                    R[f"{ax}_error"] = "picture did not move: axis stuck, too little texture, or servo off"; continue
                big = (2, 5, 10, 20) if ax == "pan" else (2, 5, 10, 15)
                step(f"{ax}_steps", lambda: self.steps(ax, ppd, (0.5,) + big if not self.quick else (1, 5, big[-1])))
                step(f"{ax}_backlash", lambda: self.backlash(ax, ppd, reps))
                step(f"{ax}_resolution", lambda: self.resolution(ax, ppd))
                for spd in (120, 240):
                    step(f"{ax}_dyn_{spd}", lambda: self.dynamics(ax, ppd, 10, spd, reps))
            end = self.shift(ref0, self.go(0, 0, 1.0))
            ppd_p, ppd_t = R.get("pan_scale", {}).get("px_per_deg") or 1, R.get("tilt_scale", {}).get("px_per_deg") or 1
            R["home_drift_deg"] = [round(end[0] / ppd_p, 2), round(end[1] / ppd_t, 2)]
        finally:
            self.g.send("SPD:120"); self.g.send("PT:0,0")
        return R


def recommend(R):
    w, h = R["gimbal"]["image"]
    c = {}
    if "pan_scale" in R and abs(R["pan_scale"]["px_per_deg"]) >= 0.5:
        ppd = R["pan_scale"]["px_per_deg"]
        c["hfov"] = round(w / abs(ppd), 1)
        c["pan_sign"] = 1 if ppd < 0 else -1          # pan + moved the scene left (-x) -> a target right of centre needs pan +
    if "tilt_scale" in R and abs(R["tilt_scale"]["px_per_deg"]) >= 0.5:
        c["tilt_sign"] = 1 if R["tilt_scale"]["px_per_deg"] > 0 else -1   # tilt + (up) moves the scene down (+y)
        c["vfov_deg"] = round(h / abs(R["tilt_scale"]["px_per_deg"]), 1)   # in COMMANDED degrees: absorbs tilt linkage ratio
    dyn = [R[k] for k in ("pan_dyn_240", "tilt_dyn_240") if R.get(k, {}).get("ok") and R[k]["t50_s"]]
    dyn += [R[k] for k in ("pan_dyn_120", "tilt_dyn_120") if R.get(k, {}).get("ok") and R[k]["t50_s"]]
    if dyn: c["latency_s"] = round(max(d["t50_s"] for d in dyn), 2)   # ponytail: slower axis; too low a latency makes the tracker run away
    for ax, key in (("pan", "deadband_deg"), ("tilt", "deadband_tilt_deg")):   # ponytail: 3/4 of the play, PID+ramp ride out the rest
        b = R.get(f"{ax}_backlash", {})
        c[key] = round(max(0.5, b.get("backlash_deg", 0) * 0.75, 3 * b.get("repeat_sd_deg", 0)), 1)
    return c


def report(R, C, path):
    L = [f"# Gimbal motion test — {R['when']}", "",
         f"Image {R['gimbal']['image'][0]}×{R['gimbal']['image'][1]} (Fast preset), {R['still']['fps']} fps. "
         f"Gimbal calibrated: **{R['gimbal']['cal']}**, limits pan {R['gimbal']['lp']}, tilt {R['gimbal']['lt']}. "
         f"Scene texture (phase-correlation response, >0.1 is good): {R['still']['texture']}.", "",
         "## Recommended tracker settings (written to Tracker/calibration.json)", "", "```json", json.dumps(C, indent=1), "```", "",
         f"Still picture jitter: {R['still']['jitter_px']} px. Home drift after the whole run: {R.get('home_drift_deg')} deg (pan, tilt).", ""]
    for ax in ("pan", "tilt"):
        if f"{ax}_error" in R: L += [f"## {ax.title()}: **{R[ax + '_error']}**", ""]; continue
        if f"{ax}_scale" not in R: continue
        sc = R[f"{ax}_scale"]
        L += [f"## {ax.title()}", "", f"Scale {sc['px_per_deg']} px/deg (cross-axis {sc['cross_px_per_deg']} px/deg), match quality {sc['resp']}.", "",
              "| Commanded ° | Measured ° | Error ° |", "|---|---|---|"]
        L += [f"| {s['cmd']:+} | {s['meas']:+} | {s['err']:+} |" if s["meas"] is not None else f"| {s['cmd']:+} | not measurable (overlap {s['resp']}) | |" for s in R.get(f"{ax}_steps", [])]
        b = R.get(f"{ax}_backlash", {})
        L += ["", f"Backlash **{b.get('backlash_deg')}°**, repeatability SD {b.get('repeat_sd_deg')}° (from right {b.get('right')}, from left {b.get('left')}).",
              "Smallest moves: " + ", ".join(f"{r['cmd']}° → {r['meas']}°" for r in R.get(f"{ax}_resolution", [])), "",
              "| Speed °/s | Onset s | 50 % s | Settled s | Overshoot % | fps |", "|---|---|---|---|---|---|"]
        for spd in (120, 240):
            d = R.get(f"{ax}_dyn_{spd}", {})
            L.append(f"| {spd} | {d.get('onset_s')} | {d.get('t50_s')} | {d.get('settle_s')} | {d.get('overshoot_pct')} | {d.get('fps')} |" if d.get("ok") else f"| {spd} | no clean measurement | | | | |")
        L.append("")
    L += ["Onset = command sent → first frame that moved >0.6°: Wi-Fi + servo start + capture + stream. "
          "This is what the tracker's *latency* setting must cover.", ""]
    open(path, "w", encoding="utf-8").write("\n".join(L))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gimbal", default="192.168.1.8")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--recompute", action="store_true", help="redo calibration.json + report from motion_test_raw.json, no motion")
    a = ap.parse_args()
    if a.recompute:
        R = json.load(open(os.path.join(HERE, "motion_test_raw.json")))
        C = recommend(R)
        json.dump(dict(C, measured=R["when"]), open(os.path.join(HERE, "calibration.json"), "w"), indent=1)
        report(R, C, os.path.join(ROOT, "docs", f"MOTION_TEST_{R['when'][:10]}.md")); print(json.dumps(C, indent=1)); return
    print("motion test: keep the scene still; Ctrl+C returns the gimbal home")
    R = Test(a.gimbal, a.quick).run()
    C = recommend(R)
    json.dump(dict(C, measured=R["when"]), open(os.path.join(HERE, "calibration.json"), "w"), indent=1)
    json.dump(R, open(os.path.join(HERE, "motion_test_raw.json"), "w"), indent=1)
    rp = os.path.join(ROOT, "docs", f"MOTION_TEST_{datetime.date.today()}.md")
    report(R, C, rp)
    print(json.dumps(C, indent=1)); print("report:", rp)


if __name__ == "__main__":
    main()
