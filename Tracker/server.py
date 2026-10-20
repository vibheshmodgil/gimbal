"""ANVYXO person-tracking server.

    python Tracker/server.py [--gimbal 192.168.1.8] [--port 8000]
    then open http://localhost:8000

Takes ONE of the gimbal's two video slots and re-serves those JPEGs untouched (no re-encode)
to any number of browsers. Three threads keep the video from ever waiting on the AI:
  stream   newest JPEG from http://<gimbal>:82/stream
  detect   runs the model on the newest frame only, `detect_hz` times a second
  control  30 Hz: alpha-beta filter predicts the locked target between detections -> PID -> PT:pan,tilt

Models (switch live in the UI):
  person  YOLO11n (ultralytics, COCO "person") at 320 px, ~25-40 ms on CPU. Sees people from the
          side, from behind and in dim light; aims at the head ("aim height" = fraction from box top).
          yolo11n-pose.pt (6 MB, same boxes + nose/eye keypoints) downloads next to this file on first use.
          Aims at the FACE (nose + eyes) when they're visible, else at "aim height" in the box (back turned).
  face    MediaPipe BlazeFace full-range, ~5 ms, but only faces turned to the camera in decent light.

Everything is done in gimbal angles, not pixels: each detection becomes an absolute pan/tilt
using where the gimbal pointed when that frame was shot (`latency`). So the gimbal's own motion
doesn't move targets, IDs stay stable while panning, and the PID error is in degrees
(pixel error is shown too). PID output is a turn rate: Kp 2 = 2 deg/s per deg of error.

Self-check without hardware:  python Tracker/server.py --selftest
"""
import argparse, json, os, threading, time, urllib.request
from collections import deque

import cv2, numpy as np

cv2.setNumThreads(2)   # default = one per core; they spin and starve the stream/control threads

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = os.path.join(HERE, "params.json")
PAN_LIM, TILT_LIM = 90.0, 45.0     # CAD collision-checked envelope (gimbal.py): never track past it, whatever the firmware says
FACE_KP_CONF = 0.5                 # pose keypoint confidence for nose/eyes to count as "face visible"
SEARCH_TILTS = (0.0, 12.0, -6.0)   # tilt rows of the repeat passes, relative to home
SEARCH_STEP_FOV, SEARCH_MARGIN = 0.6, 5.0   # stop-and-look views 0.6 x HFOV apart (overlap); stay 5 deg inside limits
# Search moves ramp up/down at this rate. An instant 90 deg/s start of the MG996R browned the ESP32 out (reset reason
# BROWNOUT, 2026-10-19 test): the gimbal rebooted and was gone ~12 s per jump. Raise only with a stiffer 5 V supply.
SEARCH_ACCEL = 150.0               # deg/s^2
STATUS_STALE_S = 0.6               # firmware sends status every 150 ms; older = gimbal not answering (rebooting / Wi-Fi)
LASER_MARGIN = 0.10   # a person's box within 10 % of the picture centre counts as "in the beam" (laser sits above the lens)
# "Dot at feet": track the floor just below the locked person's feet so the laser can stay on while tracking.
FEET_AIM = 1.10       # aim point, fraction of box height from its top: 10 % below the feet = floor in front of them
FEET_ZONE = 0.85      # beam (minus FEET_MARGIN) must sit below 85 % of the box height: under the knees, never near chest/face
FEET_MARGIN = 0.05    # calibration knob: laser-above-lens offset + box sloppiness, fraction of picture height
FEET_EDGE = 0.95      # box bottom must end above this: feet really visible, not a close-up cut off at the frame edge
FEET_FRESH_S = 0.4    # locked person must have been detected this recently
# the UI may send these straight to the gimbal (laser stays with laser_loop: it enforces the beam safety checks)
GIMBAL_ALLOWED = ("CAM:", "CAMP:", "CAMGET", "CAMSAVE", "PT:", "MV:", "CAL:", "SPD:", "MODE:", "SRV:", "SAVE")
GIMBAL_MOVES = ("PT:", "MV:", "CAL:", "MODE:")   # these stop tracking first, or the two would fight

DEFAULTS = dict(
    tracking=False, detector="person", detect_hz=8.0, conf=0.35, aim=0.15,
    kp_pan=1.5, ki_pan=0.0, kd_pan=0.05, kp_tilt=1.5, ki_tilt=0.0, kd_tilt=0.05,
    # latency: too LOW makes it run away (it thinks you are further along its own motion); too high only feels sluggish.
    # Camera->PC delay measured 0.19-0.44 s at Fast 320, so start at the high end.
    deadband=1.0, max_speed=45.0, latency=0.40, smooth=0.5,
    vfov=0.0,                      # tilt: deg of COMMAND per image height; 0 = hfov*h/w. Measured, because the tilt
                                   # linkage turns only ~70% of the commanded angle on this head
    ff=0.7,                        # velocity feed-forward 0..1: cuts lag on a walking person (0 = pure PID)
    deadband_tilt=1.0,             # tilt has ~4 deg backlash (motion test): smaller deadband makes it hunt
    hfov=61.0,                     # 60.9 measured on the old OV5640; module is now an OV3660 -> calibration.json overrides
    pan_sign=1, tilt_sign=1, lost_s=1.2, release_s=4.0, reacquire_s=4.0,   # walk test: people dropped out for 3-4 s at the frame edge
    # search = stop-and-look: jump between views at search_speed, hold each for search_dwell s (sharp frames for the
    # detector), first toward where the person was heading. Was a slow continuous sweep: 6-17 s to find someone.
    search=True, search_after=0.3, search_speed=60.0, search_dwell=0.7,
    home_pan=0.0, home_tilt=0.0,   # offset from the calibrated 0,0 ("Set home = current position")
    allow_uncalibrated=False,
    laser_mode="off", laser_period=5.0,   # laser: off|on|flash|blink|strobe|sos; never saved, always off at start
    laser_feet=False)                     # aim at the feet + allow the laser while tracking; never saved, off at start
# Results of the gimbal motion test (written by that test, not by hand) become the defaults; params.json still overrides.
CALIB_FILE = os.path.join(HERE, "calibration.json")
if os.path.exists(CALIB_FILE):
    _cal = json.load(open(CALIB_FILE))
    for _src, _k in (("hfov", "hfov"), ("latency_s", "latency"), ("pan_sign", "pan_sign"), ("tilt_sign", "tilt_sign"), ("deadband_deg", "deadband"), ("vfov_deg", "vfov"), ("deadband_tilt_deg", "deadband_tilt")):
        if _cal.get(_src) is not None: DEFAULTS[_k] = type(DEFAULTS[_k])(_cal[_src])
P = dict(DEFAULTS)


def clip(x, lo, hi): return max(lo, min(hi, x))


def limits(st):
    """Firmware soft limits ("lp"/"lt") AND servo reach ("rp"/"rt", fw 3.4+) AND the CAD envelope.
    The firmware's own limits alone are not enough: an off-centre horn left them at pan -17..163 (2026-10-19)."""
    lo, hi = np.array([-PAN_LIM, -TILT_LIM]), np.array([PAN_LIM, TILT_LIM])
    for k, (a, b) in enumerate((("lp", "rp"), ("lt", "rt"))):
        for key in (a, b):
            if st.get(key): lo[k], hi[k] = max(lo[k], st[key][0]), min(hi[k], st[key][1])
    return lo, hi


def home(st):
    lo, hi = limits(st)
    return np.clip([P["home_pan"], P["home_tilt"]], lo, hi)


# ------------------------------------------------------------------ gimbal I/O
class Stream(threading.Thread):
    """Keeps only the newest JPEG; old frames are lag."""
    def __init__(self, ip, url=None):
        super().__init__(daemon=True)
        self.url, self.jpg, self.t, self.seq, self.fps, self.err = url or f"http://{ip}:82/stream", None, 0.0, 0, 0.0, "connecting"
        self.cond = threading.Condition()

    def run(self):
        while True:
            try:
                r, buf, times = urllib.request.urlopen(self.url, timeout=5), b"", deque(maxlen=30)
                self.err = ""
                while True:
                    chunk = r.read(8192)
                    if not chunk: raise IOError("stream closed")
                    buf += chunk
                    while True:
                        a = buf.find(b"\xff\xd8")
                        if a < 0: buf = buf[-1:]; break
                        b = buf.find(b"\xff\xd9", a + 2)
                        if b < 0: buf = buf[a:]; break
                        jpg, buf, now = buf[a:b + 2], buf[b + 2:], time.time()
                        times.append(now)
                        self.fps = (len(times) - 1) / max(times[-1] - times[0], 1e-3)
                        with self.cond:
                            self.jpg, self.t, self.seq = jpg, now, self.seq + 1
                            self.cond.notify_all()
            except Exception as e:
                # 503 = both gimbal video slots taken (close a gimbal-UI tab)
                self.err, self.fps = str(e)[:80], 0.0
                time.sleep(1)


class Gimbal(threading.Thread):
    def __init__(self, ip):
        super().__init__(daemon=True)
        self.url, self.ws, self.status, self.cam, self.connected = f"ws://{ip}:81/", None, {}, {}, False
        self.status_t = 0.0

    def run(self):
        import websocket
        while True:
            self.ws = websocket.WebSocketApp(self.url, on_message=self._msg, on_open=self._open)
            self.ws.run_forever(ping_interval=10, ping_timeout=5)
            self.connected = False
            time.sleep(1)

    def _open(self, ws): self.connected = True; self.send("CAMGET")

    def _msg(self, _, m):
        try:
            if m.startswith('{"t":"s"'): self.status, self.status_t = json.loads(m), time.time()
            elif m.startswith('{"t":"c"'): self.cam = json.loads(m)
        except ValueError:
            pass

    def send(self, c):
        try: self.ws.send(c); return True
        except Exception: return False


class CmdHistory:
    """Where the gimbal was told to point, over the last few seconds (for latency compensation)."""
    def __init__(self): self.h, self.lock = deque(maxlen=300), threading.Lock()
    def add(self, t, cmd):
        with self.lock: self.h.append((t, cmd.copy()))
    def at(self, t):
        with self.lock:
            best = self.h[0][1] if self.h else np.zeros(2)
            for ht, c in reversed(self.h):
                if ht <= t: return c.copy()
            return best.copy()


# ------------------------------------------------------------------ detectors
class PersonDet:
    def __init__(self):
        import torch
        torch.set_num_threads(2)   # all 8 threads spin and starve the stream/control threads
        torch.set_num_interop_threads(1)
        from ultralytics import YOLO
        self.m = YOLO(os.path.join(HERE, "yolo11n-pose.pt"))
        # NVIDIA GPU (needs the CUDA build of torch): ~5 ms instead of 60-90 ms and frees the CPU
        self.gpu = torch.cuda.is_available()
        self.device = 0 if self.gpu else "cpu"
        print("person detector on", torch.cuda.get_device_name(0) if self.gpu else "CPU")
    def __call__(self, img, conf):
        r = self.m.predict(img, imgsz=320, conf=conf, classes=[0], verbose=False, device=self.device)[0]
        h, w = img.shape[:2]
        kps = r.keypoints.data.tolist() if r.keypoints is not None else [[]] * len(r.boxes)
        return [(x1 / w, y1 / h, x2 / w, y2 / h, s, face_point(k, w, h))
                for (x1, y1, x2, y2), s, k in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), kps)]


class FaceDet:
    def __init__(self):
        import mediapipe as mp
        self.d = mp.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.2)
    def __call__(self, img, conf):
        out = []
        for d in self.d.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)).detections or []:
            b = d.location_data.relative_bounding_box
            if d.score[0] >= conf: out.append((b.xmin, b.ymin, b.xmin + b.width, b.ymin + b.height, float(d.score[0])))
        return out


# ------------------------------------------------------------------ tracking + control
class Track:
    def __init__(self, tid, d, t):
        self.id, self.w, self.v, self.size, self.last, self.hits, self.box, self.score = tid, d["w"].copy(), np.zeros(2), d["size"], t, 1, d["box"], d["score"]
        self.face = d.get("face", False)   # aimed at the face (pose keypoints) on the last detection, else box aim height
        self.born = t


class Tracker:
    """IDs by nearest-neighbour matching in gimbal angles + alpha-beta (EMA with velocity) smoothing."""
    def __init__(self): self.tracks, self.next_id, self.locked, self.lock = {}, 1, None, threading.Lock()

    def clear(self):
        with self.lock: self.tracks, self.locked = {}, None

    def update(self, dets, t):
        a = clip(1.0 - P["smooth"], 0.05, 1.0)               # trust in the new measurement
        b = 2 * (2 - a) - 4 * np.sqrt(1 - a)                 # matching velocity gain (critically damped)
        with self.lock:
            pairs = []
            for i, d in enumerate(dets):
                for tr in self.tracks.values():
                    pred = tr.w + tr.v * min(t - tr.last, 0.5)
                    if (dist := float(np.hypot(*(d["w"] - pred)))) < max(8.0, 1.2 * max(d["size"], tr.size)):
                        pairs.append((dist, i, tr.id))
            used_d, used_t = set(), set()
            for _, i, tid in sorted(pairs):                  # ponytail: greedy, fine for a handful of people
                if i in used_d or tid in used_t: continue
                used_d.add(i); used_t.add(tid)
                tr, d = self.tracks[tid], dets[i]
                dt = max(1e-3, t - tr.last)
                pred = tr.w + tr.v * dt
                r = d["w"] - pred
                tr.w, tr.v = pred + a * r, np.clip(tr.v + b * r / dt, -60, 60)
                tr.size, tr.last, tr.hits, tr.box, tr.score = 0.7 * tr.size + 0.3 * d["size"], t, tr.hits + 1, d["box"], d["score"]
                tr.face = d.get("face", False)
            # re-acquire: the locked person vanished briefly (blur, turned away, gimbal moving) and a detection shows
            # up near where they were -> it's them; keep their ID instead of handing the lock to someone else.
            lk, free = self.tracks.get(self.locked), [i for i in range(len(dets)) if i not in used_d]
            if lk is not None and lk.last < t and t - lk.last <= min(P["reacquire_s"], P["release_s"]) and free:
                i = min(free, key=lambda i: np.hypot(*(dets[i]["w"] - lk.w)))
                d = dets[i]
                if np.hypot(*(d["w"] - lk.w - np.clip(lk.v * (t - lk.last), -15, 15))) < max(20.0, 2.0 * max(d["size"], lk.size)):   # where they were heading
                    lk.w, lk.v, lk.size, lk.last, lk.hits, lk.box, lk.score = d["w"].copy(), np.zeros(2), d["size"], t, lk.hits + 1, d["box"], d["score"]
                    lk.face = d.get("face", False)
                    used_d.add(i)
            for i, d in enumerate(dets):
                if i not in used_d:
                    self.tracks[self.next_id] = Track(self.next_id, d, t); self.next_id += 1
            for tid in [k for k, tr in self.tracks.items() if t - tr.last > P["release_s"]]:
                del self.tracks[tid]
            # auto-lock: nobody locked, or the locked track has been lost for lost_s (a fast mover often comes back as a
            # NEW track outside the match gate; waiting release_s for the old one froze the camera with them in view).
            # New lock = a track that APPEARED after the locked one vanished (not a bystander who was already there),
            # nearest to where the lost one was; with nobody locked, the biggest (closest) person.
            lk = self.tracks.get(self.locked)
            if lk is None or t - lk.last > P["lost_s"]:
                cands = [tr for tr in self.tracks.values() if tr.last == t and tr.hits >= 2 and tr is not lk and (lk is None or tr.born > lk.last)]
                if cands:
                    self.locked = (min(cands, key=lambda tr: np.hypot(*(tr.w - lk.w))) if lk else max(cands, key=lambda tr: tr.size)).id
                elif lk is None:
                    self.locked = None

    def set_lock(self, tid):
        with self.lock: self.locked = tid if tid in self.tracks else None

    def target(self, now, latency):
        """Predicted angle of the locked person right now, and seconds since last seen."""
        with self.lock:
            tr = self.tracks.get(self.locked)
            if tr is None: return None, None
            age = now - tr.last
            # short look-ahead only: coasting 1 s through gaps was tried (walk test 2026-10-13) and overshot when
            # the person stopped (p90 error 10 deg vs 5) -> longer losses. Keep 0.25 s.
            return tr.w + tr.v * min(age + latency, 0.25), age

    def velocity(self):
        with self.lock:
            tr = self.tracks.get(self.locked)
            return np.zeros(2) if tr is None else tr.v.copy()


class PID:
    def __init__(self): self.reset()
    def reset(self): self.i, self.prev, self.d = 0.0, None, 0.0
    def step(self, e, dt, kp, ki, kd, limit):
        if ki > 0: self.i = clip(self.i + e * dt, -0.5 * limit / ki, 0.5 * limit / ki)   # anti-windup
        else: self.i = 0.0
        if self.prev is not None: self.d = 0.7 * self.d + 0.3 * (e - self.prev) / dt     # filtered derivative
        self.prev = e
        return kp * e, ki * self.i, kd * self.d


def search_plan(start, heading, lo, hi, home_tilt, hfov):
    """Stop-and-look views [(pan, tilt), ...]: first pass at the lost tilt, from where the person vanished out toward
    `heading` (+1 right / -1 left) to that limit, then the other side; then full-width repeat passes at the tilt rows."""
    a, b = lo[0] + SEARCH_MARGIN, hi[0] - SEARCH_MARGIN
    step, d = max(5.0, SEARCH_STEP_FOV * hfov), 1 if heading >= 0 else -1
    clip_t = lambda t: clip(t, lo[1] + SEARCH_MARGIN, hi[1] - SEARCH_MARGIN)

    def run(p0, direction, limit):   # p0 + step, p0 + 2 step ... ending exactly at the limit
        out, p = [], p0
        while (limit - p) * direction > 1.0:
            p = clip(p + direction * step, min(a, b), max(a, b))
            out.append(p)
        return out
    p0 = clip(start[0], a, b)
    first = run(p0, d, b if d > 0 else a) + run(p0, -d, a if d > 0 else b)
    plan = [(p, clip_t(start[1])) for p in first]
    full = [a] + run(a, 1, b)
    for k, r in enumerate(SEARCH_TILTS):
        plan += [(p, clip_t(home_tilt + r)) for p in (full if k % 2 else full[::-1])]
    return plan


def laser_pattern(mode, t):
    if mode == "on": return True
    if mode == "flash": return t % max(P["laser_period"], 0.5) < 0.25     # short flash every N s
    if mode == "blink": return t % 1.0 < 0.5
    if mode == "strobe": return t % 0.2 < 0.1
    if mode == "sos":
        u = 0.2; seq = [1,1,1, 3,3,3, 1,1,1]                              # dots/dashes in units, 1 unit gaps, 7 gap at end
        total = sum(seq) * u + len(seq) * u + 6 * u
        tt, pos = t % total, 0.0
        for d in seq:
            if pos <= tt < pos + d * u: return True
            pos += (d + 1) * u
        return False
    return False


def beam_hits(box, beam):
    x1, y1, x2, y2 = box
    return x1 - LASER_MARGIN < beam[0] < x2 + LASER_MARGIN and y1 - LASER_MARGIN < beam[1] < y2 + LASER_MARGIN


def feet_block(box, beam):
    """Why a beam at `beam` (picture fractions, y down) is NOT safely at this person's feet ('' = it is)."""
    x1, y1, x2, y2 = box
    if y2 > FEET_EDGE: return "feet not visible (too close / off the bottom of the picture)"
    if beam[1] - FEET_MARGIN < y1 + FEET_ZONE * (y2 - y1): return "beam above the feet"
    return ""


def face_point(kp, w, h):
    """Mean of the confident nose / left eye / right eye keypoints (COCO 0-2), as picture fractions; None if not visible."""
    pts = [(x / w, y / h) for x, y, c in kp[:3] if c >= FACE_KP_CONF]
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)) if pts else None


def make_dets(boxes, ref, shape):
    h, w = shape[:2]
    vfov = P["vfov"] or P["hfov"] * h / w
    aim = (FEET_AIM if P["laser_feet"] else P["aim"]) if P["detector"] == "person" else 0.5
    out = []
    for x1, y1, x2, y2, s, *face in boxes:
        face = face[0] if face else None
        ax, ay = face if face and not P["laser_feet"] else ((x1 + x2) / 2, y1 + aim * (y2 - y1))
        off = np.array([(ax - 0.5) * P["hfov"] * P["pan_sign"], -(ay - 0.5) * vfov * P["tilt_sign"]])
        out.append(dict(w=ref + off, size=(x2 - x1) * P["hfov"], box=(x1, y1, x2, y2), score=s, face=bool(face)))
    return out


# ------------------------------------------------------------------ server
def _try(fn):
    try: fn()
    except Exception: pass   # webhook receiver down: drop the event, never stall the tracker


def detections_payload(tracks, tf, ref, latency, detector):
    """perception.detections for the device service: every track detected in the frame that arrived at tf.
    Bearing = the track's latency-compensated gimbal angles (pan + right, tilt + up). Direction only, no range."""
    dets = [dict(label="person" if detector == "person" else "face", confidence=round(float(t.score), 3), track_id=t.id,
                 box_norm=[round(min(1.0, max(0.0, float(v))), 4) for v in t.box],
                 bearing=dict(pan_deg=round(float(t.w[0]), 2), tilt_deg=round(float(t.w[1]), 2)))
            for t in tracks if t.last == tf]
    return dict(type="perception.detections", data=dict(
        observed_at=tf - latency, latency_s=latency, detections=dets,
        model="yolo11n-pose" if detector == "person" else "mediapipe-blazeface-full-range",
        sensor_pose=dict(pan_deg=round(float(ref[0]), 2), tilt_deg=round(float(ref[1]), 2))))


def main(gimbal_ip, port, webhook=None, stream_url=None, device_url=None, host="127.0.0.1"):
    from flask import Flask, Response, jsonify, request, send_file
    import logging; logging.getLogger("werkzeug").setLevel(logging.WARNING)

    if os.path.exists(PARAMS_FILE): P.update({k: v for k, v in json.load(open(PARAMS_FILE)).items() if k in DEFAULTS})
    P["tracking"], P["laser_mode"], P["laser_feet"] = False, "off", False   # never start moving or lasing by itself
    stream, gimbal, hist, tracker = Stream(gimbal_ip, stream_url), Gimbal(gimbal_ip), CmdHistory(), Tracker()
    S = dict(go_home=True, mode="OFF", det_hz=0.0, det_ms=0.0, det_err="", shape=(240, 320), e=[0, 0], out=[0, 0], terms=[[0] * 3, [0] * 3], cmd=[0, 0])

    # newest detections -> device service (its /v1/perception/events). One slot, one sender thread: a slow or dead
    # service never stalls detection, and stale results are dropped, not queued.
    push = dict(t=0.0, body=None, ev=threading.Event())

    def push_loop():
        while True:
            push["ev"].wait(); push["ev"].clear()
            body = json.dumps(push["body"]).encode()
            _try(lambda: urllib.request.urlopen(urllib.request.Request(device_url, body, {"Content-Type": "application/json"}), timeout=1))

    def detect_loop():
        dets_cache, seq, times = {}, 0, deque(maxlen=10)
        while True:
            t0 = time.time()
            with stream.cond:
                stream.cond.wait_for(lambda: stream.seq != seq, timeout=1)
                jpg, tf, seq = stream.jpg, stream.t, stream.seq
            if jpg is None: continue
            name = P["detector"]
            try:
                if name not in dets_cache:
                    S["det_err"] = f"loading {name} model..."
                    dets_cache[name] = PersonDet() if name == "person" else FaceDet()
                    tracker.clear()
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                # ponytail: no brightening for the model. Measured on dark live frames: raw 24/24 people found,
                # gain 19/24, gamma 8/24 (it amplifies JPEG noise). The viewer brightens in the browser instead.
                ti = time.time()
                boxes = dets_cache[name](img, P["conf"])
                S["det_ms"], S["shape"], S["det_err"] = (time.time() - ti) * 1000, img.shape[:2], ""
                ref = hist.at(tf - P["latency"])
                tracker.update(make_dets(boxes, ref, img.shape), tf)
                if device_url and tf - push["t"] >= 0.25:     # <= 4/s, empty lists too ("nobody in view" is news)
                    with tracker.lock:
                        push["body"] = detections_payload(list(tracker.tracks.values()), tf, ref, P["latency"], name)
                    push["t"] = tf; push["ev"].set()
            except Exception as e:
                S["det_err"] = f"{name}: {e}"[:120]; time.sleep(1)
            times.append(time.time())
            S["det_hz"] = (len(times) - 1) / max(times[-1] - times[0], 1e-3)
            time.sleep(max(0.0, 1.0 / P["detect_hz"] - (time.time() - t0)))

    def control_loop():
        # the only thread that sends PT, so "go home" requests come in through S["go_home"] (no races)
        pids, cmd, last, sent, was = (PID(), PID()), np.zeros(2), time.time(), None, False
        last_seen, last_pos, heading, search, was_alive = time.time(), np.zeros(2), 1, None, True
        mood_sent, mood_t, was_seen = None, 0.0, False
        while True:
            time.sleep(1 / 30)
            now = time.time(); dt = clip(now - last, 1e-3, 0.1); last = now
            st = gimbal.status
            lo, hi = limits(st)
            if P["tracking"] != was:
                for p in pids: p.reset()
                gimbal.send("SPD:240" if P["tracking"] else "SPD:120")   # firmware ramp must not be the limit; our max_speed is
                sent, was = None, P["tracking"]
            if not P["tracking"] and "pan" in st: cmd = np.array([st["pan"], st["tilt"]], float)   # follow manual moves
            if not P["tracking"] and "pan" in st and S.pop("go_home", False):
                h = home(st); gimbal.send(f"PT:{h[0]:.2f},{h[1]:.2f}")
            hist.add(now, cmd)
            tgt, age = tracker.target(now, P["latency"])
            seen = tgt is not None and age <= P["lost_s"]
            e, out, terms = np.zeros(2), np.zeros(2), [[0.0] * 3, [0.0] * 3]
            if seen:
                e = tgt - cmd
                if P["tracking"]:
                    vel = tracker.velocity()
                    for k, ax in enumerate(("pan", "tilt")):
                        ek = e[k] if abs(e[k]) > P["deadband" if k == 0 else "deadband_tilt"] else 0.0
                        terms[k] = list(pids[k].step(ek, dt, P["kp_" + ax], P["ki_" + ax], P["kd_" + ax], P["max_speed"]))
                        # feed-forward: turn at the person's own (estimated) speed so the PID only fixes what's left
                        out[k] = clip(sum(terms[k]) + P["ff"] * vel[k], -P["max_speed"], P["max_speed"])
                    cmd = np.clip(cmd + out * dt, lo, hi)
                    for k in range(2):   # pinned at an end stop with the person further out: don't let Ki wind up
                        if (cmd[k] <= lo[k] and out[k] < 0) or (cmd[k] >= hi[k] and out[k] > 0): pids[k].i = 0.0
                    if S.get("drift") and now < S["drift"][1]:   # tuning: slide the aim away = a person walking at that speed
                        cmd = np.clip(cmd + np.array(S["drift"][0]) * dt, lo, hi)
                    if S.get("kick") is not None:            # tuning: knock the aim off target, watch the PID bring it back
                        cmd = np.clip(cmd + np.array(S.pop("kick")), lo, hi)
                        for p in pids: p.reset()
            else:
                for p in pids: p.reset()                     # lost: hold still, forget the integral
            if seen:   # which way to look first if they vanish: where they were walking, else the frame side they were on
                v = tracker.velocity()[0]
                heading = np.sign(v) if abs(v) > 3 else np.sign(e[0]) if abs(e[0]) > 3 else heading
            if seen or not P["tracking"]: last_seen, last_pos, search = now, cmd.copy(), None
            mode = "OFF" if not P["tracking"] else f"TRACKING ID {tracker.locked}" if seen else "LOST - holding"
            alive = now - gimbal.status_t < STATUS_STALE_S
            if P["tracking"] and not alive:
                # gimbal rebooting (brownout) or Wi-Fi gap: its status is frozen, so neither track nor search on it.
                # Hold the plan where it is and re-send the target as soon as it answers again.
                mode, sent, was_alive = "GIMBAL NOT RESPONDING", None, False
                S.update(e=e.tolist(), out=out.tolist(), terms=terms, cmd=cmd.tolist(), mode=mode)
                continue
            if not was_alive and "pan" in st:
                # back after a reboot: it woke up centred. Carry on from where it IS (no big jump = no new brownout)
                cmd = last_pos = np.clip([st["pan"], st["tilt"]], lo, hi); search, was_alive = None, True
            if P["tracking"] and not seen and now - last_seen > P["search_after"]:
                h, step = home(st), P["search_speed"] * dt
                if P["search"]:
                    if search is None:
                        search = dict(plan=search_plan(last_pos, heading, lo, hi, h[1], P["hfov"]), i=0, until=None, v=0.0)
                    plan = search["plan"]
                    goal = np.array(plan[search["i"] % len(plan)])
                    # move with a speed ramp (accel + brake), then stand still and look
                    dist = float(np.abs(goal - cmd).max())
                    search["v"] = min(search["v"] + SEARCH_ACCEL * dt, P["search_speed"], np.sqrt(2 * SEARCH_ACCEL * dist))
                    cmd = cmd + np.clip(goal - cmd, -search["v"] * dt, search["v"] * dt)
                    if dist < 0.5: search["v"] = 0.0
                    if np.abs(goal - cmd).max() < 0.5:
                        search["until"] = search["until"] or now + P["search_dwell"]
                        if now >= search["until"]: search["i"], search["until"] = search["i"] + 1, None
                    mode = f"SEARCHING view {search['i'] % len(plan) + 1}/{len(plan)}"
                elif tracker.locked is None:                 # search off and the person is gone for good: drift home
                    cmd = cmd + np.clip(h - cmd, -step, step)
                    mode = "HOME - waiting" if np.abs(h - cmd).max() < 0.1 else "RETURNING HOME"
                cmd = np.clip(cmd, lo, hi)
            # the gimbal's OLED eyes show what we're doing (firmware 3.5+; expires to neutral if we stop sending)
            mood = None if not P["tracking"] else "HAPPY" if seen else "SEARCH" if mode.startswith("SEARCH") else "SAD"
            if seen and not was_seen: gimbal.send("MOOD:SURPRISED")      # just (re)found someone
            was_seen = seen
            if mood and (mood != mood_sent or now - mood_t > 2.0):
                gimbal.send("MOOD:" + mood); mood_sent, mood_t = mood, now
            if P["tracking"] and (sent is None or np.abs(cmd - sent).max() > 0.05):
                gimbal.send(f"PT:{cmd[0]:.2f},{cmd[1]:.2f}"); sent = cmd.copy()
            S.update(e=e.tolist(), out=out.tolist(), terms=terms, cmd=cmd.tolist(), mode=mode)

    def laser_block(now):
        """Why the laser must be off right now ('' = allowed). The laser sits just above the camera, so the beam
        lands at the picture centre: never on anyone's box; while tracking only in "Dot at feet", with the beam
        below the locked person's knees and their feet in view."""
        feet = P["tracking"] and P["laser_feet"]
        if P["tracking"] and not feet: return "tracking is on - the beam would be on the person (use Dot at feet)"
        if feet and P["detector"] != "person": return "Dot at feet needs the Person detector"
        if not gimbal.connected: return "gimbal offline"
        if S["det_err"] or S["det_hz"] < 2: return "person detector not running - can't check the beam path"
        h, w = S["shape"]
        vfov, now_cmd = P["vfov"] or P["hfov"] * h / w, hist.at(now)
        with tracker.lock:
            lk = tracker.tracks.get(tracker.locked)
            if feet and (lk is None or now - lk.last > FEET_FRESH_S): return "Dot at feet: locked person not seen right now"
            for t in tracker.tracks.values():
                if now - t.last >= 0.6: continue
                ref = hist.at(t.last - P["latency"])         # where the gimbal pointed when this box's frame was taken
                for c in (ref, now_cmd):                     # the servo is somewhere between then and the latest command
                    beam = (0.5 + (c[0] - ref[0]) / P["hfov"] * P["pan_sign"], 0.5 - (c[1] - ref[1]) / vfov * P["tilt_sign"])
                    if feet and t.id == tracker.locked:
                        if why := feet_block(t.box, beam): return why
                    elif beam_hits(t.box, beam): return f"person ID {t.id} in the beam"
        return ""

    def laser_loop():
        sent, sent_t, mode, start = None, 0.0, "off", time.time()
        while True:
            time.sleep(0.02)
            now = time.time()
            if P["laser_mode"] != mode: mode, start = P["laser_mode"], now
            block = laser_block(now)
            want = not block and laser_pattern(mode, now - start)
            on_now = bool(gimbal.status.get("laser"))
            if want != sent or (block and on_now and now - sent_t > 0.5):    # block also kills a laser lit from the gimbal UI
                gimbal.send("L:1" if want else "L:0"); sent, sent_t = want, now
            if mode == "on" and sent and not on_now and now - sent_t > 1.5:   # firmware 60 s safety timer fired
                P["laser_mode"], S["laser_msg"] = "off", "switched off by the gimbal's 60 s safety timer"
            S["laser_block"] = block if mode != "off" or on_now else ""

    def state():
        now = time.time()
        h, w = S["shape"]
        vfov = (P["vfov"] or P["hfov"] * h / w)
        with tracker.lock:
            tracks = [dict(id=t.id, box=t.box, score=round(t.score, 2), locked=t.id == tracker.locked, fresh=now - t.last < 0.6, face=t.face)
                      for t in tracker.tracks.values()]
        tgt, age = tracker.target(now, 0.0)
        aim = None
        if tgt is not None:                                 # predicted target drawn where it is in the shown frame
            ref = hist.at(now - P["latency"])
            aim = [0.5 + (tgt[0] - ref[0]) / P["hfov"] * P["pan_sign"], 0.5 - (tgt[1] - ref[1]) / vfov * P["tilt_sign"]]
        st = gimbal.status
        return dict(P=P, laser=dict(on=bool(gimbal.status.get("laser")), block=S.get("laser_block", ""), msg=S.get("laser_msg", "")), mode=S["mode"], tracks=tracks, locked=tracker.locked, aim=aim, lost=age is not None and age > P["lost_s"],
                    e=S["e"], epx=[S["e"][0] / P["hfov"] * w, S["e"][1] / vfov * h], out=S["out"], terms=S["terms"], cmd=S["cmd"],
                    gimbal=dict(st, ip=gimbal_ip, connected=gimbal.connected), home=home(st).tolist(),
                    stream=dict(fps=round(stream.fps, 1), err=stream.err), det=dict(hz=round(S["det_hz"], 1), ms=round(S["det_ms"]), err=S["det_err"]),
                    cam=gimbal.cam)

    app = Flask(__name__)

    @app.get("/")
    def index(): return send_file(os.path.join(HERE, "index.html"))

    @app.get("/video")
    def video():
        def gen():
            seq = 0
            while True:
                with stream.cond:
                    stream.cond.wait_for(lambda: stream.seq != seq, timeout=2)
                    jpg, seq = stream.jpg, stream.seq
                if jpg: yield b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n%s\r\n" % (len(jpg), jpg)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")

    @app.get("/events")
    def events():
        def gen():
            while True:
                yield f"data: {json.dumps(state())}\n\n"
                time.sleep(1 / 15)
        return Response(gen(), mimetype="text/event-stream", headers={"Cache-Control": "no-cache"})

    @app.post("/api/params")
    def set_params():
        d, reason = request.get_json(force=True) or {}, ""
        if d.get("tracking") and not P["tracking"]:          # tracking an uncalibrated gimbal = wrong angles, hits end stops
            st = gimbal.status
            if "pan" not in st: reason = "gimbal not connected yet"
            elif not st.get("cal", True) and not d.get("allow_uncalibrated", P["allow_uncalibrated"]):
                reason = "Gimbal NOT calibrated: open the Setup tab (mount, zero, direction, limits, Finish), or tick 'Track anyway'"
            if reason: d.pop("tracking")
        for k, v in d.items():
            if k in DEFAULTS: P[k] = type(DEFAULTS[k])(v)
        return jsonify(ok=not reason, reason=reason)

    @app.post("/api/home")
    def go_home():
        P["tracking"] = False
        S["go_home"] = True
        return jsonify(ok=True)

    @app.post("/api/kick")
    def kick():
        d = request.get_json(force=True) or {}
        S["kick"] = [clip(float(d.get("pan", 0)), -20, 20), clip(float(d.get("tilt", 0)), -10, 10)]
        return jsonify(ok=True)

    @app.post("/api/drift")
    def drift():
        d = request.get_json(force=True) or {}
        S["drift"] = ([clip(float(d.get("pan", 0)), -60, 60), clip(float(d.get("tilt", 0)), -30, 30)], time.time() + clip(float(d.get("secs", 2)), 0, 5))
        return jsonify(ok=True)

    @app.post("/api/home_here")
    def home_here():
        st = gimbal.status
        if "pan" not in st: return jsonify(ok=False, reason="gimbal not connected"), 409
        P["home_pan"], P["home_tilt"] = float(st["pan"]), float(st["tilt"])
        return jsonify(ok=True, home=[P["home_pan"], P["home_tilt"]])

    @app.get("/api/stacks")   # diagnostics: what every server thread is doing (open it if the UI stops responding)
    def stacks():
        import sys, traceback
        nl = chr(10)
        return (nl * 2).join(f"--- {tid}{nl}" + "".join(traceback.format_stack(f)[-6:]) for tid, f in sys._current_frames().items()), 200, {"Content-Type": "text/plain"}

    @app.post("/api/save")
    def save():
        json.dump({k: v for k, v in P.items() if k not in ("tracking", "laser_mode", "laser_feet")}, open(PARAMS_FILE, "w"), indent=1)
        return jsonify(ok=True, file=PARAMS_FILE)

    @app.post("/api/reset")
    def reset():
        P.update({k: v for k, v in DEFAULTS.items() if k != "tracking"})
        return jsonify(ok=True)

    @app.post("/api/lock")
    def lock():
        tracker.set_lock((request.get_json(force=True) or {}).get("id"))
        return jsonify(ok=True, locked=tracker.locked)

    @app.post("/api/gimbal")
    def gimbal_cmd():
        c = str((request.get_json(force=True) or {}).get("c", ""))
        if not (c == "C" or c.startswith(GIMBAL_ALLOWED)): return jsonify(ok=False, err="not allowed"), 403
        if c == "C" or c.startswith(GIMBAL_MOVES): P["tracking"] = False
        return jsonify(ok=gimbal.send(c))

    # ---- simple API for other local apps (BOB voice robot): status, move, home, track, motors, snapshot, presence webhook
    S["webhook"], S["present"] = webhook, False

    def presence_loop():
        # ponytail: presence = any detection in the last 1 s; "left" after LEFT_S without one. Detector runs even when not tracking.
        LEFT_S, gone_since = 3.0, None
        while True:
            time.sleep(0.2)
            now = time.time()
            with tracker.lock:
                n = sum(now - t.last < 1.0 for t in tracker.tracks.values())
            ev = None
            if n and not S["present"]: S["present"], ev = True, "person_entered"
            elif n: gone_since = None
            elif S["present"]:
                gone_since = gone_since or now
                if now - gone_since >= LEFT_S: S["present"], ev, gone_since = False, "person_left", None
            if ev:
                S["last_event"] = dict(event=ev, t=now, count=n, detector=P["detector"])
                if S["webhook"]:
                    body = json.dumps(S["last_event"]).encode()
                    req = urllib.request.Request(S["webhook"], body, {"Content-Type": "application/json"})
                    threading.Thread(target=lambda: _try(lambda: urllib.request.urlopen(req, timeout=2)), daemon=True).start()

    @app.get("/status")
    def api_status():
        st, now = gimbal.status, time.time()
        with tracker.lock:
            people = sum(now - t.last < 1.0 for t in tracker.tracks.values())
        return jsonify(connected=gimbal.connected, pan=st.get("pan"), tilt=st.get("tilt"), calibrated=st.get("cal"),
                       tracking=P["tracking"], mode=S["mode"], people=people, present=S["present"],
                       last_event=S.get("last_event"), motors=dict(pan=st.get("panEn"), tilt=st.get("tiltEn")),
                       home=home(st).tolist(), limits=[list(x) for x in limits(st)], webhook=S["webhook"])

    @app.post("/move")
    def api_move():
        d = request.get_json(force=True) or {}
        if not gimbal.connected: return jsonify(ok=False, reason="gimbal offline"), 409
        P["tracking"] = False                               # a manual move wins over tracking
        pan, tilt = float(d.get("pan", 0)), float(d.get("tilt", 0))
        c = f"MV:{pan:.2f},{tilt:.2f}" if d.get("relative") else f"PT:{pan:.2f},{tilt:.2f}"   # firmware clamps to its limits
        gimbal.send("SRV:P,1"); gimbal.send("SRV:T,1")      # wake relaxed motors
        return jsonify(ok=gimbal.send(c), sent=c)

    @app.post("/home")
    def api_home(): return go_home()

    @app.post("/track")
    def api_track():
        d = request.get_json(force=True) or {}
        if not d.get("on"):
            P["tracking"] = False
            return jsonify(ok=True, tracking=False)
        gimbal.send("SRV:P,1"); gimbal.send("SRV:T,1")
        with app.test_request_context(json={"tracking": True, **({"allow_uncalibrated": True} if d.get("allow_uncalibrated") else {})}):
            r = set_params().get_json()
        return jsonify(ok=r["ok"], tracking=P["tracking"], reason=r["reason"])

    @app.post("/motors")
    def api_motors():
        on = bool((request.get_json(force=True) or {}).get("on"))
        if not on: P["tracking"] = False                    # relaxed servos can't track
        ok = gimbal.send(f"SRV:P,{int(on)}") and gimbal.send(f"SRV:T,{int(on)}")
        return jsonify(ok=ok, motors=on)

    @app.get("/snapshot.jpg")
    def snapshot():
        if not stream.jpg: return jsonify(ok=False, reason="no camera frame yet"), 503
        return Response(stream.jpg, mimetype="image/jpeg", headers={"Cache-Control": "no-cache"})

    @app.post("/webhook")
    def set_webhook():
        url = str((request.get_json(force=True) or {}).get("url") or "")
        if url and not url.startswith(("http://", "https://")): return jsonify(ok=False, reason="url must be http(s)"), 400
        S["webhook"] = url or None
        return jsonify(ok=True, webhook=S["webhook"])

    for th in (stream, gimbal, threading.Thread(target=detect_loop, daemon=True), threading.Thread(target=control_loop, daemon=True),
               threading.Thread(target=laser_loop, daemon=True), threading.Thread(target=presence_loop, daemon=True),
               threading.Thread(target=push_loop, daemon=True)):
        th.start()
    print(f"ANVYXO tracker: http://{host}:{port}  (gimbal {gimbal_ip})")
    app.run(host=host, port=port, threaded=True)   # default localhost only: it can move the gimbal, no auth


def selftest():
    P.update(DEFAULTS)
    tr = Tracker()
    for k in range(20):                                    # A walks right at 20 deg/s, B stands still; order shuffles
        t = k * 0.125
        a = dict(w=np.array([10 + 20 * t, 0.0]), size=12.0, box=(0, 0, 1, 1), score=0.9)
        b = dict(w=np.array([-20.0, 5.0]), size=8.0, box=(0, 0, 1, 1), score=0.9)
        tr.update([a, b] if k % 2 else [b, a], t)
    assert sorted(tr.tracks) == [1, 2], tr.tracks.keys()
    walker = min(tr.tracks.values(), key=lambda x: x.w[1])
    assert abs(walker.v[0] - 20) < 3, walker.v                          # velocity estimate
    assert tr.locked == walker.id                                       # biggest person auto-locked
    pid, cmd, peak = PID(), 0.0, 0.0
    for _ in range(300):                                   # 10 s at 30 Hz toward a fixed 20 deg target
        cmd += clip(sum(pid.step(20 - cmd, 1 / 30, 2.0, 0.0, 0.05, 90)), -90, 90) / 30
        peak = max(peak, cmd)
    assert abs(cmd - 20) < 0.5 and peak < 21, (cmd, peak)

    # re-acquire: locked (smaller) person vanishes 1.5 s, comes back 12 deg away (outside the normal 8 deg match gate)
    # while a bigger person stays in view -> lock and ID stay with them, no new track
    tr, t = Tracker(), 0.0
    me = lambda x: dict(w=np.array([x, 0.0]), size=6.0, box=(0, 0, 1, 1), score=0.9)
    big = dict(w=np.array([-30.0, 0.0]), size=20.0, box=(0, 0, 1, 1), score=0.9)
    for _ in range(4): tr.update([me(0.0), big], t); t += 0.125
    lid = next(k for k, x in tr.tracks.items() if x.size < 10)
    tr.set_lock(lid)
    for _ in range(12): tr.update([big], t); t += 0.125    # 1.5 s gone
    tr.update([me(12.0), big], t)
    assert tr.locked == lid and len(tr.tracks) == 2 and abs(tr.tracks[lid].w[0] - 12) < 1e-6, (tr.locked, tr.tracks.keys())
    # calibrated soft limits from the status JSON clamp home (and every command)
    st = dict(lp=[-30.0, 40.0], lt=[-10.0, 20.0])
    P.update(home_pan=100.0, home_tilt=-50.0)
    assert home(st).tolist() == [40.0, -10.0] and home({}).tolist() == [90.0, -45.0], home(st)
    lo, hi = limits(dict(lp=[-17.0, 163.0], rp=[-17.0, 163.0], lt=[-27.0, 63.0], rt=[-72.0, 108.0]))   # the real lopsided gimbal
    assert lo.tolist() == [-17.0, -27.0] and hi.tolist() == [90.0, 45.0], (lo, hi)
    # search: lost at pan 40 heading right, limits +-90 (85 with margin), 52.8 deg FOV -> right first, then left, then rows
    plan = search_plan(np.array([40.0, 5.0]), 1, np.array([-90.0, -45.0]), np.array([90.0, 45.0]), 0.0, 52.8)
    pans = [p for p, _ in plan]
    assert pans[0] > 40 and pans[1] == 85 and pans[2] < 40 and min(pans[:6]) == -85, pans[:8]
    assert all(t == 5.0 for _, t in plan[:6]) and {t for _, t in plan[6:]} == {0.0, 12.0, -6.0}, plan
    assert all(abs(q - p) <= 0.6 * 52.8 + 1e-9 for p, q in zip(pans[2:5], pans[3:6]))   # neighbouring views overlap
    # lock handoff: locked person lost > lost_s, comes back as a NEW track far away -> lock moves to it, no 4 s freeze
    P.update(DEFAULTS); tr, t = Tracker(), 0.0
    for _ in range(3): tr.update([dict(w=np.array([0.0, 0.0]), size=10.0, box=(0, 0, 1, 1), score=0.9)], t); t += 0.1
    old = tr.locked
    t += P["lost_s"] + 0.1
    for _ in range(2): tr.update([dict(w=np.array([45.0, 0.0]), size=10.0, box=(0, 0, 1, 1), score=0.9)], t); t += 0.1
    assert tr.locked != old and tr.tracks[tr.locked].w[0] == 45.0, (old, tr.locked)
    # face aim: confident nose/eyes win over the box's aim height; none visible -> box aim
    assert face_point([[100, 50, 0.9], [110, 40, 0.8], [90, 40, 0.2]], 200, 100) == (0.525, 0.45)
    assert face_point([[100, 50, 0.1]] * 3, 200, 100) is None
    P.update(DEFAULTS)
    d = make_dets([(0.2, 0.4, 0.6, 1.0, 0.9, (0.5, 0.6)), (0.2, 0.4, 0.6, 1.0, 0.9, None)], np.zeros(2), (240, 320))
    assert d[0]["w"][1] != d[1]["w"][1] and abs(d[0]["w"][1] - (-(0.6 - 0.5) * (P["vfov"] or P["hfov"] * 240 / 320))) < 1e-9
    P.update(DEFAULTS)
    P["laser_period"] = 5.0                                 # laser patterns
    assert laser_pattern("flash", 0.1) and not laser_pattern("flash", 1.0) and laser_pattern("flash", 5.1)
    assert laser_pattern("on", 99) and not laser_pattern("off", 0) and laser_pattern("strobe", 0.05) and not laser_pattern("strobe", 0.15)
    assert laser_pattern("sos", 0.1) and not laser_pattern("sos", 0.3) and laser_pattern("sos", 1.3)   # dot, gap, first dash
    # dot at feet: person 0.2..0.8 tall, beam at centre = on the body -> blocked; centred below the feet -> allowed
    assert feet_block((0.4, 0.2, 0.6, 0.8), (0.5, 0.5)) == "beam above the feet"
    assert feet_block((0.4, 0.2, 0.6, 0.8), (0.5, 0.80)) == ""            # 0.80 - 0.05 >= 0.2 + 0.85*0.6 = 0.71
    assert feet_block((0.4, -0.1, 0.6, 0.6), (0.5, 0.66)) == ""           # tracked: aim 1.1 puts the feet just above centre
    assert feet_block((0.4, 0.0, 0.6, 0.99), (0.5, 0.99)).startswith("feet not visible")
    assert beam_hits((0.4, 0.2, 0.6, 0.8), (0.5, 0.5)) and not beam_hits((0.0, 0.0, 0.2, 0.3), (0.5, 0.5))
    # detections push: only tracks seen in THIS frame, boxes clipped to 0..1, bearing = track angles
    t1 = Track(1, dict(w=np.array([12.5, -3.0]), size=10.0, box=(-0.01, 0.2, 0.5, 1.02), score=0.81), 5.0)
    t2 = Track(2, dict(w=np.array([0.0, 0.0]), size=10.0, box=(0, 0, 1, 1), score=0.9), 4.0)
    d = detections_payload([t1, t2], 5.0, np.array([10.0, 0.0]), 0.31, "person")["data"]
    assert [x["track_id"] for x in d["detections"]] == [1] and d["detections"][0]["box_norm"] == [0.0, 0.2, 0.5, 1.0]
    assert d["detections"][0]["bearing"] == {"pan_deg": 12.5, "tilt_deg": -3.0} and abs(d["observed_at"] - 4.69) < 1e-9
    print("selftest ok")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gimbal", default="192.168.1.8")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--webhook", help="POST person_entered / person_left JSON here, e.g. http://localhost:8090/gimbal/event")
    ap.add_argument("--device-url", help="push detections to the device service, e.g. "
                    "http://127.0.0.1:8100/v1/perception/events?token=...  (enables its perception.detections)")
    ap.add_argument("--stream-url", help="read video here instead of the gimbal's :82 (e.g. the device service: "
                    "http://127.0.0.1:8100/v1/camera/stream?token=...), so both don't use up the gimbal's 2 stream slots")
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 = reachable from the LAN (e.g. headless Jetson). "
                    "NO login: anyone on the network can move the gimbal and fire the laser. Prefer an SSH tunnel.")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    selftest() if a.selftest else main(a.gimbal, a.port, a.webhook, a.stream_url, a.device_url, a.host)
