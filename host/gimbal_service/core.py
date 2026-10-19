"""Device logic: validation, operations (move / home / survey), stop, observations, events, health.

Everything physical goes through the firmware, which keeps its own soft limits, servo-reach clamp, speed cap and
laser timer. This layer adds what the firmware can't know: request validation (reject, never silently clamp),
one motion operation at a time, acceptance vs completion, timeouts, cancellation and timestamped observations.

The servos are open-loop hobby servos: "completed" means the firmware's commanded trajectory finished and the
settle time passed, never that the head was measured there. Every pose says so (source + physically_verified).
"""
import hashlib, math, os, threading, time, uuid
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone

CONTRACT = "anvyxo-device-proposal/0.1"   # until the orchestrator's canonical contract exists (docs/INTEGRATION.md)
RESOLUTIONS = ["320x240", "400x296", "480x320", "640x480", "800x600", "1024x768", "1280x720", "1280x1024", "1600x1200"]
POSE_SOURCE = "firmware_commanded_open_loop"
EXPOSURE_MARGIN_S = 0.15   # latency jitter: a frame counts as "at rest" only if the head was still this much longer
REJECTED_EVENT_TYPES = {"perception.entry_candidate", "perception.entry_confirmed", "perception.exit_candidate",
                        "perception.exit_confirmed"}   # reserved: needs doorway coverage + a classifier (perception.py)


def iso(t): return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
def new_id(prefix): return f"{prefix}_{uuid.uuid4().hex[:16]}"


def _env(name, default, cast=str):
    v = os.environ.get(name)
    return default if v in (None, "") else (v.lower() in ("1", "true", "yes", "on") if cast is bool else cast(v))


@dataclass
class Config:
    device_id: str = "gimbal-01"
    host: str = "192.168.1.6"                 # firmware IP / anvyxo.local, or "mock"
    token: str = ""                           # bearer token; empty = only allowed when bound to loopback
    bind: str = "127.0.0.1"
    port: int = 8100
    pan_limit_deg: float = 90.0               # host cap = CAD collision-checked envelope (gimbal.py), on top of firmware limits
    tilt_limit_deg: float = 45.0
    max_speed_dps: float = 120.0              # host cap; firmware hard cap is 360
    default_speed_dps: float = 60.0
    allow_uncalibrated: bool = False
    settle_s: float = 0.3                     # after the commanded slew ends (servo + head wobble)
    ack_timeout_s: float = 1.0                # firmware must echo the new target in its status within this
    status_stale_s: float = 1.0               # firmware sends status every 150 ms
    frame_stale_s: float = 1.0
    camera_latency_s: float = 0.31            # exposure -> host arrival, measured 2026-10-13 (Tracker/calibration.json)
    hfov_deg: float = 52.8                    # measured on the OV3660 (Tracker/calibration.json); 0 = unknown
    vfov_deg: float = 0.0
    survey_max_points: int = 60
    survey_settle_s: float = 0.8              # >= camera_latency_s so the captured frame was exposed at rest
    max_observations: int = 200               # kept in memory (images included)
    persist_dir: str = ""                     # empty = memory only (privacy default); else JPEG + JSON per observation
    retention_hours: float = 24.0
    lopsided_deg: float = 30.0                # servo reach asymmetry that means the horn is mounted off-centre

    @classmethod
    def from_env(cls):
        c, d = cls(), cls.__dataclass_fields__
        for k, f in d.items():
            setattr(c, k, _env("GIMBAL_" + k.upper(), getattr(c, k), f.type if f.type in (bool, int, float) else str))
        return c


class ApiError(Exception):
    def __init__(self, http, code, message, **details):
        super().__init__(message)
        self.http, self.code, self.message, self.details = http, code, message, details

    def body(self, request_id=None):
        return {"error": {"code": self.code, "message": self.message, "details": self.details, "request_id": request_id}}


def num(d, key, default=None, lo=None, hi=None):
    v = d.get(key, default)
    if v is None: raise ApiError(422, "INVALID_ARGUMENT", f"'{key}' is required")
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ApiError(422, "INVALID_ARGUMENT", f"'{key}' must be a finite number", field=key)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ApiError(422, "OUT_OF_RANGE", f"'{key}'={v} outside {lo}..{hi}", field=key, min=lo, max=hi)
    return float(v)


def only(d, *keys):
    """Reject unknown fields: a typo must fail loudly, not run with defaults."""
    extra = sorted(set(d) - set(keys) - {"request_id"})
    if extra: raise ApiError(422, "UNKNOWN_FIELD", f"unknown field(s) {extra}", allowed=sorted(keys))
    return d


def jpeg_size(jpg):
    """(width, height) from the JPEG SOF marker, or (None, None)."""
    i = 2
    while jpg and i + 9 < len(jpg):
        if jpg[i] != 0xFF: i += 1; continue
        m, ln = jpg[i + 1], int.from_bytes(jpg[i + 2:i + 4], "big")
        if m in (0xC0, 0xC1, 0xC2): return int.from_bytes(jpg[i + 7:i + 9], "big"), int.from_bytes(jpg[i + 5:i + 7], "big")
        i += 2 + ln
    return None, None


class EventBus:
    def __init__(self, device_id, keep=1000):
        self.device_id, self.seq, self.ring, self.cond = device_id, 0, deque(maxlen=keep), threading.Condition()

    def publish(self, type_, data, severity="info"):
        with self.cond:
            self.seq += 1
            ev = {"schema": "event/v0", "event_id": new_id("evt"), "seq": self.seq, "type": type_, "time": iso(time.time()),
                  "device_id": self.device_id, "severity": severity, "data": data}
            self.ring.append(ev)
            self.cond.notify_all()
        return ev

    def since(self, seq, timeout=0.0):
        with self.cond:
            if timeout: self.cond.wait_for(lambda: self.seq > seq, timeout=timeout)
            return [e for e in self.ring if e["seq"] > seq]


@dataclass
class Operation:
    kind: str
    params: dict
    request_id: str
    operation_id: str = field(default_factory=lambda: new_id("op"))
    state: str = "accepted"          # accepted -> running -> succeeded | failed | cancelled | preempted
    created_at: float = field(default_factory=time.time)
    started_at: float = None
    device_ack_at: float = None      # firmware echoed the new target (command accepted by the DEVICE)
    ended_at: float = None
    progress: dict = field(default_factory=dict)
    result: dict = None
    error: dict = None
    cancel: threading.Event = field(default_factory=threading.Event)
    cancel_reason: str = ""

    @property
    def active(self): return self.state in ("accepted", "running")

    def to_dict(self):
        t = lambda v: v and iso(v)
        return {"schema": "operation/v0", "operation_id": self.operation_id, "request_id": self.request_id, "kind": self.kind,
                "state": self.state, "params": self.params, "created_at": t(self.created_at), "started_at": t(self.started_at),
                "device_ack_at": t(self.device_ack_at), "ended_at": t(self.ended_at), "progress": self.progress,
                "result": self.result, "error": self.error,
                "completion_evidence": "commanded trajectory finished + settle time (open loop, no position sensor)"}


class _OpEnd(Exception):
    def __init__(self, state, code=None, message=""):
        super().__init__(message); self.state, self.code, self.message = state, code, message


class Device:
    def __init__(self, cfg, link):
        self.cfg, self.link = cfg, link
        self.bus = EventBus(cfg.device_id)
        self.ops, self.active, self.lock = OrderedDict(), None, threading.Lock()
        self.obs, self.obs_lock = OrderedDict(), threading.Lock()
        self.hist = deque(maxlen=200)                       # (t, pan, tilt, tp, tt) for pose-at-exposure
        self.presence = {"state": "unknown", "since": None, "source": None}
        self.sensor_id, self.frame_id = f"{cfg.device_id}/camera0", f"{cfg.device_id}/pan_tilt_base"
        self._closing = False
        if cfg.persist_dir: os.makedirs(cfg.persist_dir, exist_ok=True)
        threading.Thread(target=self._monitor, daemon=True, name="device-monitor").start()

    # ------------------------------------------------------------------ state
    def _st(self): return self.link.status or {}

    def status_fresh(self): return self.link.connected and time.time() - self.link.status_t < self.cfg.status_stale_s

    def limits(self):
        """Usable range = firmware soft limits AND servo reach (firmware 3.4+) AND the host's CAD envelope."""
        st, out = self._st(), {}
        for ax, l, r in (("pan", "lp", "rp"), ("tilt", "lt", "rt")):
            cap = self.cfg.pan_limit_deg if ax == "pan" else self.cfg.tilt_limit_deg
            lo, hi = st.get(l) or [-cap, cap]
            lo, hi = max(lo, -cap), min(hi, cap)
            if st.get(r): lo, hi = max(lo, st[r][0]), min(hi, st[r][1])
            out[ax] = [lo, hi]
        return out

    def faults(self):
        st, f = self._st(), []
        add = lambda code, sev, msg: f.append({"code": code, "severity": sev, "message": msg})
        if not self.link.connected: add("DEVICE_DISCONNECTED", "error", f"no WebSocket to firmware at {self.link.host}")
        elif not self.status_fresh(): add("STATUS_STALE", "error", "firmware status older than %.1f s" % self.cfg.status_stale_s)
        cam = self.camera_status()
        if not cam["ready"]: add("CAMERA_UNAVAILABLE", "error", cam["stream"]["error"] or "no fresh frame")
        if st.get("calOn"): add("CALIBRATION_IN_PROGRESS", "error", "calibration wizard active: soft limits are off")
        if st and not st.get("cal"):
            add("NOT_CALIBRATED", "warning" if self.cfg.allow_uncalibrated else "error", "zero/limits not confirmed (UI Setup tab)")
        if st and (st.get("panEn") is False or st.get("tiltEn") is False): add("MOTORS_DISABLED", "error", "servo signal off (limp)")
        for ax, r in (("pan", st.get("rp")), ("tilt", st.get("rt"))):
            if r and abs(r[0] + r[1]) > self.cfg.lopsided_deg:
                add("LOPSIDED_MOUNT", "warning", f"{ax} servo reach {r[0]:.0f}..{r[1]:.0f} deg: horn mounted off-centre")
        return f

    def ready_reasons(self, include_camera=True):
        return [x["code"] for x in self.faults() if x["severity"] == "error" and (include_camera or x["code"] != "CAMERA_UNAVAILABLE")]

    def pose(self, st=None):
        st = st or self._st()
        if "pan" not in st: return None
        moving = abs(st["pan"] - st["tp"]) > 0.15 or abs(st["tilt"] - st["tt"]) > 0.15
        return {"frame_id": self.frame_id, "representation": "pan_tilt_relative", "pan_deg": st["pan"], "tilt_deg": st["tilt"],
                "target_pan_deg": st["tp"], "target_tilt_deg": st["tt"], "moving": moving, "source": POSE_SOURCE,
                "physically_verified": False}

    def status(self):
        st, now, cal = self._st(), time.time(), self.cfg
        lim = self.limits()
        return {"schema": "device_status/v0", "device_id": cal.device_id, "device_type": "pan_tilt_camera", "time": iso(now),
                "connection": {"state": "connected" if self.link.connected else "disconnected", "host": self.link.host,
                               "last_status_at": iso(self.link.status_t) if self.link.status_t else None,
                               "status_age_ms": round((now - self.link.status_t) * 1000) if self.link.status_t else None},
                "ready": not self.ready_reasons(), "not_ready_reasons": self.ready_reasons(),
                "calibration": {"calibrated": st.get("cal"), "in_progress": st.get("calOn"),
                                "direction_reversed": {"pan": st.get("invP"), "tilt": st.get("invT")},
                                "uncalibrated_motion_allowed": cal.allow_uncalibrated},
                "pose": self.pose(st),
                "limits": {"pan_deg": lim["pan"], "tilt_deg": lim["tilt"], "soft_pan_deg": st.get("lp"), "soft_tilt_deg": st.get("lt"),
                           "servo_reach_pan_deg": st.get("rp"), "servo_reach_tilt_deg": st.get("rt"), "max_speed_dps": cal.max_speed_dps,
                           "acceleration_limited": False},
                "motion": {"firmware_mode": st.get("mode"), "speed_dps": st.get("speed"),
                           "motors_enabled": {"pan": st.get("panEn"), "tilt": st.get("tiltEn")},
                           "active_operation_id": self.active.operation_id if self.active and self.active.active else None},
                "laser": {"on": st.get("laser"), "controlled_by": "tracker/UI (not exposed by this API)"},
                "presence": self.presence, "camera": self.camera_status(), "faults": self.faults()}

    def camera_status(self):
        jpg, t, seq = self.link.frame()
        now, cam = time.time(), self.link.cam or {}
        age = now - t if jpg else None
        fresh = jpg is not None and age < self.cfg.frame_stale_s
        w, h = jpeg_size(jpg) if jpg else (None, None)
        res = cam.get("res")
        return {"sensor_id": self.sensor_id, "sensor_type": "camera.rgb", "model": cam.get("sensor"), "ready": bool(self.link.stream_ok and fresh),
                "stream": {"connected": bool(self.link.stream_ok), "error": self.link.stream_err or None, "fps": round(self.link.fps, 1),
                           "path": "/v1/camera/stream", "encoding": "image/jpeg (MJPEG, frames re-served unchanged)"},
                "last_frame_at": iso(t) if jpg else None, "frame_age_ms": round(age * 1000) if age is not None else None,
                "stale": not fresh, "frame_seq": seq, "width": w, "height": h,
                "config": {"resolution_index": res, "resolution": RESOLUTIONS[res] if isinstance(res, int) and 0 <= res < len(RESOLUTIONS) else None,
                           "fps_cap": cam.get("fps"), "jpeg_quality": cam.get("q")},
                "timing": {"timestamp_source": "host_receive_time", "estimated_capture_latency_s": self.cfg.camera_latency_s},
                "intrinsics": {"calibration_ref": None, "hfov_deg": self.cfg.hfov_deg or None, "vfov_deg": self.cfg.vfov_deg or None,
                               "note": "FOV measured by motion test; no lens intrinsics/distortion calibration exists"}}

    # ------------------------------------------------------------------ operations
    def _require_motion_ready(self):
        if not self.status_fresh(): raise ApiError(503, "DEVICE_UNAVAILABLE", "gimbal firmware not connected or status stale")
        bad = self.ready_reasons(include_camera=False)   # moving doesn't need the camera
        if bad:
            raise ApiError(409, bad[0], "gimbal not ready for motion: " + ", ".join(bad), reasons=bad)

    def _check_target(self, pan, tilt):
        lim = self.limits()
        for ax, v in (("pan", pan), ("tilt", tilt)):
            lo, hi = lim[ax]
            if not lo - 1e-6 <= v <= hi + 1e-6:
                raise ApiError(422, "OUT_OF_LIMITS", f"{ax} {v:.2f} deg outside allowed {lo:.1f}..{hi:.1f} deg", axis=ax, min=lo, max=hi)

    def _speed(self, d):
        return num(d, "speed_dps", self.cfg.default_speed_dps, 10.0, self.cfg.max_speed_dps)

    def move(self, d, request_id):
        only(d, "mode", "pan_deg", "tilt_deg", "speed_dps", "preempt")
        self._require_motion_ready()
        mode = d.get("mode", "absolute")
        if mode not in ("absolute", "relative"): raise ApiError(422, "INVALID_ARGUMENT", "mode must be absolute|relative", field="mode")
        st, speed = self._st(), self._speed(d)
        dp, dt = num(d, "pan_deg", 0.0 if mode == "relative" else None), num(d, "tilt_deg", 0.0 if mode == "relative" else None)
        pan, tilt = (st["tp"] + dp, st["tt"] + dt) if mode == "relative" else (dp, dt)
        self._check_target(pan, tilt)
        params = {"mode": mode, "pan_deg": dp, "tilt_deg": dt, "target_pan_deg": pan, "target_tilt_deg": tilt, "speed_dps": speed}
        return self._start("move", params, request_id, d.get("preempt", False), lambda op: self._run_move(op, pan, tilt, speed))

    def home(self, d, request_id):
        only(d, "speed_dps", "preempt")
        self._require_motion_ready()
        speed = self._speed(d)
        self._check_target(0.0, 0.0)
        return self._start("home", {"target_pan_deg": 0.0, "target_tilt_deg": 0.0, "speed_dps": speed, "home": "calibrated zero"},
                           request_id, d.get("preempt", False), lambda op: self._run_move(op, 0.0, 0.0, speed))

    def survey(self, d, request_id):
        only(d, "waypoints", "grid", "restore", "speed_dps", "settle_s", "preempt")
        if "grid" in d: only(d["grid"] if isinstance(d["grid"], dict) else {}, "pan_from_deg", "pan_to_deg", "pan_step_deg", "tilts_deg")
        self._require_motion_ready()
        if not self.camera_status()["ready"]: raise ApiError(409, "CAMERA_UNAVAILABLE", "camera has no fresh frames")
        pts = d.get("waypoints")
        if pts is None:   # grid: pan from..to step, at each tilt
            g = d.get("grid") or {}
            a, b = num(g, "pan_from_deg"), num(g, "pan_to_deg")
            step = num(g, "pan_step_deg", 20.0, 1.0, 180.0)
            tilts = g.get("tilts_deg", [0.0])
            if not isinstance(tilts, list) or not tilts: raise ApiError(422, "INVALID_ARGUMENT", "grid.tilts_deg must be a non-empty list")
            n = int(abs(b - a) // step) + 1
            pans = [a + math.copysign(step, b - a) * i for i in range(n)]
            pts = [[p, num({"t": t}, "t")] for k, t in enumerate(tilts) for p in (pans if k % 2 == 0 else pans[::-1])]   # serpentine
        if not isinstance(pts, list) or not pts: raise ApiError(422, "INVALID_ARGUMENT", "waypoints must be a non-empty list of [pan, tilt]")
        if len(pts) > self.cfg.survey_max_points:
            raise ApiError(422, "OUT_OF_RANGE", f"{len(pts)} waypoints > max {self.cfg.survey_max_points}", max=self.cfg.survey_max_points)
        clean = []
        for i, p in enumerate(pts):
            if not (isinstance(p, (list, tuple)) and len(p) == 2): raise ApiError(422, "INVALID_ARGUMENT", f"waypoint {i} must be [pan, tilt]")
            pan, tilt = num({"p": p[0]}, "p"), num({"t": p[1]}, "t")
            self._check_target(pan, tilt)
            clean.append([pan, tilt])
        restore = d.get("restore", "previous")
        if restore not in ("none", "previous", "home"): raise ApiError(422, "INVALID_ARGUMENT", "restore must be none|previous|home")
        if restore == "home": self._check_target(0.0, 0.0)
        speed, settle = self._speed(d), num(d, "settle_s", self.cfg.survey_settle_s, self.cfg.camera_latency_s, 10.0)
        params = {"waypoints": clean, "speed_dps": speed, "settle_s": settle, "restore": restore}
        return self._start("survey", params, request_id, d.get("preempt", False), lambda op: self._run_survey(op, clean, speed, settle, restore))

    def _start(self, kind, params, request_id, preempt, runner):
        if not isinstance(preempt, bool): raise ApiError(422, "INVALID_ARGUMENT", "preempt must be true/false")
        with self.lock:
            old = self.active
            if old and old.active:
                if not preempt:
                    raise ApiError(409, "BUSY", f"operation {old.operation_id} ({old.kind}) is still {old.state}; stop it or send preempt=true",
                                   active_operation_id=old.operation_id)
                old.cancel_reason = "preempted"; old.cancel.set()
            op = Operation(kind, params, request_id)
            self.ops[op.operation_id] = op
            while len(self.ops) > 200: self.ops.popitem(last=False)
            self.active = op
        if old and old.active:
            for _ in range(100):                # the old runner stops at its next check (<= 20 ms)
                if not old.active: break
                time.sleep(0.02)
        self.bus.publish("operation.accepted", op.to_dict())
        threading.Thread(target=self._run, args=(op, runner), daemon=True, name=f"op-{op.operation_id}").start()
        return op

    def _run(self, op, runner):
        op.state, op.started_at = "running", time.time()
        self.bus.publish("operation.started", {"operation_id": op.operation_id, "kind": op.kind})
        try:
            op.result = runner(op)
            op.state = "succeeded"
        except _OpEnd as e:
            op.state = e.state
            if e.code: op.error = {"code": e.code, "message": e.message}
        except Exception as e:   # bug or link exception: never leave an operation "running"
            op.state, op.error = "failed", {"code": "INTERNAL_ERROR", "message": str(e)[:200]}
        op.ended_at = time.time()
        sev = "info" if op.state == "succeeded" else "warning" if op.state in ("cancelled", "preempted") else "error"
        self.bus.publish("operation." + op.state, op.to_dict(), sev)

    def _check_cancel(self, op):
        if op.cancel.is_set():
            if op.cancel_reason != "preempted": self.link.send("MODE:MANUAL")   # freeze where it is now
            raise _OpEnd("preempted" if op.cancel_reason == "preempted" else "cancelled", "CANCELLED" if op.cancel_reason != "preempted" else "PREEMPTED",
                         op.cancel_reason or "cancelled")

    def _sleep(self, op, s):
        end = time.time() + s
        while time.time() < end:
            self._check_cancel(op)
            time.sleep(0.02)

    def _goto(self, op, pan, tilt, speed):
        """Command a target, confirm the firmware accepted it, wait for the commanded slew to end + settle."""
        self._check_cancel(op)
        st0 = self._st()
        dist = max(abs(pan - st0.get("pan", pan)), abs(tilt - st0.get("tilt", tilt)))
        t_send = time.time()
        if not (self.link.send(f"SPD:{speed:.0f}") and self.link.send(f"PT:{pan:.2f},{tilt:.2f}")):
            raise _OpEnd("failed", "LINK_LOST", "could not send to firmware")
        hit = lambda st: abs(st.get("tp", 1e9) - pan) <= 0.15 and abs(st.get("tt", 1e9) - tilt) <= 0.15
        while not (self.link.status_t > t_send and hit(self._st())):            # acceptance = firmware echoes the target
            self._check_cancel(op)
            if time.time() - t_send > self.cfg.ack_timeout_s:
                raise _OpEnd("failed", "COMMAND_NOT_ACKNOWLEDGED", "firmware status never showed the new target")
            time.sleep(0.02)
        op.device_ack_at = op.device_ack_at or time.time()
        deadline = t_send + dist / speed * 1.5 + 2.0
        while True:
            self._check_cancel(op)
            st = self._st()
            if not self.status_fresh(): raise _OpEnd("failed", "LINK_LOST", "firmware status lost during motion (firmware finishes the slew on its own)")
            if not hit(st): raise _OpEnd("failed", "PREEMPTED_EXTERNALLY", f"firmware target changed by another client to {st.get('tp')},{st.get('tt')}")
            if abs(st["pan"] - pan) <= 0.15 and abs(st["tilt"] - tilt) <= 0.15: break
            if time.time() > deadline:
                self.link.send("MODE:MANUAL")
                raise _OpEnd("failed", "MOTION_TIMEOUT", f"commanded slew not finished after {deadline - t_send:.1f} s; stopped")
            time.sleep(0.02)
        t_arrive = time.time()
        self._sleep(op, self.cfg.settle_s)
        return t_arrive

    def _run_move(self, op, pan, tilt, speed):
        prev = self._st().get("speed")
        try:
            self._goto(op, pan, tilt, speed)
        finally:
            if prev and self.link.connected: self.link.send(f"SPD:{prev:.0f}")   # leave the UI's speed setting as it was
        return {"pose": self.pose()}

    def _run_survey(self, op, pts, speed, settle, restore):
        st0, prev = self._st(), self._st().get("speed")
        start = (st0["pan"], st0["tilt"])
        steps = [{"index": i, "target_pan_deg": p, "target_tilt_deg": t, "status": "pending", "observation_id": None, "error": None}
                 for i, (p, t) in enumerate(pts)]
        op.progress = {"completed_steps": 0, "total_steps": len(pts)}
        op.result = {"survey_id": op.operation_id, "steps": steps, "observation_ids": [], "complete": False,
                     "note": "observations from several directions; NOT a map (no depth, no localisation)"}
        self.bus.publish("survey.started", {"survey_id": op.operation_id, "total_steps": len(pts)})
        try:
            for s in steps:
                self._goto(op, s["target_pan_deg"], s["target_tilt_deg"], speed)
                t_rest = time.time()
                self._sleep(op, max(0.0, settle - self.cfg.settle_s))
                try:   # the frame must have been EXPOSED after the head came to rest
                    o = self.observe(after_t=t_rest + self.cfg.camera_latency_s + EXPOSURE_MARGIN_S, timeout=3.0,
                                     source={"survey_id": op.operation_id, "operation_id": op.operation_id, "step_index": s["index"]})
                    s["status"], s["observation_id"] = "captured", o["observation_id"]
                    op.result["observation_ids"].append(o["observation_id"])
                except ApiError as e:
                    s["status"], s["error"] = "capture_failed", {"code": e.code, "message": e.message}
                op.progress = {"completed_steps": s["index"] + 1, "total_steps": len(pts)}
                self.bus.publish("operation.progress", {"operation_id": op.operation_id, **op.progress, "step": s})
            failed = [s for s in steps if s["status"] != "captured"]
            if restore != "none":
                self._goto(op, *(start if restore == "previous" else (0.0, 0.0)), speed)
            op.result["complete"] = not failed
            self.bus.publish("survey.completed", {"survey_id": op.operation_id, "captured": len(op.result["observation_ids"]),
                                                  "total_steps": len(pts), "complete": not failed})
            if failed: raise _OpEnd("failed", "SURVEY_INCOMPLETE", f"{len(failed)} of {len(pts)} captures failed (partial result kept)")
            return op.result
        except _OpEnd as e:
            for s in steps:
                if s["status"] == "pending": s["status"] = "not_reached"
            if e.code != "SURVEY_INCOMPLETE":
                self.bus.publish("survey.completed", {"survey_id": op.operation_id, "captured": len(op.result["observation_ids"]),
                                                      "total_steps": len(pts), "complete": False, "ended": e.state})
            raise
        finally:
            if prev and self.link.connected: self.link.send(f"SPD:{prev:.0f}")

    def stop(self, request_id):
        op = self.active if self.active and self.active.active else None
        if op: op.cancel_reason = "stop requested"; op.cancel.set()
        t_send = time.time()
        sent = self.link.send("MODE:MANUAL")                 # firmware: target := current position (no jump)
        if op:
            for _ in range(100):
                if not op.active: break
                time.sleep(0.02)
        # report where it actually froze: a status sent AFTER the stop, with target == position
        confirmed, end = False, time.time() + 1.0
        while sent and time.time() < end:
            p = self.pose()
            if self.link.status_t > t_send + 0.05 and p and not p["moving"]: confirmed = True; break
            time.sleep(0.02)
        self.bus.publish("motion.stopped", {"request_id": request_id, "cancelled_operation_id": op and op.operation_id, "sent": sent},
                         "info" if sent else "error")
        if not sent: raise ApiError(503, "DEVICE_UNAVAILABLE", "stop could not be sent: firmware not connected (it holds its last target)",
                                    cancelled_operation_id=op and op.operation_id)
        return {"stopped": True, "confirmed_by_device": confirmed, "cancelled_operation_id": op and op.operation_id, "pose": self.pose(),
                "note": "firmware froze its target at the commanded position; confirmed_by_device = a later status showed it holding there"}

    def cancel(self, op_id, request_id):
        op = self.ops.get(op_id)
        if not op: raise ApiError(404, "NOT_FOUND", f"no operation {op_id}")
        if not op.active: return op
        return self.stop(request_id) and op

    # ------------------------------------------------------------------ observations
    def _pose_at(self, t):
        """Commanded pose when the frame was exposed (~latency before it arrived) and whether it was at rest.
        Status is a step function: the pose at exposure = last status at/before it, plus any that came during it."""
        t0 = t - self.cfg.camera_latency_s - EXPOSURE_MARGIN_S
        hist = sorted(self.hist)
        before = [h for h in hist if h[0] <= t0]
        win = before[-1:] + [h for h in hist if t0 < h[0] <= t]
        if not win:
            st = self._st()
            win = [(t, st.get("pan", 0.0), st.get("tilt", 0.0), st.get("tp", 1e9), st.get("tt", 1e9))]   # unknown -> not at rest
        expo = [h for h in win if h[0] <= t - self.cfg.camera_latency_s] or win[:1]
        _, p, ti, _, _ = expo[-1]
        still = all(abs(h[1] - win[0][1]) < 0.15 and abs(h[2] - win[0][2]) < 0.15 and abs(h[1] - h[3]) < 0.15 and abs(h[2] - h[4]) < 0.15 for h in win)
        return {"frame_id": self.frame_id, "representation": "pan_tilt_relative", "pan_deg": p, "tilt_deg": ti, "source": POSE_SOURCE,
                "physically_verified": False, "at_rest": still, "global_pose": None,
                "note": "angles relative to this gimbal's calibrated zero; no transform to base_link/map is known"}, still

    def observe(self, after_t=None, max_age_s=None, source=None, timeout=3.0, store=True):
        """A camera observation. after_t: the frame must have arrived after this host time (fresh capture)."""
        if not self.link.stream_ok: raise ApiError(503, "CAMERA_UNAVAILABLE", self.link.stream_err or "camera stream not connected")
        end = time.time() + timeout
        jpg, t, seq = self.link.frame()
        while after_t is not None and (jpg is None or t < after_t):
            if time.time() > end: raise ApiError(504, "FRAME_TIMEOUT", "no fresh frame arrived in time")
            jpg, t, seq = self.link.wait_frame(seq, max(0.01, end - time.time()))
        if jpg is None: raise ApiError(503, "CAMERA_UNAVAILABLE", "no frame received yet")
        now, max_age = time.time(), self.cfg.frame_stale_s if max_age_s is None else max_age_s
        if now - t > max_age: raise ApiError(503, "FRAME_STALE", f"newest frame is {now - t:.2f} s old (> {max_age} s)", frame_age_ms=round((now - t) * 1000))
        pose, still = self._pose_at(t)
        w, h = jpeg_size(jpg)
        oid = new_id("obs")
        cs = self.camera_status()
        o = {"schema": "observation/v0", "observation_id": oid, "created_at": iso(now), "timestamp": iso(t - self.cfg.camera_latency_s),
             "received_at": iso(t), "device_id": self.cfg.device_id, "sensor_id": self.sensor_id, "sensor_type": "camera.rgb",
             "data": {"media_type": "image/jpeg", "ref": f"/v1/observations/{oid}/image", "width": w, "height": h, "bytes": len(jpg),
                      "sha256": hashlib.sha256(jpg).hexdigest()},
             "camera": {"model": cs["model"], "calibration_ref": None, "hfov_deg": cs["intrinsics"]["hfov_deg"], "config": cs["config"]},
             "pose": pose,
             "timing": {"timestamp_source": "host_receive_time minus estimated_capture_latency_s", "estimated_capture_latency_s": self.cfg.camera_latency_s,
                        "frame_age_ms_at_creation": round((now - t) * 1000), "clock": "host UTC (not synchronised with firmware)"},
             "validity": {"valid": True, "stale": False, "pose_at_rest": still, "reasons": [] if still else ["gimbal moving during exposure window"]},
             "confidence": None, "processing": {"status": "raw", "model": None, "model_version": None, "results": []},
             "source": source or {}}
        if store: self._store(o, jpg)
        self.bus.publish("observation.created", {k: o[k] for k in ("observation_id", "timestamp", "sensor_id", "source")} | {"ref": o["data"]["ref"]})
        return o

    def _store(self, o, jpg):
        with self.obs_lock:
            self.obs[o["observation_id"]] = (o, jpg)
            while len(self.obs) > self.cfg.max_observations: self.obs.popitem(last=False)
        if self.cfg.persist_dir:
            import json
            base = os.path.join(self.cfg.persist_dir, o["observation_id"])
            with open(base + ".jpg", "wb") as f: f.write(jpg)
            with open(base + ".json", "w") as f: json.dump(o, f)

    def get_obs(self, oid):
        with self.obs_lock:
            if oid in self.obs: return self.obs[oid]
        if self.cfg.persist_dir and oid.replace("_", "").isalnum():
            import json
            base = os.path.join(self.cfg.persist_dir, oid)
            if os.path.exists(base + ".json"):
                return json.load(open(base + ".json")), open(base + ".jpg", "rb").read()
        raise ApiError(404, "NOT_FOUND", f"no observation {oid} (expired or never existed)")

    def _prune(self):
        if not self.cfg.persist_dir: return
        cutoff = time.time() - self.cfg.retention_hours * 3600
        for n in os.listdir(self.cfg.persist_dir):
            p = os.path.join(self.cfg.persist_dir, n)
            if os.path.getmtime(p) < cutoff: os.remove(p)

    # ------------------------------------------------------------------ monitor: connection, camera, state events
    def _monitor(self):
        was = {"link": None, "cam": None, "cal": None, "calOn": None}
        last_state, last_prune = (None, 0.0), 0.0
        while not self._closing:
            time.sleep(0.1)
            now, st = time.time(), self._st()
            if "pan" in st: self.hist.append((self.link.status_t, st["pan"], st["tilt"], st["tp"], st["tt"]))
            link, cam = self.status_fresh(), self.camera_status()["ready"]
            if link != was["link"]:
                if was["link"] is not None or not link:
                    self.bus.publish("device.connected" if link else "device.fault", {"code": None if link else "DEVICE_DISCONNECTED", "host": self.link.host},
                                     "info" if link else "error")
                was["link"] = link
            if cam != was["cam"]:
                if was["cam"] is not None or not cam:
                    self.bus.publish("camera.available" if cam else "camera.unavailable", {"sensor_id": self.sensor_id, "error": self.link.stream_err or None},
                                     "info" if cam else "error")
                was["cam"] = cam
            for k in ("cal", "calOn"):
                if st.get(k) != was[k] and k in st:
                    if was[k] is not None: self.bus.publish("device.calibration_changed", {"calibrated": st.get("cal"), "in_progress": st.get("calOn")})
                    was[k] = st.get(k)
            p = self.pose(st)
            if p:   # state: on change, at most 2 Hz (pose is the commanded open-loop value)
                key = (round(p["pan_deg"]), round(p["tilt_deg"]), p["moving"], st.get("mode"))
                if key != last_state[0] and now - last_state[1] >= 0.5:
                    self.bus.publish("device.state", {"pose": p, "firmware_mode": st.get("mode")}); last_state = (key, now)
            if now - last_prune > 60: self._prune(); last_prune = now

    def close(self):
        """Shutdown: stop our own operation (freeze), leave everything else to the firmware."""
        if self.active and self.active.active: self.stop("shutdown")
        self._closing = True
        self.link.close()

    # ------------------------------------------------------------------ perception (external detectors feed in here)
    def ingest(self, d):
        from .perception import normalise
        ev_type, data = normalise(d)
        if ev_type in REJECTED_EVENT_TYPES:
            raise ApiError(422, "UNSUPPORTED_EVENT", f"{ev_type} needs a doorway entry/exit classifier, which this device does not have")
        if ev_type in ("perception.person_visible", "perception.person_not_visible"):
            self.presence = {"state": "visible" if ev_type == "perception.person_visible" else "not_visible",
                             "since": iso(time.time()), "source": data.get("detector")}
            if ev_type == "perception.person_visible":
                try: data["observation_id"] = self.observe(source={"event": ev_type})["observation_id"]
                except ApiError as e: data["observation_error"] = e.code
        elif ev_type == "perception.presence_unknown":
            self.presence = {"state": "unknown", "since": iso(time.time()), "source": data.get("detector")}
        return self.bus.publish(ev_type, data)

    def descriptor(self):
        return {"schema": "device_descriptor/v0", "contract": CONTRACT, "device_id": self.cfg.device_id, "device_type": "pan_tilt_camera",
                "model": "ANVYXO ESP32-S3 pan/tilt gimbal", "api_version": "v1",
                "axes": [{"name": "pan", "unit": "deg", "positive": "right, seen from behind", "actuator": "MG996R hobby servo", "feedback": "none (open loop)"},
                         {"name": "tilt", "unit": "deg", "positive": "up", "actuator": "MG90S hobby servo", "feedback": "none (open loop)"}],
                "sensors": [{"sensor_id": self.sensor_id, "sensor_type": "camera.rgb", "frame_id": f"{self.cfg.device_id}/camera_optical",
                             "mounted_on": self.frame_id, "calibration_ref": None}],
                "coordinate_frames": {self.frame_id: "origin at pan/tilt axes; zero = calibrated straight ahead + level; not linked to any map/base_link"},
                "capabilities": ["health", "status", "move.absolute", "move.relative", "stop", "home", "camera.status", "camera.snapshot",
                                 "camera.stream.mjpeg", "camera.config", "survey.bounded", "events.sse", "perception.ingest.person_visibility"],
                "unsupported": ["closed_loop_position_feedback", "acceleration_limiting", "entry_exit_classification", "scene_interpretation",
                                "mapping", "global_pose", "lens_intrinsics", "laser_control_via_api"]}
