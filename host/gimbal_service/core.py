"""Device logic: validation, operations (move / home / motors / survey), stop, observations, events, health.

Shapes follow the canonical Robotics Device API v1 (vendored in contracts/v1, see contracts/v1/SOURCE.md).

Everything physical goes through the firmware, which keeps its own soft limits, servo-reach clamp, speed cap and
laser timer. This layer adds what the firmware can't know: request validation (reject, never silently clamp),
one motion operation at a time, acceptance vs completion, timeouts, cancellation and timestamped observations.

The servos are open-loop hobby servos: "succeeded" means the firmware's commanded trajectory finished and the
settle time passed (outcome "commanded_reached"), never that the head was measured there.
"""
import hashlib, math, os, threading, time, uuid
from collections import OrderedDict, deque
from dataclasses import dataclass, field

SCHEMA_VERSION = "1.0"
SOFTWARE_VERSION = "gimbal_service 1.0"
RESOLUTIONS = ["320x240", "400x296", "480x320", "640x480", "800x600", "1024x768", "1280x720", "1280x1024", "1600x1200"]
POSE_SOURCE = "firmware_commanded_open_loop"
EXPOSURE_MARGIN_S = 0.15   # latency jitter: a frame counts as "at rest" only if the head was still this much longer
IDEMPOTENCY_S = 600        # a repeated request_id returns the original operation for 10 min
EVENT_RETENTION_S = 24 * 3600
DETECTIONS_FRESH_S = 10.0  # advertise perception.detections only while the tracker fed us this recently
REJECTED_EVENT_TYPES = {"perception.entry_candidate", "perception.entry_confirmed", "perception.exit_candidate",
                        "perception.exit_confirmed"}   # reserved: needs doorway coverage + a classifier (perception.py)
RETRYABLE = {"BUSY", "OFFLINE", "LINK_LOST", "CAMERA_UNAVAILABLE", "FRAME_STALE", "FRAME_TIMEOUT", "TIMEOUT", "MOTION_TIMEOUT",
             "COMMAND_NOT_ACKNOWLEDGED", "CALIBRATION_IN_PROGRESS", "UNAVAILABLE"}


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
    event_webhook: str = ""                   # optional: POST every room_event JSON here (e.g. the assistant's /gimbal/event)

    @classmethod
    def from_env(cls):
        c, d = cls(), cls.__dataclass_fields__
        for k, f in d.items():
            setattr(c, k, _env("GIMBAL_" + k.upper(), getattr(c, k), f.type if f.type in (bool, int, float) else str))
        return c


class ApiError(Exception):
    """Canonical error. reason = the specific cause when the canonical code is generic (e.g. unknown_field)."""
    def __init__(self, http, code, message, reason=None, **details):
        super().__init__(message)
        self.http, self.code, self.message = http, code, message
        self.details = {**({"reason": reason} if reason else {}), **details}

    def err(self): return err(self.code, self.message, **self.details)


def err(code, message, **details):
    return {"code": code, "message": message, "retryable": code in RETRYABLE, "details": details}


def num(d, key, default=None, lo=None, hi=None):
    v = d.get(key, default)
    if v is None: raise ApiError(422, "INVALID_ARGUMENT", f"'{key}' is required", "missing_field", field=key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ApiError(422, "INVALID_ARGUMENT", f"'{key}' must be a finite number", "not_a_number", field=key)
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ApiError(422, "OUT_OF_RANGE", f"'{key}'={v} outside {lo}..{hi}", "argument_range", field=key, min=lo, max=hi)
    return float(v)


def flag(d, key, default=False):
    v = d.get(key, default)
    if not isinstance(v, bool): raise ApiError(422, "INVALID_ARGUMENT", f"'{key}' must be true/false", "not_a_boolean", field=key)
    return v


def only(d, *keys):
    """Reject unknown fields: a typo must fail loudly, not run with defaults."""
    if not isinstance(d, dict): raise ApiError(422, "INVALID_ARGUMENT", "args must be a JSON object", "not_an_object")
    extra = sorted(set(d) - set(keys) - {"request_id"})
    if extra: raise ApiError(422, "INVALID_ARGUMENT", f"unknown field(s) {extra}", "unknown_field", fields=extra, allowed=sorted(keys))
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
    """Internal, fine-grained events (operation.*, device.state ...): the optional SSE stream. Room events are separate."""
    def __init__(self, device_id, keep=1000):
        self.device_id, self.seq, self.ring, self.cond = device_id, 0, deque(maxlen=keep), threading.Condition()

    def publish(self, type_, data, severity="info"):
        with self.cond:
            self.seq += 1
            ev = {"event_id": new_id("evt"), "seq": self.seq, "type": type_, "t": time.time(), "device_id": self.device_id,
                  "severity": severity, "data": data}
            self.ring.append(ev)
            self.cond.notify_all()
        return ev

    def since(self, seq, timeout=0.0):
        with self.cond:
            if timeout: self.cond.wait_for(lambda: self.seq > seq, timeout=timeout)
            return [e for e in self.ring if e["seq"] > seq]


# ------------------------------------------------------------------ operation args (advertised in /v1/info, enforced here)
_NUM = {"type": "number"}
OPERATION_ARGS = {
    "gimbal.move": {"type": "object", "additionalProperties": False, "required": ["pan_deg", "tilt_deg"],
                    "properties": {"mode": {"enum": ["absolute", "relative"], "default": "absolute"}, "pan_deg": _NUM, "tilt_deg": _NUM,
                                   "speed_dps": {"type": "number", "minimum": 10}, "preempt": {"type": "boolean"}}},
    "gimbal.home": {"type": "object", "additionalProperties": False,
                    "properties": {"speed_dps": {"type": "number", "minimum": 10}, "preempt": {"type": "boolean"}}},
    "gimbal.stop": {"type": "object", "additionalProperties": False, "properties": {}},
    "gimbal.motors": {"type": "object", "additionalProperties": False, "required": ["enabled"], "properties": {"enabled": {"type": "boolean"}}},
    "survey.visual": {"type": "object", "additionalProperties": False, "required": ["pan_from_deg", "pan_to_deg", "step_deg"],
                      "properties": {"pan_from_deg": _NUM, "pan_to_deg": _NUM, "step_deg": {"type": "number", "minimum": 1, "maximum": 180},
                                     "tilt_deg": _NUM, "tilts_deg": {"type": "array", "items": _NUM, "minItems": 1},
                                     "restore": {"enum": ["previous", "home", "none"], "default": "previous"},
                                     "settle_s": {"type": "number"}, "speed_dps": {"type": "number", "minimum": 10}, "preempt": {"type": "boolean"}}},
}


@dataclass
class Operation:
    operation: str                   # canonical name, e.g. "gimbal.move"
    params: dict
    request_id: str
    device_id: str
    operation_id: str = field(default_factory=lambda: new_id("op"))
    state: str = "accepted"          # accepted -> running -> succeeded | failed | cancelled | preempted | timed_out
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)
    device_ack_at: float = None      # firmware echoed the new target (command accepted by the DEVICE)
    commanded: bool = False          # a motion command left this host (so "nothing happened" can no longer be claimed)
    deadline: float = None           # absolute host time from deadline_s
    progress: float = 0.0
    result: dict = field(default_factory=dict)
    error: dict = None
    observation_ids: list = field(default_factory=list)
    cancel: threading.Event = field(default_factory=threading.Event)
    cancel_reason: str = ""

    @property
    def active(self): return self.state in ("accepted", "running")

    def set(self, **kw):
        for k, v in kw.items(): setattr(self, k, v)
        self.updated_at = time.time()

    def to_dict(self):
        """operation_result.schema.json"""
        outcome = ("commanded_reached" if self.state == "succeeded" else "unconfirmed" if self.commanded or self.device_ack_at else "none")
        d = {"operation_id": self.operation_id, "request_id": self.request_id, "device_id": self.device_id, "operation": self.operation,
             "state": self.state, "outcome": outcome, "progress": round(self.progress, 3), "result": {"params": self.params, **self.result},
             "error": self.error, "observation_ids": list(self.observation_ids), "created_at": self.created_at, "updated_at": self.updated_at}
        if self.device_ack_at: d["device_ack_at"] = self.device_ack_at
        return d


class _OpEnd(Exception):
    def __init__(self, state, code=None, message="", **details):
        super().__init__(message); self.state, self.code, self.message, self.details = state, code, message, details


class Device:
    def __init__(self, cfg, link):
        self.cfg, self.link = cfg, link
        self.bus = EventBus(cfg.device_id)
        self.room, self.room_lock = deque(maxlen=20000), threading.Lock()     # room_event.schema.json, kept 24 h
        self.ops, self.active, self.lock = OrderedDict(), None, threading.Lock()
        self.by_request = {}                                                 # request_id -> (time, operation_id)
        self.obs, self.obs_lock = OrderedDict(), threading.Lock()
        self.detections, self.detections_t = None, 0.0                       # newest tracker detections observation
        self.hist = deque(maxlen=200)                                        # (t, pan, tilt, tp, tt) for pose-at-exposure
        self.presence = {"state": "unknown", "since": None, "source": None}
        self.sensor_id = f"{cfg.device_id}/camera0"
        self.frame_id, self.cam_frame_id = f"{cfg.device_id}/pan_tilt_base", f"{cfg.device_id}/camera_optical"
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
        if not self.link.connected: add("OFFLINE", "error", f"no WebSocket to firmware at {self.link.host}")
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
        """common.schema.json gimbal_pose (+ target/moving extras)."""
        st = st or self._st()
        if "pan" not in st: return None
        moving = abs(st["pan"] - st["tp"]) > 0.15 or abs(st["tilt"] - st["tt"]) > 0.15
        return {"frame_id": self.frame_id, "representation": "pan_tilt_relative", "pan_deg": st["pan"], "tilt_deg": st["tilt"],
                "source": POSE_SOURCE, "physically_verified": False, "target_pan_deg": st["tp"], "target_tilt_deg": st["tt"], "moving": moving}

    def status(self):
        """device_status.schema.json"""
        st, cfg = self._st(), self.cfg
        faults, pose, lim = self.faults(), self.pose(st), self.limits()
        errs = [x for x in faults if x["severity"] == "error"]
        cam_down = any(x["code"] == "CAMERA_UNAVAILABLE" for x in errs)
        act = self.active if self.active and self.active.active else None
        state = ("unknown" if not st else "calibrating" if st.get("calOn") else
                 "disabled" if st.get("panEn") is False or st.get("tiltEn") is False else
                 "surveying" if act and act.operation == "survey.visual" else "moving" if act or (pose and pose["moving"]) else "idle")
        d = {"device_id": cfg.device_id, "observed_at": self.link.status_t or 0.0,
             "connection": "offline" if not self.status_fresh() else "degraded" if cam_down else "online",
             "health": "fault" if errs else "warning" if faults else "ok", "state": state,
             "calibrated": bool(st.get("cal")), "motors_enabled": bool(st.get("panEn") and st.get("tiltEn")),
             "faults": faults, "limits": {"pan_deg": lim["pan"], "tilt_deg": lim["tilt"]},
             "active_operation_ids": [act.operation_id] if act else [],
             "extra": {"ready": not self.ready_reasons(), "not_ready_reasons": self.ready_reasons(), "host": self.link.host,
                       "calibration": {"in_progress": st.get("calOn"), "direction_reversed": {"pan": st.get("invP"), "tilt": st.get("invT")},
                                       "uncalibrated_motion_allowed": cfg.allow_uncalibrated},
                       "limits_detail": {"soft_pan_deg": st.get("lp"), "soft_tilt_deg": st.get("lt"), "servo_reach_pan_deg": st.get("rp"),
                                         "servo_reach_tilt_deg": st.get("rt"), "cad_cap_deg": [cfg.pan_limit_deg, cfg.tilt_limit_deg],
                                         "max_speed_dps": cfg.max_speed_dps, "acceleration_limited": False},
                       "firmware_mode": st.get("mode"), "speed_dps": st.get("speed"), "laser_on": st.get("laser"),
                       "presence": self.presence, "camera": self.camera_status()}}
        if pose: d["pose"] = pose
        return d

    def camera_status(self):
        jpg, t, seq = self.link.frame()
        now, cam = time.time(), self.link.cam or {}
        age = now - t if jpg else None
        fresh = jpg is not None and age < self.cfg.frame_stale_s
        w, h = jpeg_size(jpg) if jpg else (None, None)
        res = cam.get("res")
        return {"sensor_id": self.sensor_id, "sensor_type": "camera", "frame_id": self.cam_frame_id, "model": cam.get("sensor"),
                "ready": bool(self.link.stream_ok and fresh),
                "stream": {"connected": bool(self.link.stream_ok), "error": self.link.stream_err or None, "fps": round(self.link.fps, 1),
                           "path": "/v1/camera/stream", "encoding": "image/jpeg (MJPEG, frames re-served unchanged)"},
                "last_frame_received_at": t if jpg else None, "frame_age_s": round(age, 3) if age is not None else None,
                "stale": not fresh, "frame_seq": seq, "width": w, "height": h,
                "config": {"resolution_index": res, "resolution": RESOLUTIONS[res] if isinstance(res, int) and 0 <= res < len(RESOLUTIONS) else None,
                           "fps_cap": cam.get("fps"), "jpeg_quality": cam.get("q")},
                "timing": {"timestamp_source": "host_receive_minus_latency", "estimated": True, "latency_s": self.cfg.camera_latency_s},
                "intrinsics": {"calibration_ref": None, "hfov_deg": self.cfg.hfov_deg or None, "vfov_deg": self.cfg.vfov_deg or None,
                               "note": "FOV measured by motion test; no lens intrinsics/distortion calibration exists"}}

    # ------------------------------------------------------------------ commands (POST /v1/commands)
    def command(self, body):
        """task_command.schema.json -> (Operation, http_status)."""
        only(body, "schema_version", "operation", "args", "deadline_s", "issued_at", "issued_by")
        sv = body.get("schema_version")
        if not isinstance(sv, str) or sv.split(".")[0] != "1":
            raise ApiError(400, "VERSION_MISMATCH", f"schema_version {sv!r}: this device speaks 1.x")
        rid, name, args = body.get("request_id"), body.get("operation"), body.get("args")
        if not isinstance(rid, str) or not 1 <= len(rid) <= 64:
            raise ApiError(422, "INVALID_ARGUMENT", "request_id (1..64 chars) is required", "missing_field", field="request_id")
        if not isinstance(name, str): raise ApiError(422, "INVALID_ARGUMENT", "operation is required", "missing_field", field="operation")
        if not isinstance(args, dict): raise ApiError(422, "INVALID_ARGUMENT", "args must be an object", "not_an_object", field="args")
        deadline = num(body, "deadline_s", lo=0.001, hi=600) if body.get("deadline_s") is not None else None
        handlers = {"gimbal.move": self.move, "gimbal.home": self.home, "survey.visual": self.survey, "gimbal.motors": self.motors}
        if name not in handlers and name != "gimbal.stop":
            raise ApiError(400, "UNSUPPORTED", f"operation {name!r} not supported", supported=sorted(OPERATION_ARGS))
        prior = self.replay(rid)
        if prior: return prior, 200
        if name == "gimbal.stop":
            only(args)
            op = self.stop(rid)
        else:
            op = handlers[name](args, rid, deadline)
        return op, 202 if op.active else 200

    def replay(self, rid):
        now = time.time()
        for k in [k for k, (t, _) in self.by_request.items() if now - t > IDEMPOTENCY_S]: del self.by_request[k]
        hit = self.by_request.get(rid)
        return self.ops.get(hit[1]) if hit else None

    # ------------------------------------------------------------------ operations
    def _require_motion_ready(self):
        if not self.status_fresh(): raise ApiError(503, "OFFLINE", "gimbal firmware not connected or status stale")
        bad = self.ready_reasons(include_camera=False)   # moving doesn't need the camera
        if bad: raise ApiError(409, bad[0], "gimbal not ready for motion: " + ", ".join(bad), reasons=bad)

    def _check_target(self, pan, tilt):
        lim = self.limits()
        for ax, v in (("pan", pan), ("tilt", tilt)):
            lo, hi = lim[ax]
            if not lo - 1e-6 <= v <= hi + 1e-6:
                raise ApiError(422, "OUT_OF_RANGE", f"{ax} {v:.2f} deg outside allowed {lo:.1f}..{hi:.1f} deg", "out_of_limits", axis=ax, min=lo, max=hi)

    def _speed(self, d):
        return num(d, "speed_dps", self.cfg.default_speed_dps, 10.0, self.cfg.max_speed_dps)

    def move(self, d, request_id, deadline=None):
        only(d, "mode", "pan_deg", "tilt_deg", "speed_dps", "preempt")
        if (prior := self.replay(request_id)): return prior
        mode = d.get("mode", "absolute")
        if mode not in ("absolute", "relative"): raise ApiError(422, "INVALID_ARGUMENT", "mode must be absolute|relative", "bad_enum", field="mode")
        speed, preempt = self._speed(d), flag(d, "preempt")
        dp, dt = num(d, "pan_deg", 0.0 if mode == "relative" else None), num(d, "tilt_deg", 0.0 if mode == "relative" else None)
        self._require_motion_ready()
        st = self._st()
        pan, tilt = (st["tp"] + dp, st["tt"] + dt) if mode == "relative" else (dp, dt)
        self._check_target(pan, tilt)
        params = {"mode": mode, "pan_deg": dp, "tilt_deg": dt, "target_pan_deg": pan, "target_tilt_deg": tilt, "speed_dps": speed}
        return self._start("gimbal.move", params, request_id, preempt, deadline, lambda op: self._run_move(op, pan, tilt, speed))

    def home(self, d, request_id, deadline=None):
        only(d, "speed_dps", "preempt")
        if (prior := self.replay(request_id)): return prior
        speed, preempt = self._speed(d), flag(d, "preempt")
        self._require_motion_ready()
        self._check_target(0.0, 0.0)
        return self._start("gimbal.home", {"target_pan_deg": 0.0, "target_tilt_deg": 0.0, "speed_dps": speed, "home": "calibrated zero"},
                           request_id, preempt, deadline, lambda op: self._run_move(op, 0.0, 0.0, speed))

    def survey(self, d, request_id, deadline=None):
        only(d, "pan_from_deg", "pan_to_deg", "step_deg", "tilt_deg", "tilts_deg", "restore", "settle_s", "speed_dps", "preempt")
        if (prior := self.replay(request_id)): return prior
        a, b = num(d, "pan_from_deg"), num(d, "pan_to_deg")
        step = num(d, "step_deg", None, 1.0, 180.0)
        if "tilts_deg" in d and "tilt_deg" in d: raise ApiError(422, "INVALID_ARGUMENT", "give tilt_deg or tilts_deg, not both", "conflicting_fields")
        tilts = d.get("tilts_deg", [d.get("tilt_deg", 0.0)])
        if not isinstance(tilts, list) or not tilts: raise ApiError(422, "INVALID_ARGUMENT", "tilts_deg must be a non-empty list", "bad_list")
        tilts = [num({"tilt_deg": t}, "tilt_deg") for t in tilts]
        n = int(abs(b - a) // step) + 1
        pans = [a + math.copysign(step, b - a) * i for i in range(n)]
        pts = [[p, t] for k, t in enumerate(tilts) for p in (pans if k % 2 == 0 else pans[::-1])]   # serpentine
        if len(pts) > self.cfg.survey_max_points:
            raise ApiError(422, "OUT_OF_RANGE", f"{len(pts)} views > max {self.cfg.survey_max_points}", "too_many_views", max=self.cfg.survey_max_points)
        restore = d.get("restore", "previous")
        if restore not in ("none", "previous", "home"): raise ApiError(422, "INVALID_ARGUMENT", "restore must be none|previous|home", "bad_enum")
        speed, preempt = self._speed(d), flag(d, "preempt")
        settle = num(d, "settle_s", self.cfg.survey_settle_s, self.cfg.camera_latency_s, 10.0)
        for p, t in pts: self._check_target(p, t)
        if restore == "home": self._check_target(0.0, 0.0)
        self._require_motion_ready()
        if not self.camera_status()["ready"]: raise ApiError(409, "CAMERA_UNAVAILABLE", "camera has no fresh frames")
        params = {"pan_from_deg": a, "pan_to_deg": b, "step_deg": step, "tilts_deg": tilts, "waypoints": pts, "speed_dps": speed,
                  "settle_s": settle, "restore": restore}
        return self._start("survey.visual", params, request_id, preempt, deadline, lambda op: self._run_survey(op, pts, speed, settle, restore))

    def motors(self, d, request_id, deadline=None):
        """Servo signal on/off. Off = limp (also ends our motion op); servos otherwise hold torque, there is no auto-relax."""
        only(d, "enabled")
        if (prior := self.replay(request_id)): return prior
        on = flag(d, "enabled", None) if "enabled" in d else None
        if on is None: raise ApiError(422, "INVALID_ARGUMENT", "'enabled' is required", "missing_field", field="enabled")
        if not self.status_fresh(): raise ApiError(503, "OFFLINE", "gimbal firmware not connected or status stale")
        if not on and self.active and self.active.active: self.stop(request_id + ":motors_off")
        op = self._record(Operation("gimbal.motors", {"enabled": on}, request_id, self.cfg.device_id))
        t_send = time.time()
        op.commanded = True
        if not (self.link.send(f"SRV:P,{int(on)}") and self.link.send(f"SRV:T,{int(on)}")):
            op.set(state="failed", error=err("LINK_LOST", "could not send to firmware")); return op
        end = time.time() + self.cfg.ack_timeout_s
        while time.time() < end:
            st = self._st()
            if self.link.status_t > t_send and st.get("panEn") is on and st.get("tiltEn") is on:
                op.set(state="succeeded", device_ack_at=time.time(), progress=1.0, result={"motors_enabled": on}); break
            time.sleep(0.02)
        else:
            op.set(state="failed", error=err("COMMAND_NOT_ACKNOWLEDGED", "firmware status never showed the motor change"))
        self._emit("operation." + op.state, op.to_dict())
        return op

    def _record(self, op):
        with self.lock:
            self.ops[op.operation_id] = op
            while len(self.ops) > 200: self.ops.popitem(last=False)
            self.by_request[op.request_id] = (time.time(), op.operation_id)
        return op

    def _start(self, name, params, request_id, preempt, deadline, runner):
        with self.lock:
            old = self.active
            if old and old.active:
                if not preempt:
                    raise ApiError(409, "BUSY", f"operation {old.operation_id} ({old.operation}) is still {old.state}; stop it or send preempt=true",
                                   active_operation_id=old.operation_id)
                old.cancel_reason = "preempted"; old.cancel.set()
            op = Operation(name, params, request_id, self.cfg.device_id, deadline=time.time() + deadline if deadline else None)
            self.active = op
        self._record(op)
        if old and old.active:
            for _ in range(100):                # the old runner stops at its next check (<= 20 ms)
                if not old.active: break
                time.sleep(0.02)
        self._emit("operation.accepted", op.to_dict())
        threading.Thread(target=self._run, args=(op, runner), daemon=True, name=f"op-{op.operation_id}").start()
        return op

    def _run(self, op, runner):
        op.set(state="running")
        self._emit("operation.started", {"operation_id": op.operation_id, "operation": op.operation})
        try:
            op.result.update(runner(op) or {})
            op.set(state="succeeded", progress=1.0)
        except _OpEnd as e:
            op.set(state=e.state, error=err(e.code, e.message, **e.details) if e.code else None)
        except Exception as e:   # bug or link exception: never leave an operation "running"
            op.set(state="failed", error=err("INTERNAL", str(e)[:200]))
        sev = "info" if op.state == "succeeded" else "warning" if op.state in ("cancelled", "preempted") else "error"
        self._emit("operation." + op.state, op.to_dict(), sev)

    def _check_cancel(self, op):
        if op.deadline and time.time() > op.deadline and not op.cancel.is_set():
            self.link.send("MODE:MANUAL")
            raise _OpEnd("timed_out", "TIMEOUT", "deadline_s passed; stopped where it was")
        if op.cancel.is_set():
            if op.cancel_reason == "preempted": raise _OpEnd("preempted", "PREEMPTED", "preempted by a newer operation")
            self.link.send("MODE:MANUAL")   # freeze where it is now
            raise _OpEnd("cancelled", "CANCELLED", op.cancel_reason or "cancelled")

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
        op.commanded = True
        if not (self.link.send(f"SPD:{speed:.0f}") and self.link.send(f"PT:{pan:.2f},{tilt:.2f}")):
            raise _OpEnd("failed", "LINK_LOST", "could not send to firmware")
        hit = lambda st: abs(st.get("tp", 1e9) - pan) <= 0.15 and abs(st.get("tt", 1e9) - tilt) <= 0.15
        while not (self.link.status_t > t_send and hit(self._st())):            # acceptance = firmware echoes the target
            self._check_cancel(op)
            if time.time() - t_send > self.cfg.ack_timeout_s:
                raise _OpEnd("failed", "COMMAND_NOT_ACKNOWLEDGED", "firmware status never showed the new target")
            time.sleep(0.02)
        if not op.device_ack_at: op.set(device_ack_at=time.time())
        deadline = t_send + dist / speed * 1.5 + 2.0
        while True:
            self._check_cancel(op)
            st = self._st()
            if not self.status_fresh(): raise _OpEnd("failed", "LINK_LOST", "firmware status lost during motion (firmware finishes the slew on its own)")
            if not hit(st): raise _OpEnd("failed", "PREEMPTED", f"firmware target changed by another client to {st.get('tp')},{st.get('tt')}",
                                         reason="external_client")
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
        views = []
        op.result.update(survey_id=op.operation_id, steps=steps, views=views, complete=False,
                         note="views from several directions; NOT a map (no depth, no localisation)")
        self._emit("survey.started", {"survey_id": op.operation_id, "total_steps": len(pts)})

        def coverage():
            got = [s for s in steps if s["status"] == "captured"]
            if not got: return None
            p = [s["target_pan_deg"] for s in got]; t = [s["target_tilt_deg"] for s in got]
            return {"frame_id": self.frame_id, "pan_deg": [min(p), max(p)], "tilt_deg": [min(t), max(t)], "hfov_deg": self.cfg.hfov_deg or None}
        try:
            for s in steps:
                self._goto(op, s["target_pan_deg"], s["target_tilt_deg"], speed)
                t_rest = time.time()
                self._sleep(op, max(0.0, settle - self.cfg.settle_s))
                try:   # the frame must have been EXPOSED after the head came to rest
                    o = self.observe(after_t=t_rest + self.cfg.camera_latency_s + EXPOSURE_MARGIN_S, timeout=3.0, kind="survey_view",
                                     source={"survey_id": op.operation_id, "operation_id": op.operation_id, "step_index": s["index"]})
                    s["status"], s["observation_id"] = "captured", o["observation_id"]
                    views.append(o); op.observation_ids.append(o["observation_id"])
                except ApiError as e:
                    s["status"], s["error"] = "capture_failed", e.err()
                op.set(progress=(s["index"] + 1) / len(pts), result={**op.result, "coverage": coverage()})
                self._emit("operation.progress", {"operation_id": op.operation_id, "progress": op.progress, "step": s})
            failed = [s for s in steps if s["status"] != "captured"]
            if restore != "none":
                self._goto(op, *(start if restore == "previous" else (0.0, 0.0)), speed)
            op.result["complete"] = not failed
            self._emit("survey.completed", {"survey_id": op.operation_id, "captured": len(views), "total_steps": len(pts),
                                            "complete": not failed, "observation_ids": list(op.observation_ids)})
            if failed:
                raise _OpEnd("failed", failed[0]["error"]["code"], f"{len(failed)} of {len(pts)} captures failed (partial views kept)",
                             reason="survey_incomplete")
            return {"coverage": coverage()}
        except _OpEnd as e:
            for s in steps:
                if s["status"] == "pending": s["status"] = "not_reached"
            if e.details.get("reason") != "survey_incomplete":
                self._emit("survey.completed", {"survey_id": op.operation_id, "captured": len(views), "total_steps": len(pts),
                                                "complete": False, "ended": e.state, "observation_ids": list(op.observation_ids)})
            raise
        finally:
            if prev and self.link.connected: self.link.send(f"SPD:{prev:.0f}")

    def stop(self, request_id):
        """Always accepted. -> a finished gimbal.stop Operation (operation_result shape)."""
        prior = self.replay(request_id) if request_id else None
        if prior: return prior
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
        s = self._record(Operation("gimbal.stop", {}, request_id or new_id("req"), self.cfg.device_id))
        s.set(progress=1.0, result={"cancelled_operation_id": op and op.operation_id, "pose": self.pose(), "confirmed_by_device": confirmed,
                                    "note": "firmware froze its target at the commanded position; confirmed_by_device = a later status showed it holding"})
        if sent: s.set(state="succeeded", device_ack_at=time.time() if confirmed else None)
        else: s.set(state="failed", error=err("OFFLINE", "stop could not be sent: firmware not connected (it holds its last target)"))
        self._emit("motion.stopped", {"request_id": s.request_id, "cancelled_operation_id": op and op.operation_id, "sent": sent},
                   "info" if sent else "error")
        return s

    def cancel(self, op_id, request_id):
        op = self.ops.get(op_id)
        if not op: raise ApiError(404, "NOT_FOUND", f"no operation {op_id}")
        if op.active: self.stop(request_id)
        return op

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
                "physically_verified": False, "at_rest": still}, still

    def observe(self, after_t=None, max_age_s=None, source=None, timeout=3.0, store=True, kind="frame_ref"):
        """observation.schema.json frame observation. after_t: the frame must have arrived after this host time (fresh)."""
        if not self.link.stream_ok: raise ApiError(503, "CAMERA_UNAVAILABLE", self.link.stream_err or "camera stream not connected")
        end = time.time() + timeout
        jpg, t, seq = self.link.frame()
        while after_t is not None and (jpg is None or t < after_t):
            if time.time() > end: raise ApiError(504, "FRAME_TIMEOUT", "no fresh frame arrived in time")
            jpg, t, seq = self.link.wait_frame(seq, max(0.01, end - time.time()))
        if jpg is None: raise ApiError(503, "CAMERA_UNAVAILABLE", "no frame received yet")
        now, max_age = time.time(), self.cfg.frame_stale_s if max_age_s is None else max_age_s
        if now - t > max_age: raise ApiError(503, "FRAME_STALE", f"newest frame is {now - t:.2f} s old (> {max_age} s)", frame_age_s=round(now - t, 3))
        pose, still = self._pose_at(t)
        w, h = jpeg_size(jpg)
        oid, cs = new_id("obs"), self.camera_status()
        quality = {"width": w, "height": h, "bytes": len(jpg), "sha256": hashlib.sha256(jpg).hexdigest(), "camera_model": cs["model"],
                   "resolution": cs["config"]["resolution"], "hfov_deg": self.cfg.hfov_deg or None, "vfov_deg": self.cfg.vfov_deg or None}
        o = {"schema_version": SCHEMA_VERSION, "observation_id": oid, "device_id": self.cfg.device_id, "sensor_id": self.sensor_id,
             "sensor_type": "camera", "kind": kind, "observed_at": t - self.cfg.camera_latency_s, "frame_id": self.cam_frame_id,
             "sensor_pose": pose,
             "validity": {"valid": True, "stale": False, "pose_at_rest": still, "reasons": [] if still else ["gimbal moving during exposure window"]},
             "quality": {k: v for k, v in quality.items() if v is not None}, "inferred": False, "data_ref": f"/v1/observations/{oid}/image",
             "timing": {"timestamp_source": "host_receive_minus_latency", "estimated": True, "latency_s": self.cfg.camera_latency_s,
                        "received_at": t, "created_at": now, "clock": "host (not synchronised with firmware)"},
             "source": source or {}}
        if store: self._store(o, jpg)
        self._emit("observation.created", {"observation_id": oid, "kind": kind, "observed_at": o["observed_at"], "data_ref": o["data_ref"],
                                           "source": o["source"]})
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

    def latest(self, kind=None, sensor_id=None):
        if sensor_id not in (None, self.sensor_id): raise ApiError(404, "NOT_FOUND", f"no sensor {sensor_id}")
        if kind == "detections":
            if not self.detections: raise ApiError(404, "NOT_FOUND", "no detections yet (is the tracker pushing to /v1/perception/events?)")
            d = dict(self.detections)
            age = time.time() - self.detections_t
            d["validity"] = {**d["validity"], "stale": age > 2.0, "reasons": d["validity"]["reasons"] + (["older than 2 s"] if age > 2.0 else [])}
            return d
        with self.obs_lock:
            for o, _ in reversed(self.obs.values()):
                if kind in (None, o["kind"]): return o
        raise ApiError(404, "NOT_FOUND", f"no {kind or ''} observation yet".replace("  ", " "))

    def detections_live(self): return self.detections is not None and time.time() - self.detections_t < DETECTIONS_FRESH_S

    def _prune(self):
        if not self.cfg.persist_dir: return
        cutoff = time.time() - self.cfg.retention_hours * 3600
        for n in os.listdir(self.cfg.persist_dir):
            p = os.path.join(self.cfg.persist_dir, n)
            if os.path.getmtime(p) < cutoff: os.remove(p)

    # ------------------------------------------------------------------ events: internal bus + room_event log
    def _emit(self, type_, data, severity="info"):
        ev = self.bus.publish(type_, data, severity)
        room = self._to_room(type_, data)
        if room:
            with self.room_lock: self.room.append(room)
            if self.cfg.event_webhook:   # fire-and-forget: a slow receiver never blocks the device
                threading.Thread(target=self._post_event, args=(room,), daemon=True).start()
        return ev

    def _to_room(self, type_, data):
        """Internal event -> room_event.schema.json (or None). Only what real logic supports; never entry/exit."""
        oids = [data["observation_id"]] if data.get("observation_id") else list(data.get("observation_ids") or [])
        if type_ == "perception.person_visible": rt, method = "presence_started", "detector_presence"
        elif type_ == "perception.person_not_visible": rt, method = "presence_ended", "detector_presence"
        elif type_ == "perception.presence_unknown": rt, method = "observation_gap", "detector_presence"
        elif type_ == "survey.started": rt, method = "survey_started", "operation_result"
        elif type_ == "survey.completed": rt, method = ("survey_completed" if data.get("complete") else "survey_failed"), "operation_result"
        elif type_ == "device.connected": rt, method = "device_online", "heartbeat"
        elif type_ == "device.fault": rt, method = ("device_offline" if data.get("code") == "OFFLINE" else "device_fault"), "heartbeat"
        elif type_ == "camera.unavailable": rt, method, data = "device_fault", "device_report", {**data, "code": "CAMERA_UNAVAILABLE"}
        elif type_ == "operation.succeeded" and data.get("operation") in ("gimbal.move", "gimbal.home"):
            rt, method, data = "gimbal_moved", "operation_result", {"operation_id": data["operation_id"], "pose": data["result"].get("pose"),
                                                                     "outcome": data["outcome"]}
        else: return None
        ev = {"schema_version": SCHEMA_VERSION, "event_id": new_id("evt"), "type": rt, "t": time.time(), "device_id": self.cfg.device_id,
              "confirmed": False, "inferred": False, "evidence": {"method": method, "observation_ids": oids}, "data": data}
        if method == "detector_presence": ev["sensor_id"] = self.sensor_id
        return ev

    def _post_event(self, ev):
        import json, urllib.request
        try:
            urllib.request.urlopen(urllib.request.Request(self.cfg.event_webhook, json.dumps(ev).encode(),
                                                          {"Content-Type": "application/json"}), timeout=2)
        except Exception:
            pass   # receiver down: the event is still in GET /v1/events (24 h)

    def room_events(self, since=0.0, until=None, types=None, limit=200):
        cutoff = time.time() - EVENT_RETENTION_S
        with self.room_lock:
            while self.room and self.room[0]["t"] < cutoff: self.room.popleft()
            # since is INCLUSIVE: clients round it (the orchestrator sends 3 decimals); dedupe by event_id
            out = [e for e in self.room if e["t"] >= since and (until is None or e["t"] <= until) and (not types or e["type"] in types)]
        return out[:limit]

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
                    self._emit("device.connected" if link else "device.fault", {"code": None if link else "OFFLINE", "host": self.link.host},
                               "info" if link else "error")
                was["link"] = link
            if cam != was["cam"]:
                if was["cam"] is not None or not cam:
                    self._emit("camera.available" if cam else "camera.unavailable", {"sensor_id": self.sensor_id, "error": self.link.stream_err or None},
                               "info" if cam else "error")
                was["cam"] = cam
            for k in ("cal", "calOn"):
                if st.get(k) != was[k] and k in st:
                    if was[k] is not None: self._emit("device.calibration_changed", {"calibrated": st.get("cal"), "in_progress": st.get("calOn")})
                    was[k] = st.get(k)
            p = self.pose(st)
            if p:   # state: on change, at most 2 Hz (pose is the commanded open-loop value)
                key = (round(p["pan_deg"]), round(p["tilt_deg"]), p["moving"], st.get("mode"))
                if key != last_state[0] and now - last_state[1] >= 0.5:
                    self._emit("device.state", {"pose": p, "firmware_mode": st.get("mode")}); last_state = (key, now)
            if now - last_prune > 60: self._prune(); last_prune = now

    def close(self):
        """Shutdown: stop our own operation (freeze), leave everything else to the firmware."""
        if self.active and self.active.active: self.stop("shutdown_" + new_id("req"))
        self._closing = True
        self.link.close()

    # ------------------------------------------------------------------ perception (external detectors feed in here)
    def ingest(self, d):
        from .perception import normalise, detections_observation
        ev_type, data = normalise(d)
        if ev_type in REJECTED_EVENT_TYPES:
            raise ApiError(422, "UNSUPPORTED_EVENT", f"{ev_type} needs a doorway entry/exit classifier, which this device does not have")
        if ev_type == "perception.detections":
            self.detections = detections_observation(self, data)
            self.detections_t = time.time()
            return self.detections
        if ev_type in ("perception.person_visible", "perception.person_not_visible"):
            self.presence = {"state": "visible" if ev_type == "perception.person_visible" else "not_visible",
                             "since": time.time(), "source": data.get("detector")}
            if ev_type == "perception.person_visible":
                try: data["observation_id"] = self.observe(source={"event": ev_type})["observation_id"]
                except ApiError as e: data["observation_error"] = e.code
        elif ev_type == "perception.presence_unknown":
            self.presence = {"state": "unknown", "since": time.time(), "source": data.get("detector")}
        return self._emit(ev_type, data)

    def info(self):
        """device_capabilities.schema.json. Advertises only what works right now."""
        caps = ["gimbal.move", "gimbal.home", "gimbal.stop", "gimbal.motors", "camera.capture", "camera.frame", "survey.visual", "events.query"]
        if self.detections_live(): caps.append("perception.detections")
        dur = {"gimbal.move": 30, "gimbal.home": 30, "gimbal.stop": 3, "gimbal.motors": 3, "survey.visual": 600}
        cap = {"gimbal.move": "gimbal.move", "gimbal.home": "gimbal.home", "gimbal.stop": "gimbal.stop", "gimbal.motors": "gimbal.motors",
               "survey.visual": "survey.visual"}
        fw = (getattr(self.link, "diag", None) or {}).get("fw")
        fps = round(self.link.fps, 1)
        return {"schema_version": SCHEMA_VERSION, "device_id": self.cfg.device_id, "device_type": "gimbal", "name": "ANVYXO ESP32-S3 pan/tilt gimbal",
                "software_version": SOFTWARE_VERSION + (f"; firmware {fw}" if fw else ""), "capabilities": caps,
                "operations": [{"name": n, "capability": cap[n], "args": OPERATION_ARGS[n], "long_running": n not in ("gimbal.stop", "gimbal.motors"),
                                "max_duration_s": dur[n], "moves_hardware": n != "gimbal.stop"} for n in OPERATION_ARGS],
                "sensors": [{"sensor_id": self.sensor_id, "sensor_type": "camera", "frame_id": self.cam_frame_id, **({"nominal_rate_hz": fps} if fps else {})}],
                "event_types": ["presence_started", "presence_ended", "observation_gap", "survey_started", "survey_completed", "survey_failed",
                                "gimbal_moved", "device_online", "device_offline", "device_fault"],
                "event_retention_s": EVENT_RETENTION_S,
                "unsupported": ["closed_loop_position_feedback", "acceleration_limiting", "automatic_motor_relax", "entry_exit_classification",
                                "scene_interpretation", "mapping", "global_pose", "lens_intrinsics", "laser_control", "gimbal.tracking"],
                "axes": [{"name": "pan", "unit": "deg", "positive": "right, seen from behind", "actuator": "MG996R hobby servo", "feedback": "none (open loop)"},
                         {"name": "tilt", "unit": "deg", "positive": "up", "actuator": "MG90S hobby servo", "feedback": "none (open loop)"}],
                "coordinate_frames": {self.frame_id: "origin at pan/tilt axes; zero = calibrated straight ahead + level; not linked to any map/base_link",
                                      self.cam_frame_id: "camera optical frame on the head (offset in gimbal.py CAM_Z/Y_FRONT, not yet published)"}}
