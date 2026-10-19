"""Perception extension point. Detectors run OUTSIDE this service (Tracker/server.py today, a Jetson later) and
POST to /v1/perception/events; this module checks their input and turns it into canonical shapes.

Implemented (backed by real detection logic):
    perception.detections           YOLO11n boxes + bearings from the tracker -> observation kind "detections"
                                    (GET /v1/observations/latest?kind=detections)
    perception.person_visible       a person detector saw >= 1 person in the last ~1 s   -> room event presence_started
    perception.person_not_visible   none for >= 3 s (Tracker presence_loop)              -> presence_ended (NOT an exit)
    perception.presence_unknown     detector stopped / not running                      -> observation_gap

Reserved, rejected until a classifier exists (a visible person is NOT an entry or exit):
    perception.entry_candidate / entry_confirmed / exit_candidate / exit_confirmed
    Needs: a doorway inside a fixed, calibrated view (gimbal parked at a known pose), a door zone polygon,
    per-track crossing direction over time, and ideally a second cue (door sensor / RC car LiDAR).
    Plug-in shape: class EntryExitClassifier: def update(tracks, pose, t) -> list[(event_type, data)]

Not here on purpose: scene interpretation, mapping, identity ("owner") recognition, long-term memory.
"""
import math, time

# Tracker/server.py --webhook sends {"event": "person_entered"|"person_left", "t", "count", "detector"}.
# Its names predate the contract: they mean visible / not visible, not room entry / exit.
_LEGACY = {"person_entered": "perception.person_visible", "person_left": "perception.person_not_visible"}
_MODELS = {"person": "yolo11n", "face": "mediapipe-blazeface-full-range"}
ACCEPTED = {"perception.person_visible", "perception.person_not_visible", "perception.presence_unknown", "perception.detections"}
MAX_DETECTIONS = 50


def normalise(d):
    """(event_type, data) from a canonical {"type", "data"} or a legacy tracker webhook body."""
    from .core import ApiError, REJECTED_EVENT_TYPES
    if not isinstance(d, dict): raise ApiError(422, "INVALID_ARGUMENT", "body must be a JSON object", "not_an_object")
    if "event" in d:
        t = _LEGACY.get(d["event"])
        data = {"count": d.get("count"), "detector": d.get("detector"), "detector_time": d.get("t")}
    else:
        t, data = d.get("type"), dict(d.get("data") or {})
    if t not in ACCEPTED | REJECTED_EVENT_TYPES:
        raise ApiError(422, "UNSUPPORTED_EVENT", f"unknown perception event {t or d.get('event')!r}", accepted=sorted(ACCEPTED))
    data.setdefault("model", _MODELS.get(data.get("detector")))
    data["identity"] = None   # generic person detection never identifies who it is
    return t, data


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def detections_observation(dev, data):
    """Tracker detections -> observation.schema.json kind "detections" (bearings in the pan/tilt frame, no range)."""
    from .core import ApiError, SCHEMA_VERSION, POSE_SOURCE, new_id
    bad = lambda m: ApiError(422, "INVALID_ARGUMENT", m, "bad_detections")
    dets, pose, t = data.get("detections"), data.get("sensor_pose") or {}, data.get("observed_at")
    if not isinstance(dets, list) or len(dets) > MAX_DETECTIONS: raise bad(f"detections must be a list of <= {MAX_DETECTIONS}")
    if not _fin(t) or abs(time.time() - t) > 60: raise bad("observed_at must be epoch seconds within 60 s of this host's clock")
    if not (_fin(pose.get("pan_deg")) and _fin(pose.get("tilt_deg"))): raise bad("sensor_pose {pan_deg, tilt_deg} required")
    out = []
    for x in dets:
        if not isinstance(x, dict) or not isinstance(x.get("label"), str) or not _fin(x.get("confidence")) or not 0 <= x["confidence"] <= 1:
            raise bad("each detection needs label and confidence 0..1")
        y = {"label": x["label"], "confidence": float(x["confidence"])}
        if isinstance(x.get("track_id"), (int, str)) and not isinstance(x.get("track_id"), bool): y["track_id"] = x["track_id"]
        b = x.get("box_norm")
        if b is not None:
            if not (isinstance(b, list) and len(b) == 4 and all(_fin(v) for v in b)): raise bad("box_norm must be [x0,y0,x1,y1]")
            y["box_norm"] = [min(1.0, max(0.0, float(v))) for v in b]   # YOLO boxes can poke a pixel past the edge
        br = x.get("bearing")
        if br is not None:
            if not (isinstance(br, dict) and _fin(br.get("pan_deg")) and _fin(br.get("tilt_deg"))): raise bad("bearing needs pan_deg, tilt_deg")
            y["bearing"] = {"pan_deg": float(br["pan_deg"]), "tilt_deg": float(br["tilt_deg"])}
        out.append(y)
    lat = data.get("latency_s")
    return {"schema_version": SCHEMA_VERSION, "observation_id": new_id("obs"), "device_id": dev.cfg.device_id, "sensor_id": dev.sensor_id,
            "sensor_type": "camera", "kind": "detections", "observed_at": float(t), "frame_id": dev.frame_id,
            "sensor_pose": {"frame_id": dev.frame_id, "representation": "pan_tilt_relative", "pan_deg": float(pose["pan_deg"]),
                            "tilt_deg": float(pose["tilt_deg"]), "source": POSE_SOURCE, "physically_verified": False},
            "validity": {"valid": True, "stale": False, "reasons": []}, "inferred": False, "model": data.get("model") or "unknown",
            "detections": out, "count": len(out),
            "timing": {"timestamp_source": "host_receive_minus_latency", "estimated": True, **({"latency_s": float(lat)} if _fin(lat) else {})},
            "quality": {"hfov_deg": dev.cfg.hfov_deg or None, "note": "bearing = direction only, no range"}}
