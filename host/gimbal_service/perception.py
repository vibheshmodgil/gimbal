"""Perception extension point. Detectors run OUTSIDE this service (Tracker/server.py today, a Jetson later) and
POST their events to /v1/perception/events; this module turns them into canonical event types.

Implemented (backed by real detection logic):
    perception.person_visible       a person detector saw >= 1 person in the last ~1 s
    perception.person_not_visible   none for >= 3 s   (Tracker/server.py presence_loop)
    perception.presence_unknown     detector stopped / not running

Reserved, rejected until a classifier exists (a visible person is NOT an entry or exit):
    perception.entry_candidate / entry_confirmed / exit_candidate / exit_confirmed
    Needs: a doorway inside a fixed, calibrated view (gimbal parked at a known pose), a door zone polygon,
    per-track crossing direction over time, and ideally a second cue (door sensor / RC car LiDAR).
    Plug-in shape: class EntryExitClassifier: def update(tracks, pose, t) -> list[(event_type, data)]

Not here on purpose: scene interpretation, mapping, identity ("owner") recognition, long-term memory.
Those belong to the central perception / memory services and consume /v1 observations.
"""

# Tracker/server.py --webhook sends {"event": "person_entered"|"person_left", "t", "count", "detector"}.
# Its names predate this contract: they mean visible / not visible, not room entry / exit.
_LEGACY = {"person_entered": "perception.person_visible", "person_left": "perception.person_not_visible"}
_MODELS = {"person": "ultralytics-yolo11n", "face": "mediapipe-blazeface-full-range"}
ACCEPTED = {"perception.person_visible", "perception.person_not_visible", "perception.presence_unknown"}


def normalise(d):
    """(event_type, data) from a canonical {"type", "data"} or a legacy tracker webhook body."""
    from .core import ApiError, REJECTED_EVENT_TYPES
    if not isinstance(d, dict): raise ApiError(422, "INVALID_ARGUMENT", "body must be a JSON object")
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
