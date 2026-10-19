"""Levels 2-4: unit, mock-device and contract tests. No hardware.   cd host && python -m pytest -q"""
import json, os, sys, time

import jsonschema, pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gimbal_service.api import create_app
from gimbal_service.core import ApiError, Config, Device, jpeg_size, num
from gimbal_service.links import MOCK_JPEG, MockLink

SCHEMA = json.load(open(os.path.join(os.path.dirname(__file__), "..", "contracts", "device_v0.schema.json")))


def valid(obj, name):
    jsonschema.validate(obj, {**SCHEMA, "$ref": f"#/$defs/{name}"})


@pytest.fixture
def env():
    cfg = Config(token="t0k", settle_s=0.05, camera_latency_s=0.05, survey_settle_s=0.1, max_speed_dps=360, default_speed_dps=200)
    link = MockLink(calibrated=True, fps=30)
    dev = Device(cfg, link)
    c = create_app(dev).test_client()
    h = {"Authorization": "Bearer t0k"}
    deadline = time.time() + 3
    while not dev.camera_status()["ready"] and time.time() < deadline: time.sleep(0.02)
    yield dev, link, c, h
    dev.close()


def wait_op(c, h, oid, timeout=8):
    end = time.time() + timeout
    while time.time() < end:
        o = c.get(f"/v1/operations/{oid}", headers=h).get_json()
        if o["state"] not in ("accepted", "running"): return o
        time.sleep(0.03)
    raise AssertionError(f"operation {oid} still {o['state']}")


# ---------------------------------------------------------------- level 2: unit
def test_validation_helpers():
    assert num({"a": 3}, "a") == 3.0
    for bad in ("x", True, None, float("nan"), float("inf")):
        with pytest.raises(ApiError) as e: num({"a": bad}, "a")
        assert e.value.http == 422
    with pytest.raises(ApiError) as e: num({"a": 500}, "a", lo=10, hi=120)
    assert e.value.code == "OUT_OF_RANGE"
    assert jpeg_size(MOCK_JPEG) == (64, 48) and jpeg_size(b"junk") == (None, None)


def test_limits_are_soft_limits_and_servo_reach(env):
    dev, link, *_ = env
    link._s.update(lp=[-90, 90], rp=[-17, 163])   # the real 2026-10-19 lopsided mount
    time.sleep(0.2)   # next firmware status (150 ms)
    assert dev.limits()["pan"] == [-17, 90]
    link._s.update(lp=[-17, 163]); time.sleep(0.2)   # next firmware status (150 ms)      # stale firmware soft limit: host CAD cap still holds
    assert dev.limits()["pan"] == [-17, 90]
    assert any(f["code"] == "LOPSIDED_MOUNT" for f in dev.faults())


# ---------------------------------------------------------------- level 3/4: mock device + contract
def test_health_status_descriptor_match_contract(env):
    dev, link, c, h = env
    r = c.get("/v1/health")                                   # no auth needed
    assert r.status_code == 200 and r.get_json()["ready"]; valid(r.get_json(), "health")
    valid(c.get("/v1/status", headers=h).get_json(), "device_status")
    valid(c.get("/v1/device", headers=h).get_json(), "device_descriptor")
    valid(c.get("/v1/camera/status", headers=h).get_json(), "camera_status")


def test_auth(env):
    _, _, c, _ = env
    r = c.get("/v1/status")
    assert r.status_code == 401; valid(r.get_json(), "error")
    assert c.get("/v1/status", headers={"Authorization": "Bearer nope"}).status_code == 401
    assert c.get("/v1/status?token=t0k").status_code == 200


def test_move_absolute_ack_then_completion(env):
    dev, link, c, h = env
    r = c.post("/v1/motion/move", json={"pan_deg": 20, "tilt_deg": -10, "request_id": "req_abc"}, headers=h)
    assert r.status_code == 202 and r.headers["X-Request-ID"] == "req_abc"
    op = wait_op(c, h, r.get_json()["operation_id"]); valid(op, "operation")
    assert op["state"] == "succeeded" and op["device_ack_at"] and op["request_id"] == "req_abc"
    assert op["result"]["pose"]["physically_verified"] is False
    assert abs(link.status["pan"] - 20) < 0.2 and "PT:20.00,-10.00" in link.sent


def test_move_relative(env):
    dev, link, c, h = env
    wait_op(c, h, c.post("/v1/motion/move", json={"pan_deg": 10, "tilt_deg": 0}, headers=h).get_json()["operation_id"])
    op = wait_op(c, h, c.post("/v1/motion/move", json={"mode": "relative", "pan_deg": 5}, headers=h).get_json()["operation_id"])
    assert op["state"] == "succeeded" and op["params"]["target_pan_deg"] == 15


@pytest.mark.parametrize("body,code", [({"pan_deg": 91, "tilt_deg": 0}, "OUT_OF_LIMITS"), ({"pan_deg": 0, "tilt_deg": -46}, "OUT_OF_LIMITS"),
                                       ({"pan_deg": "10", "tilt_deg": 0}, "INVALID_ARGUMENT"), ({"pan_deg": 10}, "INVALID_ARGUMENT"),
                                       ({"pan_deg": 0, "tilt_deg": 0, "speed_dps": 1000}, "OUT_OF_RANGE"),
                                       ({"pan_deg": 0, "tilt_deg": 0, "mode": "fly"}, "INVALID_ARGUMENT"),
                                       ({"pan_deg": 0, "tilt_deg": 0, "spead_dps": 5}, "UNKNOWN_FIELD")])
def test_invalid_moves_rejected_before_anything_is_sent(env, body, code):
    dev, link, c, h = env
    n = len(link.sent)
    r = c.post("/v1/motion/move", json=body, headers=h)
    assert r.status_code == 422 and r.get_json()["error"]["code"] == code
    valid(r.get_json(), "error")
    assert not [s for s in link.sent[n:] if s.startswith("PT:")]


def test_bad_json(env):
    _, _, c, h = env
    r = c.post("/v1/motion/move", data="{nope", headers={**h, "Content-Type": "application/json"})
    assert r.status_code == 400 and r.get_json()["error"]["code"] == "BAD_JSON"


def test_overlap_busy_and_preempt(env):
    dev, link, c, h = env
    a = c.post("/v1/motion/move", json={"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20}, headers=h).get_json()
    r = c.post("/v1/motion/move", json={"pan_deg": -80, "tilt_deg": 0}, headers=h)
    assert r.status_code == 409 and r.get_json()["error"]["code"] == "BUSY"
    b = c.post("/v1/motion/move", json={"pan_deg": 5, "tilt_deg": 0, "preempt": True}, headers=h).get_json()
    assert wait_op(c, h, a["operation_id"])["state"] == "preempted"
    assert wait_op(c, h, b["operation_id"])["state"] == "succeeded"


def test_stop_cancels_and_freezes(env):
    dev, link, c, h = env
    a = c.post("/v1/motion/move", json={"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20}, headers=h).get_json()
    time.sleep(0.4)
    r = c.post("/v1/motion/stop", headers=h).get_json()
    assert r["stopped"] and r["confirmed_by_device"] and r["cancelled_operation_id"] == a["operation_id"]
    time.sleep(0.4)
    assert abs(link.status["pan"] - r["pose"]["pan_deg"]) < 0.3        # reported stop pose = where it really holds
    op = wait_op(c, h, a["operation_id"])
    assert op["state"] == "cancelled" and op["error"]["code"] == "CANCELLED"
    p = link.status["pan"]; time.sleep(0.2)
    assert 2 < p < 30 and abs(link.status["pan"] - p) < 0.3   # stayed where it stopped, no jump
    assert "MODE:MANUAL" in link.sent


def test_cancel_endpoint(env):
    dev, link, c, h = env
    a = c.post("/v1/motion/move", json={"pan_deg": -80, "tilt_deg": 0, "speed_dps": 20}, headers=h).get_json()
    time.sleep(0.2)
    assert c.post(f"/v1/operations/{a['operation_id']}/cancel", headers=h).get_json()["state"] == "cancelled"
    assert c.post("/v1/operations/op_missing/cancel", headers=h).status_code == 404


@pytest.mark.parametrize("fault,code", [("stall", "MOTION_TIMEOUT"), ("deaf", "COMMAND_NOT_ACKNOWLEDGED")])
def test_faults_fail_the_operation(env, fault, code):
    dev, link, c, h = env
    setattr(link, fault, True)
    op = wait_op(c, h, c.post("/v1/motion/move", json={"pan_deg": 3, "tilt_deg": 0}, headers=h).get_json()["operation_id"])
    assert op["state"] == "failed" and op["error"]["code"] == code


def test_link_lost_mid_move(env):
    dev, link, c, h = env
    a = c.post("/v1/motion/move", json={"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20}, headers=h).get_json()
    time.sleep(0.3); link.online = False
    op = wait_op(c, h, a["operation_id"], timeout=5)
    assert op["state"] == "failed" and op["error"]["code"] == "LINK_LOST"
    r = c.post("/v1/motion/move", json={"pan_deg": 0, "tilt_deg": 0}, headers=h)
    assert r.status_code == 503 and r.get_json()["error"]["code"] == "DEVICE_UNAVAILABLE"
    assert c.post("/v1/motion/stop", headers=h).status_code == 503
    assert not c.get("/v1/health").get_json()["ready"]


def test_external_client_takes_over(env):
    dev, link, c, h = env
    a = c.post("/v1/motion/move", json={"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20}, headers=h).get_json()
    time.sleep(0.3); link.send("PT:-30,0")                     # e.g. the tracker or the browser UI
    op = wait_op(c, h, a["operation_id"])
    assert op["state"] == "failed" and op["error"]["code"] == "PREEMPTED_EXTERNALLY"


@pytest.mark.parametrize("st,code", [({"cal": False}, "NOT_CALIBRATED"), ({"calOn": True}, "CALIBRATION_IN_PROGRESS"), ({"panEn": False}, "MOTORS_DISABLED")])
def test_motion_gated_on_calibration_and_motors(env, st, code):
    dev, link, c, h = env
    link._s.update(st); time.sleep(0.2)   # next firmware status (150 ms)
    r = c.post("/v1/motion/home", json={}, headers=h)
    assert r.status_code == 409 and r.get_json()["error"]["code"] == code


def test_uncalibrated_allowed_by_config(env):
    dev, link, c, h = env
    link._s["cal"] = False; dev.cfg.allow_uncalibrated = True; time.sleep(0.2)   # next firmware status (150 ms)
    assert wait_op(c, h, c.post("/v1/motion/home", json={}, headers=h).get_json()["operation_id"])["state"] == "succeeded"


def test_fresh_snapshot_and_observation(env):
    dev, link, c, h = env
    t0 = time.time()
    r = c.get("/v1/camera/snapshot", headers=h)
    assert r.status_code == 200 and r.data[:2] == b"\xff\xd8" and r.headers["X-Observation-ID"].startswith("obs_")
    r = c.post("/v1/observations", json={}, headers=h)
    o = r.get_json(); assert r.status_code == 201; valid(o, "observation")
    assert o["pose"]["representation"] == "pan_tilt_relative" and o["pose"]["global_pose"] is None
    assert o["data"]["width"] == 64 and o["validity"]["valid"] and not o["validity"]["stale"]
    from datetime import datetime
    assert datetime.fromisoformat(o["received_at"].replace("Z", "+00:00")).timestamp() >= t0 + dev.cfg.camera_latency_s - 0.01
    assert c.get(o["data"]["ref"], headers=h).data == MOCK_JPEG
    assert c.get("/v1/observations/obs_nothere", headers=h).status_code == 404


def test_camera_down_and_stale_frames_are_errors(env):
    dev, link, c, h = env
    link.camera = False
    r = c.get("/v1/camera/snapshot", headers=h)
    assert r.status_code == 503 and r.get_json()["error"]["code"] == "CAMERA_UNAVAILABLE"
    link.camera = True; link._fps = 0.5; time.sleep(1.2)          # frames now arrive every 2 s
    r = c.get("/v1/camera/snapshot?fresh=0&max_age_ms=200", headers=h)
    assert r.status_code in (200, 503)
    if r.status_code == 503: assert r.get_json()["error"]["code"] == "FRAME_STALE"


def test_survey_grid_restores_pose(env):
    dev, link, c, h = env
    r = c.post("/v1/survey", json={"grid": {"pan_from_deg": -20, "pan_to_deg": 20, "pan_step_deg": 20, "tilts_deg": [0, 10]}, "restore": "previous"}, headers=h)
    assert r.status_code == 202
    op = wait_op(c, h, r.get_json()["operation_id"], timeout=15); valid(op, "operation")
    assert op["state"] == "succeeded" and op["result"]["complete"] and len(op["result"]["observation_ids"]) == 6
    assert [s["target_pan_deg"] for s in op["result"]["steps"]] == [-20, 0, 20, 20, 0, -20]   # serpentine
    o = c.get(f"/v1/observations/{op['result']['observation_ids'][2]}", headers=h).get_json()
    assert o["source"]["survey_id"] == op["operation_id"] and o["source"]["step_index"] == 2 and o["pose"]["at_rest"]
    assert abs(o["pose"]["pan_deg"] - 20) < 0.3 and abs(o["pose"]["tilt_deg"]) < 0.3
    assert abs(link.status["pan"]) < 0.3 and abs(link.status["tilt"]) < 0.3   # restored


def test_survey_cancel_keeps_partial_results(env):
    dev, link, c, h = env
    r = c.post("/v1/survey", json={"waypoints": [[-60, 0], [0, 0], [60, 0]], "speed_dps": 60}, headers=h).get_json()
    end = time.time() + 10
    while not c.get(f"/v1/operations/{r['operation_id']}", headers=h).get_json()["result"]["observation_ids"] and time.time() < end: time.sleep(0.2)   # next firmware status (150 ms)
    c.post("/v1/motion/stop", headers=h)
    op = wait_op(c, h, r["operation_id"])
    assert op["state"] == "cancelled" and 1 <= len(op["result"]["observation_ids"]) < 3
    assert "not_reached" in [s["status"] for s in op["result"]["steps"]]


def test_survey_capture_failure_is_partial_failure(env):
    dev, link, c, h = env
    r = c.post("/v1/survey", json={"waypoints": [[5, 0], [10, 0]], "restore": "none"}, headers=h).get_json()
    end = time.time() + 10
    while not c.get(f"/v1/operations/{r['operation_id']}", headers=h).get_json()["result"]["observation_ids"] and time.time() < end: time.sleep(0.02)
    link.camera = False
    op = wait_op(c, h, r["operation_id"])
    assert op["state"] == "failed" and op["error"]["code"] == "SURVEY_INCOMPLETE" and len(op["result"]["observation_ids"]) == 1


@pytest.mark.parametrize("body", [{"waypoints": []}, {"waypoints": [[0, 0]] * 61}, {"waypoints": [[0, 99]]}, {"waypoints": [[0]]},
                                  {"waypoints": [[0, 0]], "restore": "maybe"}, {"waypoints": [[0, 0]], "settle_s": 0.0},
                                  {"waypoints": [[0, 0]], "tilts": 0}, {"grid": {"pan_from_deg": 0, "pan_to_deg": 10, "tilt": [0]}}])
def test_survey_validation(env, body):
    _, link, c, h = env
    r = c.post("/v1/survey", json=body, headers=h)
    assert r.status_code == 422, r.get_json()


def test_events_follow_contract(env):
    dev, link, c, h = env
    wait_op(c, h, c.post("/v1/motion/move", json={"pan_deg": 4, "tilt_deg": 0}, headers=h).get_json()["operation_id"])
    c.post("/v1/observations", json={}, headers=h)
    evs = c.get("/v1/events?once=1", headers=h).get_json()["events"]
    for e in evs: valid(e, "event")
    types = [e["type"] for e in evs]
    assert {"operation.accepted", "operation.started", "operation.succeeded", "observation.created"} <= set(types)
    assert [e["seq"] for e in evs] == sorted(e["seq"] for e in evs)
    assert c.get(f"/v1/events?once=1&after={evs[-1]['seq']}", headers=h).get_json()["events"] == []


def test_perception_ingest(env):
    dev, link, c, h = env
    r = c.post("/v1/perception/events?token=t0k", json={"event": "person_entered", "t": 1.0, "count": 1, "detector": "person"})   # tracker webhook
    e = r.get_json(); assert r.status_code == 201; valid(e, "event")
    assert e["type"] == "perception.person_visible" and e["data"]["model"] == "ultralytics-yolo11n" and e["data"]["identity"] is None
    assert e["data"]["observation_id"].startswith("obs_")
    assert c.get("/v1/status", headers=h).get_json()["presence"]["state"] == "visible"
    c.post("/v1/perception/events", json={"event": "person_left"}, headers=h)
    assert c.get("/v1/status", headers=h).get_json()["presence"]["state"] == "not_visible"
    for bad in ({"type": "perception.entry_confirmed"}, {"type": "perception.owner_home"}, {"event": "dance"}):
        r = c.post("/v1/perception/events", json=bad, headers=h)
        assert r.status_code == 422 and r.get_json()["error"]["code"] == "UNSUPPORTED_EVENT"


def test_camera_config(env):
    dev, link, c, h = env
    assert c.post("/v1/camera/config", json={"resolution": "320x240", "fps_cap": 20}, headers=h).status_code == 202
    assert link.cam["res"] == 0 and link.cam["fps"] == 20
    assert c.post("/v1/camera/config", json={"resolution": "4k"}, headers=h).status_code == 422
    assert c.post("/v1/camera/config", json={"fps_cap": 99}, headers=h).status_code == 422


def test_restream_does_not_drop_burst_frames(env):
    dev, link, c, h = env
    link.camera = False; time.sleep(0.1)                        # mock camera quiet: only our burst
    seq0 = link.frame()[2]
    for _ in range(3): link.push(MOCK_JPEG)                     # burst: 3 frames before any reader wakes
    assert [f[2] for f in link.frames_after(seq0)][:3] == [seq0 + 1, seq0 + 2, seq0 + 3]


def test_persistence_and_retention(env, tmp_path):
    dev, link, c, h = env
    dev.cfg.persist_dir, dev.cfg.max_observations = str(tmp_path), 1
    a = c.post("/v1/observations", json={}, headers=h).get_json()["observation_id"]
    c.post("/v1/observations", json={}, headers=h)                     # evicts `a` from memory
    assert c.get(f"/v1/observations/{a}/image", headers=h).data == MOCK_JPEG   # served from disk
    old = time.time() - 48 * 3600
    for f in tmp_path.iterdir(): os.utime(f, (old, old))
    dev._prune()
    assert not list(tmp_path.iterdir()) and c.get(f"/v1/observations/{a}", headers=h).status_code == 404


def test_unknown_endpoint_is_structured(env):
    _, _, c, h = env
    r = c.get("/v1/nope", headers=h)
    assert r.status_code == 404 and r.get_json()["error"]["code"] == "NOT_FOUND"
