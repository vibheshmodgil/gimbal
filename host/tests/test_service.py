"""Levels 2-4: unit, mock-device and contract tests against the canonical Robotics Device API v1. No hardware.

    cd host && python -m pytest -q
Every JSON response is checked against the vendored canonical schemas (contracts/v1). When the assistant repo is on
this machine, its own conformance checker (robotics.devices.conformance) also runs against the service.
"""
import json, os, sys, time, uuid

import jsonschema, pytest
from referencing import Registry, Resource

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from gimbal_service.api import create_app
from gimbal_service.core import ApiError, Config, Device, jpeg_size, num
from gimbal_service.links import MOCK_JPEG, MockLink

V1 = os.path.join(HERE, "..", "contracts", "v1")
SCHEMAS = {f[:-12]: json.load(open(os.path.join(V1, f))) for f in os.listdir(V1) if f.endswith(".schema.json")}
REG = Registry().with_resources([(s["$id"], Resource.from_contents(s)) for s in SCHEMAS.values()])
CANON = os.environ.get("ROBOTICS_REPO", r"C:\Users\vibhe\Documents\Codex\2026-09-27\can-you-integrate-to-solidworks-to\outputs"
                                        r"\AI_Assistant_Robot\otto_base_robo_code-main")


def valid(obj, name):
    schema = {"$ref": f"common.schema.json#/$defs/{name}"} if name in ("envelope", "error", "gimbal_pose") else {"$ref": SCHEMAS[name]["$id"]}
    jsonschema.Draft202012Validator(schema, registry=REG).validate(obj)


class Client:
    """Test client that checks the v1 envelope on every JSON response."""
    def __init__(self, c, token): self.c, self.h = c, {"Authorization": f"Bearer {token}"}

    def __call__(self, method, path, body=None, auth=True, **kw):
        r = getattr(self.c, method.lower())(path, json=body, headers={**(self.h if auth else {}), **kw.pop("headers", {})}, **kw)
        if r.is_json:
            valid(r.get_json(), "envelope")
            e = r.get_json()
            if not e["ok"]: valid(e["error"], "error")
            return r.status_code, e
        return r.status_code, r

    def cmd(self, op, args, rid=None, **extra):
        return self("POST", "/v1/commands", {"schema_version": "1.0", "request_id": rid or f"req_{uuid.uuid4().hex[:16]}", "operation": op, "args": args, **extra})

    def wait(self, oid, timeout=8):
        end = time.time() + timeout
        while time.time() < end:
            _, e = self("GET", f"/v1/operations/{oid}")
            valid(e["data"], "operation_result")
            if e["data"]["state"] not in ("accepted", "running"): return e["data"]
            time.sleep(0.03)
        raise AssertionError(f"operation {oid} still {e['data']['state']}")

    def run(self, op, args, **kw):
        code, e = self.cmd(op, args, **kw)
        assert code == 202 and e["ok"], e
        return self.wait(e["data"]["operation_id"], kw.pop("timeout", 15))


@pytest.fixture
def env():
    cfg = Config(token="t0k", settle_s=0.05, camera_latency_s=0.05, survey_settle_s=0.1, max_speed_dps=360, default_speed_dps=200)
    link = MockLink(calibrated=True, fps=30)
    dev = Device(cfg, link)
    c = Client(create_app(dev).test_client(), "t0k")
    deadline = time.time() + 3
    while not dev.camera_status()["ready"] and time.time() < deadline: time.sleep(0.02)
    yield dev, link, c
    dev.close()


def settle_status(): time.sleep(0.2)   # next firmware status (150 ms)


# ---------------------------------------------------------------- level 2: unit
def test_validation_helpers():
    assert num({"a": 3}, "a") == 3.0
    for bad in ("x", True, None, float("nan"), float("inf")):
        with pytest.raises(ApiError) as e: num({"a": bad}, "a")
        assert e.value.http == 422 and e.value.code == "INVALID_ARGUMENT"
    with pytest.raises(ApiError) as e: num({"a": 500}, "a", lo=10, hi=120)
    assert e.value.code == "OUT_OF_RANGE"
    assert jpeg_size(MOCK_JPEG) == (64, 48) and jpeg_size(b"junk") == (None, None)


def test_limits_are_soft_limits_and_servo_reach_and_cad_cap(env):
    dev, link, _ = env
    link._s.update(lp=[-90, 90], rp=[-17, 163]); settle_status()   # the real 2026-10-19 lopsided mount
    assert dev.limits()["pan"] == [-17, 90]
    link._s.update(lp=[-17, 163]); settle_status()                 # stale firmware soft limit: host CAD cap still holds
    assert dev.limits()["pan"] == [-17, 90]
    assert any(f["code"] == "LOPSIDED_MOUNT" for f in dev.faults())


def test_vendored_schemas_match_canonical():
    src = os.path.join(CANON, "robotics", "shared", "schemas")
    if not os.path.isdir(src): pytest.skip("canonical assistant repo not on this machine")
    for f in os.listdir(src):
        if f.endswith(".schema.json"):
            assert json.load(open(os.path.join(src, f))) == json.load(open(os.path.join(V1, f))), f"{f} drifted: re-vendor contracts/v1"


# ---------------------------------------------------------------- level 4: canonical contract
def test_canonical_conformance_checker(env):
    """The orchestrator's own checker (read-only + one capture), driven through the Flask test client."""
    if not os.path.isdir(os.path.join(CANON, "robotics")): pytest.skip("canonical assistant repo not on this machine")
    sys.path.insert(0, CANON)
    from robotics.devices.conformance import check
    dev, link, c = env

    def transport(method, path, body, timeout):
        r = getattr(c.c, method.lower())(path, json=body, headers={**c.h, "X-Robotics-Api": "1"})
        return r.status_code, dict(r.headers), r.get_json() if r.is_json else r.data
    sent = len(link.sent)
    assert check(transport) == []
    assert not [s for s in link.sent[sent:] if s.startswith(("PT:", "MV:", "SRV:"))]   # never moved anything


def test_info_status_health_match_contract(env):
    dev, link, c = env
    code, e = c("GET", "/v1/health", auth=False)                     # open
    assert code == 200 and e["data"]["ready"]
    _, e = c("GET", "/v1/info"); valid(e["data"], "device_capabilities")
    i = e["data"]
    assert i["device_type"] == "gimbal" and "gimbal.stop" in i["capabilities"] and "perception.detections" not in i["capabilities"]
    assert all(o["capability"] in i["capabilities"] and "moves_hardware" in o for o in i["operations"])
    assert i["sensors"][0]["frame_id"] == "gimbal-01/camera_optical" and "closed_loop_position_feedback" in i["unsupported"]
    _, e = c("GET", "/v1/status"); valid(e["data"], "device_status")
    s = e["data"]
    assert s["connection"] == "online" and s["state"] == "idle" and s["pose"]["frame_id"] == "gimbal-01/pan_tilt_base"
    assert s["pose"]["physically_verified"] is False and s["pose"]["source"] == "firmware_commanded_open_loop"
    assert abs(s["observed_at"] - time.time()) < 1 and s["limits"] == {"pan_deg": [-90, 90], "tilt_deg": [-45, 45]}


def test_auth_and_version(env):
    _, _, c = env
    code, e = c("GET", "/v1/status", auth=False)
    assert code == 401 and e["error"]["code"] == "UNAUTHORIZED"
    assert c("GET", "/v1/status", auth=False, headers={"Authorization": "Bearer nope"})[0] == 401
    assert c("GET", "/v1/status?token=t0k", auth=False)[0] == 200
    code, e = c("GET", "/v1/status", headers={"X-Robotics-Api": "2"})
    assert code == 400 and e["error"]["code"] == "VERSION_MISMATCH"
    code, e = c.cmd("gimbal.home", {}, schema_version="2.0")
    assert e["error"]["code"] == "VERSION_MISMATCH"


def test_move_ack_then_commanded_reached(env):
    dev, link, c = env
    op = c.run("gimbal.move", {"pan_deg": 20, "tilt_deg": -10}, rid="req_abc")
    assert op["state"] == "succeeded" and op["outcome"] == "commanded_reached" and op["request_id"] == "req_abc"
    assert op["device_ack_at"] >= op["created_at"] and op["updated_at"] >= op["device_ack_at"]
    assert op["result"]["pose"]["physically_verified"] is False and op["result"]["params"]["target_pan_deg"] == 20
    assert abs(link.status["pan"] - 20) < 0.2 and "PT:20.00,-10.00" in link.sent


def test_relative_move_and_alias_route(env):
    dev, link, c = env
    c.run("gimbal.move", {"pan_deg": 10, "tilt_deg": 0})
    code, e = c("POST", "/v1/motion/move", {"mode": "relative", "pan_deg": 5, "request_id": "req_alias"})
    assert code == 202
    op = c.wait(e["data"]["operation_id"])
    assert op["state"] == "succeeded" and op["result"]["params"]["target_pan_deg"] == 15 and op["operation"] == "gimbal.move"


def test_idempotent_request_id(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": 30, "tilt_deg": 0}, rid="req_same")
    n = len([s for s in link.sent if s.startswith("PT:")])
    code, b = c.cmd("gimbal.move", {"pan_deg": 30, "tilt_deg": 0}, rid="req_same")
    assert code == 200 and b["data"]["operation_id"] == a["data"]["operation_id"]          # not executed twice
    c.wait(a["data"]["operation_id"])
    assert len([s for s in link.sent if s.startswith("PT:")]) == n


@pytest.mark.parametrize("args,code,reason", [
    ({"pan_deg": 91, "tilt_deg": 0}, "OUT_OF_RANGE", "out_of_limits"), ({"pan_deg": 0, "tilt_deg": -46}, "OUT_OF_RANGE", "out_of_limits"),
    ({"pan_deg": "10", "tilt_deg": 0}, "INVALID_ARGUMENT", "not_a_number"), ({"pan_deg": 10}, "INVALID_ARGUMENT", "missing_field"),
    ({"pan_deg": 0, "tilt_deg": 0, "speed_dps": 1000}, "OUT_OF_RANGE", "argument_range"),
    ({"pan_deg": 0, "tilt_deg": 0, "mode": "fly"}, "INVALID_ARGUMENT", "bad_enum"),
    ({"pan_deg": 0, "tilt_deg": 0, "spead_dps": 5}, "INVALID_ARGUMENT", "unknown_field"),
    ({"pan_deg": 0, "tilt_deg": 0, "preempt": "yes"}, "INVALID_ARGUMENT", "not_a_boolean")])
def test_invalid_moves_rejected_before_anything_is_sent(env, args, code, reason):
    dev, link, c = env
    n = len(link.sent)
    http, e = c.cmd("gimbal.move", args)
    assert http == 422 and e["error"]["code"] == code and e["error"]["details"]["reason"] == reason and not e["error"]["retryable"]
    assert not [s for s in link.sent[n:] if s.startswith("PT:")]


@pytest.mark.parametrize("body,code", [({"request_id": "r1", "operation": "gimbal.home", "args": {}}, "VERSION_MISMATCH"),
                                       ({"schema_version": "1.0", "operation": "gimbal.home", "args": {}}, "INVALID_ARGUMENT"),
                                       ({"schema_version": "1.0", "request_id": "r2", "operation": "gimbal.dance", "args": {}}, "UNSUPPORTED"),
                                       ({"schema_version": "1.0", "request_id": "r3", "operation": "gimbal.home", "args": []}, "INVALID_ARGUMENT"),
                                       ({"schema_version": "1.0", "request_id": "r4", "operation": "gimbal.home", "args": {}, "x": 1}, "INVALID_ARGUMENT")])
def test_command_envelope_validation(env, body, code):
    _, _, c = env
    http, e = c("POST", "/v1/commands", body)
    assert not e["ok"] and e["error"]["code"] == code


def test_bad_json(env):
    _, _, c = env
    r = c.c.post("/v1/commands", data="{nope", headers={**c.h, "Content-Type": "application/json"})
    assert r.status_code == 400 and r.get_json()["error"]["code"] == "INVALID_ARGUMENT" and r.get_json()["error"]["details"]["reason"] == "bad_json"


def test_overlap_busy_and_preempt(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20})
    http, e = c.cmd("gimbal.move", {"pan_deg": -80, "tilt_deg": 0})
    assert http == 409 and e["error"]["code"] == "BUSY" and e["error"]["retryable"]
    _, b = c.cmd("gimbal.move", {"pan_deg": 5, "tilt_deg": 0, "preempt": True})
    old = c.wait(a["data"]["operation_id"])
    assert old["state"] == "preempted" and old["error"]["code"] == "PREEMPTED" and old["outcome"] == "unconfirmed"
    assert c.wait(b["data"]["operation_id"])["state"] == "succeeded"


def test_stop_cancels_and_freezes(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20})
    time.sleep(0.4)
    code, e = c("POST", "/v1/stop", {"request_id": "req_stop"})
    s = e["data"]; valid(s, "operation_result")
    assert code == 200 and s["operation"] == "gimbal.stop" and s["state"] == "succeeded" and s["result"]["confirmed_by_device"]
    assert s["result"]["cancelled_operation_id"] == a["data"]["operation_id"]
    op = c.wait(a["data"]["operation_id"])
    assert op["state"] == "cancelled" and op["error"]["code"] == "CANCELLED"
    time.sleep(0.4)
    assert abs(link.status["pan"] - s["result"]["pose"]["pan_deg"]) < 0.3 and 2 < link.status["pan"] < 30   # holds where it stopped
    assert "MODE:MANUAL" in link.sent
    assert c("POST", "/v1/motion/stop", {})[0] == 200                                                  # alias
    assert c.cmd("gimbal.stop", {})[1]["data"]["operation"] == "gimbal.stop"                           # as a command


def test_cancel_endpoint(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": -80, "tilt_deg": 0, "speed_dps": 20})
    time.sleep(0.2)
    _, e = c("POST", f"/v1/operations/{a['data']['operation_id']}/cancel", {})
    assert e["data"]["state"] == "cancelled"
    assert c("POST", "/v1/operations/op_missing/cancel", {})[0] == 404


def test_deadline_times_out(env):
    dev, link, c = env
    op = c.run("gimbal.move", {"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20}, deadline_s=0.5)
    assert op["state"] == "timed_out" and op["error"]["code"] == "TIMEOUT" and op["outcome"] == "unconfirmed"


@pytest.mark.parametrize("fault,code", [("stall", "MOTION_TIMEOUT"), ("deaf", "COMMAND_NOT_ACKNOWLEDGED")])
def test_faults_fail_the_operation(env, fault, code):
    dev, link, c = env
    setattr(link, fault, True)
    op = c.run("gimbal.move", {"pan_deg": 3, "tilt_deg": 0})
    assert op["state"] == "failed" and op["error"]["code"] == code
    assert op["outcome"] == "unconfirmed"          # the command was sent: completion unknown, not "nothing happened"


def test_link_lost_mid_move(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20})
    time.sleep(0.3); link.online = False
    op = c.wait(a["data"]["operation_id"], timeout=5)
    assert op["state"] == "failed" and op["error"]["code"] == "LINK_LOST"
    http, e = c.cmd("gimbal.move", {"pan_deg": 0, "tilt_deg": 0})
    assert http == 503 and e["error"]["code"] == "OFFLINE" and e["error"]["retryable"]
    _, e = c("POST", "/v1/stop", {})
    assert e["data"]["state"] == "failed" and e["data"]["error"]["code"] == "OFFLINE"
    _, e = c("GET", "/v1/status")
    assert e["data"]["connection"] == "offline" and e["data"]["health"] == "fault"


def test_external_client_takes_over(env):
    dev, link, c = env
    _, a = c.cmd("gimbal.move", {"pan_deg": 80, "tilt_deg": 0, "speed_dps": 20})
    time.sleep(0.3); link.send("PT:-30,0")                     # e.g. the tracker or the browser UI
    op = c.wait(a["data"]["operation_id"])
    assert op["state"] == "failed" and op["error"]["code"] == "PREEMPTED" and op["error"]["details"]["reason"] == "external_client"


def test_outcome_none_only_when_nothing_was_sent():
    from gimbal_service.core import Operation
    op = Operation("gimbal.move", {}, "r", "gimbal-01", state="cancelled")
    assert op.to_dict()["outcome"] == "none"
    op.commanded = True
    assert op.to_dict()["outcome"] == "unconfirmed"


@pytest.mark.parametrize("st,code,state", [({"cal": False}, "NOT_CALIBRATED", "idle"), ({"calOn": True}, "CALIBRATION_IN_PROGRESS", "calibrating"),
                                           ({"panEn": False}, "MOTORS_DISABLED", "disabled")])
def test_motion_gated_on_calibration_and_motors(env, st, code, state):
    dev, link, c = env
    link._s.update(st); settle_status()
    http, e = c.cmd("gimbal.home", {})
    assert http == 409 and e["error"]["code"] == code
    assert c("GET", "/v1/status")[1]["data"]["state"] == state


def test_uncalibrated_allowed_by_config(env):
    dev, link, c = env
    link._s["cal"] = False; dev.cfg.allow_uncalibrated = True; settle_status()
    assert c.run("gimbal.home", {})["state"] == "succeeded"


def test_motors_command(env):
    dev, link, c = env
    code, e = c.cmd("gimbal.motors", {"enabled": False})
    assert code == 200 and e["data"]["state"] == "succeeded" and "SRV:P,0" in link.sent
    settle_status()
    assert c("GET", "/v1/status")[1]["data"]["motors_enabled"] is False
    assert c.cmd("gimbal.motors", {"enabled": True})[1]["data"]["state"] == "succeeded"
    assert c.cmd("gimbal.motors", {"enabled": "yes"})[1]["error"]["code"] == "INVALID_ARGUMENT"


def test_fresh_capture_observation(env):
    dev, link, c = env
    t0 = time.time()
    code, e = c("POST", "/v1/observations", {"sensor_id": dev.sensor_id})
    o = e["data"]; valid(o, "observation")
    assert code == 201 and o["kind"] == "frame_ref" and o["sensor_type"] == "camera" and o["frame_id"] == "gimbal-01/camera_optical"
    assert o["sensor_pose"]["representation"] == "pan_tilt_relative" and o["sensor_pose"]["physically_verified"] is False
    assert o["timing"] == {**o["timing"], "timestamp_source": "host_receive_minus_latency", "estimated": True, "latency_s": 0.05}
    assert o["timing"]["received_at"] >= t0 + dev.cfg.camera_latency_s - 0.01 and o["quality"]["width"] == 64
    code, r = c("GET", o["data_ref"])
    assert code == 200 and r.data == MOCK_JPEG and float(r.headers["X-Observed-At"]) == pytest.approx(o["observed_at"], abs=1e-3)
    assert c("GET", "/v1/observations/obs_nothere")[0] == 404
    assert c("POST", "/v1/observations", {"sensor_id": "lidar9"})[0] == 404
    _, e = c("GET", "/v1/observations/latest?kind=frame_ref")
    assert e["data"]["observation_id"] == o["observation_id"]


def test_frame_jpg_and_snapshot_headers(env):
    dev, link, c = env
    code, r = c("GET", f"/v1/sensors/{dev.sensor_id}/frame.jpg")
    assert code == 200 and r.data[:2] == b"\xff\xd8" and abs(float(r.headers["X-Server-Time"]) - time.time()) < 1
    assert float(r.headers["X-Observed-At"]) < float(r.headers["X-Server-Time"])
    code, r = c("GET", "/v1/camera/snapshot")
    assert code == 200 and r.headers["X-Observation-ID"].startswith("obs_")
    assert c("GET", "/v1/sensors/nope/frame.jpg")[0] == 404


def test_camera_down_and_stale_frames_are_errors(env):
    dev, link, c = env
    link.camera = False
    code, e = c("GET", "/v1/camera/snapshot")
    assert code == 503 and e["error"]["code"] == "CAMERA_UNAVAILABLE"
    assert c("GET", "/v1/status")[1]["data"]["connection"] == "degraded"
    link.camera = True; link._fps = 0.5; time.sleep(1.2)          # frames now arrive every 2 s
    code, e = c("GET", f"/v1/sensors/{dev.sensor_id}/frame.jpg")
    assert code in (200, 503)
    if code == 503: assert e["error"]["code"] == "FRAME_STALE"


def test_survey_visual_views_coverage_restore(env):
    dev, link, c = env
    op = c.run("survey.visual", {"pan_from_deg": -20, "pan_to_deg": 20, "step_deg": 20, "tilts_deg": [0, 10], "restore": "previous"})
    assert op["state"] == "succeeded" and op["outcome"] == "commanded_reached" and op["result"]["complete"]
    views = op["result"]["views"]
    assert len(views) == 6 and op["observation_ids"] == [v["observation_id"] for v in views]
    for v in views: valid(v, "observation"); assert v["kind"] == "survey_view" and v["sensor_pose"]["at_rest"]
    assert [s["target_pan_deg"] for s in op["result"]["steps"]] == [-20, 0, 20, 20, 0, -20]   # serpentine
    assert abs(views[2]["sensor_pose"]["pan_deg"] - 20) < 0.3 and views[2]["source"]["step_index"] == 2
    assert op["result"]["coverage"] == {"frame_id": "gimbal-01/pan_tilt_base", "pan_deg": [-20, 20], "tilt_deg": [0, 10], "hfov_deg": 52.8}
    assert abs(link.status["pan"]) < 0.3 and abs(link.status["tilt"]) < 0.3   # restored


def test_survey_cancel_keeps_partial_views(env):
    dev, link, c = env
    _, a = c.cmd("survey.visual", {"pan_from_deg": -60, "pan_to_deg": 60, "step_deg": 60, "speed_dps": 60})
    oid, end = a["data"]["operation_id"], time.time() + 10
    while not c("GET", f"/v1/operations/{oid}")[1]["data"]["observation_ids"] and time.time() < end: time.sleep(0.05)
    c("POST", "/v1/stop", {})
    op = c.wait(oid)
    assert op["state"] == "cancelled" and 1 <= len(op["result"]["views"]) < 3 and 0 < op["progress"] < 1
    assert "not_reached" in [s["status"] for s in op["result"]["steps"]]


def test_survey_capture_failure_is_partial_failure(env):
    dev, link, c = env
    _, a = c.cmd("survey.visual", {"pan_from_deg": 5, "pan_to_deg": 10, "step_deg": 5, "restore": "none"})
    oid, end = a["data"]["operation_id"], time.time() + 10
    while not c("GET", f"/v1/operations/{oid}")[1]["data"]["observation_ids"] and time.time() < end: time.sleep(0.02)
    link.camera = False
    op = c.wait(oid)
    assert op["state"] == "failed" and op["error"]["code"] in ("CAMERA_UNAVAILABLE", "FRAME_TIMEOUT")
    assert op["error"]["details"]["reason"] == "survey_incomplete" and len(op["result"]["views"]) == 1


@pytest.mark.parametrize("args", [{"pan_from_deg": 0, "pan_to_deg": 10}, {"pan_from_deg": -90, "pan_to_deg": 90, "step_deg": 1},
                                  {"pan_from_deg": 0, "pan_to_deg": 10, "step_deg": 5, "tilt_deg": 99},
                                  {"pan_from_deg": 0, "pan_to_deg": 10, "step_deg": 5, "restore": "maybe"},
                                  {"pan_from_deg": 0, "pan_to_deg": 10, "step_deg": 5, "settle_s": 0.0},
                                  {"pan_from_deg": 0, "pan_to_deg": 10, "step_deg": 5, "tilts": 0},
                                  {"pan_from_deg": 0, "pan_to_deg": 10, "step_deg": 5, "tilt_deg": 0, "tilts_deg": [0]}])
def test_survey_validation(env, args):
    _, link, c = env
    code, e = c.cmd("survey.visual", args)
    assert code == 422 and e["error"]["code"] in ("INVALID_ARGUMENT", "OUT_OF_RANGE"), e


def test_room_events(env):
    dev, link, c = env
    t0 = time.time() - 0.01
    c.run("gimbal.move", {"pan_deg": 4, "tilt_deg": 0})
    c.run("survey.visual", {"pan_from_deg": 0, "pan_to_deg": 5, "step_deg": 5})
    _, e = c("GET", f"/v1/events?since={t0:.3f}")
    evs = e["data"]["events"]
    for x in evs: valid(x, "room_event"); assert x["confirmed"] is False
    types = [x["type"] for x in evs]
    assert "gimbal_moved" in types and "survey_started" in types and "survey_completed" in types
    done = next(x for x in evs if x["type"] == "survey_completed")
    assert len(done["evidence"]["observation_ids"]) == 2
    _, e = c("GET", f"/v1/events?since={t0:.3f}&types=survey_started&limit=5")
    assert [x["type"] for x in e["data"]["events"]] == ["survey_started"]
    assert c("GET", f"/v1/events?since={time.time() + 10:.3f}")[1]["data"]["events"] == []
    assert c("GET", "/v1/events?since=abc")[0] == 422
    _, e = c("GET", "/v1/events?internal=1")
    assert {"operation.accepted", "operation.succeeded", "observation.created"} <= {x["type"] for x in e["data"]["events"]}


def test_presence_ingest_maps_to_room_events(env):
    dev, link, c = env
    t0 = time.time() - 0.01
    code, e = c("POST", "/v1/perception/events?token=t0k", {"event": "person_entered", "t": 1.0, "count": 1, "detector": "person"}, auth=False)
    assert code == 201 and e["data"]["type"] == "perception.person_visible" and e["data"]["data"]["identity"] is None
    c("POST", "/v1/perception/events", {"event": "person_left"})
    c("POST", "/v1/perception/events", {"type": "perception.presence_unknown"})
    evs = c("GET", f"/v1/events?since={t0:.3f}")[1]["data"]["events"]
    for x in evs: valid(x, "room_event")
    assert [x["type"] for x in evs] == ["presence_started", "presence_ended", "observation_gap"]
    assert evs[0]["evidence"]["method"] == "detector_presence" and evs[0]["confirmed"] is False and evs[0]["evidence"]["observation_ids"]
    for bad in ({"type": "perception.entry_confirmed"}, {"type": "perception.owner_home"}, {"event": "dance"}):
        code, e = c("POST", "/v1/perception/events", bad)
        assert code == 422 and e["error"]["code"] == "UNSUPPORTED_EVENT"


def test_room_event_webhook(env):
    import http.server, threading as th
    got = []
    class H(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            got.append(json.loads(self.rfile.read(int(self.headers["Content-Length"])))); self.send_response(200); self.end_headers()
        def log_message(self, *a): pass
    srv = http.server.HTTPServer(("127.0.0.1", 0), H)
    th.Thread(target=srv.serve_forever, daemon=True).start()
    dev, link, c = env
    dev.cfg.event_webhook = f"http://127.0.0.1:{srv.server_port}/gimbal/event"
    c("POST", "/v1/perception/events", {"event": "person_entered", "count": 1, "detector": "person"})
    end = time.time() + 3
    while not got and time.time() < end: time.sleep(0.02)
    srv.shutdown()
    assert got and got[0]["type"] == "presence_started"; valid(got[0], "room_event")


def test_detections_ingest_and_latest(env):
    dev, link, c = env
    assert c("GET", "/v1/observations/latest?kind=detections")[1]["error"]["code"] == "NOT_FOUND"
    det = {"type": "perception.detections", "data": {"observed_at": time.time() - 0.3, "model": "yolo11n", "latency_s": 0.31,
           "sensor_pose": {"pan_deg": 10.0, "tilt_deg": 0.0},
           "detections": [{"label": "person", "confidence": 0.82, "track_id": 3, "box_norm": [-0.002, 0.1, 0.4, 1.001],
                           "bearing": {"pan_deg": 2.5, "tilt_deg": 4.0}}]}}
    assert c("POST", "/v1/perception/events", det)[0] == 201
    _, e = c("GET", "/v1/observations/latest?kind=detections")
    o = e["data"]; valid(o, "observation")
    assert o["kind"] == "detections" and o["frame_id"] == "gimbal-01/pan_tilt_base" and o["model"] == "yolo11n"
    assert o["detections"][0]["box_norm"] == [0.0, 0.1, 0.4, 1.0] and "position_m" not in o["detections"][0]   # bearing only
    assert "perception.detections" in c("GET", "/v1/info")[1]["data"]["capabilities"]
    for bad in ({"detections": [{"label": "x", "confidence": 2}]}, {"detections": "x"}, {"detections": [], "observed_at": 5}):
        d = {**det["data"], **bad}
        assert c("POST", "/v1/perception/events", {"type": "perception.detections", "data": d})[1]["error"]["code"] == "INVALID_ARGUMENT"


def test_camera_config(env):
    dev, link, c = env
    assert c("POST", "/v1/camera/config", {"resolution": "320x240", "fps_cap": 20})[0] == 202
    assert link.cam["res"] == 0 and link.cam["fps"] == 20
    assert c("POST", "/v1/camera/config", {"resolution": "4k"})[0] == 422
    assert c("POST", "/v1/camera/config", {"fps_cap": 99})[0] == 422


def test_restream_does_not_drop_burst_frames(env):
    dev, link, c = env
    link.camera = False; time.sleep(0.1)                        # mock camera quiet: only our burst
    seq0 = link.frame()[2]
    for _ in range(3): link.push(MOCK_JPEG)                     # burst: 3 frames before any reader wakes
    assert [f[2] for f in link.frames_after(seq0)][:3] == [seq0 + 1, seq0 + 2, seq0 + 3]


def test_persistence_and_retention(env, tmp_path):
    dev, link, c = env
    dev.cfg.persist_dir, dev.cfg.max_observations = str(tmp_path), 1
    a = c("POST", "/v1/observations", {})[1]["data"]["observation_id"]
    c("POST", "/v1/observations", {})                                       # evicts `a` from memory
    assert c("GET", f"/v1/observations/{a}/image")[1].data == MOCK_JPEG     # served from disk
    old = time.time() - 48 * 3600
    for f in tmp_path.iterdir(): os.utime(f, (old, old))
    dev._prune()
    assert not list(tmp_path.iterdir()) and c("GET", f"/v1/observations/{a}")[0] == 404


def test_unknown_endpoint_and_method_are_enveloped(env):
    _, _, c = env
    code, e = c("GET", "/v1/nope")
    assert code == 404 and e["error"]["code"] == "NOT_FOUND"
    code, e = c("DELETE", "/v1/status")
    assert code == 405 and e["error"]["code"] == "UNSUPPORTED"
