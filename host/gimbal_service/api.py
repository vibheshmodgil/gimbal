"""HTTP service, Robotics Device API v1.   python -m gimbal_service.api   (config: GIMBAL_* env vars, see README)

Every JSON response is the v1 envelope {schema_version, ok, data, error, server_time}. Binary routes (JPEG, MJPEG,
SSE) are not wrapped; their errors are.
"""
import hmac, ipaddress, json, signal, sys, time, uuid

from flask import Flask, Response, g, jsonify, request

from .core import ApiError, Config, Device, RESOLUTIONS, SCHEMA_VERSION, num, only


def create_app(device):
    app, cfg = Flask(__name__), device.cfg

    def env(data=None, http=200, error=None):
        return jsonify({"schema_version": SCHEMA_VERSION, "ok": error is None, "data": data, "error": error,
                        "server_time": time.time(), "request_id": g.get("rid")}), http

    def _err(e):
        return env(error={**e.err(), "details": {**e.details, "request_id": g.get("rid")}}, http=e.http)

    @app.before_request
    def _auth():
        body = request.get_json(silent=True) if request.is_json else None
        g.rid = request.headers.get("X-Request-ID") or (body.get("request_id") if isinstance(body, dict) else None) or f"req_{uuid.uuid4().hex[:16]}"
        api = request.headers.get("X-Robotics-Api")
        if api and api.split(".")[0] != "1": return _err(ApiError(400, "VERSION_MISMATCH", f"client speaks {api}, this device speaks 1.x"))
        if request.path == "/v1/health" or not cfg.token: return None
        h = request.headers.get("Authorization", "")
        tok = h[7:] if h.startswith("Bearer ") else request.headers.get("X-API-Token") or request.args.get("token", "")
        if not hmac.compare_digest(tok.encode(), cfg.token.encode()):
            return _err(ApiError(401, "UNAUTHORIZED", "missing or wrong bearer token"))

    @app.after_request
    def _rid(r):
        r.headers["X-Request-ID"] = g.get("rid", "")
        return r

    app.register_error_handler(ApiError, _err)
    app.register_error_handler(404, lambda e: _err(ApiError(404, "NOT_FOUND", f"no endpoint {request.method} {request.path}")))
    app.register_error_handler(405, lambda e: _err(ApiError(405, "UNSUPPORTED", f"{request.method} not allowed on {request.path}")))

    def body():
        d = request.get_json(silent=True)
        if d is None and request.data: raise ApiError(400, "INVALID_ARGUMENT", "body is not valid JSON", "bad_json")
        if d is not None and not isinstance(d, dict): raise ApiError(400, "INVALID_ARGUMENT", "body must be a JSON object", "bad_json")
        return d or {}

    def args_only(d):   # resource aliases: the body is the args (+ optional request_id)
        d = dict(d); d.pop("request_id", None)
        return d

    def op_env(op, http=None):
        return env(op.to_dict(), http or (202 if op.active else 200))

    # ---------------------------------------------------------------- device
    @app.get("/v1/health")
    def health():
        r = device.ready_reasons()
        return env({"device_id": cfg.device_id, "alive": True, "ready": not r, "not_ready_reasons": r, "t": time.time()})

    @app.get("/v1/info")
    def info(): return env(device.info())

    app.add_url_rule("/v1/device", "device", info)            # v0 name, same content

    @app.get("/v1/status")
    def status(): return env(device.status())

    # ---------------------------------------------------------------- commands + operations
    @app.post("/v1/commands")
    def commands():
        op, http = device.command(body())
        return op_env(op, http)

    @app.get("/v1/operations")
    def ops(): return env({"operations": [o.to_dict() for o in list(device.ops.values())[-50:]]})

    @app.get("/v1/operations/<oid>")
    def op_get(oid):
        op = device.ops.get(oid)
        if not op: raise ApiError(404, "NOT_FOUND", f"no operation {oid}")
        return op_env(op, 200)

    @app.post("/v1/operations/<oid>/cancel")
    def op_cancel(oid): return op_env(device.cancel(oid, g.rid), 200)

    @app.post("/v1/stop")
    def stop_v1():
        only(body(), "request_id")
        return op_env(device.stop(g.rid), 200)   # a failed stop is state "failed" + error OFFLINE inside the result

    # resource-style aliases (v0), same handlers and result shape
    @app.post("/v1/motion/move")
    def move(): return op_env(device.move(args_only(body()), g.rid))

    @app.post("/v1/motion/home")
    def home(): return op_env(device.home(args_only(body()), g.rid))

    app.add_url_rule("/v1/motion/stop", "motion_stop", stop_v1, methods=["POST"])

    @app.post("/v1/survey")
    def survey(): return op_env(device.survey(args_only(body()), g.rid))

    # ---------------------------------------------------------------- camera
    @app.get("/v1/camera/status")
    def cam_status(): return env(device.camera_status())

    @app.post("/v1/camera/config")
    def cam_config():
        d, sent = only(body(), "resolution", "fps_cap"), []
        if "resolution" in d:
            if d["resolution"] not in RESOLUTIONS: raise ApiError(422, "INVALID_ARGUMENT", "resolution must be one of " + ", ".join(RESOLUTIONS), "bad_enum")
            sent.append(f"CAM:res,{RESOLUTIONS.index(d['resolution'])}")
        if "fps_cap" in d: sent.append(f"CAM:fps,{int(num(d, 'fps_cap', lo=1, hi=30))}")
        if not sent: raise ApiError(422, "INVALID_ARGUMENT", "give resolution and/or fps_cap", "missing_field")
        for c in sent + ["CAMGET"]:
            if not device.link.send(c): raise ApiError(503, "OFFLINE", "firmware not connected")
        return env({"sent": sent, "note": "applied live, not saved to flash; check /v1/camera/status"}, 202)

    def jpeg(o, jpg):
        return Response(jpg, mimetype="image/jpeg", headers={"X-Observation-ID": o["observation_id"], "X-Observed-At": f"{o['observed_at']:.3f}",
                                                             "X-Server-Time": f"{time.time():.3f}", "X-Timestamp-Source": o["timing"]["timestamp_source"],
                                                             "Cache-Control": "no-store"})

    @app.get("/v1/camera/snapshot")
    def snapshot():
        """JPEG of a frame exposed after this request (fresh=1, default) or the newest one (fresh=0, max_age_ms)."""
        fresh = request.args.get("fresh", "1") != "0"
        try: max_age = float(request.args.get("max_age_ms", cfg.frame_stale_s * 1000))
        except ValueError: raise ApiError(422, "INVALID_ARGUMENT", "max_age_ms must be a number", "not_a_number")
        max_age = num({"max_age_ms": max_age}, "max_age_ms", lo=0, hi=60000) / 1000
        o = device.observe(after_t=time.time() + cfg.camera_latency_s if fresh else None, max_age_s=max_age, source={"request_id": g.rid})
        return jpeg(o, device.get_obs(o["observation_id"])[1])

    @app.get("/v1/sensors/<path:sid>/frame.jpg")
    def frame(sid):
        """camera.frame: newest frame if younger than GIMBAL_FRAME_STALE_S, with its own (estimated) capture time."""
        if sid != device.sensor_id: raise ApiError(404, "NOT_FOUND", f"no sensor {sid}")
        o = device.observe(source={"request_id": g.rid, "route": "frame.jpg"})
        return jpeg(o, device.get_obs(o["observation_id"])[1])

    @app.get("/v1/camera/stream")
    def stream():
        """The firmware's JPEGs, re-served unchanged (one firmware slot shared by any number of clients)."""
        def gen():
            device.link.viewers += 1
            try:
                seq = 0
                while True:
                    device.link.wait_frame(seq, 2.0)
                    for jpg, _, seq in device.link.frames_after(seq):   # every frame, bursts included
                        yield b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n%s\r\n" % (len(jpg), jpg)
            finally:
                device.link.viewers -= 1
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")

    # ---------------------------------------------------------------- observations
    @app.post("/v1/observations")
    def observe():
        """camera.capture: a frame exposed AFTER this request -> frame_ref observation (+ data_ref to the JPEG)."""
        d = only(body(), "sensor_id", "fresh", "source")
        if d.get("sensor_id") not in (None, device.sensor_id, "cam", "camera"): raise ApiError(404, "NOT_FOUND", f"no sensor {d['sensor_id']}")
        fresh = d.get("fresh", True)
        src = d.get("source") if isinstance(d.get("source"), dict) else {}
        o = device.observe(after_t=time.time() + cfg.camera_latency_s if fresh else None, source={"request_id": g.rid, **src})
        return env(o, 201)

    @app.get("/v1/observations/latest")
    def obs_latest(): return env(device.latest(request.args.get("kind"), request.args.get("sensor_id")))

    @app.get("/v1/observations/<oid>")
    def obs_get(oid): return env(device.get_obs(oid)[0])

    @app.get("/v1/observations/<oid>/image")
    def obs_img(oid):
        o, jpg = device.get_obs(oid)
        return jpeg(o, jpg)

    # ---------------------------------------------------------------- events
    @app.get("/v1/events")
    def events():
        """JSON room events (room_event.schema.json): ?since=<epoch>&until=&types=a,b&limit=.
        Accept: text/event-stream = live SSE of the finer internal events (resume with Last-Event-ID)."""
        if "text/event-stream" in request.headers.get("Accept", ""):
            after = int(request.headers.get("Last-Event-ID") or 0)
            def gen():
                seq = after
                while True:
                    evs = device.bus.since(seq, timeout=15)
                    if not evs: yield ": keep-alive\n\n"
                    for e in evs:
                        seq = e["seq"]
                        yield f"id: {e['seq']}\nevent: {e['type']}\ndata: {json.dumps(e)}\n\n"
            return Response(gen(), mimetype="text/event-stream", headers={"Cache-Control": "no-cache"})
        if request.args.get("internal"):   # backlog of the internal stream as JSON (debugging, tests)
            return env({"events": device.bus.since(int(request.args.get("after", 0)))})
        try:
            since, until = float(request.args.get("since", 0)), request.args.get("until")
            until, limit = float(until) if until else None, int(request.args.get("limit", 200))
        except ValueError:
            raise ApiError(422, "INVALID_ARGUMENT", "since/until must be epoch seconds, limit an integer", "not_a_number")
        if not 1 <= limit <= 1000: raise ApiError(422, "OUT_OF_RANGE", "limit must be 1..1000", "argument_range")
        types = [t for t in request.args.get("types", "").split(",") if t]
        return env({"events": device.room_events(since, until, types, limit), "retention_s": 24 * 3600})

    @app.post("/v1/perception/events")
    def perception(): return env(device.ingest(body()), 201)

    return app


def main():
    cfg = Config.from_env()
    loopback = ipaddress.ip_address(cfg.bind if cfg.bind != "localhost" else "127.0.0.1").is_loopback
    if not cfg.token and not loopback:
        sys.exit("refusing to listen on %s without GIMBAL_TOKEN (camera + motion would be open to the network)" % cfg.bind)
    if cfg.host == "mock":
        from .links import MockLink
        link = MockLink(calibrated=True)
    else:
        from .links import FirmwareLink
        link = FirmwareLink(cfg.host)
    device = Device(cfg, link)
    signal.signal(signal.SIGTERM, lambda *_: (device.close(), sys.exit(0)))
    print(f"gimbal service {cfg.device_id}: http://{cfg.bind}:{cfg.port}/v1  (firmware {cfg.host}, auth {'on' if cfg.token else 'OFF (loopback only)'})")
    try:
        create_app(device).run(host=cfg.bind, port=cfg.port, threaded=True)
    finally:
        device.close()


if __name__ == "__main__":
    main()
