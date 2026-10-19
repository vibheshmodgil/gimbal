"""HTTP/JSON + SSE service for the gimbal.   python -m gimbal_service.api   (config: GIMBAL_* env vars, see README)"""
import hmac, ipaddress, json, signal, sys, time, uuid

from flask import Flask, Response, g, jsonify, request

from .core import ApiError, Config, Device, RESOLUTIONS, iso, num


def create_app(device):
    app, cfg = Flask(__name__), device.cfg

    @app.before_request
    def _auth():
        g.rid = request.headers.get("X-Request-ID") or (request.get_json(silent=True) or {}).get("request_id") or f"req_{uuid.uuid4().hex[:16]}"
        if request.path == "/v1/health" or not cfg.token: return None
        h = request.headers.get("Authorization", "")
        tok = h[7:] if h.startswith("Bearer ") else request.headers.get("X-API-Token") or request.args.get("token", "")
        if not hmac.compare_digest(tok.encode(), cfg.token.encode()):
            return _err(ApiError(401, "UNAUTHORIZED", "missing or wrong bearer token"))

    @app.after_request
    def _rid(r):
        r.headers["X-Request-ID"] = g.get("rid", "")
        return r

    def _err(e):
        return jsonify(e.body(g.get("rid"))), e.http

    app.register_error_handler(ApiError, _err)
    app.register_error_handler(404, lambda e: _err(ApiError(404, "NOT_FOUND", f"no endpoint {request.method} {request.path}")))
    app.register_error_handler(405, lambda e: _err(ApiError(405, "METHOD_NOT_ALLOWED", f"{request.method} not allowed on {request.path}")))

    def body():
        d = request.get_json(silent=True)
        if d is None and request.data: raise ApiError(400, "BAD_JSON", "body is not valid JSON")
        if d is not None and not isinstance(d, dict): raise ApiError(400, "BAD_JSON", "body must be a JSON object")
        return d or {}

    def op_resp(op):
        return jsonify(op.to_dict() | {"links": {"self": f"/v1/operations/{op.operation_id}", "cancel": f"/v1/operations/{op.operation_id}/cancel"}}), 202

    @app.get("/v1/health")
    def health():
        reasons = device.ready_reasons()
        return jsonify({"schema": "health/v0", "device_id": cfg.device_id, "time": iso(time.time()), "alive": True,
                        "ready": not reasons, "not_ready_reasons": reasons}), 200

    @app.get("/v1/device")
    def descriptor(): return jsonify(device.descriptor())

    @app.get("/v1/status")
    def status(): return jsonify(device.status())

    @app.post("/v1/motion/move")
    def move(): return op_resp(device.move(body(), g.rid))

    @app.post("/v1/motion/home")
    def home(): return op_resp(device.home(body(), g.rid))

    @app.post("/v1/motion/stop")
    def stop(): return jsonify(device.stop(g.rid))

    @app.post("/v1/survey")
    def survey(): return op_resp(device.survey(body(), g.rid))

    @app.get("/v1/operations")
    def ops(): return jsonify({"operations": [o.to_dict() for o in list(device.ops.values())[-50:]]})

    @app.get("/v1/operations/<oid>")
    def op_get(oid):
        op = device.ops.get(oid)
        if not op: raise ApiError(404, "NOT_FOUND", f"no operation {oid}")
        return jsonify(op.to_dict())

    @app.post("/v1/operations/<oid>/cancel")
    def op_cancel(oid): return jsonify(device.cancel(oid, g.rid).to_dict())

    @app.get("/v1/camera/status")
    def cam_status(): return jsonify(device.camera_status())

    @app.post("/v1/camera/config")
    def cam_config():
        d, sent = body(), []
        if "resolution" in d:
            if d["resolution"] not in RESOLUTIONS: raise ApiError(422, "INVALID_ARGUMENT", "resolution must be one of " + ", ".join(RESOLUTIONS))
            sent.append(f"CAM:res,{RESOLUTIONS.index(d['resolution'])}")
        if "fps_cap" in d: sent.append(f"CAM:fps,{int(num(d, 'fps_cap', lo=1, hi=30))}")
        if not sent: raise ApiError(422, "INVALID_ARGUMENT", "give resolution and/or fps_cap")
        for c in sent + ["CAMGET"]:
            if not device.link.send(c): raise ApiError(503, "DEVICE_UNAVAILABLE", "firmware not connected")
        return jsonify({"sent": sent, "note": "applied live, not saved to flash; check /v1/camera/status"}), 202

    @app.get("/v1/camera/snapshot")
    def snapshot():
        """JPEG of a frame exposed after this request (fresh=1, default) or the newest one (fresh=0, max_age_ms)."""
        fresh = request.args.get("fresh", "1") != "0"
        max_age = num({"m": float(request.args.get("max_age_ms", cfg.frame_stale_s * 1000))}, "m", lo=0, hi=60000) / 1000
        o = device.observe(after_t=time.time() + cfg.camera_latency_s if fresh else None, max_age_s=max_age, source={"request_id": g.rid})
        _, jpg = device.get_obs(o["observation_id"])
        return Response(jpg, mimetype="image/jpeg", headers={"X-Observation-ID": o["observation_id"], "X-Timestamp": o["timestamp"],
                                                             "X-Received-At": o["received_at"], "Cache-Control": "no-store"})

    @app.post("/v1/observations")
    def observe():
        """Fresh camera observation (JSON metadata + image ref): the acceptance-test call."""
        d = body()
        fresh = d.get("fresh", True)
        o = device.observe(after_t=time.time() + cfg.camera_latency_s if fresh else None, source={"request_id": g.rid, **(d.get("source") or {})})
        return jsonify(o), 201

    @app.get("/v1/observations/<oid>")
    def obs_get(oid): return jsonify(device.get_obs(oid)[0])

    @app.get("/v1/observations/<oid>/image")
    def obs_img(oid): return Response(device.get_obs(oid)[1], mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

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

    @app.get("/v1/events")
    def events():
        """SSE. Resume with Last-Event-ID (= seq). ?once=1 returns the backlog as JSON (polling clients, tests)."""
        after = int(request.headers.get("Last-Event-ID") or request.args.get("after") or 0)
        if request.args.get("once"): return jsonify({"events": device.bus.since(after)})
        def gen():
            seq = after
            while True:
                evs = device.bus.since(seq, timeout=15)
                if not evs: yield ": keep-alive\n\n"
                for e in evs:
                    seq = e["seq"]
                    yield f"id: {e['seq']}\nevent: {e['type']}\ndata: {json.dumps(e)}\n\n"
        return Response(gen(), mimetype="text/event-stream", headers={"Cache-Control": "no-cache"})

    @app.post("/v1/perception/events")
    def perception(): return jsonify(device.ingest(body())), 201

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
