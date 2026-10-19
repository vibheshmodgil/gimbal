"""Reference client for the orchestrator (stdlib only) + the acceptance / hardware-validation run.

    python client.py --url http://127.0.0.1:8100 [--token T] acceptance          status -> small move -> fresh observation
    python client.py ... hardware --pan 10 --tilt 5                              + stop mid-move, short survey, limit rejection
Both print one JSON report. Moves stay within +-pan/+-tilt of the CURRENT pose and inside the service's limits.
"""
import argparse, json, time, urllib.error, urllib.request, uuid


class GimbalClient:
    def __init__(self, url, token="", timeout=10):
        self.url, self.token, self.timeout = url.rstrip("/"), token, timeout

    def call(self, method, path, body=None, raw=False):
        h = {"X-Request-ID": f"req_{uuid.uuid4().hex[:16]}"}
        if self.token: h["Authorization"] = f"Bearer {self.token}"
        data = None if body is None else json.dumps(body).encode()
        if data: h["Content-Type"] = "application/json"
        req = urllib.request.Request(self.url + path, data, h, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                b = r.read()
                return r.status, (b, dict(r.headers)) if raw else json.loads(b)
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    def wait(self, op_id, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            _, op = self.call("GET", f"/v1/operations/{op_id}")
            if op.get("state") not in ("accepted", "running"): return op
            time.sleep(0.1)
        raise TimeoutError(op_id)

    def run(self, method, path, body=None, timeout=30):
        code, op = self.call(method, path, body)
        return (code, self.wait(op["operation_id"], timeout)) if code == 202 else (code, op)


def acceptance(c, pan=5.0, tilt=0.0):
    rep = {}
    rep["health"] = c.call("GET", "/v1/health")
    code, st = c.call("GET", "/v1/status")
    rep["status_before"] = {"code": code, "ready": st.get("ready"), "pose": st.get("pose"), "limits": st.get("limits"), "faults": st.get("faults")}
    code, op = c.run("POST", "/v1/motion/move", {"mode": "relative", "pan_deg": pan, "tilt_deg": tilt, "speed_dps": 40})
    rep["move"] = {"code": code, "state": op.get("state"), "error": op.get("error"), "device_ack_at": op.get("device_ack_at"),
                   "pose": (op.get("result") or {}).get("pose")}
    code, o = c.call("POST", "/v1/observations", {})
    rep["observation"] = {"code": code, **({k: o.get(k) for k in ("observation_id", "timestamp", "received_at", "pose", "validity", "data")} if code == 201 else o)}
    if code == 201:
        code, (img, _) = c.call("GET", o["data"]["ref"], raw=True)
        rep["image"] = {"code": code, "bytes": len(img), "jpeg": img[:2] == b"\xff\xd8"}
    _, back = c.run("POST", "/v1/motion/move", {"mode": "relative", "pan_deg": -pan, "tilt_deg": -tilt, "speed_dps": 40})
    rep["move_back"] = back.get("state")
    rep["passed"] = rep["move"]["state"] == "succeeded" and rep.get("image", {}).get("jpeg", False)
    return rep


def hardware(c, pan, tilt):
    rep = acceptance(c, pan, tilt)
    _, st = c.call("GET", "/v1/status")
    p0, lim = st["pose"], st["limits"]
    # stop: start a slow long move, stop after 0.5 s, pose must hold
    tgt = max(lim["pan_deg"][0], min(lim["pan_deg"][1], p0["pan_deg"] + 3 * pan))
    code, op = c.call("POST", "/v1/motion/move", {"pan_deg": tgt, "tilt_deg": p0["tilt_deg"], "speed_dps": 10})
    time.sleep(0.5)
    _, stop = c.call("POST", "/v1/motion/stop")
    time.sleep(0.6)
    _, st2 = c.call("GET", "/v1/status")
    rep["stop"] = {"op_state": c.wait(op["operation_id"])["state"], "stopped_at_pan": stop.get("pose", {}).get("pan_deg"),
                   "pan_after_0_6s": st2["pose"]["pan_deg"], "target": tgt}
    # limit: one degree past the pan limit must be refused without motion
    code, err = c.call("POST", "/v1/motion/move", {"pan_deg": lim["pan_deg"][1] + 1, "tilt_deg": 0})
    rep["limit_rejection"] = {"code": code, "error": err.get("error", {}).get("code")}
    # short survey around the start pose, back to where it began
    pts = [[p0["pan_deg"] - pan, p0["tilt_deg"]], [p0["pan_deg"], p0["tilt_deg"]], [p0["pan_deg"] + pan, p0["tilt_deg"]]]
    code, sv = c.run("POST", "/v1/survey", {"waypoints": pts, "speed_dps": 40, "restore": "previous"}, timeout=60)
    rep["survey"] = {"code": code, "state": sv.get("state"), "error": sv.get("error"),
                     "steps": [(s["status"], s["observation_id"]) for s in (sv.get("result") or {}).get("steps", [])]}
    _, evs = c.call("GET", "/v1/events?once=1")
    rep["event_types_seen"] = sorted({e["type"] for e in evs.get("events", [])})
    rep["passed"] = (rep["passed"] and rep["stop"]["op_state"] == "cancelled" and rep["limit_rejection"]["error"] == "OUT_OF_LIMITS"
                     and rep["survey"]["state"] == "succeeded")
    return rep


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8100")
    ap.add_argument("--token", default="")
    ap.add_argument("what", choices=["acceptance", "hardware"])
    ap.add_argument("--pan", type=float, default=5.0)
    ap.add_argument("--tilt", type=float, default=0.0)
    a = ap.parse_args()
    c = GimbalClient(a.url, a.token)
    print(json.dumps(acceptance(c, a.pan, a.tilt) if a.what == "acceptance" else hardware(c, a.pan, a.tilt), indent=1))
