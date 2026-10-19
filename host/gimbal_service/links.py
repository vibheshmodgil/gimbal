"""Transport to the gimbal: the real ANVYXO firmware (WebSocket :81 + MJPEG :82) or a mock with the same interface.

Both links expose:
    connected, status (firmware status dict), status_t (host time it arrived), cam (camera JSON)
    send(cmd) -> bool                        fire-and-forget firmware command (see firmware HELP)
    frame() -> (jpeg, t_recv, seq)           newest JPEG and the host time it arrived
    wait_frame(after_seq, timeout) -> same   first frame newer than after_seq (or newest on timeout)
    stream_ok, stream_err, fps, viewers      stream health; viewers = extra clients reading our re-served stream
    close()
The firmware has no position feedback: status "pan"/"tilt" is its own smoothed COMMAND, not a measured angle.
"""
import base64, json, threading, time, urllib.request
from collections import deque


class _Frames:
    """Newest-frame buffer shared by both links; old frames are only lag."""
    def __init__(self):
        self.jpg, self.t, self.seq, self.fps = None, 0.0, 0, 0.0
        self.cond, self._times, self.ring = threading.Condition(), deque(maxlen=30), deque(maxlen=8)

    def push(self, jpg):
        now = time.time()
        self._times.append(now)
        self.fps = (len(self._times) - 1) / max(self._times[-1] - self._times[0], 1e-3)
        with self.cond:
            self.jpg, self.t, self.seq = jpg, now, self.seq + 1
            self.ring.append((jpg, now, self.seq))   # frames arrive in TCP bursts: re-streaming must not skip them
            self.cond.notify_all()

    def frames_after(self, seq):
        with self.cond: return [f for f in self.ring if f[2] > seq]

    def frame(self):
        with self.cond: return self.jpg, self.t, self.seq

    def wait_frame(self, after_seq, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after_seq, timeout=timeout)
            return self.jpg, self.t, self.seq


class FirmwareLink(_Frames):
    def __init__(self, host, ws_port=81, stream_port=82):
        super().__init__()
        self.host, self.ws_url, self.stream_url = host, f"ws://{host}:{ws_port}/", f"http://{host}:{stream_port}/stream"
        self.connected, self.status, self.status_t, self.cam, self.diag = False, {}, 0.0, {}, {}
        self.stream_ok, self.stream_err, self.viewers, self._ws, self._stop = False, "connecting", 0, None, False
        threading.Thread(target=self._ws_loop, daemon=True, name="gimbal-ws").start()
        threading.Thread(target=self._stream_loop, daemon=True, name="gimbal-stream").start()

    def _ws_loop(self):
        import websocket
        while not self._stop:
            self._ws = websocket.WebSocketApp(self.ws_url, on_open=self._open, on_message=self._msg)
            self._ws.run_forever(ping_interval=10, ping_timeout=5)
            self.connected = False
            time.sleep(1)

    def _open(self, _):
        self.connected = True
        self.send("CAMGET")
        self.send("DIAG")   # firmware version etc.

    def _msg(self, _, m):
        if not m.startswith("{"): return
        try: j = json.loads(m)
        except ValueError: return
        if j.get("t") == "s": self.status, self.status_t = j, time.time()
        elif j.get("t") == "c": self.cam = j
        elif j.get("t") == "d": self.diag = j

    def send(self, cmd):
        try:
            self._ws.send(cmd)
            return True
        except Exception:
            return False

    def _stream_loop(self):
        while not self._stop:
            try:
                r, buf = urllib.request.urlopen(self.stream_url, timeout=5), b""
                self.stream_ok, self.stream_err = True, ""
                while not self._stop:
                    chunk = r.read(8192)
                    if not chunk: raise IOError("stream closed")
                    buf += chunk
                    while True:
                        a = buf.find(b"\xff\xd8")
                        if a < 0: buf = buf[-1:]; break
                        b = buf.find(b"\xff\xd9", a + 2)
                        if b < 0: buf = buf[a:]; break
                        self.push(buf[a:b + 2]); buf = buf[b + 2:]
            except Exception as e:   # 503 = both firmware stream slots taken (close a browser tab on the gimbal page)
                self.stream_ok, self.stream_err, self.fps = False, str(e)[:120], 0.0
                time.sleep(1)

    def close(self):
        self._stop = True
        try: self._ws.close()
        except Exception: pass


# 64x48 grey-blue JPEG: the mock camera's frame
MOCK_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA0JCgsKCA0LCgsODg0PEyAVExISEyccHhcgLikxMC4pLSwzOko+MzZGNywtQFdBRkxOUlNSMj5aYVpQYEpRUk//2wBDAQ4ODhMREyYV"
    "FSZPNS01T09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0//wAARCAAwAEADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcI"
    "CQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlq"
    "c3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAEC"
    "AwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpj"
    "ZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwCjRRRX"
    "lHrBRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAf/2Q==")


class MockLink(_Frames):
    """Simulated firmware: same status keys and command semantics (soft limits, servo reach, constant-speed slew,
    MODE:MANUAL = freeze). Fault switches for tests: online, camera, stall (servo target never reached), deaf
    (commands ignored)."""
    def __init__(self, calibrated=True, fps=15.0, lp=(-90.0, 90.0), lt=(-45.0, 45.0), status_interval=0.15):
        super().__init__()
        self.host, self.online, self.camera, self.stall, self.deaf = "mock", True, True, False, False
        self.cam, self.diag = {"t": "c", "sensor": "MOCK", "ok": True, "res": 3, "fps": 25, "q": 12}, {"t": "d", "fw": "mock"}
        self._s = dict(t="s", cal=calibrated, calOn=False, invP=False, invT=False, lp=list(lp), lt=list(lt),
                       rp=[-90.0, 90.0], rt=[-90.0, 90.0], pan=0.0, tilt=0.0, tp=0.0, tt=0.0, laser=False, mode="MANUAL",
                       speed=120.0, trimPan=0, trimTilt=0, panEn=True, tiltEn=True, fps=fps)
        self.status, self.status_t, self.sent, self._fps, self._stop = dict(self._s), time.time(), [], fps, False
        self.viewers, self._status_iv = 0, status_interval   # firmware STATUS_INTERVAL_MS = 150
        threading.Thread(target=self._run, daemon=True, name="mock-gimbal").start()

    @property
    def connected(self): return self.online

    @property
    def stream_ok(self): return self.online and self.camera

    @property
    def stream_err(self): return "" if self.stream_ok else "mock camera offline"

    def _clamp(self, v, ax):
        lo, hi = self._s["l" + ax]
        rlo, rhi = self._s["r" + ax]
        if self._s["calOn"]: lo, hi = -999, 999
        return max(rlo, min(rhi, max(lo, min(hi, v))))

    def send(self, cmd):
        if not self.online: return False
        self.sent.append(cmd)
        if self.deaf: return True
        s, c = self._s, cmd.strip().upper()
        if c.startswith("PT:"): p, t = map(float, c[3:].split(",")); s["tp"], s["tt"], s["mode"] = self._clamp(p, "p"), self._clamp(t, "t"), "MANUAL"
        elif c.startswith("MV:"): p, t = map(float, c[3:].split(",")); s["tp"], s["tt"] = self._clamp(s["tp"] + p, "p"), self._clamp(s["tt"] + t, "t")
        elif c == "C": s["tp"], s["tt"] = self._clamp(0, "p"), self._clamp(0, "t")
        elif c == "MODE:MANUAL": s["tp"], s["tt"], s["mode"] = s["pan"], s["tilt"], "MANUAL"
        elif c.startswith("SPD:"): s["speed"] = max(10.0, min(360.0, float(c[4:])))
        elif c.startswith("SRV:") and len(c) >= 7: s["panEn" if c[4] == "P" else "tiltEn"] = c[6] == "1"
        elif c.startswith("CAM:"):
            k, v = cmd[4:].split(","); self.cam[k.lower()] = int(v)
        return True

    def _run(self):
        last_frame, last_status, last = 0.0, 0.0, time.time()
        while not self._stop:
            time.sleep(0.02)
            now = time.time(); dt, last = now - last, now
            s, step = self._s, self._s["speed"] * dt
            if not self.stall:
                for a, tg in (("pan", "tp"), ("tilt", "tt")):
                    s[a] += max(-step, min(step, s[tg] - s[a]))
            if self.online and now - last_status >= self._status_iv: self.status, self.status_t, last_status = dict(s), now, now
            if self.online and self.camera and now - last_frame >= 1 / self._fps:
                last_frame = now; self.push(MOCK_JPEG)

    def close(self): self._stop = True
