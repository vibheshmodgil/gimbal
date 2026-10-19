# Gimbal device service (`host/`)

Host adapter that makes the ANVYXO gimbal a registered **device** for the robot platform's orchestrator:
validated motion, stop, home, camera status / fresh frames / stream, bounded surveys, timestamped
observations and an event stream. It talks to the unchanged firmware (`Firmware/ANVYXO_Gimbal_Phase3`, v3.4+)
over its existing WebSocket (`:81`) and MJPEG stream (`:82`). No AI packages: detectors stay in `Tracker/`.

Architecture, audit, contract proposal and fusion notes: [`../docs/DEVICE_INTEGRATION.md`](../docs/DEVICE_INTEGRATION.md).

## Run

```bash
cd host
pip install -r requirements.txt                       # flask, websocket-client (+ pytest, jsonschema for tests)

# real gimbal (PowerShell: $env:GIMBAL_HOST="192.168.1.6"; $env:GIMBAL_TOKEN="..."; python -m gimbal_service.api)
GIMBAL_HOST=192.168.1.6 GIMBAL_TOKEN=change-me python -m gimbal_service.api
# simulated gimbal + camera, no hardware
GIMBAL_HOST=mock python -m gimbal_service.api
```
Service: `http://127.0.0.1:8100/v1`. Auth: `Authorization: Bearer $GIMBAL_TOKEN` (also `X-API-Token` or
`?token=` for `<img>`/webhook clients). `/v1/health` is open. Without a token it refuses any non-loopback bind.

Only one process should own the firmware stream; the firmware has **2 stream slots**. Run the tracker on the
adapter's re-served stream and let it report presence to the adapter:
```bash
python Tracker/server.py --gimbal 192.168.1.6 \
  --stream-url "http://127.0.0.1:8100/v1/camera/stream?token=change-me" \
  --webhook    "http://127.0.0.1:8100/v1/perception/events?token=change-me"
```

## Configuration (environment)

| Variable | Default | Meaning |
|---|---|---|
| `GIMBAL_HOST` | `192.168.1.6` | firmware IP / `anvyxo.local`, or `mock` |
| `GIMBAL_DEVICE_ID` | `gimbal-01` | id used in every status, event, observation |
| `GIMBAL_TOKEN` | *(empty)* | bearer token; empty = loopback-only |
| `GIMBAL_BIND` / `GIMBAL_PORT` | `127.0.0.1` / `8100` | listen address |
| `GIMBAL_PAN_LIMIT_DEG` / `GIMBAL_TILT_LIMIT_DEG` | `90` / `45` | host cap = CAD collision-checked envelope, on top of firmware limits |
| `GIMBAL_MAX_SPEED_DPS` / `GIMBAL_DEFAULT_SPEED_DPS` | `120` / `60` | speed cap / default for API moves |
| `GIMBAL_ALLOW_UNCALIBRATED` | `0` | `1` = allow motion before the Setup wizard is finished (tests only) |
| `GIMBAL_SETTLE_S` | `0.3` | wait after the commanded slew ends before "succeeded" |
| `GIMBAL_ACK_TIMEOUT_S` | `1.0` | firmware must echo a new target within this |
| `GIMBAL_STATUS_STALE_S` / `GIMBAL_FRAME_STALE_S` | `1.0` / `1.0` | older = not ready / stale frame error |
| `GIMBAL_CAMERA_LATENCY_S` | `0.31` | exposure→host delay (measured 2026-10-13) |
| `GIMBAL_HFOV_DEG` / `GIMBAL_VFOV_DEG` | `52.8` / `0` | measured FOV metadata (0 = unknown) |
| `GIMBAL_SURVEY_MAX_POINTS` / `GIMBAL_SURVEY_SETTLE_S` | `60` / `0.8` | survey bounds |
| `GIMBAL_MAX_OBSERVATIONS` | `200` | observations (with JPEG) kept in memory |
| `GIMBAL_PERSIST_DIR` | *(empty)* | empty = memory only (privacy default); else JPEG+JSON per observation |
| `GIMBAL_RETENTION_HOURS` | `24` | persisted observations older than this are deleted |
| `GIMBAL_LOPSIDED_DEG` | `30` | servo-reach asymmetry reported as `LOPSIDED_MOUNT` |

## API (v1, contract proposal 0.1, JSON Schemas in `contracts/device_v0.schema.json`)

Every response carries `X-Request-ID` (yours if sent, else generated). Errors are always
`{"error": {"code", "message", "details", "request_id"}}`.

| Method + path | Does | Success |
|---|---|---|
| `GET /v1/health` | liveness + readiness (open) | 200 `health/v0` |
| `GET /v1/device` | descriptor: axes, sensors, frames, capabilities, **unsupported** list | 200 |
| `GET /v1/status` | connection, readiness, calibration, pose, limits, motion, presence, camera, faults | 200 `device_status/v0` |
| `POST /v1/motion/move` | `{mode: absolute\|relative, pan_deg, tilt_deg, speed_dps?, preempt?}` | 202 `operation/v0` |
| `POST /v1/motion/home` | to calibrated zero `{speed_dps?, preempt?}` | 202 operation |
| `POST /v1/motion/stop` | cancel active op, firmware freezes in place; reports `confirmed_by_device` | 200 |
| `POST /v1/survey` | `{waypoints:[[pan,tilt],..]}` or `{grid:{pan_from_deg,pan_to_deg,pan_step_deg,tilts_deg}}`, `restore: previous\|home\|none`, `settle_s`, `speed_dps` | 202 operation |
| `GET /v1/operations[/{id}]`, `POST /v1/operations/{id}/cancel` | lifecycle, progress, result, error | 200 |
| `GET /v1/camera/status` | ready, stream fps/error, frame age, size, sensor, config, timing | 200 |
| `POST /v1/camera/config` | `{resolution: "640x480", fps_cap: 1..30}` (live, not saved) | 202 |
| `POST /v1/observations` | **fresh** frame (exposed after the request) → `observation/v0` JSON + image ref | 201 |
| `GET /v1/camera/snapshot?fresh=1` | fresh JPEG; `fresh=0&max_age_ms=` = newest if young enough | 200 image/jpeg |
| `GET /v1/observations/{id}[/image]` | stored observation / its JPEG | 200 |
| `GET /v1/camera/stream` | firmware MJPEG re-served unchanged (no re-encode) | 200 multipart |
| `GET /v1/events` | SSE, resume with `Last-Event-ID`; `?once=1&after=seq` = JSON backlog | 200 |
| `POST /v1/perception/events` | detector input (tracker webhook or `{type, data}`) | 201 event |

Operation states: `accepted → running → succeeded | failed | cancelled | preempted`. `device_ack_at` = the
firmware echoed the target (accepted by the **device**); `succeeded` = commanded slew finished + settle.
**No position sensor exists**: poses are `firmware_commanded_open_loop`, `physically_verified: false`.

Error codes: `INVALID_ARGUMENT OUT_OF_RANGE OUT_OF_LIMITS UNKNOWN_FIELD BAD_JSON UNAUTHORIZED NOT_FOUND BUSY
NOT_CALIBRATED CALIBRATION_IN_PROGRESS MOTORS_DISABLED DEVICE_UNAVAILABLE CAMERA_UNAVAILABLE FRAME_STALE
FRAME_TIMEOUT UNSUPPORTED_EVENT` (HTTP) and, inside operations, `COMMAND_NOT_ACKNOWLEDGED MOTION_TIMEOUT
LINK_LOST PREEMPTED_EXTERNALLY CANCELLED PREEMPTED SURVEY_INCOMPLETE INTERNAL_ERROR`.

## Test

```bash
cd host
python -m pytest -q                                                   # L2-4: unit + mock device + schema (44 tests)
GIMBAL_HOST=mock GIMBAL_TOKEN=m GIMBAL_PORT=8101 python -m gimbal_service.api &
python client.py --url http://127.0.0.1:8101 --token m hardware --pan 10 --tilt 5     # L6 over HTTP, mock
# L5/L8 REAL hardware (moves the head +-pan deg around where it is; keep hands clear):
python client.py --url http://127.0.0.1:8100 --token $GIMBAL_TOKEN hardware --pan 5
python client.py --url ... acceptance            # the orchestrator acceptance test: status, move, fresh observation
```
Recorded results: `test_results/` (see `docs/DEVICE_INTEGRATION.md` §7).

## Troubleshooting

- `DEVICE_DISCONNECTED`: gimbal off / new IP. `arp -a | findstr e0-72-a1-f6-30-80`, then `GIMBAL_HOST=<ip>`.
- `CAMERA_UNAVAILABLE` with `HTTP Error 503`: both firmware stream slots busy. Close gimbal-page browser tabs,
  point the tracker at `/v1/camera/stream`.
- `NOT_CALIBRATED`: run the Setup tab in the tracker UI (mount → direction → zero → limits → Finish).
- `LOPSIDED_MOUNT` warning: the servo horn is off-centre; Setup step 1 (Mount) fixes it.
- `PREEMPTED_EXTERNALLY`: the tracker or a browser moved the gimbal during an API operation. Stop tracking first.
- OTA firmware updates fail while a stream is open: stop this service and the tracker first.
