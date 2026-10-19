# Gimbal as a device subsystem: audit, architecture, contract, results (2026-10-19)

Branch `feature/device-adapter` in this repo (local git created 2026-10-19; baseline checkpoint `f20347f` on `main`).
Service code and operator docs: [`host/`](../host/README.md).

## 1. Audit: what the hardware and firmware actually do

| Item | Finding (from the code + live device) |
|---|---|
| Axes | pan (MG996R, base) and tilt (MG90S, yoke). 2 DOF. |
| Drive | LEDC 50 Hz PWM, 500–2500 µs = servo 0–180°; `servo = 90 + dir·(angle + trim)` (`gimbal.cpp`). |
| Feedback | **None.** Hobby servos, no encoder/IMU. Firmware `pan/tilt` = its own smoothed *command*. Tilt linkage moves ~70% of command with ~3.4° backlash (2026-10-13 motion test). |
| Motion | constant-speed slew toward target, 10–360 °/s; **no acceleration limit**. Last command wins. `MODE:MANUAL` freezes target at current position. |
| Limits | firmware soft limits `lp/lt` (flash) + new servo-reach clamp (fw 3.4) + host CAD cap ±90/±45 (`gimbal.py` collision check). |
| Calibration | wizard: mount (new in 3.4), direction, zero, limits, done; saved in flash. **Live device: NOT calibrated; pan horn ~73° off-centre (reach 17° vs 163°).** |
| Homing | no sensor homing; "home" = calibrated zero. Boot = servos to zero immediately (firmware, unchanged). |
| Safety in firmware | limits clamp, speed clamp, laser 60 s auto-off and off when no viewer, BOOT long-press = centre + laser off. |
| Camera | OV3660 on ESP32-S3 (8 MB PSRAM), JPEG, MJPEG on `:82` in its own task, **max 2 stream clients**, no single-frame endpoint, no capture timestamp in frames. Live: 320×240, 20–26 fps. Measured exposure→host latency 0.31 s. |
| Control channel | WebSocket `:81` text commands (`HELP`), status JSON every 150 ms, `/cmd?c=` HTTP, OTA (`anvyxo`). |
| Host apps | `Tracker/server.py` (Flask :8000, YOLO11n person tracking, PID, laser safety, UI, small BOB API + presence webhook). |
| Assistant repo | `Desktop/Claude/SHIV_AI-shiv_claude_vibhesh` (github `vibheshmodgil/robo_test`, master only, **uncommitted user changes → not touched**). Its `vision_service/.../esp32_controller.py` targets a *different* motor board (`POST :80/move`, UDP 4210). **No canonical device/observation contract exists there or on the remote**; no `AI_Assistant/` monorepo exists locally. `gh` is not authenticated, so no PR/issue channel was available. |

## 2. Architecture

```
 orchestrator (other session)  ──HTTP/JSON + SSE──▶  host/gimbal_service  ──WS :81 / MJPEG :82──▶  firmware 3.4 (real-time: slew, limits, laser)
                                                    │  validation, ops, stop,                      ▲
 Tracker/server.py (YOLO, PID) ──/v1/camera/stream──┘  observations, events,       (tracker still drives PT for closed-loop tracking)
                               ──webhook──▶ /v1/perception/events
```
- `links.py`: `FirmwareLink` (real) and `MockLink` (same interface, firmware-like 150 ms status, fault switches).
- `core.py`: config, validation, operations, survey, observations, events, monitor. `api.py`: Flask routes.
- `perception.py`: detector input, plus the reserved entry/exit plug-in point.
- Physical control never depends on the PC: the firmware finishes or holds its last target if the host or link dies.

## 3. Motion safety (device-side, not prompt-side)

- **Validation:** finite numbers only, unknown fields rejected, targets outside `limits()` = firmware soft ∩ servo
  reach ∩ CAD cap → `422 OUT_OF_LIMITS` (never silently clamped), speed 10..`MAX_SPEED`.
- **Gating:** no motion when disconnected or status is stale (`503`), calibrating, motors limp, or not calibrated
  (unless `GIMBAL_ALLOW_UNCALIBRATED=1`).
- **One motion operation at a time:** `409 BUSY`, or `preempt: true` (the old one ends `preempted`).
  Other clients (tracker/UI) changing the target mid-operation → `PREEMPTED_EXTERNALLY`.
- **Accepted vs done:** `202` = accepted by the service; `device_ack_at` = the firmware status echoed the target
  (≤1 s, else `COMMAND_NOT_ACKNOWLEDGED`); `succeeded` = commanded slew finished + settle. Never "physically verified".
- **Timeouts:** travel time ×1.5 + 2 s, then the service stops it (`MOTION_TIMEOUT`). Link loss → `LINK_LOST`.
- **Stop:** always accepted; cancels the operation, firmware freezes; the reply waits for a post-stop status
  (`confirmed_by_device`). Hardware: stop pose = pose 0.6 s later (5.0° = 5.0°).
- **Startup/shutdown:** the service never moves on start; on shutdown it stops only its own operation.
- **Not done:** acceleration limiting (firmware has none; the speed cap is the protection).

## 4. Contract proposal (until the orchestrator's canonical one exists)

Schemas: `host/contracts/device_v0.schema.json` (`health/v0`, `device_status/v0`, `device_descriptor/v0`,
`operation/v0`, `observation/v0`, `event/v0`, `error`). Every test validates responses against it.

**Designed to be device-generic** (RC car later = another device with the same envelope):
- `device_id`, `sensor_id` (`gimbal-01/camera0`), `sensor_type` enum already includes `lidar.2d|lidar.3d|imu|odometry`.
- Observation = metadata + `data.ref` (image URL), never inline pixels; `sha256`, size, media type.
- `pose.representation = "pan_tilt_relative"` in frame `gimbal-01/pan_tilt_base`; `global_pose: null`. No claim of
  relation to `base_link`/`map`. A car would publish `representation: "se3"` in its own frame with its own source.
- `timestamp` = host receive time − measured latency; `timing` says so; host clock only (no firmware clock sync).
- `validity` (`valid`, `stale`, `pose_at_rest`, `reasons`), `confidence`, `processing{status, model, model_version, results}`,
  `source{survey_id|operation_id|event|request_id, step_index}`.

Frames (for the shared `coordinate_frames/`): `gimbal-01/pan_tilt_base`: origin at the pan/tilt axis intersection,
zero = calibrated straight-ahead + level, pan + = right seen from behind, tilt + = up. To REP-103 (x fwd, y left, z up):
`yaw = −pan`, `pitch = −tilt`. Camera optical frame offset from the axes is in `gimbal.py` (`CAM_Z`, `Y_FRONT`);
not yet published as a transform.

## 5. Fusion, presence, mapping: what this device does and doesn't do

- Provides raw, timestamped, source-labelled camera observations and pan/tilt pose. Fusion with RC-car camera,
  LiDAR, odometry or IMU belongs to the central perception/mapping service. That needs: lens intrinsics +
  distortion (not measured), camera↔pan/tilt extrinsics, the gimbal's placement in the room/map frame
  (needs localisation or manual survey), and clock sync (NTP on the PC; the firmware has no capture clock).
- A survey = frames from several directions with poses. **Not a map**: no depth, no metric geometry.
- Presence: `perception.person_visible / person_not_visible` (YOLO11n tracker, real) and `presence_unknown`.
  `identity` is always `null`: generic detection never says *who*.
- Entry/exit: **reserved, rejected (`UNSUPPORTED_EVENT`)**. Needs a doorway in a fixed calibrated view, a door
  zone, crossing direction over time, ideally a second sensor. Plug-in shape is in `perception.py`.
- Scene interpretation, mapping, long-term memory: not here, by design.

## 6. Deployment / edge

- PC (Windows) today: `host/` needs only Flask + websocket-client (pure Python, no Windows APIs; runs unchanged on a
  Jetson/Linux). AI stays in `Tracker/` (torch/ultralytics) and can stay on the PC while `host/` runs on an edge box.
- Firmware is standalone (ESP32): it keeps limits and holds position with no host.
- Monorepo path (when `AI_Assistant/` exists): `git subtree add --prefix devices/gimbal <this repo> main` keeps
  history and the Arduino/CAD build untouched; move `host/contracts/*` to `shared/contracts/` once agreed.

## 7. Test results (2026-10-19, exact commands in `host/README.md`)

| Level | What | Result | Real or mocked |
|---|---|---|---|
| 1 | firmware `arduino-cli compile` (esp32s3, 8M OPI) | OK, 38% flash | real build |
| 1 | `pyflakes gimbal_service tests client.py`, `compileall` | clean | static |
| 1 | `python Tracker/server.py --selftest` | ok | unit |
| 2–4 | `python -m pytest -q`: validation, limits, ack/completion, BUSY/preempt, stop, cancel, stall timeout, deaf firmware, link loss, external takeover, calibration gating, fresh/stale frames, survey (grid, cancel→partial, capture failure→partial), events, perception ingest, auth, persistence/retention, re-stream bursts, all schema-validated | **44 passed**, 10× repeat stable after fixes | **mock** |
| 6 | real HTTP server (mock link) driven by `client.py hardware` | passed (`test_results/mock_http_validation.json`) | mock |
| 5 | camera status + fresh observation + snapshot from the real OV3660 | 320×240 JPEG, frame age ~130 ms, 20–26 fps; re-served stream 25.8 fps vs 26.2 in | **real** |
| 7/8 | `client.py hardware --pan 5` on the real gimbal (uncalibrated, allowed by env): relative move + back, fresh observation, stop mid-move, limit rejection, 3-point survey with restore, home | **passed** (`test_results/REAL_hardware_validation_2026-10-19.json`) | **real motion**, open-loop: commanded poses only, not measured |
| 9 | assistant → gimbal end to end | **not run**: no orchestrator exists yet | — |

Bugs found by the hardware run and fixed: stop reported a pose up to 150 ms stale (1.6° off); the re-served stream
dropped ~50% of frames (TCP bursts); unknown request fields were silently ignored; pose-at-exposure had no
sample with 150 ms status spacing. The mock now uses the firmware's 150 ms status timing.

## 8. Capability matrix

- **Real + tested on hardware:** status, readiness/faults (including the lopsided mount), absolute/relative move, home,
  stop, limit rejection, fresh frame + metadata, MJPEG re-stream, 3-point survey, events.
- **Mock-tested only:** timeouts, deaf firmware, link loss mid-move, external takeover, camera loss mid-survey,
  calibration gating, perception ingest, persistence/retention, camera config.
- **Unverified:** position accuracy (no sensor; tilt is known to under-travel ~30%), long-run stability, tracker
  running on `--stream-url` with real detections, behaviour after the horn re-mount and calibration.
- **Unsupported (by hardware or not built):** closed-loop position, acceleration limits, lens intrinsics, global
  pose, entry/exit, scene interpretation, mapping, owner identity, laser via API (left with the tracker's safety logic).

## 9. For the orchestrator session

Not accepted by anyone yet: this is a proposal. To align: give me the canonical device-registry and observation
contract (file, branch or doc). Field renames are mechanical (`core.py` builders + schema). Registration payload =
`GET /v1/device`. Before physical use the gimbal needs: horn re-mount (Setup step 1) and calibration (Setup steps 2–5).
