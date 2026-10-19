# Person tracker: tuned settings — 13 October 2026

Everything below is saved and loads automatically after a power cycle:

| What | Where it lives | Loaded by |
|---|---|---|
| PID gains, detector, motion settings | `Tracker/params.json` | `Tracker/server.py` on start (UI **Save settings** rewrites it) |
| Camera FOV, directions, latency, deadbands (measured) | `Tracker/calibration.json` | `server.py` defaults; `params.json` overrides |
| Camera picture settings | gimbal flash (`CAMSAVE`) | gimbal on boot |
| Camera orientation defaults | `Firmware/.../config.h` (`CAMERA_VFLIP 1`, `CAMERA_HMIRROR 1`) | only after a factory reset / reflash |

Start: `python Tracker/server.py`, open http://localhost:8000, close any gimbal-UI tab first (the gimbal serves 2 video viewers). Tracking always starts OFF.

## Tracker settings (`params.json`)

| Setting | Pan | Tilt | Why |
|---|---|---|---|
| Kp / Ki / Kd | 2.5 / 0.8 / 0.35 | 2.5 / 0 / 0.3 | grid re-tune on the GPU (below); tilt Ki off (backlash) |
| Deadband | 0.5 deg | 2.5 deg | tilt has 3.4 deg backlash, smaller makes it hunt |
| Feed-forward | 0.5 | | cuts walking lag 5.4 -> ~1.6 deg |
| Smoothing (speed estimate) | 0.75 | | at 12 detections/s, 0.5 made feed-forward oscillate (16-22 deg overshoot) |
| Latency | 0.31 s | | measured command -> picture delay |
| Max speed | 45 deg/s | | |
| Detector | YOLO11n person, 320 px, conf 0.35, 15/s on the GTX 1650 | | 416/480 px and conf 0.30 found the person *less* often on this camera |
| Lost lock | hold 1.2 s, re-acquire within 4 s, release 4 s, search after 3 s | | |
| FOV | 52.8 deg H, 56.8 deg tilt-command | | OV3660, measured by `Tracker/motion_test.py` |

Gimbal still **not calibrated**, so `allow_uncalibrated` is saved as true. Calibrate in the gimbal UI (Calibrate tab), then untick "Track anyway".

## Camera (saved in gimbal flash)

Fast preset 320x240, JPEG q10, manual exposure (`aec 0`, `aecv` 783), gain ceiling 6, brightness +1, exposure level +1, `vflip 1`, `hm 1` (the OV3660 picture was mirrored without it). Manual exposure keeps ~28 fps in a dark room; auto exposure dropped to 2-9 fps.

Do not brighten frames for the detector: on dark live frames raw found the person 24/24, gain 19/24, gamma 8/24. The UI's *View brightness* slider only changes what you see.

## Results (real walking, 45 s, final settings)

| Movement | Median pan error | 90th percentile |
|---|---|---|
| Still | 1.8 deg | 4.0 deg |
| Slow | 1.6 deg | 3.6 deg |
| Walking | 1.6 deg | 4.3 deg |
| Fast | 2.8 deg | 7.4 deg |

Step (12 deg kick): back in 0.5-1.0 s, 3-5 deg overshoot. Lock held ~75% of the time; the rest are ~3 s drop-outs at the far sides of the room, where the person is small and dark (detector limit, not PID). More room light is the biggest remaining improvement.

## Tried and rejected

- Coasting through detection gaps (1 s look-ahead): overshot when the person stopped, p90 error 10 deg vs 5 deg.
- Brightening detector input: fewer detections (see above).
- Bigger detector input (416/480 px): fewer detections.

## Grid re-tune on the GPU (12 detections/s), person seated, 4 kicks per setting

| Pan Kp / Kd | Return | Overshoot | Score |
|---|---|---|---|
| 2.0 / 0.2 | 0.52 s | 0.6 deg | 1.90 |
| 2.5 / 0.2 | 0.62 s | 1.9 deg | 1.30 |
| **2.5 / 0.35** | **0.41 s** | **1.6 deg** | **0.97** |
| 3.0 / 0.2 | 0.63 s | 2.1 deg | 1.17 |
| 3.0 / 0.35 | 0.56 s | 1.3 deg | 1.11 |

Pan Ki at 2.5/0.35: 0.4 and 0.8 tie (1.57 / 1.58), 1.2 worse (2.39). Kept 0.8 (halves lag on continuous motion).

| Tilt Kp / Kd | Return | Overshoot | Score |
|---|---|---|---|
| 1.5 / 0.15 | 0.62 s | 1.7 deg | 0.90 |
| 2.0 / 0.15 | 0.71 s | 0.6 deg | 0.81 |
| 2.5 / 0.15 | 0.50 s | 0.5 deg | 0.57 |
| **2.5 / 0.3** | **0.33 / 0.28 s** | **0.6 / 0.3 deg** | **0.42 / 0.96** (two runs) |
| 3.0 / 0.3 | 0.38 s | 0.0 deg | 1.42 |
| 3.0 / 0.45 | 0.39 s | 0.4 deg | 0.53 |

Scores beyond 2.5 were dominated by random detection drop-outs, not the controller; 2.5/0.3 was the most consistent.
Confirmation run with the saved set: pan return 0.56 s / overshoot 6.9 deg, tilt 0.45 s / 3.6 deg — noisier than the grid
(4 kicks per setting, one bad kick moves the mean). Score = return s + 0.15 x overshoot deg + 0.02 x lost frames.

## Room walk with the saved gains (60 s)

Inside the gimbal's range: pan error median 1.5 deg (p90 6.0), tilt 2.2 deg (p90 3.6), far person 3.6 deg — tracking is on target.
But 24 % of tracking time the gimbal sat at its -90 deg pan end stop: the person's direction spanned -117..+35 deg (median -42).
**Turn the gimbal base ~40 deg to the left** (counter-clockwise seen from above) so the room fits inside +-90; software can't extend servo travel.
Added: the pan/tilt integral is cleared while the head is pinned at an end stop (no wind-up -> no overshoot when the person comes back).
