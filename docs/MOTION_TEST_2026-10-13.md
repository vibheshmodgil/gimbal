# Gimbal motion test — 2026-10-13T22:25:07

Image 320×240 (Fast preset), 28.1 fps. Gimbal calibrated: **False**, limits pan [-90.0, 90.0], tilt [-45.0, 45.0]. Scene texture (phase-correlation response, >0.1 is good): 0.233.

## Recommended tracker settings (written to Tracker/calibration.json)

```json
{
 "hfov": 52.8,
 "pan_sign": 1,
 "tilt_sign": 1,
 "vfov_deg": 56.8,
 "latency_s": 0.31,
 "deadband_deg": 0.5,
 "deadband_tilt_deg": 2.5
}
```

Still picture jitter: 11.491 px. Home drift after the whole run: [-0.11, 3.02] deg (pan, tilt).

## Pan

Scale -6.057 px/deg (cross-axis 0.274 px/deg), match quality 0.766.

| Commanded ° | Measured ° | Error ° |
|---|---|---|
| +0.5 | +0.01 | -0.49 |
| -0.5 | -0.36 | +0.14 |
| +2 | +2.43 | +0.43 |
| -2 | -1.49 | +0.51 |
| +5 | +5.23 | +0.23 |
| -5 | -4.74 | +0.26 |
| +10 | +10.64 | +0.64 |
| -10 | -9.82 | +0.18 |
| +20 | not measurable (overlap 0.01) | |
| -20 | not measurable (overlap 0.006) | |

Backlash **0.31°**, repeatability SD 0.175° (from right [-0.43, -0.06, -0.51, -0.43], from left [-0.11, -0.12, -0.12, -0.13]).
Smallest moves: 0.25° → 0.03°, 0.5° → 0.06°, 1.0° → 0.72°, 2.0° → 2.24°

| Speed °/s | Onset s | 50 % s | Settled s | Overshoot % | fps |
|---|---|---|---|---|---|
| 120 | no clean measurement | | | | |
| 240 | no clean measurement | | | | |

## Tilt

Scale 4.224 px/deg (cross-axis -0.113 px/deg), match quality 0.823.

| Commanded ° | Measured ° | Error ° |
|---|---|---|
| +0.5 | -0.0 | -0.5 |
| -0.5 | +0.01 | +0.51 |
| +2 | +2.53 | +0.53 |
| -2 | -2.67 | -0.67 |
| +5 | +5.62 | +0.62 |
| -5 | -7.29 | -2.29 |
| +10 | +11.33 | +1.33 |
| -10 | -15.06 | -5.06 |
| +15 | not measurable (overlap 0.056) | |
| -15 | not measurable (overlap 0.048) | |

Backlash **3.37°**, repeatability SD 0.069° (from right [3.36, 3.5, 3.54, 3.53], from left [0.16, 0.18, 0.07, 0.14]).
Smallest moves: 0.25° → -0.01°, 0.5° → 0.0°, 1.0° → 1.32°, 2.0° → 3.65°

| Speed °/s | Onset s | 50 % s | Settled s | Overshoot % | fps |
|---|---|---|---|---|---|
| 120 | 0.283 | 0.306 | 0.353 | 0.65 | 9.7 |
| 240 | 0.284 | 0.293 | 0.312 | 0.35 | 9.4 |

Onset = command sent → first frame that moved >0.6°: Wi-Fi + servo start + capture + stream. This is what the tracker's *latency* setting must cover.
