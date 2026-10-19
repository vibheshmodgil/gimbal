# Live gimbal test in Chrome + forward strategy — 13 October 2026

First run where the Chrome bridge worked. Board (MAC e0:72:a1:f6:30:80) is now at **192.168.1.8** (was .2). Flashed firmware is still **3.1**. One viewer, laser off throughout.

Method: a WebSocket on :81 opened inside the page sent `PT:` moves and logged 150 ms status. The page's own `<img id=cam>` stream (CORS `*`) was sampled to a canvas every 10 ms. "Picture onset" = first frame that differs by more than the sensor-noise floor (≈1.5) after the command. "Settle" = last such frame. That makes it a real **command → visible-on-screen** measure: it includes Wi-Fi, servo start, capture, JPEG and browser.

## Motors (software ramp, 120 °/s)

| Move | Status ack | Software reached target |
|---|---|---|
| ±10° pan | 46–430 ms | 300–435 ms |
| 60° pan reversal | 74 ms | 776 ms |
| 120° pan reversal | 224 ms | 1204 ms |
| ±15° / 30° tilt | 33–183 ms | 320–370 ms |
| 60° tilt reversal | 47 ms | 732 ms |
| diagonal 20°/12° and reversal | 155–168 ms | 349–489 ms |

All moves reached their targets, and the timings match the 120 °/s ramp plus 50–250 ms command overhead. **The motors are not the source of lag.** The camera showed real physical motion for every pan and tilt step, including ±3°.

WebSocket PING round trip: **24–401 ms** (median ~150 ms). On 12 Oct it was 9–57 ms. RSSI was −55 to −66 dBm, so this jitter comes from the board side (CPU/stream contention), not from signal strength.

## Camera lag: the real problem

| Preset | Frames/s seen in browser | Picture onset after command | Picture settled |
|---|---|---|---|
| Balanced 640×480 q12 | **2–5** | **370–2339 ms** | 650–2781 ms |
| Fast 320×240 | **8–13** | **190–440 ms** | 410–1050 ms |

On Balanced, the video lags the motors by up to ~2 s. With frames 200–500 ms apart, the picture can look frozen while the head is already moving. Fast is about 4× quicker, close to motor-limited.

## Best and worst conditions

- **Best:** Fast 320 preset, small moves (≤20°), one viewer, cool board. You see motion within ~0.2–0.4 s.
- **Worst (crash):** Fast preset + continuous motion. After ~45 s the **board rebooted with reset reason `watchdog`**. The browser video froze 13 s before the WebSocket dropped, and the gimbal stayed parked at the last angle. Board temperature went **64 → 81 °C** during the session.
- **Worst (usability):** Balanced/Sharp during motion. 2–5 fps, up to 2.3 s visual lag, heavy motion blur, and a washed-out left side.

## Fixed today

1. **Upside-down camera.** The control already existed: Camera tab → *Flip vertical*. It was ON, which is wrong for this mount. I turned it OFF and pressed *Save as default*, and it survived the reboot. `config.h` now defaults `CAMERA_VFLIP 0`. If the picture also looks left/right swapped, tick *Mirror* and save.
2. **Watchdog crash (likely cause).** The MJPEG task (core 0, priority 5) only slept when it was ahead of the fps cap. At 320×240 frames are ready instantly, so the task never yielded, IDLE0 starved, and the task watchdog fired. It now always yields ≥5 ms per frame (`camera.cpp` streamTask). This compiles, but it is **not flashed**. Verify with a 5-minute Fast-preset patrol after OTA.

## Forward strategy (in order)

1. **OTA-flash 3.3 + the watchdog fix** (`anvyxo`, rollback binary kept). Acceptance: 5 min Fast-preset patrol with no reset, and `reset` ≠ watchdog/brownout afterwards.
2. **Make Fast the default viewing preset** while aiming. Switch to Sharp only for stills or snapshots, when the head is stationary. ponytail: no auto-switching yet; add a "drop to Fast while moving" option only if manual switching annoys you.
3. **Thermal.** 81 °C during streaming is too close to throttling/instability. Add a heatsink or airflow at the ESP32-S3 module and re-measure under the same Fast patrol. Target < 70 °C.
4. **Use the 3.3 metrics** (`captureMs`, `sendMs`, `loopGapMs`) to decide whether Balanced is capture-bound or Wi-Fi-bound before tuning JPEG quality/resolution further.
5. **Picture quality, after 1–4.** Set exposure level −2 (live shows 0), and fix gain ceiling 248, which is invalid; 3.3 load-validates it. Test under even lighting.
6. **Motion tuning last.** The motors are fine at 120 °/s. Only raise speed once the video can keep up (≥10 fps). Check the 5 V rail under simultaneous pan+tilt before any full-range sweep.
