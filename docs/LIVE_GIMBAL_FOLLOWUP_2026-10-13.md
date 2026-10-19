# Live motor follow-up and firmware changes — 13 October 2026

## Live test on firmware 3.1

The browser automation bridge remained unavailable, so commands went to the same WebSocket and HTTP endpoints used by the browser UI. One camera viewer stayed connected. Laser stayed off. The sequence used pan ±10° and ±30°, tilt ±15°, and diagonal ±20°/±12° moves, with a return to center. No full end-stop sweep was attempted while the reported board temperature was 86.3°C.

- 12 WebSocket `PING` round trips: **9–57 ms**.
- Command target appeared in status in **76–319 ms**. Status is broadcast every 150 ms, so this includes sampling delay.
- Software position reached target in **201–700 ms**. The longest move was a 60° pan reversal; at the configured 120°/s ramp, 500 ms is expected before status delay. These values do **not** measure actual servo shaft motion.
- The camera stream reported **0–6.9 fps** during portions of the motor sequence. A low video rate can make responsive motors look delayed.
- The board remained up, stayed at **86.3°C** in before/after diagnostics, and returned to **0°, 0°** with both servo signals on. Its historical reset reason was still `BROWNOUT`; no new reset was observed during this run.
- An HTTP diagnostics request timed out after the sequence, while a subsequent request succeeded. This reinforces that occasional HTTP stalls exist; the WebSocket ping samples did not show comparable delay.

## Source changes prepared as firmware 3.3

- Camera status now reports the last frame capture time, MJPEG send time, capture failure count, and largest main-loop gap since the previous status. These fields will let a post-update run distinguish camera acquisition, network send, and control-loop stalls.
- Camera settings loaded from flash are checked against each setting's valid range. Saving camera settings removes an invalid sensor-reported value instead of persisting it. This matters because the live OV5640 reported gain ceiling `248`, outside the configured valid `0..6` range.
- Status JSON buffer was enlarged and truncation is checked before broadcast.
- Firmware version advanced to **3.3** so the flashed build can be identified.

The modified sketch and libraries **compiled successfully** with the installed Arduino ESP32-S3 toolchain. The binary uses 1,277,254 bytes of program storage (38%) and 66,252 bytes of global RAM (20%). The new `3.3` binary is at `Firmware/ANVYXO_Gimbal_Phase3/build/esp32.esp32.esp32s3/ANVYXO_Gimbal_Phase3.ino.bin`. A previously flashed `3.1` binary remains in the same directory as `ANVYXO_Gimbal_Phase3.ino_flashed.bin` for rollback if OTA connectivity remains available.

**Deployment status:** the live gimbal still runs firmware 3.1. The new metrics and persistence fix have not yet been verified on hardware.

## Next decision

After updating to 3.3, repeat a one-viewer stationary and motor test and log `captureMs`, `sendMs`, `captureFails`, and `loopGapMs`. If capture time dominates, address camera exposure/sensor timing and lighting. If send time dominates, address stream delivery and Wi-Fi. If loop gaps are high, isolate synchronous HTTP/OLED work. Independently measure the 5 V rail under servo load before increasing motor speed or doing full-range patrol tests.
