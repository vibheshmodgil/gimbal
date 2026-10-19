# Live gimbal test and forward strategy — 12 October 2026

## Scope and method

The Chrome control bridge was unavailable, so the live ESP32 was tested through the same local HTTP, WebSocket, and MJPEG endpoints used by its web page. The camera feed already had one viewer. I kept the laser off, limited motion to pan ±20° and tilt ±12°, and returned the head to 0°, 0°. A stream frame was captured at selected positions. This proves the image changed with physical pan/tilt movement, but it does not give an accurate physical servo angle or exact glass-to-glass latency.

The live board reports firmware **3.1**. The workspace `config.h` labels the current firmware **3.2**. Treat source-based explanations as hypotheses until the flashed build is identified.

## Observations

| Area | Live result | Interpretation |
|---|---|---|
| Power and health | Last reset reason `BROWNOUT`; temperature rose from 75.3°C to 82.3°C during this session; Wi-Fi RSSI about −65 to −61 dBm; camera, OLED, PSRAM and both servo outputs reported present. | Power integrity and heat deserve first investigation. The reset reason is historical; this test did not prove the movement caused it. |
| Pan | 0° → +20° → −20° → 0°; telemetry and captured views changed. A 40° reversal reached the software target in about 0.55 s. | Motor is responsive in this small range. Telemetry is the commanded servo position, not measured shaft position. |
| Tilt | 0° → +12°; telemetry reached target in about 0.20 s and the view changed. A later command timed out, so the test was stopped and a center command was sent. Final status confirmed 0°, 0°. | Small upward tilt works. Downward physical response was not verified. |
| Command path | Most local HTTP commands returned in 56–133 ms; intermittent responses took 1.1–3.8 s, and one request exceeded 4 s. | This is a credible source of perceived lag, though Chrome uses WebSocket for movement. Measure WebSocket latency separately before blaming the motor. |
| Camera at VGA | 640×480, JPEG quality 12, cap 25 fps: actual single-viewer stream varied roughly 1–10 fps, often 4–7 fps; frames around 19–21 KB. | The cap is not achieved. A second viewer and frame capture can disturb throughput, so compare at one viewer under fixed conditions. |
| Camera presets | Fast 320×240: mean 10.7 fps, 5.2 KB/frame. Balanced 640×480: mean 6.1 fps, 19.3 KB/frame. HQ 800×600: mean 5.0 fps, 40.7 KB/frame. Each mean came from roughly 20 status samples over about 3.3 s with one viewer. | Fast was the best measured motion-viewing setting; HQ was the slowest completed test. The MAX command timed out; no MAX fps result is claimed. |
| Image quality | Saved frames show left-side washout and substantial shadow noise. The live camera reports exposure level 0, whereas workspace defaults specify −2. | Check lighting, exposure configuration, and the live firmware before tuning quality. The current scene is not suitable for fine visual tracking. |

After the preset test, live `CAMGET` confirmed the original 640×480 resolution, quality 12, and 25 fps cap. Final status confirmed pan 0°, tilt 0°, laser off, and both servo signals enabled.

## Best and worst observed conditions

- **Best measured video responsiveness:** Fast preset with one viewer, about 10.7 fps. Image detail is limited to 320×240.
- **Best detail/speed compromise tested:** Balanced at 640×480, about 6.1 fps in this session. Its performance still needs improvement for smooth manual aiming.
- **Worst completed preset:** HQ at 800×600, about 5.0 fps. MAX could not be benchmarked because its request timed out.
- **Worst reliability clues:** prior brownout, rising board temperature, occasional multi-second command response, and severe mixed bright/dark image areas. These are observations, not established root causes.

## Forward strategy

1. **Stabilize power and thermal behavior.** With the unit in view and the laser off, measure 5 V at the servo rail and ESP32 during pan, tilt, and simultaneous movement. Check the 5 V 3 A supply, common ground, 470–1000 µF capacitor, harness resistance and connectors. Recheck reset reason after a controlled run; a new brownout would link the issue to load. Inspect airflow and board temperature under a fixed camera workload.
2. **Make latency measurable.** Add sequence IDs and timestamps for browser send, WebSocket receive, target update, servo pulse write, camera frame capture, stream send, and browser display. Report median and 95th-percentile response. Because firmware `curPan`/`curTilt` are software ramps, use an external camera or angle marker to measure real shaft motion and backlash.
3. **Fix the video path first for usability.** Use Fast as the temporary low-lag setting and one viewer. Compare Fast, Balanced, and HQ in consistent bright, even lighting at a fixed Wi-Fi distance. Profile camera capture versus JPEG streaming and client send blocking; test whether the stream task or heat causes the observed fps collapse. Keep a lower resolution if it substantially improves end-to-end latency.
4. **Tune picture quality separately.** Under even illumination, compare exposure level 0 with −2 and verify gain ceiling/status values against the actual OV5640 sensor. Capture sample frames in bright, dim, and backlit scenes. Save a profile only after the live image is visibly better and fps is stable.
5. **Then tune motion.** Repeat single-axis ±10° and ±20° steps, diagonal moves, and reversals at multiple speeds with external motion measurement. Check for backlash, missed motion, jitter, and supply sag. Verify calibrated limits and cable clearance before any full sweep or patrol test.
6. **Reconcile and harden firmware.** Identify the exact flashed 3.1 build versus workspace 3.2, reproduce the slow-command path, then update and retest. Move Wi-Fi/OTA credentials out of source before sharing or publishing the project; rotate them if this source has left the private workspace.

## Next acceptance run

Run with one browser viewer, a stable scene, and logged supply voltage and temperature. Require no new brownout or watchdog reset, no request timeout, repeatable physical motor response, and a meaningful increase in delivered fps and visible responsiveness. Set the final numeric targets after measuring end-to-end latency with timestamps.

## Evidence and limits

- [Baseline frame](live_probe_frame.jpg), [pan right](live_probe_pan_right.jpg), [pan left](live_probe_pan_left.jpg), [tilt up](live_probe_tilt_up.jpg).
- [Reusable probe](live_probe.mjs) and [baseline telemetry](live_probe_baseline.json).
- Full end-stop sweeps, laser emission, OLED appearance, physical voltage, and actual servo angle were not observed during this session. Avoid treating the software position as physical feedback.
