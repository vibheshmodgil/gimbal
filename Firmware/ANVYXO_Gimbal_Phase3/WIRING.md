# ANVYXO Gimbal – Phase 3 Wiring

Board: **Freenove ESP32-S3 WROOM CAM (FNK0085)**. Every pin below matches `config.h`. If you change a pin there, change it here too.

## Pin map

| Part | Wire | ESP32-S3 pin | `config.h` |
|---|---|---|---|
| SSD1306 OLED (0x3C) | SDA | **GPIO 21** | `OLED_SDA_PIN 21` |
| | SCL | **GPIO 47** | `OLED_SCL_PIN 47` |
| | VCC | **3V3** | — |
| | GND | **GND** | — |
| MG996R (pan, in base) | Signal (orange) | **GPIO 14** | `PAN_PIN 14` |
| | Red (+) | **5 V WAGO bus** (buck output, not the ESP32) | — |
| | Brown (−) | **GND** | — |
| MG90S (tilt, in head) | Signal (orange) | **GPIO 2** | `TILT_PIN 2` |
| | Red (+) | **5 V** (from base harness) | — |
| | Brown (−) | **GND** | — |
| Laser switch (NPN) | Base, through 1 kΩ | **GPIO 1** | `LASER_PIN 1` (HIGH = on) |
| On-board RGB LED | — | GPIO 48 (built in, no wiring) | `RGB_LED_PIN 48` |
| BOOT button | — | GPIO 0 (built in, no wiring) | `BUTTON_PIN 0` (next screen page) |
| Camera (OV5640 on this board) | — | Built in, no wiring | GPIO 4–13, 15–18 |

**Do not use** these pins for anything else:
- 4–13 and 15–18: camera
- 0, 3, 45, 46: strapping pins
- 19 and 20: USB
- 35–37: PSRAM
- 48: RGB LED

## Laser switch

```
 5 V ──────────── Laser (+)
                  Laser (−) ──── C
                                  │  NPN (S8050 / 2N2222)
 GPIO 1 ──[ 1 kΩ ]────────────── B
                                  │
 GND ─────────────────────────── E
```

The laser shuts itself off after 60 s (`LASER_MAX_ON_MS 60000`).

## Power (all inside the base)

```
 3S pack ─ BMS ─ P+ ──┬── charge jack (+)      (12.6 V 3S Li-ion charger only)
                      └── rocker switch ── buck IN+
               P− ──┬── charge jack (−)
                    └── buck IN−

 buck OUT+ (5.1 V) ── 5 V WAGO ──┬── MG996R red
                                 ├── 1000 µF cap (+)
                                 └── harness 5 V ──┬── ESP32 5V pin
                                                   ├── MG90S red
                                                   └── Laser (+)

 buck OUT− ────────── GND WAGO ──┬── MG996R brown
                                 ├── cap (−), OLED GND
                                 └── harness GND ──┬── ESP32 GND
                                                   ├── MG90S brown
                                                   └── NPN emitter
```

- Set the buck to 5.1 V **before** connecting anything.
- **All grounds must be common.** If they aren't, the servos jitter or don't move.
- Never power the servos from the ESP32's 3V3 or 5V pin.
- **Switch the base off before plugging in USB-C to flash.** Or flash over Wi-Fi (OTA): Arduino IDE → Port → `anvyxo`, password `anvyxo`.

## Base ↔ head harness (6 wires)

| # | Wire | Gauge |
|---|---|---|
| 1 | 5 V | 22–24 AWG |
| 2 | GND | 22–24 AWG |
| 3 | 3V3 (OLED VCC, from ESP32) | 26–28 AWG |
| 4 | SDA (GPIO 21 → OLED) | 26–28 AWG |
| 5 | SCL (GPIO 47 → OLED) | 26–28 AWG |
| 6 | PAN signal (GPIO 14 → MG996R) | 26–28 AWG |

Leave an 8–10 cm service loop so the head can pan ±90° and tilt ±45°.

## Servo settings (from `config.h`)

| | Pan (MG996R) | Tilt (MG90S) |
|---|---|---|
| PWM | 50 Hz, 14-bit, LEDC ch 4 | 50 Hz, 14-bit, LEDC ch 5 |
| Pulse | 500–2500 µs | 500–2500 µs |
| Limit | ±90° | ±45° |
| Wrong direction? | set `PAN_INVERT 1` | set `TILT_INVERT 1` |

## Quick check on power-up (Serial 115200)

- `OLED: not found at 0x3C (SDA 21, SCL 47)` → SDA and SCL are swapped, or VCC/GND is loose.
- Servos twitch but don't hold position → no common ground, or the buck is too weak (MG996R stall pulls ~2.5 A) / battery flat.
- Laser stays off → check the transistor's C/B/E orientation. S8050 and 2N2222 pinouts differ by package.
