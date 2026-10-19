# ESP32-S3 CAM - OLED Phase 1

Minimal Arduino IDE project for testing one 0.96-inch 128x64 SSD1306 I2C OLED with an ESP32-S3 CAM.

## Wiring

| OLED | ESP32-S3 |
|---|---|
| VDD/VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL/SCK | GPIO47 |

## Arduino IDE libraries

Install from Library Manager:

- Adafruit GFX Library
- Adafruit SSD1306

## Configuration

All important OLED/I2C settings are in `config.h`:

- SDA
- SCL
- I2C address
- Width
- Height
- Serial baud rate

## Serial Monitor

Set baud rate to:

`115200`

The program scans the I2C bus, normally finds the OLED at `0x3C`, initializes the display, and shows:

ESP32-S3 CAM
OLED TEST
I2C OK
System Ready

## Phase 1 scope

Included:
- ESP32-S3
- One SSD1306 OLED
- I2C initialization
- I2C scanner
- OLED diagnostics
- Test display

Not included yet:
- Camera
- Wi-Fi
- Second OLED
