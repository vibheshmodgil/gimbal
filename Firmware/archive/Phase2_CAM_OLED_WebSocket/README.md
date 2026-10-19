# ESP32-S3 CAM + OLED + WebSocket

Phase 2 Arduino IDE project.

## What this version does

- Initializes one 0.96-inch 128x64 SSD1306 OLED.
- Scans I2C.
- Uses OLED SDA = GPIO21.
- Uses OLED SCL = GPIO47.
- Initializes the camera.
- Connects the ESP32-S3 to Wi-Fi.
- Starts an HTTP page on port 80.
- Starts a WebSocket server on port 81.
- Sends JPEG camera frames over the WebSocket connection.
- Displays system status on the OLED.

## IMPORTANT: camera board profile

The default `config.h` profile is:

`CAMERA_PROFILE_ESP32S3_EYE`

This is the pin map used by Espressif's current CameraWebServer example for the ESP32-S3-EYE.

Your actual ESP32-S3 CAM may use a different pin map. If the camera init fails, the first thing to verify is the exact camera-board model/pinout. Do not randomly change the OLED pins.

## Arduino IDE libraries

Install:

- Adafruit GFX Library
- Adafruit SSD1306
- WebSockets

The WebSockets library provides `WebSocketsServer`.

## Wi-Fi

Edit `config.h`:

`WIFI_SSID`
`WIFI_PASSWORD`

## Board

Start with:

`ESP32S3 Dev Module`

Set Serial Monitor to:

`115200`

For PSRAM-capable camera boards, select the appropriate PSRAM setting for the exact board.

## Wiring

OLED:

VDD -> 3V3
GND -> GND
SDA -> GPIO21
SCL -> GPIO47

## Camera test

After upload, open Serial Monitor.

When Wi-Fi connects, the ESP32 prints an IP address such as:

`192.168.1.xxx`

Open that IP in a browser:

`http://192.168.1.xxx`

The webpage connects to WebSocket port 81 and displays the JPEG camera frames sent by the ESP32.

## Important architecture note

The camera image itself is transported as binary JPEG frames over WebSocket in this test project.

The WebSocket channel also accepts text commands such as:

`PING`
`BLINK`

Those commands are placeholders for the future robot-eye/RC-car control layer.

## Phase 2 scope

Included:
- One OLED
- Camera
- Wi-Fi
- HTTP control page
- WebSocket server
- Camera JPEG frames over WebSocket

Not included yet:
- Second OLED
- Eye animation system
- Motor control
- Person tracking
- YOLO
- RC-car control protocol
