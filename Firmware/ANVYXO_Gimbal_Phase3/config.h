#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// ANVYXO Gimbal - Phase 3 configuration
// Freenove ESP32-S3 WROOM CAM + SSD1306 OLED + MG996R pan + MG90S tilt + laser
// ============================================================

#define SERIAL_BAUD 115200

// -----------------------------
// Wi-Fi (home network). If it fails, the gimbal starts its own hotspot.
// -----------------------------
#define WIFI_SSID "Airtel_kuma_9602"
#define WIFI_PASSWORD "air71417"
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define AP_SSID     "ANVYXO-Gimbal"     // fallback hotspot -> open http://192.168.4.1
#define AP_PASSWORD "anvyxo123"          // min 8 characters
#define MDNS_NAME   "anvyxo"             // http://anvyxo.local (Windows/phones that support mDNS)
#define OTA_PASSWORD "anvyxo"            // Arduino IDE -> Port -> network port "anvyxo"

#define HTTP_PORT 80
#define WEBSOCKET_PORT 81
#define STREAM_PORT 82                   // MJPEG video: http://<ip>:82/stream (own task, never blocks control)
#define STREAM_MAX_CLIENTS 2
#define STATUS_INTERVAL_MS 150           // status JSON to the web page (drives the 3D twin)
#define FW_VERSION "3.4"

// -----------------------------
// OLED screen in the base: 4 pins  GND->GND  VCC->3V3  SCL->GPIO47  SDA->GPIO21
// -----------------------------
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 47
#define OLED_I2C_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_REFRESH_MS 80

// -----------------------------
// Servos, laser, LED, button
// -----------------------------
#define PAN_PIN   14      // MG996R signal (orange)
#define TILT_PIN  2       // MG90S signal (orange)
#define LASER_PIN 1       // NPN transistor base via 1k: HIGH = laser on
#define RGB_LED_PIN 48    // Freenove on-board WS2812
#define BUTTON_PIN 0      // BOOT button: short press = next screen page

// The camera owns LEDC timer 0 (channel 0). Channels 4/5 share timer 2 at 50 Hz.
#define SERVO_LEDC_PAN_CH  4
#define SERVO_LEDC_TILT_CH 5
#define SERVO_FREQ_HZ  50
#define SERVO_RES_BITS 14

// Pulse range of each servo (0..180 deg). Typical MG996R/MG90S: 500..2500 us.
#define PAN_MIN_US  500
#define PAN_MAX_US  2500
#define TILT_MIN_US 500
#define TILT_MAX_US 2500

// Gimbal angles are 0 = centre. Limits match the CAD collision checks.
#define PAN_LIMIT_DEG  90.0f
#define TILT_LIMIT_DEG 45.0f
#define PAN_INVERT  0     // set 1 if "pan right" turns left
#define TILT_INVERT 0     // set 1 if "tilt up" looks down

#define DEFAULT_SPEED_DPS 120.0f   // smooth-motion speed (deg/s), changeable from the web page
#define PATROL_SPEED_DPS  30.0f
#define LASER_MAX_ON_MS   60000    // safety: laser switches itself off after 60 s

// -----------------------------
// Camera (Freenove ESP32-S3 WROOM = ESP32-S3-EYE pin map)
// -----------------------------
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y2_GPIO_NUM     11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM     10
#define Y6_GPIO_NUM     12
#define Y7_GPIO_NUM     18
#define Y8_GPIO_NUM     17
#define Y9_GPIO_NUM     16
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13

#define CAMERA_XCLK_FREQ    20000000
#define CAMERA_FRAME_SIZE   FRAMESIZE_VGA    // default 640x480; changeable live from the web page (saved in flash)
#define CAMERA_JPEG_QUALITY 12              // 4..63, lower = better
#define CAMERA_FPS_CAP      25
// Exposure: the OV5640 auto-exposure aims too bright (washed-out picture). Lower AE target fixes it.
// OV5640 takes -5..+5 (each step ~10 brightness units of 255), OV2640/OV3660 take -2..+2.
// Calibration knob: still too bright -> -3/-4, too dark -> -1/0. Also live on the Camera tab.
#define CAMERA_AE_LEVEL     -2
#define CAMERA_GAIN_CEILING 2      // 0..6 = 2x..128x; low = less noise and less blow-out in bright rooms
#define CAMERA_VFLIP   1      // OV3660 module (fitted 2026-10-13): flip 1 + mirror 1 = upright, not mirrored (pan test confirmed)
#define CAMERA_HMIRROR 1

#endif
