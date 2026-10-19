#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// ESP32-S3 CAM + SSD1306 OLED + Wi-Fi + WebSocket
// Phase 2 configuration
// ============================================================

// -----------------------------
// Serial
// -----------------------------
#define SERIAL_BAUD 115200

// -----------------------------
// OLED
// -----------------------------
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 47

#define OLED_I2C_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// -----------------------------
// Wi-Fi
// -----------------------------
#define WIFI_SSID "Airtel_kuma_9602"
#define WIFI_PASSWORD "air71417"

#define WIFI_CONNECT_TIMEOUT_MS 20000

// -----------------------------
// WebSocket
// -----------------------------
#define WEBSOCKET_PORT 81
#define HTTP_PORT 80

// Send a camera frame every N milliseconds.
// 100 ms ~= 10 FPS maximum requested rate.
#define CAMERA_STREAM_INTERVAL_MS 100

// -----------------------------
// Camera profile
// -----------------------------
// IMPORTANT:
// This default profile is for Espressif ESP32-S3-EYE.
//
// If your ESP32-S3 CAM is a different board, the CAMERA_*
// pin definitions below must be changed to match your board.
#define CAMERA_PROFILE_ESP32S3_EYE

// -----------------------------
// ESP32-S3-EYE camera pins
// -----------------------------
#if defined(CAMERA_PROFILE_ESP32S3_EYE)

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

#else

#error "Select the correct camera profile in config.h"

#endif

// -----------------------------
// Camera settings
// -----------------------------
#define CAMERA_XCLK_FREQ 20000000

// QVGA = 320x240.
// Good starting point for ESP32-S3 WebSocket testing.
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA

// Lower number = higher JPEG quality / larger frame.
#define CAMERA_JPEG_QUALITY 12

#endif
