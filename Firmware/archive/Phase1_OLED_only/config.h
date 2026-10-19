#ifndef CONFIG_H
#define CONFIG_H

// ESP32-S3 CAM + SSD1306 OLED
// Phase 1 Configuration

// I2C pins
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 47

// SSD1306 I2C address
#define OLED_I2C_ADDRESS 0x3C

// OLED resolution
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Serial baud rate
#define SERIAL_BAUD 115200

#endif
