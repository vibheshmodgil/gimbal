#include "config.h"
#include "oled.h"

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-S3 CAM - OLED PHASE 1");
    Serial.println("================================");
    Serial.println("Starting system...");

    if (oledBegin())
    {
        oledShowTestScreen();

        Serial.println();
        Serial.println("================================");
        Serial.println("SYSTEM READY");
        Serial.println("================================");
    }
    else
    {
        Serial.println();
        Serial.println("================================");
        Serial.println("OLED INITIALIZATION FAILED");
        Serial.println("================================");
    }
}

void loop()
{
    // Phase 1 only:
    // ESP32-S3 CAM + one SSD1306 OLED.
    // Camera, Wi-Fi and second OLED will be added later.
    delay(1000);
}
