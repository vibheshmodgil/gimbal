#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "oled.h"
#include "camera.h"
#include "web.h"
bool cameraOK = false;
bool wifiOK = false;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("ESP32-S3 CAM + OLED + WEBSOCKET");
    Serial.println("PHASE 2");
    Serial.println("==========================================");

    // -----------------------------
    // OLED / I2C
    // -----------------------------
    bool oledOK = oledBegin();

    if (oledOK)
        Serial.println("OLED: OK");
    else
        Serial.println("OLED: FAILED");

    // -----------------------------
    // Camera
    // -----------------------------
    cameraOK = cameraBegin();

    if (cameraOK)
        Serial.println("CAMERA: OK");
    else
        Serial.println("CAMERA: FAILED");

    // -----------------------------
    // Wi-Fi
    // -----------------------------
    wifiOK = wifiBegin();

    if (wifiOK)
    {
        Serial.println("WIFI: OK");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        // -----------------------------
        // HTTP page + WebSocket server
        // -----------------------------
        webBegin();

        Serial.println();
        Serial.println("Camera/WebSocket page:");
        Serial.print("http://");
        Serial.println(WiFi.localIP());
        Serial.println("WebSocket port: 81");
    }
    else
    {
        Serial.println("WIFI: FAILED");
    }

    // -----------------------------
    // OLED status
    // -----------------------------
    oledShowSystemStatus(
        oledOK,
        cameraOK,
        wifiOK,
        wifiOK ? WiFi.localIP().toString().c_str() : "WiFi failed"
    );

    Serial.println();
    Serial.println("==========================================");
    Serial.println("SYSTEM INITIALIZATION COMPLETE");
    Serial.println("==========================================");
}

void loop()
{
    if (wifiOK)
    {
        webHandle();

        // Stream one JPEG frame over WebSocket
        // at a modest rate for the first hardware test.
        static uint32_t lastFrameMs = 0;

        if (millis() - lastFrameMs >= CAMERA_STREAM_INTERVAL_MS)
        {
            lastFrameMs = millis();

            if (cameraOK)
            {
                camera_fb_t *fb = cameraGetFrame();

                if (fb != nullptr)
                {
                    webSendCameraFrame(fb->buf, fb->len);
                    cameraReleaseFrame(fb);
                }
            }
        }
    }

    delay(1);
}
