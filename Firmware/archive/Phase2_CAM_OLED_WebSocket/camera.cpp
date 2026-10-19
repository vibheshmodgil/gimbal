#include "camera.h"
#include "config.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>

bool cameraBegin()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("CAMERA INITIALIZATION");
    Serial.println("================================");

    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = CAMERA_XCLK_FREQ;

    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;

    bool hasPSRAM = psramFound();

    Serial.print("PSRAM: ");
    Serial.println(hasPSRAM ? "FOUND" : "NOT FOUND");

    if (hasPSRAM)
    {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.print("Camera init failed: 0x");
        Serial.println((uint32_t)err, HEX);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor == nullptr)
    {
        Serial.println("Camera sensor handle is NULL.");
        return false;
    }

    // ESP32-S3-EYE orientation
    sensor->set_vflip(sensor, 1);

    Serial.println("Camera initialization OK.");
    Serial.println("Frame size: QVGA 320x240");
    Serial.println("JPEG mode: ON");

    return true;
}

camera_fb_t *cameraGetFrame()
{
    return esp_camera_fb_get();
}

void cameraReleaseFrame(camera_fb_t *frame)
{
    if (frame != nullptr)
    {
        esp_camera_fb_return(frame);
    }
}