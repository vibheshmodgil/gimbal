#include "oled.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

TwoWire OLED_I2C = TwoWire(0);

Adafruit_SSD1306 display(
    OLED_WIDTH,
    OLED_HEIGHT,
    &OLED_I2C,
    -1
);

static bool oledDetected = false;

void oledScanI2C()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("I2C BUS SCAN");
    Serial.println("================================");

    uint8_t devicesFound = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        OLED_I2C.beginTransmission(address);
        uint8_t error = OLED_I2C.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at 0x");

            if (address < 0x10)
                Serial.print("0");

            Serial.println(address, HEX);

            devicesFound++;

            if (address == OLED_I2C_ADDRESS)
                oledDetected = true;
        }
    }

    Serial.println();

    Serial.print("Total I2C devices found: ");
    Serial.println(devicesFound);

    if (oledDetected)
    {
        Serial.print("OLED detected at 0x");
        if (OLED_I2C_ADDRESS < 0x10)
            Serial.print("0");
        Serial.println(OLED_I2C_ADDRESS, HEX);
    }
    else
    {
        Serial.println("OLED not detected.");
    }
}

bool oledBegin()
{
    Serial.println("Initializing I2C/OLED...");

    OLED_I2C.begin(
        OLED_SDA_PIN,
        OLED_SCL_PIN
    );

    Serial.print("OLED SDA = GPIO ");
    Serial.println(OLED_SDA_PIN);

    Serial.print("OLED SCL = GPIO ");
    Serial.println(OLED_SCL_PIN);

    oledScanI2C();

    if (!oledDetected)
        return false;

    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_I2C_ADDRESS))
    {
        Serial.println("SSD1306 init failed.");
        return false;
    }

    display.clearDisplay();
    display.display();

    Serial.println("SSD1306 initialization OK.");

    return true;
}

void oledShowSystemStatus(
    bool oledOK,
    bool cameraOK,
    bool wifiOK,
    const char *ipAddress
)
{
    if (!oledDetected)
        return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("ESP32-S3 CAM");

    display.setCursor(0, 12);
    display.print("OLED: ");
    display.println(oledOK ? "OK" : "FAIL");

    display.setCursor(0, 24);
    display.print("CAM:  ");
    display.println(cameraOK ? "OK" : "FAIL");

    display.setCursor(0, 36);
    display.print("WiFi: ");
    display.println(wifiOK ? "OK" : "FAIL");

    display.setCursor(0, 48);

    if (wifiOK)
    {
        display.println(ipAddress);
    }
    else
    {
        display.println("WiFi not connected");
    }

    display.display();
}

bool oledIsDetected()
{
    return oledDetected;
}
