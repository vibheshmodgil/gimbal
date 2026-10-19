#include "oled.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Dedicated I2C bus for the OLED.
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

    if (devicesFound == 0)
    {
        Serial.println("No I2C devices found.");
    }
    else
    {
        Serial.print("Total I2C devices found: ");
        Serial.println(devicesFound);
    }

    if (oledDetected)
    {
        Serial.print("SSD1306 OLED detected at 0x");

        if (OLED_I2C_ADDRESS < 0x10)
            Serial.print("0");

        Serial.println(OLED_I2C_ADDRESS, HEX);
    }
    else
    {
        Serial.print("SSD1306 OLED NOT detected at 0x");

        if (OLED_I2C_ADDRESS < 0x10)
            Serial.print("0");

        Serial.println(OLED_I2C_ADDRESS, HEX);
    }

    Serial.println("================================");
}

bool oledBegin()
{
    Serial.println();
    Serial.println("Initializing OLED...");

    OLED_I2C.begin(
        OLED_SDA_PIN,
        OLED_SCL_PIN
    );

    Serial.print("I2C SDA: GPIO ");
    Serial.println(OLED_SDA_PIN);

    Serial.print("I2C SCL: GPIO ");
    Serial.println(OLED_SCL_PIN);

    Serial.print("Expected OLED address: 0x");

    if (OLED_I2C_ADDRESS < 0x10)
        Serial.print("0");

    Serial.println(OLED_I2C_ADDRESS, HEX);

    oledScanI2C();

    if (!oledDetected)
    {
        Serial.println();
        Serial.println("OLED initialization skipped.");
        Serial.println("Check:");
        Serial.println("  - OLED VDD -> ESP32 3V3");
        Serial.println("  - OLED GND -> ESP32 GND");
        Serial.println("  - OLED SDA -> GPIO21");
        Serial.println("  - OLED SCL -> GPIO47");
        Serial.println("  - OLED address -> normally 0x3C");

        return false;
    }

    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_I2C_ADDRESS))
    {
        Serial.println("SSD1306 initialization FAILED.");
        return false;
    }

    Serial.println("SSD1306 initialization OK.");

    display.clearDisplay();
    display.display();

    return true;
}

void oledShowTestScreen()
{
    if (!oledDetected)
        return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("ESP32-S3 CAM");

    display.setCursor(0, 18);
    display.println("OLED TEST");

    display.setCursor(0, 36);
    display.println("I2C OK");

    display.setCursor(0, 54);
    display.println("System Ready");

    display.display();

    Serial.println("OLED test screen displayed.");
}

bool oledIsDetected()
{
    return oledDetected;
}
