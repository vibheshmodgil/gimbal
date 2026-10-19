// ============================================================
// ANVYXO Gimbal - Phase 3
// Camera stream + pan/tilt control + laser + OLED screen in the base.
// Board: "ESP32S3 Dev Module", Flash Size: "8MB", PSRAM: "OPI PSRAM",
// Partition: "8M with spiffs (3MB APP/1.5MB SPIFFS)"  (has OTA, so wireless updates work)
// Libraries: Adafruit GFX, Adafruit SSD1306, WebSockets (Markus Sattler)
// Web page: edit web_src/index.html, then run  python tools/build_web.py  (regenerates web_page.h)
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "camera.h"
#include "gimbal.h"
#include "oled.h"
#include "web.h"

static bool cameraOK = false;
static bool wifiOK = false;
static uint32_t ledTestUntil = 0;
static uint32_t maxLoopGapMs = 0;

void statusLedTest(int r, int g, int b)   // wiring test from the web page: show a colour for 3 s
{
    rgbLedWrite(RGB_LED_PIN, constrain(r, 0, 255), constrain(g, 0, 255), constrain(b, 0, 255));
    ledTestUntil = millis() + 3000;
}

static void statusLed()
{
    static uint32_t last = 0;
    if (millis() - last < 200 || millis() < ledTestUntil) return;
    last = millis();
    if (laserOn())            rgbLedWrite(RGB_LED_PIN, 40, 0, 0);    // red   = laser on
    else if (webClients())    rgbLedWrite(RGB_LED_PIN, 0, 30, 0);    // green = someone viewing
    else if (wifiOK)          rgbLedWrite(RGB_LED_PIN, 0, 0, 30);    // blue  = ready
    else                      rgbLedWrite(RGB_LED_PIN, 30, 12, 0);   // amber = no network
}

static void button()
{
    static bool was = false;
    static uint32_t downMs = 0;
    bool down = digitalRead(BUTTON_PIN) == LOW;
    if (down && !was) downMs = millis();
    if (!down && was)
    {
        uint32_t held = millis() - downMs;
        if (held > 1500) { laserSet(false); gimbalCenter(); oledMessage("Centre + laser off"); }   // long press = safe home
        else if (held > 40) oledNextPage();                                                        // short press = next page
    }
    was = down;
}

static void serialCommands()
{
    static String line;
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (line.length()) Serial.println(commandRun(line));
            line = "";
        }
        else if (line.length() < 64) line += c;
    }
}

static void sendStatus()
{
    static uint32_t last = 0;
    if (millis() - last < STATUS_INTERVAL_MS || webClients() == 0) return;
    last = millis();
    char js[1024];
    int n = snprintf(js, sizeof(js),
             "{\"t\":\"s\",%s,\"pan\":%.1f,\"tilt\":%.1f,\"tp\":%.1f,\"tt\":%.1f,\"laser\":%s,\"laserLeft\":%lu,\"mode\":\"%s\","
             "\"speed\":%.0f,\"trimPan\":%.0f,\"trimTilt\":%.0f,\"fps\":%.1f,\"kb\":%.1f,\"viewers\":%d,\"clients\":%d,"
             "\"ip\":\"%s\",\"ap\":%s,\"heap\":%lu,\"rssi\":%d,\"panEn\":%s,\"tiltEn\":%s,\"panUs\":%d,\"tiltUs\":%d,"
             "\"oled\":%s,\"cam\":%s,\"captureMs\":%lu,\"sendMs\":%lu,\"captureFails\":%lu,\"loopGapMs\":%lu}",
             gimbalCalJson().c_str(), gimbalPan(), gimbalTilt(), gimbalTargetPan(), gimbalTargetTilt(), laserOn() ? "true" : "false",
             (unsigned long)laserSecondsLeft(), gimbalModeName(), gimbalSpeed(), gimbalTrimPan(), gimbalTrimTilt(),
             cameraFps(), cameraFrameBytes() / 1024.0f, cameraViewers(), webClients(), wifiAddress().c_str(),
             wifiIsAP() ? "true" : "false", (unsigned long)ESP.getFreeHeap(), wifiIsAP() ? 0 : (int)WiFi.RSSI(),
             servoEnabled('P') ? "true" : "false", servoEnabled('T') ? "true" : "false", servoPulseUs('P'), servoPulseUs('T'),
             oledIsDetected() ? "true" : "false", cameraOK ? "true" : "false",
             (unsigned long)cameraCaptureMs(), (unsigned long)cameraSendMs(),
             (unsigned long)cameraCaptureFailures(), (unsigned long)maxLoopGapMs);
    maxLoopGapMs = 0;
    if (n > 0 && n < (int)sizeof(js)) webSendStatus(js);
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);
    Serial.println("\n=== ANVYXO GIMBAL - PHASE 3 ===");
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    bool oledOK = oledBegin();
    Serial.println(oledOK ? "OLED: OK" : "OLED: not found (check SDA 21 / SCL 47 / 3V3 / GND)");
    oledBoot("starting servos");

    gimbalBegin();                        // servos go to centre (with saved trim)

    oledBoot("starting camera");
    cameraOK = cameraBegin();
    Serial.println(cameraOK ? "CAMERA: OK" : "CAMERA: FAILED");

    oledBoot("connecting WiFi");
    wifiOK = wifiBegin();
    if (wifiOK)
    {
        webBegin();
        ArduinoOTA.setHostname(MDNS_NAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart([]() { laserSet(false); oledBoot("updating..."); });
        ArduinoOTA.begin();
    }
    oledBoot(wifiOK ? wifiAddress().c_str() : "no network");
    delay(1200);
    oledSetPage(PAGE_EYES);
    Serial.println("Ready. Type HELP in the serial monitor for commands.");
}

void loop()
{
    static uint32_t previousLoopMs = 0;
    uint32_t now = millis();
    if (previousLoopMs)
    {
        uint32_t gap = now - previousLoopMs;
        if (gap > maxLoopGapMs) maxLoopGapMs = gap;
    }
    previousLoopMs = now;
    if (wifiOK)
    {
        webHandle();
        ArduinoOTA.handle();
        sendStatus();
    }
    gimbalUpdate();
    button();
    serialCommands();
    statusLed();

    String ip = wifiOK ? wifiAddress() : String("no network");   // must outlive info
    OledInfo info = {wifiOK, wifiIsAP(), cameraOK, ip.c_str(),
                     wifiOK && !wifiIsAP() ? (int)WiFi.RSSI() : 0, cameraViewers(), cameraFps(),
                     gimbalPan(), gimbalTilt(), laserOn(), laserSecondsLeft(), gimbalModeName(), gimbalMoving()};
    oledUpdate(info);
    delay(1);
}
