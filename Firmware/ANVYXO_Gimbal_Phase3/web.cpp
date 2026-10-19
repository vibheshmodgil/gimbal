#include "web.h"
#include "config.h"
#include "gimbal.h"
#include "oled.h"
#include "camera.h"
#include "web_page.h"

#include <WiFi.h>
#include <esp_system.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>

static WebServer httpServer(HTTP_PORT);
static WebSocketsServer webSocket(WEBSOCKET_PORT);
static bool apMode = false;

// ------------------------------------------------------------------ commands
static bool parse2(const String &s, float &a, float &b)
{
    int c = s.indexOf(',');
    if (c < 0) return false;
    a = s.substring(0, c).toFloat();
    b = s.substring(c + 1).toFloat();
    return true;
}

void statusLedTest(int r, int g, int b);   // in the .ino

static const char *resetReason()
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "crash";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_DEEPSLEEP: return "deep-sleep";
        case ESP_RST_EXT:       return "reset pin";
        case ESP_RST_USB:       return "usb";
        default:                return "other";
    }
}

static String diagJson()
{
    char j[520];
    snprintf(j, sizeof(j),
             "{\"t\":\"d\",\"fw\":\"" FW_VERSION "\",\"oled\":%s,\"i2c\":\"%s\",\"cam\":%s,\"sensor\":\"%s\","
             "\"psram\":%u,\"heap\":%u,\"rssi\":%d,\"up\":%lu,\"reset\":\"%s\",\"temp\":%.1f,\"cpu\":%u,"
             "\"panEn\":%s,\"tiltEn\":%s,\"panUs\":%d,\"tiltUs\":%d,\"oledC\":%d,\"oledInv\":%s,\"oledOn\":%s,"
             "\"oledTest\":%d,\"page\":\"%s\",\"pins\":\"OLED SDA %d SCL %d | PAN %d | TILT %d | LASER %d\"}",
             oledIsDetected() ? "true" : "false", oledScan().c_str(), cameraSensorName()[0] != 'n' ? "true" : "false",
             cameraSensorName(), (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap(), apMode ? 0 : (int)WiFi.RSSI(),
             (unsigned long)(millis() / 1000), resetReason(), temperatureRead(), (unsigned)getCpuFrequencyMhz(),
             servoEnabled('P') ? "true" : "false", servoEnabled('T') ? "true" : "false", servoPulseUs('P'), servoPulseUs('T'),
             oledContrastValue(), oledInverted() ? "true" : "false", oledPowered() ? "true" : "false", oledTestPattern(),
             oledPageName(), OLED_SDA_PIN, OLED_SCL_PIN, PAN_PIN, TILT_PIN, LASER_PIN);
    return j;
}

String commandRun(const String &raw)
{
    String cmd = raw;
    cmd.trim();
    String up = cmd;
    up.toUpperCase();
    float a, b;

    if (up == "PING") return "PONG";
    if (up == "C" || up == "CENTER" || up == "CENTRE") { gimbalCenter(); return "OK centre"; }
    if (up.startsWith("PT:") && parse2(cmd.substring(3), a, b)) { gimbalSetTarget(a, b); return "OK"; }
    if (up.startsWith("MV:") && parse2(cmd.substring(3), a, b)) { gimbalMoveBy(a, b); return "OK"; }
    if (up.startsWith("L:"))
    {
        char v = up.length() > 2 ? up[2] : 'T';
        laserSet(v == '1' ? true : v == '0' ? false : !laserOn());
        oledMessage(laserOn() ? "LASER ON" : "LASER OFF", 800);
        return laserOn() ? "OK laser on" : "OK laser off";
    }
    if (up.startsWith("MODE:"))
    {
        String m = up.substring(5);
        if (m == "PATROL") gimbalSetMode(MODE_PATROL);
        else if (m == "SCAN") gimbalSetMode(MODE_SCAN);
        else if (m == "NOD") gimbalSetMode(MODE_NOD);
        else if (m == "SHAKE") gimbalSetMode(MODE_SHAKE);
        else if (m == "TESTPAN") gimbalSetMode(MODE_TEST_PAN);
        else if (m == "TESTTILT") gimbalSetMode(MODE_TEST_TILT);
        else gimbalSetMode(MODE_MANUAL);
        oledMessage(gimbalModeName(), 700);
        return String("OK mode ") + gimbalModeName();
    }
    if (up.startsWith("SPD:")) { gimbalSetSpeed(cmd.substring(4).toFloat()); return "OK speed"; }
    if (up.startsWith("TRIM:") && parse2(cmd.substring(5), a, b)) { gimbalSetTrim(a, b); return "OK trim (send SAVE to keep)"; }
    if (up == "SAVE") { gimbalSaveSettings(); oledMessage("Settings saved"); return "OK saved"; }
    if (up.startsWith("CAL:"))   // calibration wizard (see gimbal.h); jog with MV:dp,dt
    {
        String a = up.substring(4);
        if (a == "ON" || a == "OFF") gimbalCalibrate(a == "ON");
        else if (a == "ZP" || a == "ZT") gimbalZeroHere(a[1]);
        else if (a == "IP" || a == "IT") gimbalInvert(a[1]);
        else if (a.length() > 2 && a[0] == 'M' && (a[1] == 'P' || a[1] == 'T')) gimbalMountPose(a[1], a.substring(2).toFloat());
        else if (a.length() == 3 && a[0] == 'L' && (a[1] == 'P' || a[1] == 'T')) gimbalLimitHere(a[1], a[2] == '+' ? 1 : -1);
        else if (a == "DONE") { gimbalCalDone(); oledMessage("Calibrated"); }
        else if (a == "RESET") gimbalCalReset();
        else return "ERR CAL:ON|OFF|ZP|ZT|IP|IT|MP<0-180>|MT<0-180>|LP-|LP+|LT-|LT+|DONE|RESET";
        return "{" + gimbalCalJson() + "}";
    }
    if (up.startsWith("SAVEP:"))
    {
        int n = cmd.substring(6).toInt();
        gimbalSavePreset(n);
        char m[20]; snprintf(m, sizeof(m), "Preset %d saved", n); oledMessage(m);
        return "OK preset saved";
    }
    if (up.startsWith("GO:")) return gimbalGotoPreset(cmd.substring(3).toInt()) ? "OK" : "ERR preset empty";
    if (up.startsWith("OLED:")) { oledTest(0); oledSetPage(cmd.substring(5).toInt()); return String("OK page ") + oledPageName(); }

    // --- wiring test / diagnostics ---
    if (up == "DIAG") return diagJson();
    if (up.startsWith("SRV:") && up.length() >= 7)
    {
        char ax = up[4];
        servoEnable(ax, up[6] == '1');
        return String("OK ") + (ax == 'P' ? "pan" : "tilt") + (up[6] == '1' ? " on" : " off (limp)");
    }
    if (up.startsWith("LED:"))
    {
        int r = 0, g = 0, bl = 0;
        sscanf(cmd.c_str() + 4, "%d,%d,%d", &r, &g, &bl);
        statusLedTest(r, g, bl);
        return "OK led";
    }
    if (up == "REBOOT") { delay(100); ESP.restart(); }

    // --- screen ---
    if (up == "OLEDINIT") return oledReinit() ? "OK OLED found at 0x3C" : "ERR OLED not found - I2C sees: " + oledScan();
    if (up == "I2C") return "I2C: " + oledScan();
    if (up.startsWith("OLEDTEST:")) { oledTest(cmd.substring(9).toInt()); return "OK test"; }
    if (up.startsWith("OLEDC:")) { oledContrast(cmd.substring(6).toInt()); return "OK contrast"; }
    if (up.startsWith("OLEDINV:")) { oledInvert(up[8] == '1'); return "OK invert"; }
    if (up.startsWith("OLEDPWR:")) { oledPower(up[8] == '1'); return "OK power"; }
    if (up.startsWith("OLEDTXT:")) { oledTest(0); oledSetText(cmd.substring(8)); return "OK text"; }

    // --- camera ---
    if (up == "CAMGET") return cameraJson();
    if (up.startsWith("CAM:"))
    {
        int c = cmd.indexOf(',');
        if (c < 0) return "ERR CAM:key,value";
        String key = cmd.substring(4, c);
        key.toLowerCase();
        return cameraSet(key, cmd.substring(c + 1).toInt()) ? cameraJson() : "ERR camera " + key;
    }
    if (up.startsWith("CAMP:")) return cameraPreset(up.substring(5)) ? cameraJson() : "ERR preset FAST|BAL|HQ|MAX";
    if (up == "CAMSAVE") { cameraSave(); oledMessage("Camera saved"); return "OK camera saved"; }
    if (up == "CAMRESET") { cameraFactory(); return cameraJson(); }

    if (up == "HELP" || up == "?")
        return "PT:p,t MV:dp,dt C L:1/0/T MODE:PATROL|SCAN|NOD|SHAKE|TESTPAN|TESTTILT|MANUAL SPD:x TRIM:p,t SAVE SAVEP:n GO:n "
               "| CAL:ON|OFF|ZP|ZT|IP|IT|MPdeg|MTdeg|LP-|LP+|LT-|LT+|DONE|RESET "
               "| SRV:P|T,0|1 LED:r,g,b DIAG REBOOT | OLED:n OLEDINIT I2C OLEDTEST:0-3 OLEDC:0-255 OLEDINV:0|1 OLEDPWR:0|1 OLEDTXT:a|b "
               "| CAMGET CAM:key,v CAMP:FAST|BAL|HQ|MAX CAMSAVE CAMRESET | PING";
    return "ERR unknown (send HELP)";
}

// ------------------------------------------------------------------ websocket
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
        case WStype_CONNECTED:
            Serial.printf("WS client %u connected\n", num);
            oledMessage("Viewer connected", 900);
            break;
        case WStype_DISCONNECTED:
            Serial.printf("WS client %u disconnected\n", num);
            if (webSocket.connectedClients() == 0 && laserOn())
            {
                laserSet(false);   // safety: nobody watching -> laser off
                Serial.println("LASER: off (no viewers)");
            }
            break;
        case WStype_TEXT:
        {
            String cmd((const char *)payload, length);
            String reply = commandRun(cmd);
            if (!cmd.startsWith("PT:") && !cmd.startsWith("MV:")) webSocket.sendTXT(num, reply);
            break;
        }
        default:
            break;
    }
}

// ------------------------------------------------------------------ wifi / server
bool wifiBegin()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WIFI: connecting");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WIFI: home network failed -> starting hotspot " AP_SSID);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);
        apMode = WiFi.softAP(AP_SSID, AP_PASSWORD);
        return apMode;
    }
    apMode = false;
    return true;
}

bool wifiIsAP() { return apMode; }
String wifiAddress() { return apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString(); }

void webBegin()
{
    httpServer.on("/", HTTP_GET, []() {
        httpServer.sendHeader("Content-Encoding", "gzip");
        httpServer.sendHeader("Cache-Control", "no-cache");
        httpServer.send_P(200, "text/html", (const char *)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
    });
    httpServer.on("/cmd", HTTP_GET, []() {   // e.g. http://anvyxo.local/cmd?c=PT:30,10
        httpServer.send(200, "text/plain", commandRun(httpServer.arg("c")));
    });
    httpServer.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    cameraStreamBegin();
    if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", HTTP_PORT);
    Serial.printf("WEB: http://%s  (or http://%s.local)  websocket :%d\n", wifiAddress().c_str(), MDNS_NAME, WEBSOCKET_PORT);
}

void webHandle()
{
    httpServer.handleClient();
    webSocket.loop();
}

int webClients() { return webSocket.connectedClients(); }

void webSendStatus(const String &json)
{
    if (webSocket.connectedClients() > 0)
    {
        String s = json;   // broadcastTXT needs a mutable String
        webSocket.broadcastTXT(s);
    }
}
