#include "oled.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static TwoWire OLED_I2C = TwoWire(0);
static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &OLED_I2C, -1);

static bool detected = false;
static int page = PAGE_EYES;
static uint32_t lastDraw = 0;
static char msg[24] = "";
static uint32_t msgUntil = 0;
static uint32_t nextBlink = 3000, blinkUntil = 0;

static const char *PAGE_NAMES[PAGE_COUNT] = {"STATUS", "GIMBAL", "EYES", "TEXT"};
static int testPattern = 0, contrast = 207;
static bool inverted = false, powered = true;
static String text = "ANVYXO|hello";
static void centered(const char *s, int y, int size);

bool oledBegin()
{
    static bool busStarted = false;
    if (!busStarted) busStarted = OLED_I2C.begin(OLED_SDA_PIN, OLED_SCL_PIN, 400000);
    OLED_I2C.beginTransmission(OLED_I2C_ADDRESS);
    detected = (OLED_I2C.endTransmission() == 0);
    if (!detected)
    {
        Serial.printf("OLED: not found at 0x%02X (SDA %d, SCL %d)\n", OLED_I2C_ADDRESS, OLED_SDA_PIN, OLED_SCL_PIN);
        return false;
    }
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS))
    {
        detected = false;
        return false;
    }
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.display();
    oledContrast(contrast);
    oledInvert(inverted);
    return true;
}

bool oledReinit()
{
    bool was = detected;
    detected = false;
    oledBegin();
    Serial.printf("OLED: re-probe %s -> %s\n", was ? "found" : "missing", detected ? "found" : "missing");
    lastDraw = 0;
    return detected;
}

String oledScan()
{
    String r;
    for (uint8_t a = 1; a < 127; a++)
    {
        OLED_I2C.beginTransmission(a);
        if (OLED_I2C.endTransmission() == 0)
        {
            char b[6];
            snprintf(b, sizeof(b), "0x%02X", a);
            if (r.length()) r += " ";
            r += b;
        }
    }
    return r.length() ? r : String("none");
}

void oledTest(int p) { testPattern = constrain(p, 0, 3); lastDraw = 0; }
void oledContrast(int v)
{
    contrast = constrain(v, 0, 255);
    if (!detected) return;
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(contrast);
}
void oledInvert(bool on) { inverted = on; if (detected) display.invertDisplay(on); }
void oledPower(bool on) { powered = on; if (detected) display.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF); }
void oledSetText(const String &t) { text = t.substring(0, 80); page = PAGE_TEXT; lastDraw = 0; }
int oledContrastValue() { return contrast; }
bool oledInverted() { return inverted; }
bool oledPowered() { return powered; }
int oledTestPattern() { return testPattern; }

static void drawTest()
{
    if (testPattern == 1) display.fillScreen(SSD1306_WHITE);
    else if (testPattern == 2)
    {
        for (int y = 0; y < OLED_HEIGHT; y += 8)
            for (int x = ((y / 8) % 2) * 8; x < OLED_WIDTH; x += 16) display.fillRect(x, y, 8, 8, SSD1306_WHITE);
    }
    else
    {
        display.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
        for (int x = 16; x < OLED_WIDTH; x += 16) display.drawFastVLine(x, 0, OLED_HEIGHT, SSD1306_WHITE);
        for (int y = 16; y < OLED_HEIGHT; y += 16) display.drawFastHLine(0, y, OLED_WIDTH, SSD1306_WHITE);
        display.fillRect(34, 24, 60, 16, SSD1306_BLACK);
        centered("128 x 64", 28, 1);
    }
}

static void drawText()
{
    // up to 4 lines, '|' splits; size 2 if every line fits 10 chars and there are <= 3 lines
    String lines[4];
    int n = 0, start = 0;
    while (n < 4)
    {
        int bar = text.indexOf('|', start);
        lines[n++] = text.substring(start, bar < 0 ? text.length() : bar);
        if (bar < 0) break;
        start = bar + 1;
    }
    bool big = n <= 3;
    for (int i = 0; i < n; i++) big = big && lines[i].length() <= 10;
    int size = big ? 2 : 1, lh = big ? 20 : 14;
    int y0 = (OLED_HEIGHT - n * lh) / 2 + 2;
    for (int i = 0; i < n; i++) centered(lines[i].c_str(), y0 + i * lh, size);
}

bool oledIsDetected() { return detected; }

static void centered(const char *s, int y, int size)
{
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(size);
    display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
    display.setCursor((OLED_WIDTH - w) / 2, y);
    display.print(s);
}

void oledBoot(const char *line)
{
    if (!detected) return;
    display.clearDisplay();
    centered("ANVYXO", 8, 2);
    centered("GIMBAL", 30, 1);
    display.drawFastHLine(20, 42, 88, SSD1306_WHITE);
    centered(line, 50, 1);
    display.display();
}

void oledSetPage(int p) { page = ((p % PAGE_COUNT) + PAGE_COUNT) % PAGE_COUNT; lastDraw = 0; }
void oledNextPage() { oledSetPage(page + 1); oledMessage(PAGE_NAMES[page], 700); }
int oledPage() { return page; }
const char *oledPageName() { return PAGE_NAMES[page]; }

void oledMessage(const char *text, uint32_t ms)
{
    strncpy(msg, text, sizeof(msg) - 1);
    msg[sizeof(msg) - 1] = 0;
    msgUntil = millis() + ms;
    lastDraw = 0;
}

static void drawStatus(const OledInfo &s)
{
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("ANVYXO");
    display.setCursor(64, 0);
    display.print(s.apMode ? "HOTSPOT" : (s.wifiOK ? "WiFi " : "NO WIFI"));
    if (s.wifiOK && !s.apMode) { display.print(s.rssi); }
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
    display.setCursor(0, 14);  display.print(s.ip);
    display.setCursor(0, 26);  display.printf("CAM %s  %.0f fps", s.cameraOK ? "OK" : "FAIL", s.fps);
    display.setCursor(0, 38);  display.printf("Viewers: %d", s.clients);
    display.setCursor(0, 50);  display.printf("P%+4.0f T%+4.0f %s", s.pan, s.tilt, s.laser ? "LASER" : "");
}

static void drawGimbal(const OledInfo &s)
{
    // pan: half-dial at left, needle shows direction
    const int cx = 40, cy = 50, r = 34;
    display.drawCircleHelper(cx, cy, r, 0x1 | 0x2, SSD1306_WHITE);
    display.drawFastHLine(cx - r, cy, 2 * r + 1, SSD1306_WHITE);
    for (int a = -90; a <= 90; a += 45)
    {
        float rad = (a - 90) * DEG_TO_RAD;
        display.drawLine(cx + (r - 4) * cos(rad), cy + (r - 4) * sin(rad), cx + r * cos(rad), cy + r * sin(rad), SSD1306_WHITE);
    }
    float rad = (s.pan - 90) * DEG_TO_RAD;
    display.drawLine(cx, cy, cx + (r - 6) * cos(rad), cy + (r - 6) * sin(rad), SSD1306_WHITE);
    display.fillCircle(cx, cy, 3, SSD1306_WHITE);
    if (s.laser) display.fillCircle(cx + (r - 6) * cos(rad), cy + (r - 6) * sin(rad), 2, SSD1306_WHITE);

    // tilt: vertical bar at right
    const int bx = 88, by = 4, bh = 46;
    display.drawRect(bx, by, 8, bh, SSD1306_WHITE);
    int ty = by + bh / 2 - (int)(s.tilt / TILT_LIMIT_DEG * (bh / 2 - 2));
    display.fillRect(bx, ty - 2, 8, 5, SSD1306_WHITE);
    display.drawFastHLine(bx - 3, by + bh / 2, 3, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(100, 4);  display.print("P");
    display.setCursor(100, 14); display.printf("%+.0f", s.pan);
    display.setCursor(100, 28); display.print("T");
    display.setCursor(100, 38); display.printf("%+.0f", s.tilt);
    display.setCursor(0, 0);    display.print(s.mode);   // above the dial (its top is y 16)
    display.setCursor(0, 56);   display.print(s.ip);     // IP always visible
    if (s.laser) { display.setCursor(74, 56); display.printf("LASER%lus", (unsigned long)s.laserLeft); }
}

static void drawEyes(const OledInfo &s)
{
    uint32_t now = millis();
    if (now > nextBlink) { blinkUntil = now + 140; nextBlink = now + 2500 + random(3500); }
    bool blink = now < blinkUntil;

    // pupils follow where the camera looks
    int ox = (int)(s.pan / PAN_LIMIT_DEG * 9);
    int oy = (int)(-s.tilt / TILT_LIMIT_DEG * 7);
    const int eyeW = 40, eyeH = blink ? 4 : 36, cy = 30;
    for (int side = -1; side <= 1; side += 2)
    {
        int ex = 64 + side * 26 - eyeW / 2;
        display.fillRoundRect(ex, cy - eyeH / 2, eyeW, eyeH, blink ? 2 : 10, SSD1306_WHITE);
        if (!blink)
        {
            display.fillCircle(ex + eyeW / 2 + ox, cy + oy, 8, SSD1306_BLACK);
            display.fillCircle(ex + eyeW / 2 + ox + 3, cy + oy - 3, 2, SSD1306_WHITE);
        }
    }
    centered(s.ip, 56, 1);                       // IP always visible under the eyes (eyes span y 12..48)
    if (s.laser) centered("* LASER *", 2, 1);    // laser warning moves above the eyes
}

void oledUpdate(const OledInfo &info)
{
    if (!detected) return;
    uint32_t now = millis();
    if (now - lastDraw < OLED_REFRESH_MS) return;
    lastDraw = now;

    display.clearDisplay();
    if (testPattern) { drawTest(); display.display(); return; }
    if (page == PAGE_STATUS) drawStatus(info);
    else if (page == PAGE_GIMBAL) drawGimbal(info);
    else if (page == PAGE_TEXT) drawText();
    else drawEyes(info);

    if (now < msgUntil)
    {
        display.fillRoundRect(8, 20, 112, 22, 4, SSD1306_BLACK);
        display.drawRoundRect(8, 20, 112, 22, 4, SSD1306_WHITE);
        centered(msg, 27, 1);
    }
    display.display();
}
