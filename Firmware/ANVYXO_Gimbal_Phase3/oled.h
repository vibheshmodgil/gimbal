#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

struct OledInfo
{
    bool wifiOK;
    bool apMode;
    bool cameraOK;
    const char *ip;
    int rssi;
    int clients;
    float fps;
    float pan, tilt;
    bool laser;
    uint32_t laserLeft;
    const char *mode;
    bool moving;
};

enum OledPage { PAGE_STATUS, PAGE_GIMBAL, PAGE_EYES, PAGE_TEXT, PAGE_COUNT };

bool oledBegin();
bool oledIsDetected();
void oledBoot(const char *line);          // splash screen with a progress line
void oledSetPage(int page);
void oledNextPage();
int oledPage();
const char *oledPageName();
void oledMessage(const char *text, uint32_t ms = 1500);  // short pop-up (e.g. "Preset 2 saved")
void oledUpdate(const OledInfo &info);    // call every loop; redraws at OLED_REFRESH_MS

// Controls + wiring test
bool oledReinit();                        // probe the I2C bus again (after re-plugging wires)
String oledScan();                        // addresses found on the OLED bus, e.g. "0x3C"
void oledTest(int pattern);               // 0 off (normal pages), 1 all white, 2 checker, 3 frame + grid
void oledContrast(int v);                 // 0..255
void oledInvert(bool on);
void oledPower(bool on);
void oledSetText(const String &t);        // shows on the TEXT page ('|' = new line)
// Eyes page mood, set by the tracker ("MOOD:HAPPY" ...). Expires to NEUTRAL 6 s after the last set, so a stopped
// tracker never leaves the eyes stuck. SURPRISED is a 1.2 s pulse on top of the current mood.
enum OledMood { MOOD_NEUTRAL, MOOD_HAPPY, MOOD_SEARCH, MOOD_SAD, MOOD_ANGRY, MOOD_SLEEP, MOOD_SURPRISED, MOOD_COUNT };
bool oledMood(const String &name);        // false = unknown name
const char *oledMoodName();
int oledContrastValue();
bool oledInverted();
bool oledPowered();
int oledTestPattern();

#endif
