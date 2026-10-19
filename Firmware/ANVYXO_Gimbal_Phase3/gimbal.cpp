#include "gimbal.h"
#include "config.h"

#include <Preferences.h>

static Preferences prefs;

static float curPan = 0, curTilt = 0;
static float tgtPan = 0, tgtTilt = 0;
static float speedDps = DEFAULT_SPEED_DPS;
static float trimPan = 0, trimTilt = 0;
// calibration (all saved in flash): direction, soft limits, done flag
static bool invPan = PAN_INVERT, invTilt = TILT_INVERT;
static float limPan[2] = {-PAN_LIMIT_DEG, PAN_LIMIT_DEG}, limTilt[2] = {-TILT_LIMIT_DEG, TILT_LIMIT_DEG};
static bool calibrated = false, calibrating = false;
static GimbalMode mode = MODE_MANUAL;
static uint32_t lastMs = 0;

// patrol / scan / gesture state
static int sweepDir = 1;
static float scanTilt = 0;
static int gestureStep = 0;

static bool laser = false;
static uint32_t laserOnMs = 0;

// gestures: list of (pan, tilt) key positions relative to where the gesture started
static const float NOD_KEYS[][2]   = {{0, 15}, {0, -15}, {0, 15}, {0, -15}, {0, 0}};
static const float SHAKE_KEYS[][2] = {{25, 0}, {-25, 0}, {25, 0}, {-25, 0}, {0, 0}};
#define TEST_SPEED_DPS 60.0f
static float gestureBasePan = 0, gestureBaseTilt = 0;

static bool panEn = true, tiltEn = true;
static int panUs = 1500, tiltUs = 1500;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int writeServo(uint8_t pin, bool en, float servoDeg, int minUs, int maxUs)
{
    servoDeg = clampf(servoDeg, 0, 180);
    float us = minUs + (maxUs - minUs) * servoDeg / 180.0f;
    uint32_t duty = (uint32_t)(us * ((1 << SERVO_RES_BITS) - 1) / (1000000.0f / SERVO_FREQ_HZ));
    ledcWrite(pin, en ? duty : 0);   // 0 = no pulses: servo stops holding
    return (int)us;
}

static void output()
{
    float p = 90.0f + (invPan ? -1 : 1) * (curPan + trimPan);
    float t = 90.0f + (invTilt ? -1 : 1) * (curTilt + trimTilt);
    panUs = writeServo(PAN_PIN, panEn, p, PAN_MIN_US, PAN_MAX_US);
    tiltUs = writeServo(TILT_PIN, tiltEn, t, TILT_MIN_US, TILT_MAX_US);
}

static bool reached() { return fabsf(curPan - tgtPan) < 0.3f && fabsf(curTilt - tgtTilt) < 0.3f; }

// what the servo can physically reach (0..180 deg) in gimbal angles: angle + trim must stay within +-90
static float reachLo(float trim) { return -90.0f - trim; }
static float reachHi(float trim) { return 90.0f - trim; }

static void setTargetRaw(float pan, float tilt)
{
    // calibrating: whole servo range, limits ignored. Normal: soft limits, never past the servo's own end.
    float pl = calibrating ? -999 : limPan[0], ph = calibrating ? 999 : limPan[1];
    float tl = calibrating ? -999 : limTilt[0], th = calibrating ? 999 : limTilt[1];
    tgtPan = clampf(clampf(pan, pl, ph), reachLo(trimPan), reachHi(trimPan));
    tgtTilt = clampf(clampf(tilt, tl, th), reachLo(trimTilt), reachHi(trimTilt));
}

void gimbalBegin()
{
    prefs.begin("gimbal", false);
    trimPan = prefs.getFloat("trimPan", 0);
    trimTilt = prefs.getFloat("trimTilt", 0);
    speedDps = prefs.getFloat("speed", DEFAULT_SPEED_DPS);
    invPan = prefs.getBool("invP", PAN_INVERT);
    invTilt = prefs.getBool("invT", TILT_INVERT);
    limPan[0] = prefs.getFloat("lp0", -PAN_LIMIT_DEG);
    limPan[1] = prefs.getFloat("lp1", PAN_LIMIT_DEG);
    limTilt[0] = prefs.getFloat("lt0", -TILT_LIMIT_DEG);
    limTilt[1] = prefs.getFloat("lt1", TILT_LIMIT_DEG);
    calibrated = prefs.getBool("cal", false);

    // LEDC channels 4/5 -> timer 2, so the camera's timer 0 (20 MHz XCLK) is untouched
    ledcAttachChannel(PAN_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS, SERVO_LEDC_PAN_CH);
    ledcAttachChannel(TILT_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS, SERVO_LEDC_TILT_CH);
    output();

    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    lastMs = millis();

    Serial.printf("GIMBAL: trim pan %.1f tilt %.1f, limits pan %.0f..%.0f tilt %.0f..%.0f, %s\n", trimPan, trimTilt,
                  limPan[0], limPan[1], limTilt[0], limTilt[1], calibrated ? "calibrated" : "NOT CALIBRATED");
}

void gimbalUpdate()
{
    uint32_t now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    if (dt > 0.1f) dt = 0.1f;

    // --- modes pick the target ---
    switch (mode)
    {
        case MODE_PATROL:
            if (reached()) { sweepDir = -sweepDir; setTargetRaw(limPan[sweepDir > 0], tgtTilt); }
            break;
        case MODE_SCAN:
            if (reached())
            {
                sweepDir = -sweepDir;
                scanTilt += 15.0f;
                if (scanTilt > limTilt[1]) scanTilt = limTilt[0];
                setTargetRaw(limPan[sweepDir > 0], scanTilt);
            }
            break;
        case MODE_NOD:
        case MODE_SHAKE:
        case MODE_TEST_PAN:
        case MODE_TEST_TILT:
            if (reached())
            {
                // wiring tests: one axis at a time, limit to limit, back to centre
                const float tpan[3][2] = {{limPan[0], 0}, {limPan[1], 0}, {0, 0}};
                const float ttilt[3][2] = {{0, limTilt[0]}, {0, limTilt[1]}, {0, 0}};
                const float(*keys)[2] = mode == MODE_NOD ? NOD_KEYS : mode == MODE_SHAKE ? SHAKE_KEYS
                                      : mode == MODE_TEST_PAN ? tpan : ttilt;
                int n = (mode == MODE_TEST_PAN || mode == MODE_TEST_TILT) ? 3 : 5;
                if (gestureStep >= n) { mode = MODE_MANUAL; break; }
                setTargetRaw(gestureBasePan + keys[gestureStep][0], gestureBaseTilt + keys[gestureStep][1]);
                gestureStep++;
            }
            break;
        default:
            break;
    }

    // --- smooth motion toward the target ---
    float spd = (mode == MODE_PATROL || mode == MODE_SCAN) ? min(speedDps, PATROL_SPEED_DPS)
              : (mode == MODE_TEST_PAN || mode == MODE_TEST_TILT) ? min(speedDps, TEST_SPEED_DPS) : speedDps;
    float step = spd * dt;
    float dp = tgtPan - curPan, dtl = tgtTilt - curTilt;
    curPan += clampf(dp, -step, step);
    curTilt += clampf(dtl, -step, step);
    output();

    // --- laser safety ---
    if (laser && now - laserOnMs > LASER_MAX_ON_MS)
    {
        laserSet(false);
        Serial.println("LASER: auto-off (safety timer)");
    }
}

void gimbalSetTarget(float pan, float tilt) { mode = MODE_MANUAL; setTargetRaw(pan, tilt); }
void gimbalMoveBy(float dPan, float dTilt) { mode = MODE_MANUAL; setTargetRaw(tgtPan + dPan, tgtTilt + dTilt); }
void gimbalCenter() { gimbalSetTarget(0, 0); }

void gimbalSetMode(GimbalMode m)
{
    mode = m;
    sweepDir = 1;
    gestureStep = 0;
    gestureBasePan = tgtPan;
    gestureBaseTilt = clampf(tgtTilt, limTilt[0] + 15, limTilt[1] - 15);
    if (m == MODE_MANUAL) setTargetRaw(curPan, curTilt);   // Stop = freeze where it is now
    if (m == MODE_PATROL) setTargetRaw(limPan[1], tgtTilt);
    if (m == MODE_TEST_PAN) { gestureBasePan = 0; gestureBaseTilt = tgtTilt; }
    if (m == MODE_TEST_TILT) { gestureBasePan = tgtPan; gestureBaseTilt = 0; }
    if (m == MODE_SCAN) { scanTilt = limTilt[0]; setTargetRaw(limPan[0], scanTilt); sweepDir = -1; }
}

GimbalMode gimbalMode() { return mode; }

const char *gimbalModeName()
{
    switch (mode)
    {
        case MODE_PATROL: return "PATROL";
        case MODE_SCAN:   return "SCAN";
        case MODE_NOD:    return "NOD";
        case MODE_SHAKE:  return "SHAKE";
        case MODE_TEST_PAN:  return "TEST PAN";
        case MODE_TEST_TILT: return "TEST TILT";
        default:          return "MANUAL";
    }
}

void gimbalSetSpeed(float dps) { speedDps = clampf(dps, 10, 360); }
float gimbalSpeed() { return speedDps; }

void gimbalSetTrim(float panTrim, float tiltTrim)
{
    trimPan = clampf(panTrim, -90, 90);
    trimTilt = clampf(tiltTrim, -90, 90);
}

void gimbalSaveSettings()
{
    prefs.putFloat("trimPan", trimPan);
    prefs.putFloat("trimTilt", trimTilt);
    prefs.putFloat("speed", speedDps);
    prefs.putBool("invP", invPan);
    prefs.putBool("invT", invTilt);
    prefs.putFloat("lp0", limPan[0]);
    prefs.putFloat("lp1", limPan[1]);
    prefs.putFloat("lt0", limTilt[0]);
    prefs.putFloat("lt1", limTilt[1]);
    prefs.putBool("cal", calibrated);
}

// ------------------------------------------------------------------ calibration
// Every step moves the reference, never the servo: the head stays exactly where it is.
void gimbalCalibrate(bool on)
{
    mode = MODE_MANUAL;
    calibrating = on;
    setTargetRaw(curPan, curTilt);   // leaving: pull back inside the (new) limits
}

bool gimbalCalibrating() { return calibrating; }

void gimbalZeroHere(char axis)   // the current position becomes 0 (straight ahead / level)
{
    float &cur = axis == 'P' ? curPan : curTilt, &tgt = axis == 'P' ? tgtPan : tgtTilt;
    float &trim = axis == 'P' ? trimPan : trimTilt, *lim = axis == 'P' ? limPan : limTilt;
    float shift = cur;
    trim = clampf(trim + shift, -90, 90);
    lim[0] = min(lim[0] - shift, -1.0f);   // limits stay where they physically were
    lim[1] = max(lim[1] - shift, 1.0f);
    cur = 0;
    tgt -= shift;
    output();
    gimbalSaveSettings();
}

void gimbalLimitHere(char axis, int side)   // side -1 = left/down end, +1 = right/up end
{
    float cur = axis == 'P' ? curPan : curTilt, *lim = axis == 'P' ? limPan : limTilt;
    if (side < 0) lim[0] = min(cur, -1.0f);
    else lim[1] = max(cur, 1.0f);
    gimbalSaveSettings();
}

void gimbalInvert(char axis)   // flip direction; the head does not move
{
    bool &inv = axis == 'P' ? invPan : invTilt;
    float &cur = axis == 'P' ? curPan : curTilt, &tgt = axis == 'P' ? tgtPan : tgtTilt;
    float &trim = axis == 'P' ? trimPan : trimTilt, *lim = axis == 'P' ? limPan : limTilt;
    inv = !inv;
    cur = -cur;
    tgt = -tgt;
    trim = -trim;
    float lo = lim[0];
    lim[0] = -lim[1];
    lim[1] = -lo;
    output();
    gimbalSaveSettings();
}

// Re-seating the horn: drive the servo to a real servo angle (0 = one end, 90 = middle, 180 = other end) with the
// trim cleared, so the head can be pulled off and pushed back on pointing the matching way. Old zero/limits are void.
void gimbalMountPose(char axis, float servoDeg)
{
    float &cur = axis == 'P' ? curPan : curTilt, &tgt = axis == 'P' ? tgtPan : tgtTilt;
    float &trim = axis == 'P' ? trimPan : trimTilt, *lim = axis == 'P' ? limPan : limTilt;
    float def = axis == 'P' ? PAN_LIMIT_DEG : TILT_LIMIT_DEG;
    bool inv = axis == 'P' ? invPan : invTilt;
    mode = MODE_MANUAL;
    calibrating = true;
    calibrated = false;
    cur += trim;   // same servo pulse, new reference: no jump, then it drives smoothly to servoDeg
    tgt += trim;
    trim = 0;
    lim[0] = -def;
    lim[1] = def;
    tgt = (inv ? -1 : 1) * (clampf(servoDeg, 0, 180) - 90.0f);
    gimbalSaveSettings();
}

void gimbalCalDone()
{
    calibrated = true;
    gimbalCalibrate(false);
    gimbalSaveSettings();
}

void gimbalCalReset()
{
    trimPan = trimTilt = 0;
    invPan = PAN_INVERT;
    invTilt = TILT_INVERT;
    limPan[0] = -PAN_LIMIT_DEG; limPan[1] = PAN_LIMIT_DEG;
    limTilt[0] = -TILT_LIMIT_DEG; limTilt[1] = TILT_LIMIT_DEG;
    calibrated = false;
    curPan = curTilt = 0;
    setTargetRaw(0, 0);
    output();
    gimbalSaveSettings();
}

bool gimbalCalibrated() { return calibrated; }

String gimbalCalJson()
{
    char j[280];
    snprintf(j, sizeof(j), "\"cal\":%s,\"calOn\":%s,\"invP\":%s,\"invT\":%s,\"lp\":[%.1f,%.1f],\"lt\":[%.1f,%.1f],"
             "\"rp\":[%.1f,%.1f],\"rt\":[%.1f,%.1f]",
             calibrated ? "true" : "false", calibrating ? "true" : "false", invPan ? "true" : "false",
             invTilt ? "true" : "false", limPan[0], limPan[1], limTilt[0], limTilt[1],
             reachLo(trimPan), reachHi(trimPan), reachLo(trimTilt), reachHi(trimTilt));
    return j;
}

void gimbalSavePreset(int n)
{
    if (n < 1 || n > 4) return;
    char k[8];
    snprintf(k, sizeof(k), "p%dp", n); prefs.putFloat(k, tgtPan);
    snprintf(k, sizeof(k), "p%dt", n); prefs.putFloat(k, tgtTilt);
}

bool gimbalGotoPreset(int n)
{
    if (n < 1 || n > 4) return false;
    char kp[8], kt[8];
    snprintf(kp, sizeof(kp), "p%dp", n);
    snprintf(kt, sizeof(kt), "p%dt", n);
    if (!prefs.isKey(kp)) return false;
    gimbalSetTarget(prefs.getFloat(kp, 0), prefs.getFloat(kt, 0));
    return true;
}

float gimbalPan() { return curPan; }
float gimbalTilt() { return curTilt; }
float gimbalTargetPan() { return tgtPan; }
float gimbalTargetTilt() { return tgtTilt; }
float gimbalTrimPan() { return trimPan; }
float gimbalTrimTilt() { return trimTilt; }
bool gimbalMoving() { return !reached(); }

void laserSet(bool on)
{
    laser = on;
    if (on) laserOnMs = millis();
    digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

bool laserOn() { return laser; }

uint32_t laserSecondsLeft()
{
    if (!laser) return 0;
    uint32_t used = millis() - laserOnMs;
    return used >= LASER_MAX_ON_MS ? 0 : (LASER_MAX_ON_MS - used) / 1000;
}

void servoEnable(char axis, bool on)
{
    if (axis == 'P') panEn = on;
    else tiltEn = on;
    output();
}

bool servoEnabled(char axis) { return axis == 'P' ? panEn : tiltEn; }
int servoPulseUs(char axis) { return axis == 'P' ? panUs : tiltUs; }
