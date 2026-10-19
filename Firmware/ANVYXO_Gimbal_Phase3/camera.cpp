#include "camera.h"
#include "config.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <Preferences.h>

static Preferences camPrefs;
static bool ok = false;
static char sensorName[12] = "none";

static volatile int viewers = 0;
static volatile uint32_t frameCount = 0, lastBytes = 0;
static volatile uint32_t lastCaptureMs = 0, lastSendMs = 0, captureFailures = 0;
static volatile int fpsCap = CAMERA_FPS_CAP;

// Resolution list shown on the web page (same order there). Index is what "res" means.
static const framesize_t RES[] = {FRAMESIZE_QVGA, FRAMESIZE_CIF, FRAMESIZE_HVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA,
                                  FRAMESIZE_XGA, FRAMESIZE_HD, FRAMESIZE_SXGA, FRAMESIZE_UXGA};
static const int RES_N = sizeof(RES) / sizeof(RES[0]);
static int resIndex = 3;   // VGA

static int resIndexOf(framesize_t f)
{
    for (int i = 0; i < RES_N; i++)
        if (RES[i] == f) return i;
    return 3;
}

// ------------------------------------------------------------------ settings table
// key -> setter + getter on the sensor. "res" and "fps" are handled separately.
struct CamKey
{
    const char *key;
    int lo, hi;
    int (*set)(sensor_t *, int);
    int (*get)(sensor_t *);
};
#define K(name, lo, hi, setter, field) \
    {name, lo, hi, [](sensor_t *s, int v) { return s->setter(s, v); }, [](sensor_t *s) { return (int)s->status.field; }}
static const CamKey KEYS[] = {
    {"q", 4, 63, [](sensor_t *s, int v) { return s->set_quality(s, v); }, [](sensor_t *s) { return (int)s->status.quality; }},
    K("bri", -2, 2, set_brightness, brightness),
    K("con", -2, 2, set_contrast, contrast),
    K("sat", -2, 2, set_saturation, saturation),
    K("sfx", 0, 6, set_special_effect, special_effect),   // 0 none, 1 negative, 2 grey, 3 red, 4 green, 5 blue, 6 sepia
    K("awb", 0, 1, set_whitebal, awb),
    K("awbg", 0, 1, set_awb_gain, awb_gain),
    K("wbm", 0, 4, set_wb_mode, wb_mode),                 // 0 auto, 1 sunny, 2 cloudy, 3 office, 4 home
    K("aec", 0, 1, set_exposure_ctrl, aec),
    K("aec2", 0, 1, set_aec2, aec2),                       // DSP exposure: better in dim rooms
    K("ael", -5, 5, set_ae_level, ae_level),               // OV5640 -5..5, other sensors -2..2
    K("aecv", 0, 1200, set_aec_value, aec_value),          // manual exposure (aec off)
    K("agc", 0, 1, set_gain_ctrl, agc),
    K("agcg", 0, 30, set_agc_gain, agc_gain),              // manual gain (agc off)
    {"gc", 0, 6, [](sensor_t *s, int v) { return s->set_gainceiling(s, (gainceiling_t)v); }, [](sensor_t *s) { return (int)s->status.gainceiling; }},
    K("bpc", 0, 1, set_bpc, bpc),
    K("wpc", 0, 1, set_wpc, wpc),
    K("gma", 0, 1, set_raw_gma, raw_gma),
    K("lenc", 0, 1, set_lenc, lenc),
    K("vflip", 0, 1, set_vflip, vflip),
    K("hm", 0, 1, set_hmirror, hmirror),
    K("dcw", 0, 1, set_dcw, dcw),
};
#undef K

bool cameraSet(const String &key, int v)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return false;
    if (key == "res")
    {
        if (v < 0 || v >= RES_N) return false;
        if (s->set_framesize(s, RES[v]) != 0) return false;
        resIndex = v;
        return true;
    }
    if (key == "fps") { fpsCap = constrain(v, 1, 30); return true; }
    for (const CamKey &k : KEYS)
        if (key == k.key) return k.set(s, constrain(v, k.lo, k.hi)) == 0;
    return false;
}

bool cameraPreset(const String &name)
{
    // res index, quality, fps cap
    int r, q, f;
    if (name == "FAST") { r = 0; q = 10; f = 30; }        // 320x240, smooth
    else if (name == "BAL") { r = 3; q = 12; f = 25; }    // 640x480
    else if (name == "HQ") { r = 4; q = 10; f = 20; }     // 800x600, sharper
    else if (name == "MAX") { r = 8; q = 12; f = 8; }     // 1600x1200, slow
    else return false;
    return cameraSet("res", r) && cameraSet("q", q) && cameraSet("fps", f);
}

String cameraJson()
{
    sensor_t *s = esp_camera_sensor_get();
    String j = "{\"t\":\"c\",\"sensor\":\"";
    j += sensorName;
    j += "\",\"ok\":";
    j += ok ? "true" : "false";
    j += ",\"res\":" + String(resIndex) + ",\"fps\":" + String(fpsCap);
    if (s)
        for (const CamKey &k : KEYS) j += ",\"" + String(k.key) + "\":" + String(k.get(s));
    return j + "}";
}

void cameraSave()
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    camPrefs.putInt("res", resIndex);
    camPrefs.putInt("fps", fpsCap);
    for (const CamKey &k : KEYS)
    {
        int value = k.get(s);
        if (value >= k.lo && value <= k.hi) camPrefs.putInt(k.key, value);
        else camPrefs.remove(k.key);   // some sensors report unsupported/invalid status values
    }
}

static void applyDefaults()
{
    cameraSet("res", resIndexOf(CAMERA_FRAME_SIZE));
    cameraSet("q", CAMERA_JPEG_QUALITY);
    cameraSet("fps", CAMERA_FPS_CAP);
    cameraSet("vflip", CAMERA_VFLIP);
    cameraSet("hm", CAMERA_HMIRROR);
    // exposure: auto on, darker AE target, capped gain, lens shading + pixel fixes on
    cameraSet("aec", 1);
    cameraSet("agc", 1);
    cameraSet("awb", 1);
    cameraSet("lenc", 1);
    cameraSet("bpc", 1);
    cameraSet("wpc", 1);
    cameraSet("gc", CAMERA_GAIN_CEILING);
    if (!cameraSet("ael", CAMERA_AE_LEVEL)) cameraSet("ael", -2);   // sensor without -5..5 range
}

void cameraFactory()
{
    camPrefs.clear();
    applyDefaults();
}

// ------------------------------------------------------------------ init
bool cameraBegin()
{
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
    config.jpeg_quality = CAMERA_JPEG_QUALITY;

    bool hasPSRAM = psramFound();
    Serial.printf("CAMERA: PSRAM %s\n", hasPSRAM ? "found" : "NOT found");
    if (hasPSRAM)
    {
        // Buffers are sized at init, so allocate for the largest size; the stream then runs smaller.
        config.frame_size = FRAMESIZE_UXGA;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
        config.frame_size = FRAMESIZE_QVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("CAMERA: init failed 0x%x\n", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return false;
    camera_sensor_info_t *info = esp_camera_sensor_get_info(&s->id);
    strncpy(sensorName, info ? info->name : "unknown", sizeof(sensorName) - 1);
    ok = true;

    camPrefs.begin("cam", false);
    applyDefaults();            // defaults first...
    if (camPrefs.isKey("res"))  // ...then anything saved from the web page
    {
        cameraSet("res", camPrefs.getInt("res", resIndex));
        cameraSet("fps", camPrefs.getInt("fps", fpsCap));
        for (const CamKey &k : KEYS)
            if (camPrefs.isKey(k.key))
            {
                int value = camPrefs.getInt(k.key);
                if (value >= k.lo && value <= k.hi) k.set(s, value);
                else Serial.printf("CAMERA: ignored invalid saved %s=%d\n", k.key, value);
            }
    }
    if (!hasPSRAM) cameraSet("res", 0);
    Serial.printf("CAMERA: %s OK, res %d, quality %d, fps cap %d\n", sensorName, resIndex, s->status.quality, fpsCap);
    return true;
}

const char *cameraSensorName() { return sensorName; }
uint32_t cameraFrameBytes() { return lastBytes; }
int cameraViewers() { return viewers; }
uint32_t cameraCaptureMs() { return lastCaptureMs; }
uint32_t cameraSendMs() { return lastSendMs; }
uint32_t cameraCaptureFailures() { return captureFailures; }

float cameraFps()
{
    static uint32_t winStart = 0, winFrames = 0;
    static float fps = 0;
    uint32_t now = millis();
    if (now - winStart >= 1000)
    {
        uint32_t f = frameCount;
        fps = (f - winFrames) * 1000.0f / (now - winStart);
        winFrames = f;
        winStart = now;
    }
    return viewers ? fps : 0;
}

// ------------------------------------------------------------------ MJPEG stream
#define BOUNDARY "anvyxoframe"

static void streamTask(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char hdr[100];
    uint32_t last = 0;
    while (true)
    {
        uint32_t gap = 1000 / fpsCap, el = millis() - last;
        // ponytail: always yield >=5 ms; at FAST (320x240) frames are instantly ready and this prio-5 task
        // on core 0 starved IDLE0 -> task-watchdog reboot (seen live 2026-10-13). Caps stream at ~200 fps, irrelevant.
        vTaskDelay(pdMS_TO_TICKS(el < gap ? max<uint32_t>(gap - el, 5) : 5));
        last = millis();
        uint32_t captureStart = millis();
        camera_fb_t *fb = esp_camera_fb_get();
        lastCaptureMs = millis() - captureStart;
        if (!fb) { captureFailures++; vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        int n = snprintf(hdr, sizeof(hdr), "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
        uint32_t sendStart = millis();
        esp_err_t r = httpd_resp_send_chunk(req, hdr, n);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        lastSendMs = millis() - sendStart;
        lastBytes = fb->len;
        esp_camera_fb_return(fb);
        if (r != ESP_OK) break;   // viewer closed the page
        frameCount++;
    }
    httpd_req_async_handler_complete(req);
    viewers--;
    vTaskDelete(nullptr);
}

static esp_err_t streamHandler(httpd_req_t *req)
{
    if (!ok || viewers >= STREAM_MAX_CLIENTS)
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, ok ? "too many viewers" : "camera failed");
    }
    httpd_req_t *copy = nullptr;
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) return ESP_FAIL;
    viewers++;
    if (xTaskCreatePinnedToCore(streamTask, "mjpeg", 4096, copy, 5, nullptr, 0) != pdPASS)
    {
        viewers--;
        httpd_req_async_handler_complete(copy);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void cameraStreamBegin()
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = STREAM_PORT;
    cfg.ctrl_port = STREAM_PORT + 32000;
    cfg.max_open_sockets = STREAM_MAX_CLIENTS + 2;
    cfg.lru_purge_enable = true;
    httpd_handle_t h = nullptr;
    if (httpd_start(&h, &cfg) != ESP_OK) { Serial.println("STREAM: server failed"); return; }
    httpd_uri_t uri = {.uri = "/stream", .method = HTTP_GET, .handler = streamHandler, .user_ctx = nullptr};
    httpd_register_uri_handler(h, &uri);
    Serial.printf("STREAM: http://<ip>:%d/stream\n", STREAM_PORT);
}
