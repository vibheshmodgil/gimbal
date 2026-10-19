#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <esp_camera.h>

bool cameraBegin();                 // init at max size, then apply saved settings
void cameraStreamBegin();           // MJPEG server on STREAM_PORT (own tasks, one per viewer)

// Settings: key,value  e.g. "res,3"  "q,10"  "fps,20"  "bri,1"  (see KEYS in camera.cpp)
bool cameraSet(const String &key, int value);
bool cameraPreset(const String &name);   // FAST | BAL | HQ | MAX
void cameraSave();                        // current settings -> flash
void cameraFactory();                     // forget saved settings, back to defaults
String cameraJson();                      // {"t":"c",...} all settings + sensor name

const char *cameraSensorName();
float cameraFps();                  // call often (main loop); measured over ~1 s
uint32_t cameraFrameBytes();
int cameraViewers();
uint32_t cameraCaptureMs();         // last frame wait/capture time
uint32_t cameraSendMs();            // last MJPEG frame send time
uint32_t cameraCaptureFailures();   // total failed frame captures since boot

#endif
