#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <esp_camera.h>

bool cameraBegin();

camera_fb_t *cameraGetFrame();
void cameraReleaseFrame(camera_fb_t *frame);

#endif
