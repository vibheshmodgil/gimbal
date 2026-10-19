#ifndef WEB_H
#define WEB_H

#include <Arduino.h>

bool wifiBegin();

void webBegin();
void webHandle();

void webSendCameraFrame(
    const uint8_t *data,
    size_t length
);

#endif
