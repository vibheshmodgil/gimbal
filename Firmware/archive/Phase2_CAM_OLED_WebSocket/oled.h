#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

bool oledBegin();
void oledScanI2C();

void oledShowSystemStatus(
    bool oledOK,
    bool cameraOK,
    bool wifiOK,
    const char *ipAddress
);

bool oledIsDetected();

#endif
