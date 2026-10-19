#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

bool oledBegin();
void oledScanI2C();
void oledShowTestScreen();
bool oledIsDetected();

#endif
