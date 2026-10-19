#ifndef WEB_H
#define WEB_H

#include <Arduino.h>

bool wifiBegin();                 // home Wi-Fi, falls back to the ANVYXO-Gimbal hotspot
bool wifiIsAP();
String wifiAddress();

void webBegin();
void webHandle();
int webClients();

void webSendStatus(const String &json);

// One command language for the web page, the serial monitor and the PC face tracker.
// Returns a short reply ("OK ...", "ERR ...").
String commandRun(const String &cmd);

#endif
