#ifndef GIMBAL_H
#define GIMBAL_H

#include <Arduino.h>

// Angles in degrees, 0 = centre. Pan +right / -left, tilt +up / -down.
enum GimbalMode { MODE_MANUAL, MODE_PATROL, MODE_SCAN, MODE_NOD, MODE_SHAKE, MODE_TEST_PAN, MODE_TEST_TILT };

void gimbalBegin();
void gimbalUpdate();                       // call every loop: smooth motion + modes + laser safety

void gimbalSetTarget(float pan, float tilt);   // absolute (switches to MANUAL)
void gimbalMoveBy(float dPan, float dTilt);    // relative (used by joystick nudges + face tracking)
void gimbalCenter();
void gimbalSetMode(GimbalMode m);
GimbalMode gimbalMode();
const char *gimbalModeName();

void gimbalSetSpeed(float dps);
float gimbalSpeed();

void gimbalSetTrim(float panTrim, float tiltTrim);  // calibration offset so 0,0 looks straight ahead
void gimbalSaveSettings();                          // trims + speed -> flash
void gimbalSavePreset(int n);

// Calibration, done once after building. Each step saves to flash and never moves the head.
void gimbalCalibrate(bool on);              // on = jog over the full servo range, limits ignored
bool gimbalCalibrating();
void gimbalZeroHere(char axis);             // 'P'/'T': current position becomes 0
void gimbalLimitHere(char axis, int side);  // current position becomes the -1 (left/down) or +1 (right/up) limit
void gimbalInvert(char axis);               // flip direction of one axis
void gimbalMountPose(char axis, float servoDeg);   // re-seat the horn: servo to 0/90/180, trim + limits cleared
void gimbalCalDone();
void gimbalCalReset();                      // back to CAD defaults, NOT calibrated
bool gimbalCalibrated();                    // tracking should wait for this
String gimbalCalJson();                     // "cal","calOn","invP","invT","lp","lt", "rp"/"rt" = servo reach (no braces)
                                            // presets: n = 1..4
bool gimbalGotoPreset(int n);

float gimbalPan();
float gimbalTilt();
float gimbalTargetPan();
float gimbalTargetTilt();
float gimbalTrimPan();
float gimbalTrimTilt();
bool gimbalMoving();

// Wiring test: switch one servo's pulses off (it goes limp) / on, and read the pulse being sent.
void servoEnable(char axis, bool on);       // axis 'P' or 'T'
bool servoEnabled(char axis);
int servoPulseUs(char axis);

void laserSet(bool on);
bool laserOn();
uint32_t laserSecondsLeft();

#endif
