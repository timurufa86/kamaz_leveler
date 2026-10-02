#pragma once
/**
 *  arduino_ota.h — ArduinoOTA access-point mode (separate from GitHub OTA).
 */
#include <Arduino.h>

void initOTA();
void startOTAMode();
void stopOTAMode();
void handleOTA();
void displayOTAScreen();

extern int otaProgress;
extern char otaStatus[32];
