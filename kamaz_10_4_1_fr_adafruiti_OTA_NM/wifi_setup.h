#pragma once
#include <Arduino.h>

void requestWiFiSetup();
void startWiFiSetup();
bool pollWiFiScanComplete();
void displayWiFiSetupScreen();
void startFallbackAccessPoint();
void connectConfiguredWiFi();
void initializeDefaultCredentials();
