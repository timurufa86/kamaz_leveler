#pragma once
#include <Arduino.h>

void requestWiFiSetup();
void startWiFiSetup();
bool pollWiFiScanComplete();
void displayWiFiSetupScreen();
void startFallbackAccessPoint();
void connectConfiguredWiFi();
/** 10.16.3: Serial WIFI STATUS / WIFI IP <ip> <gw> <mask> [dns] / WIFI IP DHCP. */
bool wifiHandleSerialCommand(const char *line);
void initializeDefaultCredentials();
