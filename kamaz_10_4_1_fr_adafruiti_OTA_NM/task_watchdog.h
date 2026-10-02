#pragma once
#include <Arduino.h>

void watchdogTask(void *pvParameters);
void initWatchdog();
void systemMaintenanceTick();
