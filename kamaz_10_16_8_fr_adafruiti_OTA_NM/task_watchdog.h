#pragma once
#include <Arduino.h>

void watchdogTask(void *pvParameters);
void initWatchdog();
void systemMaintenanceTick();

extern uint32_t uptimeHours;
