#pragma once
/**
 *  task_display.h — FreeRTOS displayTask.
 *  Menu open/close requests are posted here and applied under xDisplayMutex.
 */

#include <Arduino.h>

enum class MenuReq : uint8_t { None = 0, Open = 1, Close = 2 };
extern volatile MenuReq g_menuReq;

constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 50;

void displayTask(void *pvParameters);
