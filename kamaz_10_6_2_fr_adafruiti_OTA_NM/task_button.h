#pragma once
/**
 *  task_button.h — FreeRTOS buttonTask (combos, menu, manual pads).
 */

#include <Arduino.h>

constexpr uint32_t MENU_COMBO_HOLD_MS = 800;
constexpr uint32_t MODE_TOGGLE_HOLD_MS = 800;
constexpr uint32_t EMERGENCY_HOLD_MS = 2000;

void buttonTask(void *pvParameters);

bool menuPairPressedNow();
bool modePairPressedNow();
bool emergencyPairPressedNow();
extern uint32_t g_menuNavIgnoreUntilMs;

void handleServiceInput();
void manualValveCloseAll();
