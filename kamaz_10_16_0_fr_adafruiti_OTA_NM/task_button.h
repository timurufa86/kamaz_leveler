#pragma once
/**
 *  task_button.h — FreeRTOS buttonTask (combos, menu, manual pads).
 */

#include <Arduino.h>

constexpr uint32_t MENU_COMBO_HOLD_MS = 800;
constexpr uint32_t MODE_TOGGLE_HOLD_MS = 800;
constexpr uint32_t EMERGENCY_HOLD_MS = 2000;
/** Pad hold before inflate/deflate start (EncButton default / README). */
constexpr uint32_t PAD_HOLD_MS = 600;
constexpr uint16_t EB_DEB_MS = 50;
constexpr uint16_t EB_CLICK_MS = 500;
constexpr uint16_t EB_STEP_MS = 200;
/** Boot: all KN1–4 must stay released this long before hold actions arm. */
constexpr uint32_t BUTTONS_ARM_IDLE_MS = 200;

void buttonTask(void *pvParameters);

bool menuPairPressedNow();
bool modePairPressedNow();
bool emergencyPairPressedNow();
extern uint32_t g_menuNavIgnoreUntilMs;

void handleServiceInput();
void manualValveCloseAll();
