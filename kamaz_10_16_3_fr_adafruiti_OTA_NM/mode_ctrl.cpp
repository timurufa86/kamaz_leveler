#include "mode_ctrl.h"
#include "app_globals.h"
#include "logger.h"
#include "mutex_guard.h"
#include "pressure_read.h"
#include "ui_screens.h"
#include <Arduino.h>

bool switchSystemMode(SystemMode target, const char *via) {
  if (target == SystemMode::MOVEMENT) return false;
  if (currentSystemMode == SystemMode::MOVEMENT) return false;
  if (currentSystemMode == target) return true;

  if (target == SystemMode::AUTO) {
    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        currentSystemMode = SystemMode::AUTO;
        currentMode = Mode::AUTO;
      }
    }
    lastLevelingCheckTime = millis();
    lastLevelingAttemptTime = millis();
    levelingAttemptsThisHour = 0;
    lastHourResetTime = millis();
    Logger::log(Logger::INFO, "MODE", "Переключено в АВТО режим");
  } else {
    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        currentSystemMode = SystemMode::MANUAL;
        currentMode = Mode::MANUAL;
      }
    }
    setAllManualTargetsFromCurrent();
    requestPressureMeasurement();
    Logger::log(Logger::INFO, "MODE", "Переключено в РУЧНОЙ режим");
  }

  forceDisplayReset(true);
  setDisplayDirty();
  SERIAL_DBG("[MODE] switch via %s > %s\n", via ? via : "?",
             target == SystemMode::AUTO ? "AUTO" : "MANUAL");
  return true;
}
