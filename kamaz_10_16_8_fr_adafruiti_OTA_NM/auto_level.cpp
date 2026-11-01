#include "auto_level.h"
#include "logger.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "valve_ctrl.h"

#include <cmath>

/* ── ConfigManager bridge functions (defined in .ino) ── */
extern float cfg_getTiltThresholdX();
extern float cfg_getTiltThresholdY();
extern float cfg_getCoarseZoneRatio();
extern float cfg_getWorseningRatio();
extern float cfg_getFineZoneRatio();
extern float cfg_getStableThresholdDeg();
extern int   cfg_getStableHoldMs();
extern int   cfg_getStableTimeoutSec();
extern int   cfg_getInflateDelay();
extern int   cfg_getReleaseDelay();
extern int   cfg_getNivCount();
extern int   cfg_getTimeInterval();
extern float cfg_getPressureMin();
extern float cfg_getPressureMax();
extern float cfg_getPressureDeadband();
extern float cfg_getMasterLowBar();
extern int   cfg_getPressureStabilizeMs();

/* ── ErrorHandler bridge ── */
extern bool cfg_errorHasActive();

/* ── Private implementation ── */

static constexpr uint32_t READ_TIMEOUT_MS = 100;
static constexpr float FINE_TUNING_SCALE = 1.0f;
static constexpr uint8_t MAX_FINE_TUNING_ITERATIONS = 10;
/** Soft ΔP floor: ignore tiny drift if pulsePad already returned true. */
static constexpr float PRESSURE_DELTA_EPS = 0.02f;

enum class LevelingStage {
  IDLE,
  COARSE_ROLL,
  COARSE_PITCH,
  FINE_TUNING,
  WAITING_STABLE,
  COMPLETED
};

static LevelingStage currentStage = LevelingStage::IDLE;
static uint8_t coarseStep = 0;
static uint8_t fineTuningIterations = 0;
static uint32_t stageStartTime = 0;

/** Locked coarse direction: +1 / -1; 0 = unset. */
static int8_t coarseSign = 0;

/** Non-blocking stability wait state. */
static uint32_t stabStartMs = 0;
static uint32_t stabHoldStartMs = 0;
static uint32_t stabLastReadMs = 0;

/** Attempt accounting for the active cycle. */
static bool cycleHadSuccessfulPulse = false;
static bool cycleCounted = false;
static bool fineLowLocked = false;
static uint8_t fineLowCorner = 0;
static float finePrevHeightDiff = 0.0f;

static void resetCycleFlags() {
  cycleHadSuccessfulPulse = false;
  cycleCounted = false;
  fineLowLocked = false;
  fineLowCorner = 0;
  finePrevHeightDiff = 0.0f;
  coarseSign = 0;
  /* 10.15.6: счётчик точных итераций принадлежит циклу и сбрасывается здесь,
     а не при каждом повторном входе в FINE_TUNING (иначе лимит был мёртв). */
  fineTuningIterations = 0;
}

static void noteSuccessfulPulse() { cycleHadSuccessfulPulse = true; }

/** Count niv attempt when a real cycle ends (success or fail after work). */
static void finishCycleAttempt(bool count) {
  if (cycleCounted) return;
  if (count) {
    levelingAttemptsThisHour++;
    lastLevelingAttemptTime = millis();
    cycleCounted = true;
    SERIAL_DBG("[AUTO] Учёт попытки: %lu/%d\n",
                  (unsigned long)levelingAttemptsThisHour, cfg_getNivCount());
  }
}

static void forceAutoIdle(const char *reason, bool countAttempt) {
  if (currentStage == LevelingStage::IDLE) return;
  SERIAL_DBG("[AUTO] → IDLE (%s)%s\n", reason ? reason : "",
                countAttempt ? " [count]" : "");
  if (countAttempt) finishCycleAttempt(true);
  currentStage = LevelingStage::IDLE;
  coarseStep = 0;
  fineTuningIterations = 0;
  coarseSign = 0;
  fineLowLocked = false;
  stabStartMs = 0;
  stabHoldStartMs = 0;
}

static void enterWaitingStable() {
  currentStage = LevelingStage::WAITING_STABLE;
  stabStartMs = millis();
  stabHoldStartMs = 0;
  stabLastReadMs = 0;
  stageStartTime = stabStartMs;
  SERIAL_DBGLN("[AUTO] Ожидание стабилизации (неблок.)");
}

/** After hold: pick next stage from current angles. */
static void decideAfterStability() {
  float currentX, currentY;
  float thX = cfg_getTiltThresholdX();
  float thY = cfg_getTiltThresholdY();
  const float coarseX = thX * cfg_getCoarseZoneRatio();
  const float coarseY = thY * cfg_getCoarseZoneRatio();
  {
    MutexGuard guard(xStateMutex);
    if (!guard) {
      currentStage = LevelingStage::COMPLETED;
      return;
    }
    currentX = angleX;
    currentY = angleY;
  }

  if (fabsf(currentX) <= thX && fabsf(currentY) <= thY) {
    SERIAL_DBG("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - ОТЛИЧНО!\n", currentX, currentY);
    currentStage = LevelingStage::COMPLETED;
  } else if (fabsf(currentX) > coarseX || fabsf(currentY) > coarseY) {
    SERIAL_DBG("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - грубая зона\n", currentX, currentY);
    currentStage = LevelingStage::COARSE_ROLL;
    coarseStep = 0;
    coarseSign = 0;
  } else {
    /* th … th×coarse → FINE, never treat as success */
    SERIAL_DBG("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - зона точной\n", currentX, currentY);
    currentStage = LevelingStage::FINE_TUNING;
    fineLowLocked = false;
  }
  stageStartTime = millis();
}

/** Tick WAITING_STABLE once per process(); returns true while still waiting. */
static bool tickStabilityWait() {
  const float stab = cfg_getStableThresholdDeg();
  const float thrX = fmaxf(stab, cfg_getTiltThresholdX());
  const float thrY = fmaxf(stab, cfg_getTiltThresholdY());
  const uint32_t holdMs = (uint32_t)cfg_getStableHoldMs();
  const uint32_t timeoutMs = (uint32_t)cfg_getStableTimeoutSec() * 1000UL;
  const uint32_t now = millis();

  if (cfg_errorHasActive() || !mpuOk || isMoving ||
      currentSystemMode != SystemMode::AUTO) {
    SERIAL_DBGLN("[AUTO] Стабилизация прервана");
    forceAutoIdle("стабилизация прервана", cycleHadSuccessfulPulse);
    return false;
  }

  if (now - stabStartMs >= timeoutMs) {
    SERIAL_DBGLN("[AUTO] Таймаут ожидания стабилизации");
    forceAutoIdle("таймаут стабилизации", true);
    return false;
  }

  if (now - stabLastReadMs < READ_TIMEOUT_MS) return true;
  stabLastReadMs = now;

  float currentX, currentY;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return true;
    currentX = angleX;
    currentY = angleY;
  }

  if (fabsf(currentX) <= thrX && fabsf(currentY) <= thrY) {
    if (stabHoldStartMs == 0) {
      stabHoldStartMs = now;
    } else if (now - stabHoldStartMs >= holdMs) {
      SERIAL_DBGLN("[AUTO] Система стабилизировалась");
      decideAfterStability();
      return false;
    }
  } else {
    stabHoldStartMs = 0;
  }
  return true;
}

/**
 * Soft ΔP: pulsePad already enforces min/max; only reject clear sensor stalls.
 * If pulse returned true and |ΔP| is tiny, still accept (pad may be near limit).
 */
static bool executeValveCommand(Pad pad, bool inflate, uint32_t durationMs) {
  SERIAL_DBG("[AUTO] Выполнение: %s %s на %d мс (t до P=%d)\n",
                padNames[pad], inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ", durationMs,
                cfg_getPressureStabilizeMs());

  float pressureBefore = 0.0f;
  float pressureAfter = 0.0f;
  if (!pulsePad(pad, inflate, durationMs, &pressureBefore, &pressureAfter)) {
    SERIAL_DBG("[AUTO] Ошибка/блок команды для %s\n", padNames[pad]);
    return false;
  }

  float pressureDelta = pressureAfter - pressureBefore;
  if (fabsf(pressureDelta) < PRESSURE_DELTA_EPS) {
    SERIAL_DBG("[AUTO] ΔP мал (%.3f), импульс принят (pulsePad=ok)\n", pressureDelta);
  }
  noteSuccessfulPulse();
  return true;
}

static bool executeSidePair(bool left, bool inflate, uint32_t durationMs) {
  SERIAL_DBG("[AUTO] Ось/сторона %s %s %lu мс\n",
                left ? "LEFT" : "RIGHT", inflate ? "INFL" : "DEFL",
                (unsigned long)durationMs);
  if (!pulseSidePair(left, inflate, durationMs)) {
    SERIAL_DBG("[AUTO] Ошибка side pair %s\n", left ? "LEFT" : "RIGHT");
    return false;
  }
  noteSuccessfulPulse();
  return true;
}

static bool executeAxlePair(bool front, bool inflate, uint32_t durationMs) {
  SERIAL_DBG("[AUTO] Ось %s %s %lu мс\n",
                front ? "FRONT" : "REAR", inflate ? "INFL" : "DEFL",
                (unsigned long)durationMs);
  if (!pulseAxlePair(front, inflate, durationMs)) {
    SERIAL_DBG("[AUTO] Ошибка axle pair %s\n", front ? "FRONT" : "REAR");
    return false;
  }
  noteSuccessfulPulse();
  return true;
}

static void onValveFailEarlyOrLate() {
  /* Fail before first successful pulse does not burn nivCount. */
  forceAutoIdle("ошибка клапана", cycleHadSuccessfulPulse);
}

/* ====================  AutoLevelingController::process  ==================== */
void AutoLevelingController::process() {
  if (currentSystemMode != SystemMode::AUTO) {
    forceAutoIdle("режим не AUTO", false);
    return;
  }
  if (currentState != SystemState::RUNNING) {
    forceAutoIdle("не RUNNING", false);
    return;
  }

  if (cfg_errorHasActive()) {
    static uint32_t lastErrorLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastErrorLog > 5000) {
        lastErrorLog = now;
        SERIAL_DBGLN("[AUTO] Выравнивание прервано из-за ошибок");
      }
      forceAutoIdle("ошибка", cycleHadSuccessfulPulse);
    }
    return;
  }

  if (!mpuOk) {
    static uint32_t lastMpuLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastMpuLog > 5000) {
        lastMpuLog = now;
        SERIAL_DBGLN("[AUTO] Выравнивание прервано: MPU не исправен");
      }
      forceAutoIdle("MPU", cycleHadSuccessfulPulse);
    }
    return;
  }

  if (isMoving) {
    static uint32_t lastMoveLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastMoveLog > 5000) {
        lastMoveLog = now;
        SERIAL_DBGLN("[AUTO] Выравнивание прервано из-за движения");
      }
      forceAutoIdle("движение", cycleHadSuccessfulPulse);
    }
    return;
  }

  float currentMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentMaster = masterPressure;
  }

  const float masterLow = cfg_getMasterLowBar();
  if (masterLow > 0.0f && currentMaster < masterLow) {
    static uint32_t lastPressureLog = 0;
    uint32_t now = millis();
    if (now - lastPressureLog > 10000) {
      lastPressureLog = now;
      SERIAL_DBG("[AUTO] Низкое давление Маг (%.1f < %.1f), выравнивание невозможно\n",
                    currentMaster, masterLow);
    }
    forceAutoIdle("низкое Маг", cycleHadSuccessfulPulse);
    return;
  }

  /* Stability wait ticks every process() call (no 500 ms gate). */
  if (currentStage == LevelingStage::WAITING_STABLE) {
    tickStabilityWait();
    return;
  }

  static uint32_t lastProcessTime = 0;
  uint32_t now = millis();

  if (now - lastProcessTime < 500) return;
  lastProcessTime = now;

  float currentX, currentY;
  float thX = cfg_getTiltThresholdX();
  float thY = cfg_getTiltThresholdY();
  const float coarseX = thX * cfg_getCoarseZoneRatio();
  const float coarseY = thY * cfg_getCoarseZoneRatio();

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentX = angleX;
    currentY = angleY;
  }

  bool needLeveling = (fabsf(currentX) > thX) || (fabsf(currentY) > thY);

  switch (currentStage) {
    case LevelingStage::IDLE:
      if (needLeveling) {
        if (levelingAttemptsThisHour >= (uint32_t)cfg_getNivCount()) {
          static uint32_t lastLimitLog = 0;
          if (now - lastLimitLog > 60000) {
            lastLimitLog = now;
            SERIAL_DBG("[AUTO] Лимит попыток: %d/%d\n",
                          levelingAttemptsThisHour, cfg_getNivCount());
          }
          return;
        }

        const uint32_t intervalMs =
            (uint32_t)constrain(cfg_getTimeInterval(), 1, 60) * 60000UL;
        if (now - lastLevelingAttemptTime < intervalMs) {
          return;
        }

        SERIAL_DBG("[AUTO] Начало выравнивания: X=%.2f° (порог=%.1f°), Y=%.2f° (порог=%.1f°)\n",
                      currentX, thX, currentY, thY);

        /* Do NOT increment levelingAttemptsThisHour here — only on cycle end. */
        resetCycleFlags();
        lastLevelingCheckTime = now;
        currentStage = LevelingStage::COARSE_ROLL;
        coarseStep = 0;
        coarseSign = 0;
        stageStartTime = now;
      }
      break;

    case LevelingStage::COARSE_ROLL:
      if (fabsf(currentX) > coarseX) {
        if (coarseStep == 0) {
          /* Lock sign on first step; keep it for inflate even if angle flips briefly. */
          coarseSign = (currentX >= 0.0f) ? 1 : -1;
          if (coarseSign > 0) {
            SERIAL_DBGLN("[AUTO] Грубо: КРЕН ВЛЕВО - стравливание правых (side)");
            if (executeSidePair(false, false, cfg_getReleaseDelay() * 1000)) {
              coarseStep = 1;
            } else {
              onValveFailEarlyOrLate();
            }
          } else {
            SERIAL_DBGLN("[AUTO] Грубо: КРЕН ВПРАВО - стравливание левых (side)");
            if (executeSidePair(true, false, cfg_getReleaseDelay() * 1000)) {
              coarseStep = 1;
            } else {
              onValveFailEarlyOrLate();
            }
          }
          stageStartTime = now;

        } else if (coarseStep == 1) {
          if (coarseSign > 0) {
            SERIAL_DBGLN("[AUTO] Грубо: КРЕН ВЛЕВО - накачка левых (side, знак зафиксирован)");
            if (executeSidePair(true, true, cfg_getInflateDelay() * 1000)) {
              coarseStep = 0;
              coarseSign = 0;
              enterWaitingStable();
            } else {
              onValveFailEarlyOrLate();
            }
          } else {
            SERIAL_DBGLN("[AUTO] Грубо: КРЕН ВПРАВО - накачка правых (side, знак зафиксирован)");
            if (executeSidePair(false, true, cfg_getInflateDelay() * 1000)) {
              coarseStep = 0;
              coarseSign = 0;
              enterWaitingStable();
            } else {
              onValveFailEarlyOrLate();
            }
          }
          stageStartTime = now;
        }
      } else if (fabsf(currentX) > thX) {
        /* Outside th but inside coarse zone → fine (via pitch handoff or direct). */
        SERIAL_DBGLN("[AUTO] Крен в зоне th…coarse → тангаж/точная");
        currentStage = LevelingStage::COARSE_PITCH;
        coarseStep = 0;
        coarseSign = 0;
        stageStartTime = now;
      } else {
        SERIAL_DBGLN("[AUTO] Крен в норме, переход к тангажу");
        currentStage = LevelingStage::COARSE_PITCH;
        coarseStep = 0;
        coarseSign = 0;
        stageStartTime = now;
      }
      break;

    case LevelingStage::COARSE_PITCH:
      if (fabsf(currentY) > coarseY) {
        if (coarseStep == 0) {
          coarseSign = (currentY >= 0.0f) ? 1 : -1;
          if (coarseSign > 0) {
            SERIAL_DBGLN("[AUTO] Грубо: НОС ВВЕРХ - стравливание передних (axle)");
            if (executeAxlePair(true, false, cfg_getReleaseDelay() * 1000)) {
              coarseStep = 1;
            } else {
              onValveFailEarlyOrLate();
            }
          } else {
            SERIAL_DBGLN("[AUTO] Грубо: НОС ВНИЗ - стравливание задних (axle)");
            if (executeAxlePair(false, false, cfg_getReleaseDelay() * 1000)) {
              coarseStep = 1;
            } else {
              onValveFailEarlyOrLate();
            }
          }
          stageStartTime = now;

        } else if (coarseStep == 1) {
          if (coarseSign > 0) {
            SERIAL_DBGLN("[AUTO] Грубо: НОС ВВЕРХ - накачка задних (axle, знак зафиксирован)");
            if (executeAxlePair(false, true, cfg_getInflateDelay() * 1000)) {
              coarseStep = 0;
              coarseSign = 0;
              enterWaitingStable();
            } else {
              onValveFailEarlyOrLate();
            }
          } else {
            SERIAL_DBGLN("[AUTO] Грубо: НОС ВНИЗ - накачка передних (axle, знак зафиксирован)");
            if (executeAxlePair(true, true, cfg_getInflateDelay() * 1000)) {
              coarseStep = 0;
              coarseSign = 0;
              enterWaitingStable();
            } else {
              onValveFailEarlyOrLate();
            }
          }
          stageStartTime = now;
        }
      } else if (fabsf(currentX) <= thX && fabsf(currentY) <= thY) {
        SERIAL_DBGLN("[AUTO] Выравнивание завершено успешно");
        currentStage = LevelingStage::COMPLETED;
      } else {
        /* Pitch in/near coarse band but still outside th (or roll residual) → FINE */
        SERIAL_DBGLN("[AUTO] Тангаж/крен в зоне th…coarse → точная настройка");
        currentStage = LevelingStage::FINE_TUNING;
        fineLowLocked = false;
        stageStartTime = now;
      }
      break;

    case LevelingStage::FINE_TUNING:
      if (fineTuningIterations >= MAX_FINE_TUNING_ITERATIONS) {
        SERIAL_DBGLN("[AUTO] Точная настройка: лимит итераций");
        currentStage = LevelingStage::COMPLETED;
        break;
      }

      {
        float cornerHeights[4];
        cornerHeights[PAD_FRONT_LEFT] = (-currentY - currentX) * FINE_TUNING_SCALE;
        cornerHeights[PAD_FRONT_RIGHT] = (-currentY + currentX) * FINE_TUNING_SCALE;
        cornerHeights[PAD_REAR_LEFT] = (+currentY - currentX) * FINE_TUNING_SCALE;
        cornerHeights[PAD_REAR_RIGHT] = (+currentY + currentX) * FINE_TUNING_SCALE;

        uint8_t highCorner = 0;
        uint8_t lowCorner = 0;
        float maxHeight = cornerHeights[0];
        float minHeight = cornerHeights[0];

        for (int i = 1; i < 4; i++) {
          if (cornerHeights[i] > maxHeight) {
            maxHeight = cornerHeights[i];
            highCorner = i;
          }
          if (cornerHeights[i] < minHeight) {
            minHeight = cornerHeights[i];
            lowCorner = i;
          }
        }

        float heightDifference = maxHeight - minHeight;
        float fineThreshold = max(thX, thY) * cfg_getFineZoneRatio();

        if (heightDifference >= fineThreshold) {
          /* Keep one target low corner until the DEFL+INFL pair finishes. */
          if (!fineLowLocked) {
            fineLowCorner = lowCorner;
            fineLowLocked = true;
            finePrevHeightDiff = heightDifference;
          }

          SERIAL_DBG("[AUTO] Точная: стравливание угла %d (высота %.2f), разница %.2f\n",
                        highCorner, maxHeight, heightDifference);

          if (!executeValveCommand(Pad(highCorner), false,
                                   cfg_getReleaseDelay() * 500)) {
            onValveFailEarlyOrLate();
            break;
          }

          SERIAL_DBG("[AUTO] Точная: накачка угла %d (зафиксирован)\n", fineLowCorner);
          if (!executeValveCommand(Pad(fineLowCorner), true,
                                   cfg_getInflateDelay() * 500)) {
            fineLowLocked = false;
            onValveFailEarlyOrLate();
            break;
          }

          float afterX = currentX, afterY = currentY;
          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              afterX = angleX;
              afterY = angleY;
            }
          }
          float afterH[4];
          afterH[PAD_FRONT_LEFT] = (-afterY - afterX) * FINE_TUNING_SCALE;
          afterH[PAD_FRONT_RIGHT] = (-afterY + afterX) * FINE_TUNING_SCALE;
          afterH[PAD_REAR_LEFT] = (+afterY - afterX) * FINE_TUNING_SCALE;
          afterH[PAD_REAR_RIGHT] = (+afterY + afterX) * FINE_TUNING_SCALE;
          float aMax = afterH[0], aMin = afterH[0];
          for (int i = 1; i < 4; i++) {
            if (afterH[i] > aMax) aMax = afterH[i];
            if (afterH[i] < aMin) aMin = afterH[i];
          }
          float afterDiff = aMax - aMin;

          /* 10.15.6: "порог хуже" из меню теперь реально влияет на решение, а
             итерация засчитывается ВСЕГДА. Раньше счётчик увеличивался только
             при улучшении и обнулялся при повторном входе в FINE_TUNING, поэтому
             лимит не достигался и клапаны могли пульсовать бесконечно. */
          const float worsening = fmaxf(1.0f, cfg_getWorseningRatio());
          const bool improved = afterDiff < finePrevHeightDiff * 0.98f;
          const bool worsened = afterDiff > finePrevHeightDiff * worsening;
          fineTuningIterations++;
          if (improved) {
            SERIAL_DBG("[AUTO] Точная: улучшение %.2f → %.2f (итер %d/%d)\n",
                          finePrevHeightDiff, afterDiff, fineTuningIterations,
                          MAX_FINE_TUNING_ITERATIONS);
          } else if (worsened) {
            SERIAL_DBG("[AUTO] Точная: стало хуже %.2f → %.2f (порог ×%.2f) — стоп\n",
                          finePrevHeightDiff, afterDiff, worsening);
          } else {
            SERIAL_DBG("[AUTO] Точная: без улучшения %.2f → %.2f (итер %d/%d)\n",
                          finePrevHeightDiff, afterDiff, fineTuningIterations,
                          MAX_FINE_TUNING_ITERATIONS);
          }
          fineLowLocked = false;
          finePrevHeightDiff = afterDiff;
          if (worsened) {
            currentStage = LevelingStage::COMPLETED;
          } else {
            enterWaitingStable();
          }
        } else {
          SERIAL_DBG("[AUTO] Точная настройка завершена (разница %.2f < %.2f)\n",
                        heightDifference, fineThreshold);
          currentStage = LevelingStage::COMPLETED;
        }
      }
      break;

    case LevelingStage::WAITING_STABLE:
      /* Handled above (early return). */
      break;

    case LevelingStage::COMPLETED:
      {
        MutexGuard guard(xStateMutex);
        if (guard) {
          float finalX = angleX;
          float finalY = angleY;
          if (fabsf(finalX) > thX || fabsf(finalY) > thY) {
            SERIAL_DBG("[AUTO] Не в норме после цикла: X=%.2f, Y=%.2f — IDLE (ждём интервал)\n",
                          finalX, finalY);
          } else {
            SERIAL_DBGLN("[AUTO] Выравнивание успешно завершено");
          }
        }
      }
      finishCycleAttempt(true);
      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
      coarseSign = 0;
      fineLowLocked = false;
      lastLevelingCheckTime = now;
      break;
  }
}

bool AutoLevelingController::isBusy() const {
  return currentStage != LevelingStage::IDLE;
}
uint8_t AutoLevelingController::fineIterations() const { return fineTuningIterations; }
const char *AutoLevelingController::stageName() const {
  switch (currentStage) {
    case LevelingStage::COARSE_ROLL: return "КРЕН";
    case LevelingStage::COARSE_PITCH: return "ТАНГ";
    case LevelingStage::FINE_TUNING: return "ТОЧНО";
    case LevelingStage::WAITING_STABLE: return "СТАБ";
    case LevelingStage::COMPLETED: return "ГОТОВО";
    default: return "ОЖИД";
  }
}

AutoLevelingController autoLevelingController;

bool autoLevelIsBusy() { return autoLevelingController.isBusy(); }
