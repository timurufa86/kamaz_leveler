#include "auto_level.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "valve_ctrl.h"
#include "pressure_read.h"

#include <cmath>

/* ── ConfigManager bridge functions (defined in .ino) ── */
extern float cfg_getTiltThresholdX();
extern float cfg_getTiltThresholdY();
extern float cfg_getCoarseZoneRatio();
extern float cfg_getWorseningRatio();
extern float cfg_getFineZoneRatio();
extern int   cfg_getInflateDelay();
extern int   cfg_getReleaseDelay();
extern int   cfg_getNivCount();
extern float cfg_getPressureMin();

/* ── ErrorHandler bridge ── */
extern bool cfg_errorHasActive();

/* ── Private implementation ── */

static constexpr float STABLE_THRESHOLD = 0.1f;
static constexpr uint32_t STABLE_DURATION_MS = 1000;
static constexpr uint32_t READ_TIMEOUT_MS = 100;
static constexpr float FINE_TUNING_SCALE = 1.0f;
static constexpr uint8_t MAX_FINE_TUNING_ITERATIONS = 10;

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

static bool waitForStability(float targetX, float targetY, uint32_t timeoutMs = 5000) {
  uint32_t startTime = millis();
  uint32_t stableStartTime = 0;
  uint32_t lastReadTime = 0;

  while (millis() - startTime < timeoutMs) {
    if (millis() - lastReadTime < READ_TIMEOUT_MS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    lastReadTime = millis();

    float currentX, currentY;
    {
      MutexGuard guard(xStateMutex);
      if (!guard) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      currentX = angleX;
      currentY = angleY;
    }

    float deltaX = abs(currentX - targetX);
    float deltaY = abs(currentY - targetY);

    if (deltaX <= STABLE_THRESHOLD && deltaY <= STABLE_THRESHOLD) {
      if (stableStartTime == 0) {
        stableStartTime = millis();
      } else if (millis() - stableStartTime >= STABLE_DURATION_MS) {
        Serial.println("[AUTO] Система стабилизировалась");
        return true;
      }
    } else {
      stableStartTime = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  Serial.println("[AUTO] Таймаут ожидания стабилизации");
  return false;
}

static bool executeValveCommand(Pad pad, bool inflate, uint32_t durationMs, uint32_t stabilizeMs = 500) {
  Serial.printf("[AUTO] Выполнение: %s %s на %d мс\n",
                padNames[pad], inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ", durationMs);

  float pressureBefore;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return false;
    pressureBefore = pressure[pad];
  }

  if (!sendValveCommandSync(pad, inflate, durationMs, stabilizeMs)) {
    Serial.printf("[AUTO] Ошибка выполнения команды для %s\n", padNames[pad]);
    return false;
  }

  requestPressureMeasurement();
  vTaskDelay(pdMS_TO_TICKS(500));

  float pressureAfter;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return false;
    pressureAfter = pressure[pad];
  }

  float pressureDelta = pressureAfter - pressureBefore;

  if (abs(pressureDelta) < 0.1f) {
    Serial.printf("[AUTO] ?? Давление не изменилось! Было: %.1f, Стало: %.1f\n",
                  pressureBefore, pressureAfter);
    return false;
  }

  return true;
}

/* ====================  AutoLevelingController::process  ==================== */
void AutoLevelingController::process() {
  if (currentSystemMode != SystemMode::AUTO) return;
  if (currentState != SystemState::RUNNING) return;

  if (cfg_errorHasActive()) {
    static uint32_t lastErrorLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastErrorLog > 5000) {
        lastErrorLog = now;
        Serial.println("[AUTO] Выравнивание прервано из-за ошибок");
      }
      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
    }
    return;
  }

  if (!mpuOk) {
    static uint32_t lastMpuLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastMpuLog > 5000) {
        lastMpuLog = now;
        Serial.println("[AUTO] Выравнивание прервано: MPU не исправен");
      }
      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
    }
    return;
  }

  if (isMoving) {
    static uint32_t lastMoveLog = 0;
    uint32_t now = millis();
    if (currentStage != LevelingStage::IDLE) {
      if (now - lastMoveLog > 5000) {
        lastMoveLog = now;
        Serial.println("[AUTO] Выравнивание прервано из-за движения");
      }
      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
    }
    return;
  }

  float currentMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentMaster = masterPressure;
  }

  float minPressure = cfg_getPressureMin();
  if (currentMaster < minPressure) {
    static uint32_t lastPressureLog = 0;
    uint32_t now = millis();
    if (now - lastPressureLog > 10000) {
      lastPressureLog = now;
      Serial.printf("[AUTO] Низкое давление в магистрали (%.1f < %.1f), выравнивание невозможно\n",
                    currentMaster, minPressure);
    }
    if (currentStage != LevelingStage::IDLE) {
      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
    }
    return;
  }

  static uint32_t lastProcessTime = 0;
  uint32_t now = millis();

  if (now - lastProcessTime < 500) return;
  lastProcessTime = now;

  float currentX, currentY;
  float thX = cfg_getTiltThresholdX();
  float thY = cfg_getTiltThresholdY();

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentX = angleX;
    currentY = angleY;
  }

  bool needLeveling = (abs(currentX) > thX) || (abs(currentY) > thY);

  switch (currentStage) {
    case LevelingStage::IDLE:
      if (needLeveling) {
        if (levelingAttemptsThisHour >= (uint32_t)cfg_getNivCount()) {
          static uint32_t lastLimitLog = 0;
          if (now - lastLimitLog > 60000) {
            lastLimitLog = now;
            Serial.printf("[AUTO] Лимит попыток: %d/%d\n",
                          levelingAttemptsThisHour, cfg_getNivCount());
          }
          return;
        }

        if (now - lastLevelingAttemptTime < 30000) {
          return;
        }

        Serial.printf("[AUTO] Начало выравнивания: X=%.2f° (порог=%.1f°), Y=%.2f° (порог=%.1f°)\n",
                      currentX, thX, currentY, thY);

        levelingAttemptsThisHour++;
        lastLevelingAttemptTime = now;
        currentStage = LevelingStage::COARSE_ROLL;
        coarseStep = 0;
        stageStartTime = now;
      }
      break;

    case LevelingStage::COARSE_ROLL:
      if (abs(currentX) > thX * cfg_getCoarseZoneRatio()) {
        float prevX = currentX;

        if (coarseStep == 0) {
          if (currentX > thX * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: КРЕН ВЛЕВО - стравливание правых");
            if (executeValveCommand(PAD_FRONT_RIGHT, false,
                                    cfg_getReleaseDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_RIGHT, false,
                                       cfg_getReleaseDelay() * 1000, 500)) {
              coarseStep = 1;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          } else if (currentX < -thX * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - стравливание левых");
            if (executeValveCommand(PAD_FRONT_LEFT, false,
                                    cfg_getReleaseDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_LEFT, false,
                                       cfg_getReleaseDelay() * 1000, 500)) {
              coarseStep = 1;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          }
          stageStartTime = now;

        } else if (coarseStep == 1) {
          if (currentX > thX * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: КРЕН ВЛЕВО - накачка левых");
            if (executeValveCommand(PAD_FRONT_LEFT, true,
                                    cfg_getInflateDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_LEFT, true,
                                       cfg_getInflateDelay() * 1000, 500)) {
              coarseStep = 0;
              currentStage = LevelingStage::WAITING_STABLE;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          } else if (currentX < -thX * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - накачка правых");
            if (executeValveCommand(PAD_FRONT_RIGHT, true,
                                    cfg_getInflateDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_RIGHT, true,
                                       cfg_getInflateDelay() * 1000, 500)) {
              coarseStep = 0;
              currentStage = LevelingStage::WAITING_STABLE;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          }
          stageStartTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            currentX = angleX;
          }
        }
        if (abs(currentX) > abs(prevX) * cfg_getWorseningRatio() && abs(prevX) > 0.1f) {
          Serial.printf("[AUTO] ?? Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
                        prevX, currentX);
          if (coarseStep > 0) {
            coarseStep--;
            Serial.printf("[AUTO] Возврат к шагу %d\n", coarseStep);
          }
        }
      } else {
        Serial.println("[AUTO] Крен в норме, переход к тангажу");
        currentStage = LevelingStage::COARSE_PITCH;
        coarseStep = 0;
        stageStartTime = now;
      }
      break;

    case LevelingStage::COARSE_PITCH:
      if (abs(currentY) > thY * cfg_getCoarseZoneRatio()) {
        float prevY = currentY;

        if (coarseStep == 0) {
          if (currentY > thY * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: НОС ВВЕРХ - стравливание передних");
            if (executeValveCommand(PAD_FRONT_LEFT, false,
                                    cfg_getReleaseDelay() * 1000, 500)
                && executeValveCommand(PAD_FRONT_RIGHT, false,
                                       cfg_getReleaseDelay() * 1000, 500)) {
              coarseStep = 1;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          } else if (currentY < -thY * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: НОС ВНИЗ - стравливание задних");
            if (executeValveCommand(PAD_REAR_LEFT, false,
                                    cfg_getReleaseDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_RIGHT, false,
                                       cfg_getReleaseDelay() * 1000, 500)) {
              coarseStep = 1;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          }
          stageStartTime = now;

        } else if (coarseStep == 1) {
          if (currentY > thY * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: НОС ВВЕРХ - накачка задних");
            if (executeValveCommand(PAD_REAR_LEFT, true,
                                    cfg_getInflateDelay() * 1000, 500)
                && executeValveCommand(PAD_REAR_RIGHT, true,
                                       cfg_getInflateDelay() * 1000, 500)) {
              coarseStep = 0;
              currentStage = LevelingStage::WAITING_STABLE;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          } else if (currentY < -thY * cfg_getCoarseZoneRatio()) {
            Serial.println("[AUTO] Грубо: НОС ВНИЗ - накачка передних");
            if (executeValveCommand(PAD_FRONT_LEFT, true,
                                    cfg_getInflateDelay() * 1000, 500)
                && executeValveCommand(PAD_FRONT_RIGHT, true,
                                       cfg_getInflateDelay() * 1000, 500)) {
              coarseStep = 0;
              currentStage = LevelingStage::WAITING_STABLE;
            } else {
              Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
              currentStage = LevelingStage::COMPLETED;
            }
          }
          stageStartTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            currentY = angleY;
          }
        }
        if (abs(currentY) > abs(prevY) * cfg_getWorseningRatio() && abs(prevY) > 0.1f) {
          Serial.printf("[AUTO] ?? Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
                        prevY, currentY);
          if (coarseStep > 0) {
            coarseStep--;
            Serial.printf("[AUTO] Возврат к шагу %d\n", coarseStep);
          }
        }
      } else {
        if (abs(currentX) <= thX && abs(currentY) <= thY) {
          Serial.println("[AUTO] Выравнивание завершено успешно");
          currentStage = LevelingStage::COMPLETED;
        } else {
          Serial.println("[AUTO] Тангаж в норме, переход к точной настройке");
          currentStage = LevelingStage::FINE_TUNING;
          fineTuningIterations = 0;
          stageStartTime = now;
        }
      }
      break;

    case LevelingStage::FINE_TUNING:
      if (fineTuningIterations >= MAX_FINE_TUNING_ITERATIONS) {
        Serial.println("[AUTO] Точная настройка: лимит итераций");
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
          Serial.printf("[AUTO] Точная: стравливание угла %d (высота %.2f), разница %.2f\n",
                        highCorner, maxHeight, heightDifference);

          if (executeValveCommand(Pad(highCorner), false,
                                  cfg_getReleaseDelay() * 500, 300)) {
            vTaskDelay(pdMS_TO_TICKS(500));

            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                currentX = angleX;
                currentY = angleY;
              }
            }

            cornerHeights[PAD_FRONT_LEFT] = (-currentY - currentX) * FINE_TUNING_SCALE;
            cornerHeights[PAD_FRONT_RIGHT] = (-currentY + currentX) * FINE_TUNING_SCALE;
            cornerHeights[PAD_REAR_LEFT] = (+currentY - currentX) * FINE_TUNING_SCALE;
            cornerHeights[PAD_REAR_RIGHT] = (+currentY + currentX) * FINE_TUNING_SCALE;

            float newMinHeight = cornerHeights[0];
            uint8_t newLowCorner = 0;
            for (int i = 1; i < 4; i++) {
              if (cornerHeights[i] < newMinHeight) {
                newMinHeight = cornerHeights[i];
                newLowCorner = i;
              }
            }

            Serial.printf("[AUTO] Точная: накачка угла %d\n", newLowCorner);
            executeValveCommand(Pad(newLowCorner), true,
                                cfg_getInflateDelay() * 500, 300);
          }
          fineTuningIterations++;
          currentStage = LevelingStage::WAITING_STABLE;
          stageStartTime = now;
        } else {
          Serial.printf("[AUTO] Точная настройка завершена (разница %.2f < %.2f)\n",
                        heightDifference, fineThreshold);
          currentStage = LevelingStage::COMPLETED;
        }
      }
      break;

    case LevelingStage::WAITING_STABLE:
      if (now - stageStartTime >= 1500) {
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            currentX = angleX;
            currentY = angleY;
          }
        }

        if (abs(currentX) <= thX && abs(currentY) <= thY) {
          Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - ОТЛИЧНО!\n", currentX, currentY);
          currentStage = LevelingStage::COMPLETED;

        } else if (abs(currentX) <= thX * cfg_getWorseningRatio() && abs(currentY) <= thY * cfg_getWorseningRatio()) {
          Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - близко к цели\n", currentX, currentY);

          if (abs(currentX) > thX * cfg_getCoarseZoneRatio() || abs(currentY) > thY * cfg_getCoarseZoneRatio()) {
            currentStage = LevelingStage::COARSE_ROLL;
            coarseStep = 0;
            Serial.println("[AUTO] Возврат к грубой настройке крена");
          } else {
            currentStage = LevelingStage::FINE_TUNING;
            fineTuningIterations = 0;
            Serial.println("[AUTO] Возврат к точной настройке");
          }

        } else {
          Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - требуется продолжение\n", currentX, currentY);

          if (abs(currentX) > thX || abs(currentY) > thY) {
            currentStage = LevelingStage::COARSE_ROLL;
            coarseStep = 0;
            Serial.println("[AUTO] Возврат к грубой настройке крена");
          }
        }
        stageStartTime = now;
      }
      break;

    case LevelingStage::COMPLETED:
      {
        MutexGuard guard(xStateMutex);
        if (guard) {
          float finalX = angleX;
          float finalY = angleY;
          if (abs(finalX) > thX || abs(finalY) > thY) {
            Serial.printf("[AUTO] ?? Выравнивание не завершено! X=%.2f, Y=%.2f (пороги: %.1f, %.1f)\n",
                          finalX, finalY, thX, thY);
            currentStage = LevelingStage::COARSE_ROLL;
            coarseStep = 0;
            fineTuningIterations = 0;
            stageStartTime = now;
            break;
          }
        }
      }

      currentStage = LevelingStage::IDLE;
      coarseStep = 0;
      fineTuningIterations = 0;
      lastLevelingCheckTime = now;
      Serial.println("[AUTO] ? Выравнивание успешно завершено!");
      break;
  }
}

bool AutoLevelingController::isBusy() const { return currentStage != LevelingStage::IDLE; }
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
