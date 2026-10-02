#include "task_control.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "task_pool.h"
#include "jhm1200.h"
#include "pressure_read.h"
#include "valve_ctrl.h"
#include "auto_level.h"

/* ── ConfigManager bridge functions (defined in .ino) ── */
extern int   cfg_getMovementCheckSec();
extern int   cfg_getManualMaxTimeSec();

/* ── ErrorHandler bridge ── */
extern bool  cfg_errorIsActive(uint8_t err);
extern bool  cfg_errorHasActive();
extern void  cfg_errorHandle(uint8_t err, const char *msg);
extern void  cfg_errorMarkCleared(uint8_t err);
extern void  cfg_errorRemove(uint8_t err);
extern void  cfg_errorUpdateTime(uint8_t err);
extern bool  cfg_errorHasCriticalPneumatic();

static constexpr uint8_t ERR_LOW_PRESSURE = 1;
static constexpr uint8_t ERR_SENSOR       = 5;
static constexpr uint8_t ERR_VALVE        = 6;

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_CONTROL_IDX = 5; // TaskMonitor::TASK_CONTROL

/* ── .ino helpers ── */
extern void runValveTestLogic();

void controlTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Control;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  IMUData imu = { 0 };
  PressureData press = { 0 };

  static uint32_t lastAdsCheck = 0;
  static bool adsErrorReported = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Control);
    cfg_taskMonitorUpdate(TASK_CONTROL_IDX);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    uint32_t currentTime = millis();

    if (cfg_errorIsActive(ERR_SENSOR)) {
      if (lastCmd.waitingForCompletion || lastCmd.commandActive) {
        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        Serial.println("[CONTROL] lastCmd сброшен из-за ошибки SENSOR");
      }
    }

#if !ENABLE_SIMULATION

    static uint32_t lastHourReset = 0;
    uint32_t now = millis();

    if (now - lastHourReset >= 3600000) {
      levelingAttemptsThisHour = 0;
      lastHourReset = now;
      Serial.println("[AUTO] Сброс счетчика попыток (прошел час)");
    }

    if (currentTime - lastAdsCheck > 100000) {
      lastAdsCheck = currentTime;

      float testBar = 0.0f;
      bool ok = false;
      {
        MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
        if (i2c) ok = Jhm1200::readBar(testBar);
      }

      static uint8_t healthFails = 0;
      if (!ok) {
        if (healthFails < 255) healthFails++;
        if (healthFails >= 3 && !cfg_errorIsActive(ERR_SENSOR) &&
            !adsErrorReported) {
          adsErrorReported = true;
          cfg_errorHandle(ERR_SENSOR, "JHM1200 не отвечает");
          Serial.println("[JHM1200] Ошибка чтения! Проверьте подключение.");
        }
      } else {
        healthFails = 0;
        if (adsErrorReported) {
          adsErrorReported = false;
          if (cfg_errorIsActive(ERR_SENSOR)) {
            cfg_errorMarkCleared(ERR_SENSOR);
            Serial.printf("[JHM1200] Датчик восстановлен (%.2f бар)\n", testBar);
          }
        }
      }
    }

#endif
    if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
      if (xQueueReceive(xIMUQueue, &imu, pdMS_TO_TICKS(10)) == pdTRUE) {
        MutexGuard guard(xStateMutex);
        if (guard) {
          angleX = imu.angleX;
          angleY = imu.angleY;
          temperature = imu.temperature;
        }
      }
      if (xQueueReceive(xPressureQueue, &press, pdMS_TO_TICKS(10)) == pdTRUE) {
        MutexGuard guard(xStateMutex);
        if (guard) {
          memcpy(pressure, press.pressure, sizeof(pressure));
          masterPressure = press.masterPressure;
        }
      }
      runValveTestLogic();
      vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
      continue;
    }

    if (xQueueReceive(xIMUQueue, &imu, pdMS_TO_TICKS(10)) == pdTRUE) {
      MutexGuard guard(xStateMutex);
      if (guard) {
        angleX = imu.angleX;
        angleY = imu.angleY;
        temperature = imu.temperature;
      }
    }
    if (xQueueReceive(xPressureQueue, &press, pdMS_TO_TICKS(10)) == pdTRUE) {
      MutexGuard guard(xStateMutex);
      if (guard) {
        memcpy(pressure, press.pressure, sizeof(pressure));
        masterPressure = press.masterPressure;
      }
    }

#if !ENABLE_SIMULATION
    if (currentSystemMode == SystemMode::MOVEMENT) {
      if (!mpuOk) {
        if (movementModeActive) {
          movementModeActive = false;
          currentSystemMode = previousMode;
          Serial.println("[CONTROL] MOVEMENT mode disabled (MPU not working)");
        }
      } else if (!cfg_errorHasCriticalPneumatic()) {
        static uint32_t lastMovementCheck = 0;
        if (currentTime - lastMovementCheck >= (uint32_t)cfg_getMovementCheckSec() * 1000UL) {
          maintainMovementPressure();
          lastMovementCheck = currentTime;
        }
      }
    } else if (currentSystemMode == SystemMode::AUTO && currentState == SystemState::RUNNING) {
      if (!cfg_errorHasActive() && mpuOk) {
        autoLevelingController.process();
      } else {
        static uint32_t lastAutoErrorLog = 0;
        if (millis() - lastAutoErrorLog > 10000) {
          lastAutoErrorLog = millis();
          if (cfg_errorHasActive()) {
            Serial.println("[AUTO] Выравнивание приостановлено из-за ошибок");
          }
          if (!mpuOk) {
            Serial.println("[AUTO] Выравнивание приостановлено: MPU не исправен");
          }
        }
      }
      checkAndAdjustMasterPressure();
    } else if (currentSystemMode == SystemMode::MANUAL && currentState == SystemState::RUNNING) {
      checkAndAdjustMasterPressure();
      static uint32_t lastManualMaintainTime = 0;
      if (currentTime - lastManualMaintainTime >= MANUAL_PRESSURE_CHECK_INTERVAL_MS) {
        maintainManualPressure();
        lastManualMaintainTime = currentTime;
      }
    }
#endif
    if (manualControlActive && (millis() - manualStartTime > (uint32_t)cfg_getManualMaxTimeSec() * 1000UL)) {
      stopManualOperation();
    }

    static uint32_t lastLeakCheck = 0;
    if (currentTime - lastLeakCheck >= 30000UL) {
      updateLeakMonitor();
      lastLeakCheck = currentTime;
    }

    bool sensorActive = cfg_errorIsActive(ERR_SENSOR);

    if (!sensorActive) {
      checkPressureLimits();
    } else {
      if (cfg_errorIsActive(ERR_LOW_PRESSURE)) {
        cfg_errorMarkCleared(ERR_LOW_PRESSURE);
        static uint32_t lastLogTime = 0;
        if (millis() - lastLogTime > 10000) {
          lastLogTime = millis();
          Serial.println("[CONTROL] LOW_PRESSURE будет удалена через 3 сек (датчик неисправен)");
        }
      }
    }
    static uint32_t stabilizeStart = 0;

    if (lastCmd.waitingForCompletion && !lastCmd.commandActive && !cfg_errorIsActive(ERR_SENSOR)) {

      if (stabilizeStart == 0) {
        stabilizeStart = millis();
      }

      if (stabilizeStart != 0 && (millis() - stabilizeStart) >= 500) {
        bool pressureChanged = false;

        MutexGuard guard(xStateMutex);
        if (guard) {
          float currentPressure = pressure[lastCmd.pad];
          float pressureDelta = abs(currentPressure - lastCmd.pressureBefore);

          if (pressureDelta > 0.3f) {
            pressureChanged = true;
          }
        }

        if (!pressureChanged) {
          valveErrorCounter.consecutiveFailures++;
          valveErrorCounter.lastFailureTime = millis();

          Serial.printf("[VALVE] Сбой #%d/%d при команде %s %s\n",
                        valveErrorCounter.consecutiveFailures,
                        valveErrorCounter.requiredFailures,
                        padNames[lastCmd.pad],
                        lastCmd.inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ");

          if (valveErrorCounter.consecutiveFailures >= valveErrorCounter.requiredFailures && !valveErrorCounter.valveErrorActive) {

            valveErrorCounter.valveErrorActive = true;
            if (!cfg_errorIsActive(ERR_VALVE)) {
              cfg_errorHandle(ERR_VALVE, "Клапанный блок не реагирует на команды");
            } else {
              cfg_errorUpdateTime(ERR_VALVE);
            }
          }
        } else {
          if (valveErrorCounter.consecutiveFailures > 0) {
            Serial.printf("[VALVE] Команда успешна, сброс счётчика (было %d сбоев)\n",
                          valveErrorCounter.consecutiveFailures);
            valveErrorCounter.consecutiveFailures = 0;
          }

          if (valveErrorCounter.valveErrorActive) {
            valveErrorCounter.valveErrorActive = false;
            if (cfg_errorIsActive(ERR_VALVE)) {
              cfg_errorRemove(ERR_VALVE);
            }
            Serial.println("[VALVE] Ошибка VALVE удалена (система восстановилась)");
          }
        }

        lastCmd.waitingForCompletion = false;
        stabilizeStart = 0;
      }
    } else {
      stabilizeStart = 0;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}
