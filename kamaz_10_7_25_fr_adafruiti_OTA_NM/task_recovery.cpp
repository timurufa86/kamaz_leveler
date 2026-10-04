#include "task_recovery.h"
#include "error_handler.h"
#include "config_manager.h"
#include "task_monitor.h"
#include "task_pool.h"
#include "pressure_read.h"
#include "imu_dmp.h"
#include "jhm1200.h"
#include "app_globals.h"
#include "mutex_guard.h"
#include <WiFi.h>
#include <Wire.h>

static constexpr char WIFI_STA_PASSWORD[] = "asd12345";
extern char sta_ssid[33];
extern bool errorScreenBlocking;

void forceDisplayReset(bool force = false);

void errorRecoveryTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_ErrRec;
  const TickType_t period = pdMS_TO_TICKS(2000);

  static SystemMode modeBeforeError = SystemMode::MANUAL;

  for (;;) {
    TaskPool::markRun(taskIndex_ErrRec);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_ERROR_RECOVERY);

    if (!ErrorHandler::hasActiveErrors()) {
      vTaskDelay(period);
      continue;
    }

    static bool modeSaved = false;
    static uint32_t watchdogStableSince = 0;
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::WATCHDOG)) {
      watchdogStableSince = 0;
    }
    if (!modeSaved) {
      MutexGuard guard(xStateMutex, pdMS_TO_TICKS(100));
      if (guard) {
        modeBeforeError = currentSystemMode;
        modeSaved = true;
        Serial.printf("[RECOVERY] Запомнен режим: %d\n", (int)modeBeforeError);
      }
    }

    bool anyCleared = false;
    int count = ErrorHandler::getActiveErrorCount();
    static int goodReads = 0;

    for (int i = 0; i < count; i++) {
      ErrorHandler::Error err = ErrorHandler::getActiveErrorAt(i);
      bool cleared = false;

      switch (err) {
        case ErrorHandler::Error::LOW_PRESSURE:
          checkPressureLimits();
          cleared = !ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE);
          if (cleared) {
            Serial.println("[RECOVERY] LOW_PRESSURE устранена (давление в норме)");
          }
          break;

        case ErrorHandler::Error::MPU:
          {
            MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(200));
            if (i2c) {
              auto probe = [](uint8_t addr) -> bool {
                Wire.beginTransmission(addr);
                return Wire.endTransmission() == 0;
              };
              cleared = probe(0x68) || probe(0x69);
            }
          }
          if (cleared) {
            Serial.println("[RECOVERY] MPU6050 на шине I2C — инициализация DMP");
            initializeDMP();
            cleared = mpuOk;
          }
          break;

        case ErrorHandler::Error::SENSOR:
          {
#if !ENABLE_SIMULATION
            float bar = 0.0f;
            bool ok = false;
            {
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
              if (i2c) {
                if (!Jhm1200::isReady()) {
                  // датчик мог появиться после boot-fail — пробуем begin
                  ok = Jhm1200::begin(Wire);
                  if (ok) jhmReady = true;
                }
                if (Jhm1200::isReady()) ok = Jhm1200::readBar(bar);
              }
            }

            if (ok) {
              goodReads++;
              if (!calibrationCompleted) {
                // Не держим SENSOR вечно: датчик жив > просим CalibTask повторить BOOT.
                if (goodReads >= 2 && getSystemState() == SystemState::RUNNING) {
                  Serial.println("[RECOVERY] JHM1200 отвечает — повтор калибровки");
                  setSystemState(SystemState::BOOT);
                  goodReads = 0;
                }
                cleared = false;
              } else if (goodReads >= 2) {
                cleared = true;
                goodReads = 0;
                Serial.printf("[RECOVERY] JHM1200 восстановлен! %.2f бар\n", bar);
              }
            } else {
              goodReads = 0;
              cleared = false;
              if (!calibrationCompleted) {
                static uint32_t lastKeepLog = 0;
                if (millis() - lastKeepLog > 5000) {
                  lastKeepLog = millis();
                  Serial.println("[RECOVERY] SENSOR: нет ответа JHM1200");
                }
              }
            }
#else
            cleared = true;
            Serial.println("[RECOVERY] SIM: Датчик давления исправен");
#endif
            break;
          }

        case ErrorHandler::Error::VALVE:
          // Не снимаем по таймеру — только после успешного теста / ручного сброса.
          cleared = false;
          break;

        case ErrorHandler::Error::WATCHDOG:
          {
            // Таймер не сбрасываем при каждом тике — иначе pendingClear сразу отменяется.
            if (watchdogStableSince == 0) {
              watchdogStableSince = millis();
            }
            if (millis() - watchdogStableSince > 30000) {
              cleared = true;
              Serial.println("[RECOVERY] WATCHDOG ошибка устранена (система стабильна)");
            }
            break;
          }

        case ErrorHandler::Error::OTA:
          {
            static uint32_t otaErrorTime = 0;
            if (otaErrorTime == 0) {
              otaErrorTime = millis();
            }
            if (millis() - otaErrorTime > 30000) {
              // Не поднимаем AP автоматически — только сброс ошибки OTA.
              if (WiFi.status() != WL_CONNECTED && sta_ssid[0] != '\0') {
                WiFi.mode(WIFI_STA);
                WiFi.begin(sta_ssid, WIFI_STA_PASSWORD);
                Serial.println("[RECOVERY] OTA: повтор STA, без softAP");
              } else {
                Serial.println("[RECOVERY] OTA ошибка снята (без авто-AP)");
              }
              cleared = true;
              otaErrorTime = 0;
            }
            break;
          }

        default:
          cleared = false;
          break;
      }

      // ? ЕСЛИ ОШИБКА УСТРАНЕНА - ПОМЕЧАЕМ НА УДАЛЕНИЕ
      if (cleared) {
        ErrorHandler::markErrorCleared(err);
        anyCleared = true;
      } else {
        // ? ЕСЛИ НЕ УДАЛЯЕМ - ОТМЕНЯЕМ PENDING CLEAR (если была установлена)
        if (ErrorHandler::isPendingClear(err)) {
          ErrorHandler::cancelClear(err);
          Serial.printf("[RECOVERY] Отменено удаление ошибки %d (условия не выполнены)\n", (int)err);
        }
      }
    }

    // ? ЕСЛИ ВСЕ ОШИБКИ УСТРАНЕНЫ - ВОССТАНАВЛИВАЕМ СИСТЕМУ
    if (anyCleared && !ErrorHandler::hasActiveErrors()) {
      Serial.println("[RECOVERY] Все ошибки устранены, восстанавливаем систему");
      vTaskDelay(pdMS_TO_TICKS(500));

      // ? ВОССТАНАВЛИВАЕМ РЕЖИМ
      currentSystemMode = modeBeforeError;
      if (modeBeforeError == SystemMode::AUTO) {
        currentMode = Mode::AUTO;
        lastLevelingCheckTime = millis();
        lastLevelingAttemptTime = millis();
      } else {
        currentMode = Mode::MANUAL;
        setAllManualTargetsFromCurrent();
      }

      errorScreenBlocking = false;
      forceDisplayReset(true);
      modeSaved = false;
    }

    vTaskDelay(period);
  }
}

