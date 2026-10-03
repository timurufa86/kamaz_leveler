#include "task_calib.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "task_pool.h"
#include "jhm1200.h"
#include "pressure_read.h"
#include "valve_ctrl.h"
#include "logger.h"

#include <cmath>

/* ── ConfigManager / ErrorHandler bridge functions (defined in .ino) ── */
extern bool cfg_errorIsActive(uint8_t err);
extern void cfg_errorHandle(uint8_t err, const char *msg);
extern void cfg_errorMarkCleared(uint8_t err);
extern void cfg_errorRemove(uint8_t err);

static constexpr uint8_t ERR_SENSOR = 5;

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_CALIB_IDX = 6; // TaskMonitor::TASK_CALIB

/* ── .ino helpers ── */
void setDisplayDirty();
extern void forceDisplayReset(bool force);
extern void initializeDMP();
constexpr uint32_t CALIB_TIME_MS = 15000;

/* ====================  ZeroCalibrator  ==================== */
class ZeroCalibrator {
private:
  bool calibrationDone_ = false;
  uint32_t stepStartTime_ = 0;
  enum CalibStep : uint8_t {
    STEP_IDLE,
    STEP_OPEN_VALVE,
    STEP_WAIT_STABILIZE,
    STEP_MEASURE,
    STEP_CLOSE_VALVE,
    STEP_DONE
  } currentStep_ = STEP_IDLE;
  float measurements_[20] = { 0 };
  uint8_t measureCount_ = 0;
  float finalZeroBar_ = 0.0f;

  static constexpr float ZERO_ABS_MAX = 1.5f;
  static constexpr float ZERO_SPREAD_MAX = 0.15f;

public:
  void start() {
    if (calibrationDone_) {
      Serial.println("[CALIB] start() ignored - already done");
      return;
    }
    if (currentStep_ != STEP_IDLE) {
      Serial.println("[CALIB] start() ignored - already started");
      return;
    }

#if ENABLE_SIMULATION
    Serial.println("[CALIB] SIM: Запуск калибровки пропущен");
    calibrationDone_ = true;
    return;
#else
    calibrationDone_ = false;
    currentStep_ = STEP_OPEN_VALVE;
    stepStartTime_ = millis();
    measureCount_ = 0;
    finalZeroBar_ = 0.0f;

    float testBar = 0.0f;
    bool ok = false;
    {
      MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(100));
      if (i2c) ok = Jhm1200::readBar(testBar);
    }
    Serial.printf("[CALIB] start(): probe=%s bar=%.3f status=0x%02X\n",
                  ok ? "ok" : "fail", testBar, Jhm1200::lastStatus());

    Serial.println("\n========================================");
    Serial.println("     АВТОМАТИЧЕСКАЯ КАЛИБРОВКА НУЛЯ");
    Serial.println("     (JHM1200 0..10 бар)");
    Serial.println("========================================");
#endif
  }

  void process() {
    if (calibrationDone_) return;

#if ENABLE_SIMULATION
    calibrationDone_ = true;
    Serial.println("[CALIB] SIM: Калибровка пропущена");
    return;
#endif

    uint32_t now = millis();

    switch (currentStep_) {
      case STEP_OPEN_VALVE:
        {
          MutexGuard guard(xValveMutex);
          if (guard) {
            closeAllValves();
            digitalWrite(PIN_DEFL, HIGH);
            Serial.println("[CALIB] Клапан сброса ОТКРЫТ");
            currentStep_ = STEP_WAIT_STABILIZE;
            stepStartTime_ = now;
          }
          break;
        }

      case STEP_WAIT_STABILIZE:
        {
          if (now - stepStartTime_ >= 3000) {
            Serial.println("[CALIB] Ожидание стабилизации завершено");
            currentStep_ = STEP_MEASURE;
            stepStartTime_ = now;
            measureCount_ = 0;
          }
          break;
        }

      case STEP_MEASURE:
        if (measureCount_ < 20) {
          static uint32_t lastMeasureTime = 0;
          if (now - lastMeasureTime >= 200) {
            float bar = 0.0f;
            bool ok = false;
            {
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
              if (i2c) ok = Jhm1200::readBar(bar);
            }
            lastMeasureTime = now;
            if (!ok) {
              Serial.println("[CALIB] JHM1200 read fail (пропуск)");
              break;
            }
            measurements_[measureCount_++] = bar;
            Serial.printf("[CALIB] Измерение %u: %.3f бар\n", measureCount_, bar);
          }
        } else {
          float sum = 0.0f;
          float minVal = measurements_[5];
          float maxVal = measurements_[5];
          for (int i = 5; i < 20; i++) {
            sum += measurements_[i];
            if (measurements_[i] < minVal) minVal = measurements_[i];
            if (measurements_[i] > maxVal) maxVal = measurements_[i];
          }
          finalZeroBar_ = sum / 15.0f;
          const float range = maxVal - minVal;
          Serial.printf("[CALIB] Среднее: %.3f бар, разброс: %.3f\n", finalZeroBar_, range);

          if (fabsf(finalZeroBar_) > ZERO_ABS_MAX) {
            Serial.printf("[CALIB] Необычный ноль |%.3f| > %.1f бар\n", finalZeroBar_, ZERO_ABS_MAX);
            if (!cfg_errorIsActive(ERR_SENSOR)) {
              cfg_errorHandle(ERR_SENSOR, "JHM1200 - аномальный ноль");
            }
            calibrationValid = false;
          } else {
            if (cfg_errorIsActive(ERR_SENSOR)) {
              cfg_errorRemove(ERR_SENSOR);
            }
            calibrationValid = true;
          }
          if (range > ZERO_SPREAD_MAX) {
            Serial.printf("[CALIB] Нестабильные измерения, разброс %.3f бар\n", range);
          }

          currentStep_ = STEP_CLOSE_VALVE;
          stepStartTime_ = now;
        }
        break;

      case STEP_CLOSE_VALVE:
        {
          MutexGuard guard(xValveMutex);
          if (guard) {
            digitalWrite(PIN_DEFL, LOW);
            Serial.println("[CALIB] Клапан сброса ЗАКРЫТ");
            currentStep_ = STEP_DONE;
          }
          break;
        }

      case STEP_DONE:
        {
          MutexGuard guard(xCalibMutex);
          if (guard) {
            g_pressureZeroBar = finalZeroBar_;
            calibrationDone_ = true;
            Serial.printf("[CALIB] Ноль сохранён: %.3f бар\n", g_pressureZeroBar);
            Serial.println("========================================");
            Serial.println("     КАЛИБРОВКА НУЛЯ ЗАВЕРШЕНА");
            Serial.println("========================================\n");

            Event event;
            event.type = EventType::CALIBRATION_DONE;
            event.timestamp = millis();
            EventBus::publish(event);
          }
          break;
        }

      default:
        break;
    }
  }

  bool isDone() const { return calibrationDone_; }
  float getZeroValue() const { return finalZeroBar_; }
};

/* ====================  calibrationTask  ==================== */
void calibrationTask(void *pvParameters) {
    extern uint8_t taskIndex_Calib;

    for (;;) {
        TaskPool::markRun(taskIndex_Calib);
        cfg_taskMonitorUpdate(TASK_CALIB_IDX);

        if (getSystemState() == SystemState::BOOT) {

#if ENABLE_SIMULATION
            Serial.println("[CALIB] SIM: Пропускаем калибровку (режим симуляции)");
            setSystemState(SystemState::RUNNING);
            calibrationCompleted = true;
            displayDirty = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
#else

            Serial.println("[CALIB] ШАГ 1/3: Инициализация JHM1200...");

            bool jhmOk = initializeJhm1200();
            if (!jhmOk) {
                Serial.println("[CALIB] JHM1200 НЕ НАЙДЕН! Повтор через 2 с…");
                cfg_errorHandle(ERR_SENSOR, "JHM1200 не найден");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);
                continue;
            }

            Serial.println("[CALIB] JHM1200 инициализирован (0x78)");

            Serial.println("[CALIB] ШАГ 2/3: Проверка датчика давления...");

            float probeBar = 0.0f;
            bool probeOk = false;
            for (int attempt = 0; attempt < 8 && !probeOk; attempt++) {
              if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(30));
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(200));
              if (i2c) probeOk = Jhm1200::readBar(probeBar);
            }

            if (!probeOk) {
                Serial.println("[CALIB] ОШИБКА ЧТЕНИЯ JHM1200! Повтор через 2 с…");
                cfg_errorHandle(ERR_SENSOR, "JHM1200 - ошибка чтения");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);
                continue;
            }

            Serial.printf("[CALIB] Датчик давления исправен (%.3f бар, status=0x%02X)\n",
                          probeBar, Jhm1200::lastStatus());

            if (cfg_errorIsActive(ERR_SENSOR)) {
                cfg_errorMarkCleared(ERR_SENSOR);
                Serial.println("[CALIB] Ошибка SENSOR помечена для удаления");
            }

#if ENABLE_MPU6050
            Serial.println("[CALIB] ШАГ 3/4: Проверка MPU6050...");
            if (!mpuOk) {
              initializeDMP();
            }
            if (!mpuOk) {
                Serial.println("[CALIB] ? MPU6050 НЕ ОТВЕЧАЕТ!");
            } else {
                Serial.println("[CALIB] ? MPU6050 инициализирован успешно");
            }
#else
            Serial.println("[CALIB] ШАГ 3/4: MPU6050 отключен (тестовый режим)");
            mpuOk = false;
#endif

            Serial.println("[CALIB] ШАГ 4/4: Калибровка нуля давления...");

            setSystemState(SystemState::CALIBRATING);
            Serial.println("[CALIB] Entering CALIBRATING state");

            displayDirty = true;

            ZeroCalibrator localCalibrator;
            bool calibratorStarted = false;
            bool sensorErrorDuringCalib = false;

            uint32_t start = millis();

            while (millis() - start < CALIB_TIME_MS) {
                cfg_taskMonitorUpdate(TASK_CALIB_IDX);

                if (cfg_errorIsActive(ERR_SENSOR)) {
                    sensorErrorDuringCalib = true;
                    Serial.println("[CALIB] ? Ошибка SENSOR во время калибровки!");
                    break;
                }

                if (!calibratorStarted && (millis() - start >= 2000)) {
                    localCalibrator.start();
                    calibratorStarted = true;
                }

                if (calibratorStarted && !localCalibrator.isDone()) {
                    localCalibrator.process();
                }

                static uint32_t lastDirtySet = 0;
                if (millis() - lastDirtySet > 100) {
                    displayDirty = true;
                    lastDirtySet = millis();
                }

                vTaskDelay(pdMS_TO_TICKS(50));
            }

            if (sensorErrorDuringCalib) {
                Serial.println("[CALIB] ? Калибровка прервана из-за ошибки SENSOR!");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);
                continue;
            }

            if (calibratorStarted && !localCalibrator.isDone()) {
                Serial.println("[CALIB] Таймаут калибровки, использую последнее значение");
                g_pressureZeroBar = localCalibrator.getZeroValue();
            } else if (localCalibrator.isDone()) {
                g_pressureZeroBar = localCalibrator.getZeroValue();
            }

            Serial.printf("[CALIB] Калибровка завершена: zero=%.3f бар\n", g_pressureZeroBar);
            Logger::log(Logger::INFO, "CALIB", "Прогрев и калибровка завершены");

            setSystemState(SystemState::RUNNING);
            calibrationCompleted = true;
            calibrationValid = true;
            displayDirty = true;
            forceDisplayReset(true);

            Serial.println("[CALIB] ? Система готова к работе!");
            Serial.println("[CALIB] Ожидание первого замера давления...");
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
