#include "task_pressure.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "task_pool.h"
#include "pressure_read.h"
#include "valve_ctrl.h"
#include "auto_level.h"

/* ── ConfigManager bridge functions (defined in .ino) ── */
extern int  cfg_getPressureStabilizeMs();
extern int  cfg_getPressureIdleMin();
extern float cfg_getRedrawPressureThr();

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_PRESSURE_IDX = 4; // TaskMonitor::TASK_PRESSURE

/* ── .ino helpers ── */
void setDisplayDirty();

#if ENABLE_SIMULATION
extern float simPressures[];
extern float simMasterPressure;
void updateSimulationData();
#endif

/* ── ErrorHandler bridge ── */
extern bool cfg_errorIsActive(uint8_t err);
static constexpr uint8_t ERR_VALVE = 6; // ErrorHandler::Error::VALVE

static volatile uint32_t pressureQueueDropCount = 0;

/* ── .ino extern (valve-test logic, called during test) ── */
extern void runValveTestLogic();
extern SystemState getSystemState();

void pressureTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Pressure;
  PressureData pd = { 0 };
  uint8_t curPad = 0;
  Event event;

  static bool firstMeasurementDone = false;
  static bool needIdlePause = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Pressure);
    cfg_taskMonitorUpdate(TASK_PRESSURE_IDX);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    const uint32_t stabilizeMs =
        (uint32_t)constrain(cfg_getPressureStabilizeMs(), 100, 2000);
    const uint32_t idleMs =
        (uint32_t)constrain(cfg_getPressureIdleMin(), 2, 30) * 60000UL;

    const bool valveWork =
        manualControlActive || autoLevelIsBusy();
    uint32_t timeout;
    if (valveWork) {
      timeout = 200;
    } else if (needIdlePause) {
      timeout = idleMs;
    } else {
      timeout = 0;
    }

    uint32_t dummy;
    bool woken = false;
    constexpr uint32_t kWdtSliceMs = 1000;
    if (timeout == 0) {
      if (xQueueReceive(xPressureWakeupQueue, &dummy, 0) == pdTRUE) {
        woken = true;
      }
    } else {
      uint32_t waited = 0;
      while (waited < timeout) {
        cfg_taskMonitorUpdate(TASK_PRESSURE_IDX);
        uint32_t chunk = timeout - waited;
        if (chunk > kWdtSliceMs) chunk = kWdtSliceMs;
        if (xQueueReceive(xPressureWakeupQueue, &dummy, pdMS_TO_TICKS(chunk)) == pdTRUE) {
          woken = true;
          break;
        }
        waited += chunk;
      }
    }
    if (woken) {
      Serial.println("[PRESS] Wakeup by request");
      needIdlePause = false;
    } else if (needIdlePause && timeout == idleMs) {
      needIdlePause = false;
      Serial.printf("[PRESS] Пауза %d мин истекла — новый опрос подушек\n",
                    cfg_getPressureIdleMin());
    }

    {
      SystemState st = getSystemState();
      if (st == SystemState::BOOT || st == SystemState::CALIBRATING || !calibrationCompleted) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
    }

#if ENABLE_SIMULATION
    updateSimulationData();

    for (int i = 0; i < PAD_COUNT; i++) {
      pd.pressure[i] = simPressures[i];
    }
    pd.masterPressure = simMasterPressure;

    if (xQueueSend(xPressureQueue, &pd, pdMS_TO_TICKS(100)) != pdTRUE) pressureQueueDropCount++;

    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        memcpy(pressure, simPressures, sizeof(pressure));
        masterPressure = simMasterPressure;
      }
    }

    event.type = EventType::PRESSURE_UPDATE;
    event.timestamp = millis();
    memcpy(event.data.pressure.pressure, simPressures, sizeof(simPressures));
    event.data.pressure.masterPressure = simMasterPressure;
    EventBus::publish(event, 0);

    if (currentState != SystemState::CALIBRATING) {
      setDisplayDirty();
    }

    needIdlePause = true;

#else
    if (cfg_errorIsActive(ERR_VALVE) || otaValveLock) {
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (valveWork) {
      vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
      float p = readPressure();
      if (p >= 0.0f) {
        if (manualControlActive) {
          const uint8_t idx = static_cast<uint8_t>(manualPadIndex);
          if (idx < PAD_COUNT) {
            pd.pressure[idx] = p;
            pressureStampMs[idx] = millis();
            pressureValid[idx] = true;
          }
        } else {
          pd.pressure[curPad] = p;
          pressureStampMs[curPad] = millis();
          pressureValid[curPad] = true;
        }
      }
    } else if (xSemaphoreTake(xValveMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      setValve(Pad(curPad), HIGH);
      vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
      float p = readPressure();
      setValve(Pad(curPad), LOW);

      if (p >= 0.0f) {
        pd.pressure[curPad] = p;
        pressureStampMs[curPad] = millis();
        pressureValid[curPad] = true;
      }

      curPad = (curPad + 1) % PAD_COUNT;
      if (curPad == 0) {
        digitalWrite(PIN_INFL, HIGH);
        vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
        float master = readPressure();
        digitalWrite(PIN_INFL, LOW);
        if (master >= 0.0f) {
          pd.masterPressure = master;
          masterStampMs = millis();
          masterValid = true;
        }

        if (!firstMeasurementDone && pd.masterPressure >= 0.0f) {
          firstMeasurementDone = true;
          firstPressureMeasurementDone = true;
          Serial.printf("[PRESS] Первый замер выполнен! Давление: %.2f бар\n", pd.masterPressure);
          checkPressureLimits();
        }
        needIdlePause = true;
      }
      xSemaphoreGive(xValveMutex);
    }
#endif

    if (xQueueSend(xPressureQueue, &pd, pdMS_TO_TICKS(100)) != pdTRUE) {
      Serial.println("[ERROR] Failed to send to pressure queue");
    }

    bool pressureChanged = false;
    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        for (int i = 0; i < PAD_COUNT; i++) {
          if (abs(pressure[i] - pd.pressure[i]) > cfg_getRedrawPressureThr()) {
            pressureChanged = true;
            break;
          }
        }
        if (abs(masterPressure - pd.masterPressure) > cfg_getRedrawPressureThr()) {
          pressureChanged = true;
        }
        memcpy(pressure, pd.pressure, sizeof(pressure));
        masterPressure = pd.masterPressure;
      }
    }

    event.type = EventType::PRESSURE_UPDATE;
    event.timestamp = millis();
    memcpy(event.data.pressure.pressure, pd.pressure, sizeof(pd.pressure));
    event.data.pressure.masterPressure = pd.masterPressure;
    EventBus::publish(event, 0);

    if (pressureChanged) {
      if (currentState != SystemState::CALIBRATING) {
        setDisplayDirty();
      }
    }
  }
}
