#include "task_valve.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "task_pool.h"
#include "valve_ctrl.h"

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_VALVE_IDX = 10; // TaskMonitor::TASK_VALVE

void valveTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  Serial.println("[DEBUG] Valve Task started");

  extern uint8_t taskIndex_Valve;

  ValveCommandMsg cmd;
  ValveCommandMsg activeCmd;
  memset(&cmd, 0, sizeof(cmd));
  memset(&activeCmd, 0, sizeof(activeCmd));
  bool cmdActive = false;
  TickType_t cmdStartTime = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_Valve);
    cfg_taskMonitorUpdate(TASK_VALVE_IDX);

    if (valveStopRequested) {
      if (cmdActive) {
        if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(100), 3, "valve/stop")) {
          closeAllValves();
          xSemaphoreGive(xValveMutex);
        } else {
          closeAllValves();
        }
        if (activeCmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(activeCmd.sync.ackQueue, &success, 0);
        }
        cmdActive = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
      while (xQueueReceive(xValveQueue, &cmd, 0) == pdTRUE) {
        if (cmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(cmd.sync.ackQueue, &success, 0);
        }
      }
      closeAllValves();
      valveStopRequested = false;
    }

    if (otaValveLock) {
      if (cmdActive) {
        if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(100), 3, "valve/ota-lock")) {
          closeAllValves();
          xSemaphoreGive(xValveMutex);
        } else {
          closeAllValves();
        }
        cmdActive = false;
        lastCmd.commandActive = false;
        lastCmd.waitingForCompletion = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
      while (xQueueReceive(xValveQueue, &cmd, 0) == pdTRUE) {
        if (cmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(cmd.sync.ackQueue, &success, 0);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (cmdActive) {
      TickType_t now = xTaskGetTickCount();
      TickType_t elapsed = now - cmdStartTime;

      if (elapsed >= activeCmd.async.duration ||
          elapsed >= pdMS_TO_TICKS(VALVE_OPERATION_TIMEOUT_MS) ||
          activeCmd.async.duration == 0) {
        MutexGuard guard(xValveMutex);
        if (guard) {
          setValve(activeCmd.async.pad, LOW);
          digitalWrite(PIN_INFL, LOW);
          digitalWrite(PIN_DEFL, LOW);
        }

        if (activeCmd.sync.ackQueue != nullptr) {
          bool success = true;
          xQueueSend(activeCmd.sync.ackQueue, &success, 0);
        }

        cmdActive = false;
        lastCmd.commandActive = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
    }

    if (!cmdActive) {
      if (xQueueReceive(xValveQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {

        if (cmd.async.duration == 0) {
          MutexGuard guard(xValveMutex);
          if (guard) {
            setValve(cmd.async.pad, LOW);
            if (cmd.async.inflate) {
              digitalWrite(PIN_INFL, LOW);
            } else {
              digitalWrite(PIN_DEFL, LOW);
            }
          }

          if (cmd.sync.ackQueue != nullptr && uxQueueSpacesAvailable(cmd.sync.ackQueue) > 0) {
            bool success = true;
            xQueueSend(cmd.sync.ackQueue, &success, 0);
          }

          if (lastCmd.commandActive) {
            lastCmd.commandActive = false;
            lastCmd.waitingForCompletion = false;
          }

          continue;
        }

        if (cmd.async.pad >= PAD_COUNT) {
          Serial.printf("[VALVE] Invalid pad: %d\n", (int)cmd.async.pad);
          if (cmd.sync.ackQueue != nullptr) {
            bool success = false;
            xQueueSend(cmd.sync.ackQueue, &success, 0);
          }
          continue;
        }

        memcpy(&activeCmd, &cmd, sizeof(ValveCommandMsg));

        MutexGuard guard(xValveMutex);
        if (guard) {
          setValve(activeCmd.async.pad, HIGH);
          digitalWrite(activeCmd.async.inflate ? PIN_DEFL : PIN_INFL, LOW);
          digitalWrite(activeCmd.async.inflate ? PIN_INFL : PIN_DEFL, HIGH);

          cmdActive = true;
          cmdStartTime = xTaskGetTickCount();
          if (activeCmd.async.pad < PAD_COUNT) {
            valveCycleCount[activeCmd.async.pad]++;
          }

          Serial.printf("[VALVE] Started: pad=%d, inflate=%d, duration=%d ticks\n",
                        (int)activeCmd.async.pad,
                        activeCmd.async.inflate,
                        (int)activeCmd.async.duration);
        } else {
          Serial.println("[VALVE] Failed to get mutex!");
          if (activeCmd.sync.ackQueue != nullptr) {
            bool success = false;
            xQueueSend(activeCmd.sync.ackQueue, &success, 0);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
