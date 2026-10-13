#pragma once
/**
 *  task_monitor.h — per-task heartbeat used by the watchdog task.
 */
#include <Arduino.h>
#include "logger.h"
#include "error_handler.h"
#include "app_globals.h"

constexpr uint8_t TASK_COUNT = 11;  // Event..Valve + ErrorRecovery (index 0..10)
constexpr uint32_t TASK_WDT_TIMEOUT_MS = 10000;

extern volatile bool s_otaWorkersPaused;

class TaskMonitor {
private:
  struct TaskInfo {
    const char *name;
    uint32_t lastRunTime;
    bool isRunning;
  };
  static TaskInfo tasks[TASK_COUNT];

public:
  enum TaskIndex : uint8_t {
    TASK_EVENT = 0,
    TASK_BUTTON,
    TASK_DISPLAY,
    TASK_IMU,
    TASK_PRESSURE,
    TASK_CONTROL,
    TASK_CALIB,
    TASK_WATCHDOG,
    TASK_OTA,
    TASK_VALVE,
    TASK_ERROR_RECOVERY
  };

  static void updateTaskStatus(TaskIndex index) {
    tasks[index].lastRunTime = millis();
    tasks[index].isRunning = true;
  }

  static void checkTasks() {
    uint32_t currentTime = millis();
    // TLS OTA blocks in http.GET/handshake — soft WDT must not emergencyStop mid-download.
    if (otaInProgress || s_otaWorkersPaused) {
      for (int i = 0; i < TASK_COUNT; i++) {
        if (tasks[i].lastRunTime != 0) tasks[i].lastRunTime = currentTime;
      }
      return;
    }
    // Grace after TaskMonitor::init — калибровка / повторный DMP не дают ложный timeout.
    static uint32_t bootMs = 0;
    if (bootMs == 0) bootMs = currentTime;
    if ((currentTime - bootMs) < 15000UL) return;

    // После срабатывания не спамить handleError (иначе cancel pending-clear → «липкий» экран).
    static uint32_t lastSoftWdtMs = 0;
    for (int i = 0; i < TASK_COUNT; i++) {
      if (tasks[i].lastRunTime == 0) continue;  // ещё не стартовала
      const uint32_t age = currentTime - tasks[i].lastRunTime;
      if (age > TASK_WDT_TIMEOUT_MS) {
        // Pressure при SENSOR / до калибровки / в паузе опроса — не зависание.
        if (i == (int)TASK_PRESSURE &&
            (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) || !calibrationCompleted)) {
          tasks[i].lastRunTime = currentTime;
          continue;
        }
        char msg[48];
        snprintf(msg, sizeof(msg), "Task %s timeout %lus", tasks[i].name,
                 (unsigned long)(age / 1000UL));
        Serial.printf("[WDT] %s | ages:", msg);
        for (int j = 0; j < TASK_COUNT; j++) {
          if (tasks[j].lastRunTime == 0) continue;
          Serial.printf(" %s=%lu", tasks[j].name,
                        (unsigned long)((currentTime - tasks[j].lastRunTime) / 1000UL));
        }
        Serial.println();
        Logger::logf(Logger::ERROR, "WATCHDOG", "%s", msg);
        // Не чаще раза в 20 с: иначе handleError снимает pending-clear и экран не гаснет.
        if (lastSoftWdtMs == 0 || (currentTime - lastSoftWdtMs) > 20000UL) {
          lastSoftWdtMs = currentTime;
          ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG, msg);
        }
        tasks[i].lastRunTime = currentTime;  // не спамить каждую секунду
      }
    }
  }

  static void init() {
    /* lastRunTime=0 → checkTasks skips until first real heartbeat. */
    tasks[TASK_EVENT] = { "Event", 0, false };
    tasks[TASK_BUTTON] = { "Button", 0, false };
    tasks[TASK_DISPLAY] = { "Display", 0, false };
    tasks[TASK_IMU] = { "IMU", 0, false };
    tasks[TASK_PRESSURE] = { "Pressure", 0, false };
    tasks[TASK_CONTROL] = { "Control", 0, false };
    tasks[TASK_CALIB] = { "Calib", 0, false };
    tasks[TASK_WATCHDOG] = { "Watchdog", 0, false };
    tasks[TASK_OTA] = { "OTA", 0, false };
    tasks[TASK_VALVE] = { "Valve", 0, false };
    tasks[TASK_ERROR_RECOVERY] = { "ErrorRecovery", 0, false };
  }
};


void cfg_taskMonitorUpdate(uint8_t idx);
void printTaskInfo();
