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
    for (int i = 0; i < TASK_COUNT; i++) {
      if (tasks[i].lastRunTime == 0) continue;  // ещё не стартовала
      if (currentTime - tasks[i].lastRunTime > TASK_WDT_TIMEOUT_MS) {
        // Pressure при SENSOR / до калибровки / в паузе опроса — не зависание.
        if (i == (int)TASK_PRESSURE &&
            (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) || !calibrationCompleted)) {
          tasks[i].lastRunTime = currentTime;
          continue;
        }
        // TLS OTA: воркеры специально на паузе — не поднимать WATCHDOG.
        if (s_otaWorkersPaused &&
            (i == (int)TASK_BUTTON || i == (int)TASK_DISPLAY || i == (int)TASK_IMU ||
             i == (int)TASK_PRESSURE || i == (int)TASK_CONTROL || i == (int)TASK_VALVE ||
             i == (int)TASK_ERROR_RECOVERY)) {
          tasks[i].lastRunTime = currentTime;
          continue;
        }
        Logger::logf(Logger::ERROR, "WATCHDOG", "Task %s is not responding", tasks[i].name);
        ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG, "Task timeout");
        tasks[i].lastRunTime = currentTime;  // не спамить каждую секунду
      }
    }
  }

  static void init() {
    const uint32_t now = millis();
    tasks[TASK_EVENT] = { "Event", now, false };
    tasks[TASK_BUTTON] = { "Button", now, false };
    tasks[TASK_DISPLAY] = { "Display", now, false };
    tasks[TASK_IMU] = { "IMU", now, false };
    tasks[TASK_PRESSURE] = { "Pressure", now, false };
    tasks[TASK_CONTROL] = { "Control", now, false };
    tasks[TASK_CALIB] = { "Calib", now, false };
    tasks[TASK_WATCHDOG] = { "Watchdog", now, false };
    tasks[TASK_OTA] = { "OTA", now, false };
    tasks[TASK_VALVE] = { "Valve", now, false };
    tasks[TASK_ERROR_RECOVERY] = { "ErrorRecovery", now, false };
  }
};


void cfg_taskMonitorUpdate(uint8_t idx);
void printTaskInfo();
