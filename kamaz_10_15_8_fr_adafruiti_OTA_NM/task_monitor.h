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
    // 10.15.6: bound check — cfg_taskMonitorUpdate() accepts a raw uint8_t.
    if (static_cast<uint8_t>(index) >= TASK_COUNT) return;
    tasks[index].lastRunTime = millis();
    tasks[index].isRunning = true;
  }

  static void checkTasks() {
    uint32_t currentTime = millis();
    // 10.15.0: задачи действительно остановлены на время OTA-сессии — метки
    // больше не подделываются (раньше монитор считал их живыми), но и
    // зависанием это не считается: проверка просто пропускается.
    static uint32_t hangFirstMs[TASK_COUNT] = {0};
    static bool pausedSinceLastCheck = false;
    if (otaInProgress || s_otaWorkersPaused) {
      pausedSinceLastCheck = true;
      static uint32_t lastPauseLog = 0;
      if (currentTime - lastPauseLog > 60000UL) {
        lastPauseLog = currentTime;
        SERIAL_DBGLN("[WDT] мониторинг задач приостановлен (OTA-сессия)");
      }
      return;
    }
    /* 10.15.6: после паузы OTA метки задач устарели на всю длительность сессии
       (> 10 с) → ложное «зависание»: closeAllValves + ошибка WATCHDOG + риск
       ESP.restart(). Один раз перебазируем метки и выходим. */
    if (pausedSinceLastCheck) {
      pausedSinceLastCheck = false;
      for (int i = 0; i < TASK_COUNT; i++) {
        if (tasks[i].lastRunTime != 0) tasks[i].lastRunTime = currentTime;
        hangFirstMs[i] = 0;
      }
      return;
    }
    // Grace after TaskMonitor::init — калибровка / повторный DMP не дают ложный timeout.
    static uint32_t bootMs = 0;
    if (bootMs == 0) bootMs = currentTime;
    if ((currentTime - bootMs) < 15000UL) return;

    // 10.15.0: зависание задачи — не только экран. Клапаны закрываются сразу,
    // а если задача не ожила за два периода монитора — устройство перезапускается.
    for (int i = 0; i < TASK_COUNT; i++) {
      if (tasks[i].lastRunTime == 0) continue;  // ещё не стартовала
      const uint32_t age = currentTime - tasks[i].lastRunTime;
      if (age <= TASK_WDT_TIMEOUT_MS) {
        hangFirstMs[i] = 0;
        continue;
      }
      // Pressure при SENSOR / до калибровки / в паузе опроса — не зависание.
      if (i == (int)TASK_PRESSURE &&
          (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) || !calibrationCompleted)) {
        tasks[i].lastRunTime = currentTime;
        hangFirstMs[i] = 0;
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
      closeAllValves();
      if (hangFirstMs[i] == 0) {
        hangFirstMs[i] = currentTime;
        ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG, msg);
      } else if (currentTime - hangFirstMs[i] > (TASK_WDT_TIMEOUT_MS * 2UL)) {
        Logger::logf(Logger::ERROR, "WATCHDOG", "Task %s hung — restart", tasks[i].name);
        closeAllValves();
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP.restart();
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
