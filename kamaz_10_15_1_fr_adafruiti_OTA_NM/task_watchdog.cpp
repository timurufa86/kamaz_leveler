#include "task_watchdog.h"
#include "logger.h"
#include "task_monitor.h"
#include "task_pool.h"
#include "memory_monitor.h"
#include <esp_task_wdt.h>

uint32_t uptimeHours = 0;

void watchdogTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    SERIAL_DBG("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    SERIAL_DBG("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_Watchdog;
  esp_task_wdt_add(NULL);
  for (;;) {
    TaskPool::markRun(taskIndex_Watchdog);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_WATCHDOG);
    TaskMonitor::checkTasks();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void initWatchdog() {
  esp_task_wdt_config_t config = {
    .timeout_ms = TASK_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t result = esp_task_wdt_init(&config);
  if (result == ESP_ERR_INVALID_STATE) {
    /* 10.15.1: Arduino-ядро уже создало TWDT, поэтому esp_task_wdt_init()
       возвращает ESP_ERR_INVALID_STATE и IDF печатает "E task_wdt: TWDT already
       initialized". Вместо повторной инициализации штатно перенастраиваем таймаут;
       если ядро не даёт — работаем на его конфигурации (как в 10.15.0). */
    const esp_err_t reconf = esp_task_wdt_reconfigure(&config);
    if (reconf == ESP_OK) {
      SERIAL_DBG("[WDT] TWDT создан ядром — перенастроен на %u ms\n", TASK_WDT_TIMEOUT_MS);
    } else {
      SERIAL_DBG("[WDT] TWDT создан ядром, reconfigure failed: %d — конфигурация ядра\n",
                    reconf);
    }
  } else if (result != ESP_OK) {
    SERIAL_DBG("[WDT] init failed: %d\n", result);
    return;
  } else {
    SERIAL_DBG("[WDT] initialized: %u ms\n", TASK_WDT_TIMEOUT_MS);
  }
}


void systemMaintenanceTick() {
  static uint32_t lastHourCounter = 0;
  static uint32_t lastMemoryCheck = 0;
  static uint32_t lastStackCheck = 0;
  static uint32_t lastTaskInfoPrint = 0;
  if (millis() - lastHourCounter >= 3600000) {
    uptimeHours++;
    lastHourCounter = millis();
  }

  if (millis() - lastMemoryCheck >= 5000) {
    MemoryMonitor::checkMemory();
    lastMemoryCheck = millis();
  }

  if (millis() - lastStackCheck > 30000) {
    lastStackCheck = millis();
    for (uint8_t i = 0; i < TaskPool::getTaskCount(); i++) {
      UBaseType_t stackFree = TaskPool::getTaskMinStack(i);
      if (stackFree < 200) {
        const char *name = TaskPool::getTaskName(i);
        SERIAL_DBG("[STACK] ?? Task '%s' has only %d bytes free!\n",
                      name ? name : "Unknown", stackFree);
      }
    }
  }

  // Verbose-only: TEST STACK calls printTaskInfo() on demand.
  if (Logger::verbose() && (millis() - lastTaskInfoPrint > 60000)) {
    lastTaskInfoPrint = millis();
    printTaskInfo();
  }

  static uint32_t lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 60000) {
    SERIAL_DBG("[MEM] Free heap: %d bytes\n", ESP.getFreeHeap());
    lastHeapCheck = millis();
  }
}
