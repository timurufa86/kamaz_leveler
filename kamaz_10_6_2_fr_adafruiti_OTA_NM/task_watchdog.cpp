#include "task_watchdog.h"
#include "task_monitor.h"
#include "task_pool.h"
#include "memory_monitor.h"
#include <esp_task_wdt.h>

uint32_t uptimeHours = 0;

void watchdogTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
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
    Serial.println("[WDT] TWDT уже инициализирован ядром, используем текущую конфигурацию");
  } else if (result != ESP_OK) {
    Serial.printf("[WDT] init failed: %d\n", result);
    return;
  } else {
    Serial.printf("[WDT] initialized: %u ms\n", TASK_WDT_TIMEOUT_MS);
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
        Serial.printf("[STACK] ?? Task '%s' has only %d bytes free!\n",
                      name ? name : "Unknown", stackFree);
      }
    }
  }

  // ? Добавлена диагностика задач раз в минуту
  if (millis() - lastTaskInfoPrint > 60000) {
    lastTaskInfoPrint = millis();
    printTaskInfo();
  }

  static uint32_t lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 60000) {
    Serial.printf("[MEM] Free heap: %d bytes\n", ESP.getFreeHeap());
    lastHeapCheck = millis();
  }
}
