#include "task_event.h"
#include "task_pool.h"
#include "task_monitor.h"
#include "event_bus.h"
#include "logger.h"
#include "app_globals.h"

void eventHandlerTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    SERIAL_DBG("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    SERIAL_DBG("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  Event event;

  extern uint8_t taskIndex_Event;

  for (;;) {
    TaskPool::markRun(taskIndex_Event);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_EVENT);
    if (EventBus::receive(event, pdMS_TO_TICKS(10))) {
      switch (event.type) {
        case EventType::ERROR_OCCURRED:
        case EventType::MODE_CHANGE:
        case EventType::DISPLAY_UPDATE:
          break;
        case EventType::CALIBRATION_DONE:
          Logger::log(Logger::INFO, "EVENT", "Калибровка завершена");
          break;
        default:
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

