#pragma once
/**
 *  memory_monitor.h — heap low/critical warnings and valve stop.
 */
#include <Arduino.h>
#include "logger.h"
#include "event_bus.h"
#include "valve_ctrl.h"

class MemoryMonitor {
private:
  static uint32_t lastCheckTime;
  static uint32_t minFreeHeap;
  static uint32_t lastWarningTime;
  static constexpr uint32_t LOW_MEMORY_THRESHOLD = 8192;
  static constexpr uint32_t CRITICAL_MEMORY_THRESHOLD = 4096;
  static bool lowMemoryReported;
  static bool criticalMemoryReported;

public:
  static void init() {
    lastCheckTime = millis();
    minFreeHeap = ESP.getFreeHeap();
    lastWarningTime = 0;
    lowMemoryReported = false;
    criticalMemoryReported = false;
    SERIAL_DBG("[MEM] Начально свободно: %d байт\n", minFreeHeap);
  }

  static void checkMemory() {
    uint32_t currentFree = ESP.getFreeHeap();
    uint32_t now = millis();

    if (currentFree < minFreeHeap) {
      minFreeHeap = currentFree;
      SERIAL_DBG("[MEM] Новый минимум: %d байт\n", minFreeHeap);
    }

    if (currentFree < CRITICAL_MEMORY_THRESHOLD) {
      if (!criticalMemoryReported) {
        criticalMemoryReported = true;
        SERIAL_DBG("[MEM] ?? КРИТИЧЕСКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

        Event event;
        event.type = EventType::CRITICAL_MEMORY;
        event.timestamp = now;
        event.data.memory.freeHeap = currentFree;
        event.data.memory.minHeap = minFreeHeap;
        EventBus::publish(event);

        emergencyStop();
        closeAllValves();
      }
    } else if (currentFree < LOW_MEMORY_THRESHOLD) {
      if (!lowMemoryReported && (now - lastWarningTime > 60000)) {
        lowMemoryReported = true;
        lastWarningTime = now;
        SERIAL_DBG("[MEM] ?? НИЗКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

        Event event;
        event.type = EventType::LOW_MEMORY_WARNING;
        event.timestamp = now;
        event.data.memory.freeHeap = currentFree;
        event.data.memory.minHeap = minFreeHeap;
        EventBus::publish(event);
      }
    } else {
      if (criticalMemoryReported && currentFree > CRITICAL_MEMORY_THRESHOLD + 2048) {
        criticalMemoryReported = false;
        SERIAL_DBG("[MEM] ? Память восстановлена: %d байт\n", currentFree);
      }
      if (lowMemoryReported && currentFree > LOW_MEMORY_THRESHOLD + 2048) {
        lowMemoryReported = false;
        SERIAL_DBG("[MEM] ? Уровень памяти нормализован: %d байт\n", currentFree);
      }
    }

    if (now - lastCheckTime > 60000) {
      lastCheckTime = now;
      SERIAL_DBG("[MEM] Свободно: %d байт (минимум: %d)\n", currentFree, minFreeHeap);
    }
  }
};

