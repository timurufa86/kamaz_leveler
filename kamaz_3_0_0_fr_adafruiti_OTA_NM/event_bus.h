#pragma once

#include "app_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

class EventBus {
private:
  static QueueHandle_t eventQueue_;
  /* 10.15.5: 128 → 32 slots of Event (~76B) frees ~7KB of heap. OTA needs free
     heap for TLS reconnects after a partial flash write (85-88KB was too little);
     the bus is advisory UI telemetry and drops events when full. */
  static constexpr size_t QUEUE_SIZE = 32;
  static SemaphoreHandle_t mutex_;
  static uint32_t droppedEvents_;
  static uint32_t maxDepth_;

public:
  static bool init();
  /** 10.15.6: accessor so the OTA worker-task freeing can guard this mutex too. */
  static SemaphoreHandle_t mutex() { return mutex_; }
  static bool publish(const Event &event, TickType_t timeout = pdMS_TO_TICKS(100));
  static bool receive(Event &event, TickType_t timeout = portMAX_DELAY);
  static size_t available();
  static void flush();
  static uint32_t getDroppedCount();
  static uint32_t dropped();
  static uint32_t maxDepth();
};
