#pragma once

#include "app_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

class EventBus {
private:
  static QueueHandle_t eventQueue_;
  static constexpr size_t QUEUE_SIZE = 128;
  static SemaphoreHandle_t mutex_;
  static uint32_t droppedEvents_;
  static uint32_t maxDepth_;

public:
  static bool init();
  static bool publish(const Event &event, TickType_t timeout = pdMS_TO_TICKS(100));
  static bool receive(Event &event, TickType_t timeout = portMAX_DELAY);
  static size_t available();
  static void flush();
  static uint32_t getDroppedCount();
  static uint32_t dropped();
  static uint32_t maxDepth();
};
