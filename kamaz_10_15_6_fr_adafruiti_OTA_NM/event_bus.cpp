#include "event_bus.h"
#include "logger.h"
#include <Arduino.h>

QueueHandle_t EventBus::eventQueue_ = nullptr;
SemaphoreHandle_t EventBus::mutex_ = nullptr;
uint32_t EventBus::droppedEvents_ = 0;
uint32_t EventBus::maxDepth_ = 0;

bool EventBus::init() {
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) return false;

  eventQueue_ = xQueueCreate(QUEUE_SIZE, sizeof(Event));
  if (eventQueue_ == nullptr) return false;

  SERIAL_DBGLN("[EVENTBUS] Queue created");
  return true;
}

bool EventBus::publish(const Event &event, TickType_t timeout) {
  if (eventQueue_ == nullptr) return false;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (uxQueueSpacesAvailable(eventQueue_) == 0) {
      droppedEvents_++;
      if (droppedEvents_ % 10 == 1) {
        SERIAL_DBG("[WARN] EventBus queue full, dropped %d events\n", droppedEvents_);
      }
      xSemaphoreGive(mutex_);
      return false;
    }
    xSemaphoreGive(mutex_);
  }

  Event eventCopy = event;
  BaseType_t result = xQueueSend(eventQueue_, &eventCopy, timeout);
  // 10.15.6: counters were updated outside the mutex (lost increments / torn maxDepth).
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    const size_t depth = uxQueueMessagesWaiting(eventQueue_);
    if (depth > maxDepth_) maxDepth_ = depth;
    if (result != pdTRUE) droppedEvents_++;
    xSemaphoreGive(mutex_);
  } else if (result != pdTRUE) {
    droppedEvents_++;  // mutex busy: count anyway (rare, harmless race)
  }
  return result == pdTRUE;
}

bool EventBus::receive(Event &event, TickType_t timeout) {
  if (eventQueue_ == nullptr) return false;
  return xQueueReceive(eventQueue_, &event, timeout) == pdTRUE;
}

size_t EventBus::available() {
  if (eventQueue_ == nullptr) return 0;
  return uxQueueMessagesWaiting(eventQueue_);
}

void EventBus::flush() {
  if (eventQueue_ == nullptr) return;
  xQueueReset(eventQueue_);
  // 10.15.6: reset the counters under the same mutex publishers use.
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    droppedEvents_ = 0;
    xSemaphoreGive(mutex_);
  }
}

uint32_t EventBus::getDroppedCount() {
  return droppedEvents_;
}

uint32_t EventBus::dropped() {
  return droppedEvents_;
}

uint32_t EventBus::maxDepth() {
  return maxDepth_;
}
