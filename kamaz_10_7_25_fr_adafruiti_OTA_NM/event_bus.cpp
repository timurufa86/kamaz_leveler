#include "event_bus.h"
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

  Serial.println("[EVENTBUS] Queue created");
  return true;
}

bool EventBus::publish(const Event &event, TickType_t timeout) {
  if (eventQueue_ == nullptr) return false;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (uxQueueSpacesAvailable(eventQueue_) == 0) {
      droppedEvents_++;
      if (droppedEvents_ % 10 == 1) {
        Serial.printf("[WARN] EventBus queue full, dropped %d events\n", droppedEvents_);
      }
      xSemaphoreGive(mutex_);
      return false;
    }
    xSemaphoreGive(mutex_);
  }

  Event eventCopy = event;
  BaseType_t result = xQueueSend(eventQueue_, &eventCopy, timeout);
  size_t depth = uxQueueMessagesWaiting(eventQueue_);
  if (depth > maxDepth_) maxDepth_ = depth;
  if (result != pdTRUE) droppedEvents_++;
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
  droppedEvents_ = 0;
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
