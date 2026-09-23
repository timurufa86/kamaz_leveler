#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/** RAII mutex take/give. Evaluates false if take failed. */
class MutexGuard {
private:
  SemaphoreHandle_t mutex_;
  bool locked_;
  static uint32_t failCount_;
  static uint32_t timeoutCount_;

public:
  explicit MutexGuard(SemaphoreHandle_t mutex, TickType_t timeout = pdMS_TO_TICKS(100))
    : mutex_(mutex), locked_(false) {
    if (mutex_ == nullptr) {
      Serial.printf("[MUTEX] FATAL: Mutex is NULL!\n");
      failCount_++;
      return;
    }
    BaseType_t result = xSemaphoreTake(mutex_, timeout);
    locked_ = (result == pdTRUE);
    if (!locked_) {
      timeoutCount_++;
      if (timeoutCount_ % 10 == 1) {
        Serial.printf("[MUTEX] TIMEOUT #%d! Task: %s, Timeout: %d ms\n",
                      timeoutCount_,
                      pcTaskGetName(xTaskGetCurrentTaskHandle()),
                      pdTICKS_TO_MS(timeout));
      }
    }
  }

  ~MutexGuard() {
    if (locked_ && mutex_ != nullptr) {
      xSemaphoreGive(mutex_);
      locked_ = false;
    }
  }

  bool isLocked() const { return locked_; }
  operator bool() const { return locked_; }
  static uint32_t getFailCount() { return failCount_; }
  static uint32_t getTimeoutCount() { return timeoutCount_; }

  MutexGuard(const MutexGuard &) = delete;
  MutexGuard &operator=(const MutexGuard &) = delete;
};

/**
 * Take mutex with retries. On final failure returns false.
 * Critical paths (valves/emergency) must not silently skip work.
 */
inline bool takeMutexWithRetry(SemaphoreHandle_t mutex, TickType_t timeoutPerTry,
                               int retries, const char *tag) {
  if (mutex == nullptr) {
    Serial.printf("[MUTEX] %s: mutex is NULL\n", tag ? tag : "?");
    return false;
  }
  for (int i = 0; i < retries; i++) {
    if (xSemaphoreTake(mutex, timeoutPerTry) == pdTRUE) return true;
    Serial.printf("[MUTEX] %s: retry %d/%d\n", tag ? tag : "?", i + 1, retries);
  }
  return false;
}
