#pragma once
/**
 *  error_handler.h — active-error buffer, pending clear, UI blocking.
 */
#include <Arduino.h>
#include <cstring>
#include "mutex_guard.h"
#include "app_globals.h"
#include "event_bus.h"
#include "valve_ctrl.h"

extern bool errorScreenBlocking;

class ErrorHandler {
public:
  enum class Error : uint8_t {
    NONE = 0,
    LOW_PRESSURE,
    MPU,
    SENSOR,
    VALVE,
    WATCHDOG,
    OTA,
    COUNT
  };

  struct ActiveError {
    Error error;
    uint32_t lastSeenTime;
  };

  struct PendingClear {
    Error error;
    uint32_t clearRequestTime;
  };

  // ========== ДОБАВЛЕН МЬЮТЕКС ==========
  static SemaphoreHandle_t mutex_;

  static ActiveError activeErrors[8];
  static uint8_t activeCount;
  static PendingClear pendingClear[8];
  static uint8_t pendingCount;
  static constexpr uint32_t ERROR_TIMEOUT_MS = 3000;
  static uint8_t currentDisplayIndex;
  static uint32_t lastDisplaySwitch;
  static bool hasError;

  // ========== ИНИЦИАЛИЗАЦИЯ МЬЮТЕКСА ==========
  static bool initMutex() {
    mutex_ = xSemaphoreCreateMutex();
    return mutex_ != nullptr;
  }

  static void handleError(Error error, const char *message) {
    if ((uint8_t)error >= (uint8_t)Error::COUNT) {
      Serial.printf("[ERROR] Invalid error code %d\n", (int)error);
      return;
    }

    if (error == Error::LOW_PRESSURE) {
      static uint32_t lastAttempt = 0;
      if (millis() - lastAttempt < 1500) return;
      lastAttempt = millis();
    }

    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) {
      Serial.println("[ERROR] Failed to lock mutex in handleError!");
      return;
    }

    Serial.printf("[ERROR] %s\n", message);
    removeFromPendingClearLocked(error);
    updateActiveBufferLocked(error);
    hasError = true;

    if (isCriticalError(error)) emergencyStop();

    Event event;
    event.type = EventType::ERROR_OCCURRED;
    event.timestamp = millis();
    event.data.error.errorCode = static_cast<uint8_t>(error);
    strncpy(event.data.error.message, message, sizeof(event.data.error.message) - 1);
    EventBus::publish(event);
  }

  static void updateErrorTime(Error error) {
    bool found = false;

    // Проверяем, есть ли уже такая ошибка
    {
      MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
      if (guard) {
        for (int i = 0; i < activeCount; i++) {
          if (activeErrors[i].error == error) {
            activeErrors[i].lastSeenTime = millis();
            found = true;
            break;
          }
        }
      }
      // Мьютекс освободится здесь автоматически
    }

    // Если не нашли - создаем новую (без мьютекса, handleError сама захватит)
    if (!found) {
      handleError(error, "Error detected");
    }
  }

  static void markErrorCleared(Error error) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return;

    bool found = false;
    for (int i = 0; i < activeCount; i++) {
      if (activeErrors[i].error == error) {
        found = true;
        break;
      }
    }
    if (!found) return;

    for (int i = 0; i < pendingCount; i++) {
      if (pendingClear[i].error == error) {
        pendingClear[i].clearRequestTime = millis();
        return;
      }
    }

    if (pendingCount < 8) {
      pendingClear[pendingCount].error = error;
      pendingClear[pendingCount].clearRequestTime = millis();
      pendingCount++;
      Serial.printf("[ERROR] Ошибка %d будет удалена через 3 сек\n", (int)error);
    }
  }

  static bool isCriticalError(Error error) {
    // MPU отсутствует на шине — не критично: давление/клапаны работают без IMU
    return error == Error::VALVE || error == Error::WATCHDOG;
  }

  /** Полноэкранная ошибка: MPU не блокирует пневматику и главный экран. */
  static bool isUiBlockingError(Error error) {
    return error != Error::MPU;
  }

  static bool hasUiBlockingErrors() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return false;
    pruneActiveBufferLocked();
    for (int i = 0; i < activeCount; i++) {
      if (isUiBlockingError(activeErrors[i].error)) return true;
    }
    return false;
  }

  static void forceClearAllErrors() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return;

    activeCount = 0;
    pendingCount = 0;
    hasError = false;
    errorScreenBlocking = false;
    currentDisplayIndex = 0;
    lastDisplaySwitch = 0;
    memset(activeErrors, 0, sizeof(activeErrors));
    memset(pendingClear, 0, sizeof(pendingClear));
    displayDirty = true;  // без fillScreen
    Serial.println("[ERROR] Все ошибки сброшены");
  }

  static bool getErrorInfo(int index, Error &error, int &total) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return false;

    pruneActiveBufferLocked();
    total = activeCount;
    if (index >= 0 && index < activeCount) {
      error = activeErrors[index].error;
      return true;
    }
    return false;
  }

  /** true при любой ошибке, мешающей пневматике (не OTA). Fail-closed при mutex. */
  static bool hasCriticalPneumaticErrors() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return true;
    pruneActiveBufferLocked();
    for (int i = 0; i < activeCount; i++) {
      const Error e = activeErrors[i].error;
      if (e == Error::LOW_PRESSURE || e == Error::SENSOR || e == Error::VALVE ||
          e == Error::WATCHDOG) {
        return true;
      }
    }
    return false;
  }

  static bool hasActiveErrors() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return false;
    pruneActiveBufferLocked();
    return activeCount > 0;
  }

  static int getActiveErrorCount() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return 0;
    pruneActiveBufferLocked();
    return activeCount;
  }

  static bool isErrorActive(Error error) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return false;
    pruneActiveBufferLocked();
    for (int i = 0; i < activeCount; i++) {
      if (activeErrors[i].error == error) return true;
    }
    return false;
  }

  static void removeError(Error error) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return;

    for (int i = 0; i < activeCount; i++) {
      if (activeErrors[i].error == error) {
        for (int j = i; j < activeCount - 1; j++) {
          activeErrors[j] = activeErrors[j + 1];
        }
        activeCount--;
        break;
      }
    }
    removeFromPendingClearLocked(error);

    if (activeCount == 0) {
      hasError = false;
      errorScreenBlocking = false;
      if (!menuVisible && !wifiSetupActive) displayDirty = true;
    }
    if (!menuVisible && !wifiSetupActive) displayDirty = true;
  }

  static Error getCurrentActiveError() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return Error::NONE;

    pruneActiveBufferLocked();
    if (activeCount == 0) return Error::NONE;

    if (currentDisplayIndex >= activeCount) {
      currentDisplayIndex = 0;
    }

    uint32_t now = millis();
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      currentDisplayIndex = 0;
      lastDisplaySwitch = now;
    }

    if (activeCount > 1 && (now - lastDisplaySwitch) >= 2000) {
      currentDisplayIndex = (currentDisplayIndex + 1) % activeCount;
      lastDisplaySwitch = now;
    }

    if (currentDisplayIndex >= activeCount) {
      currentDisplayIndex = 0;
    }

    return activeErrors[currentDisplayIndex].error;
  }

  static int getCurrentDisplayIndex() {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return 0;
    return currentDisplayIndex;
  }

  static Error getActiveErrorAt(int index) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return Error::NONE;
    pruneActiveBufferLocked();
    if (index >= 0 && index < activeCount) {
      return activeErrors[index].error;
    }
    return Error::NONE;
  }

  static const char *getErrorMessage(Error error) {
    switch (error) {
      case Error::NONE: return "НЕТ ОШИБКИ";
      case Error::LOW_PRESSURE: return "НИЗКОЕ ДАВЛЕНИЕ";
      case Error::MPU: return "ОШИБКА MPU6050";
      case Error::SENSOR: return "ОШИБКА ДАТЧИКА ДАВЛЕНИЯ";
      case Error::VALVE: return "ОШИБКА КЛАПАНА";
      case Error::WATCHDOG: return "ОШИБКА WATCHDOG";
      case Error::OTA: return "ОШИБКА OTA";
      default: return "НЕИЗВЕСТНАЯ ОШИБКА";
    }
  }

  static bool isPendingClear(Error error) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return false;
    for (int i = 0; i < pendingCount; i++) {
      if (pendingClear[i].error == error) return true;
    }
    return false;
  }

static void cancelClear(Error error) {
    MutexGuard guard(mutex_, pdMS_TO_TICKS(100));
    if (!guard) return;
    
    for (int i = 0; i < pendingCount; i++) {
        if (pendingClear[i].error == error) {
            for (int j = i; j < pendingCount - 1; j++) {
                pendingClear[j] = pendingClear[j + 1];
            }
            pendingCount--;
            Serial.printf("[ERROR] Отменено удаление ошибки %d\n", (int)error);
            return;
        }
    }
}

private:
  static void updateActiveBufferLocked(Error error) {
    uint32_t now = millis();
    for (int i = 0; i < activeCount; i++) {
      if (activeErrors[i].error == error) {
        activeErrors[i].lastSeenTime = now;
        return;
      }
    }
    if (activeCount < 8) {
      activeErrors[activeCount].error = error;
      activeErrors[activeCount].lastSeenTime = now;
      activeCount++;
    }
  }

  static void removeFromPendingClearLocked(Error error) {
    for (int i = 0; i < pendingCount; i++) {
      if (pendingClear[i].error == error) {
        for (int j = i; j < pendingCount - 1; j++) {
          pendingClear[j] = pendingClear[j + 1];
        }
        pendingCount--;
        break;
      }
    }
  }

  static void pruneActiveBufferLocked() {
    uint32_t now = millis();
    bool anyRemoved = false;

    for (int i = 0; i < pendingCount;) {
      if (now - pendingClear[i].clearRequestTime > ERROR_TIMEOUT_MS) {
        bool found = false;
        for (int j = 0; j < activeCount; j++) {
          if (activeErrors[j].error == pendingClear[i].error) {
            for (int k = j; k < activeCount - 1; k++) {
              activeErrors[k] = activeErrors[k + 1];
            }
            activeCount--;
            found = true;
            anyRemoved = true;
            if (!otaInProgress) {
              Serial.printf("[ERROR] Удалена ошибка %d\n", (int)pendingClear[i].error);
            }
            break;
          }
        }

        for (int k = i; k < pendingCount - 1; k++) {
          pendingClear[k] = pendingClear[k + 1];
        }
        pendingCount--;
      } else {
        i++;
      }
    }

    if (anyRemoved) {
      if (!menuVisible && !wifiSetupActive) displayDirty = true;
    }

    // Не вызываем forceDisplayReset (fillScreen) — это давало мерцание каждые ~3 с
    // и срывало меню. Достаточно displayDirty; hasError синхронизируем с буфером.
    if (activeCount == 0) {
      if (hasError) {
        hasError = false;
        errorScreenBlocking = false;
        if (!menuVisible && !wifiSetupActive) displayDirty = true;
        // Только после реального удаления из pending — иначе спам на каждом кадре UI,
        // когда hasError «залип», а activeCount уже 0.
        if (anyRemoved && !otaInProgress) {
          Serial.println("[ERROR] Все ошибки удалены");
        }
      }
    } else {
      hasError = true;
    }
  }
};


void resetSystemErrors();

bool cfg_errorIsActive(uint8_t err);
void cfg_errorMarkCleared(uint8_t err);
void cfg_errorRemove(uint8_t err);
void cfg_errorHandle(uint8_t err, const char *msg);
void cfg_errorUpdateTime(uint8_t err);
bool cfg_errorIsPendingClear(uint8_t err);
void cfg_errorCancelClear(uint8_t err);
bool cfg_errorHasActive();
bool cfg_errorHasCriticalPneumatic();
bool cfg_errorHasUiBlocking();
int cfg_errorActiveCount();
uint8_t cfg_errorCurrent();
int cfg_errorDisplayIndex();
const char *cfg_errorMessage(uint8_t err);
