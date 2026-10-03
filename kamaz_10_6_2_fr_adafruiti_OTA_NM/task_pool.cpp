#include "task_pool.h"
#include <Arduino.h>

TaskPool::TaskHandle TaskPool::tasks_[16];
uint8_t TaskPool::taskCount_ = 0;
SemaphoreHandle_t TaskPool::poolMutex_ = nullptr;

bool TaskPool::init() {
  poolMutex_ = xSemaphoreCreateMutex();
  return poolMutex_ != nullptr;
}

uint8_t TaskPool::addTask(const TaskConfig &config) {
  if (poolMutex_ == nullptr) return 0xFF;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(100)) != pdTRUE) return 0xFF;

  if (taskCount_ >= 16) {
    xSemaphoreGive(poolMutex_);
    return 0xFF;
  }

  TaskHandle_t handle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(
    config.function,
    config.name,
    config.stackSize,
    config.parameters,
    config.priority,
    &handle,
    config.coreId);

  if (result != pdPASS) {
    xSemaphoreGive(poolMutex_);
    return 0xFF;
  }

  tasks_[taskCount_].handle = handle;
  tasks_[taskCount_].name = config.name;
  tasks_[taskCount_].periodMs = config.periodMs;
  tasks_[taskCount_].lastRun = 0;
  tasks_[taskCount_].enabled = true;
  tasks_[taskCount_].minStackFree = uxTaskGetStackHighWaterMark(handle);

  uint8_t index = taskCount_++;
  xSemaphoreGive(poolMutex_);
  return index;
}

bool TaskPool::removeTask(uint8_t index) {
  if (poolMutex_ == nullptr) return false;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(100)) != pdTRUE) return false;

  if (index >= taskCount_) {
    xSemaphoreGive(poolMutex_);
    return false;
  }

  if (tasks_[index].handle != nullptr) {
    vTaskDelete(tasks_[index].handle);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  for (uint8_t i = index; i < taskCount_ - 1; i++) {
    tasks_[i] = tasks_[i + 1];
  }
  taskCount_--;

  xSemaphoreGive(poolMutex_);
  return true;
}

void TaskPool::enableTask(uint8_t index) {
  if (poolMutex_ == nullptr) return;
  if (index >= taskCount_) return;

  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) tasks_[index].enabled = true;
    xSemaphoreGive(poolMutex_);
  }
}

void TaskPool::disableTask(uint8_t index) {
  if (poolMutex_ == nullptr) return;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) tasks_[index].enabled = false;
    xSemaphoreGive(poolMutex_);
  }
}

bool TaskPool::isTaskEnabled(uint8_t index) {
  if (poolMutex_ == nullptr) return false;
  bool result = false;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) result = tasks_[index].enabled;
    xSemaphoreGive(poolMutex_);
  }
  return result;
}

const char *TaskPool::getTaskName(uint8_t index) {
  if (poolMutex_ == nullptr) return nullptr;
  const char *name = nullptr;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) name = tasks_[index].name;
    xSemaphoreGive(poolMutex_);
  }
  return name;
}

void TaskPool::markRun(uint8_t index) {
  if (poolMutex_ == nullptr) return;
  if (index >= taskCount_) return;

  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) {
      tasks_[index].lastRun = millis();
      if (tasks_[index].handle != nullptr) {
        UBaseType_t currentStack = uxTaskGetStackHighWaterMark(tasks_[index].handle);
        if (currentStack < tasks_[index].minStackFree) {
          tasks_[index].minStackFree = currentStack;
          if (currentStack < 200) {
            Serial.printf("[TASKPOOL] ?? Task '%s' stack critical: %d bytes free\n",
                          tasks_[index].name, currentStack);
          }
        }
      }
    }
    xSemaphoreGive(poolMutex_);
  }
}

uint32_t TaskPool::getTimeUntilNextRun(uint8_t index) {
  if (poolMutex_ == nullptr) return UINT32_MAX;
  uint32_t result = UINT32_MAX;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_ && tasks_[index].enabled) {
      uint32_t elapsed = millis() - tasks_[index].lastRun;
      if (tasks_[index].periodMs > elapsed) {
        result = tasks_[index].periodMs - elapsed;
      } else {
        result = 0;
      }
    }
    xSemaphoreGive(poolMutex_);
  }
  return result;
}

uint8_t TaskPool::getTaskCount() {
  return taskCount_;
}

UBaseType_t TaskPool::getTaskMinStack(uint8_t index) {
  if (poolMutex_ == nullptr) return 0;
  UBaseType_t result = 0;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) result = tasks_[index].minStackFree;
    xSemaphoreGive(poolMutex_);
  }
  return result;
}

TaskHandle_t TaskPool::getHandle(uint8_t index) {
  if (poolMutex_ == nullptr || index >= taskCount_) return nullptr;
  TaskHandle_t h = nullptr;
  if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (index < taskCount_) h = tasks_[index].handle;
    xSemaphoreGive(poolMutex_);
  }
  return h;
}

void TaskPool::suspendTask(uint8_t index) {
  TaskHandle_t h = getHandle(index);
  if (h) vTaskSuspend(h);
}

void TaskPool::resumeTask(uint8_t index) {
  TaskHandle_t h = getHandle(index);
  if (h) vTaskResume(h);
}
