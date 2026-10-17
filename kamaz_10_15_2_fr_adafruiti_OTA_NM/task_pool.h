#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

struct TaskConfig {
  const char *name;
  TaskFunction_t function;
  uint32_t stackSize;
  UBaseType_t priority;
  BaseType_t coreId;
  uint32_t periodMs;
  void *parameters;
};

class TaskPool {
private:
  struct TaskHandle {
    TaskHandle_t handle;
    const char *name;
    uint32_t periodMs;
    uint32_t lastRun;
    bool enabled;
    UBaseType_t minStackFree;
  };

  static TaskHandle tasks_[16];
  static uint8_t taskCount_;
  static SemaphoreHandle_t poolMutex_;

public:
  static bool init();
  static uint8_t addTask(const TaskConfig &config);
  static bool removeTask(uint8_t index);
  static void enableTask(uint8_t index);
  static void disableTask(uint8_t index);
  static bool isTaskEnabled(uint8_t index);
  static const char *getTaskName(uint8_t index);
  static void markRun(uint8_t index);
  static uint32_t getTimeUntilNextRun(uint8_t index);
  static uint8_t getTaskCount();
  static UBaseType_t getTaskMinStack(uint8_t index);
  static TaskHandle_t getHandle(uint8_t index);
  static void suspendTask(uint8_t index);
  static void resumeTask(uint8_t index);
};
