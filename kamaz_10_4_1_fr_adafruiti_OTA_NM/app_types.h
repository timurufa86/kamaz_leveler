#pragma once

#include <Arduino.h>

enum class EventType : uint8_t {
  NONE = 0,
  IMU_UPDATE,
  PRESSURE_UPDATE,
  MODE_CHANGE,
  ERROR_OCCURRED,
  ERROR_CLEARED,
  VALVE_COMMAND,
  LEVELING_START,
  LEVELING_END,
  MOVEMENT_DETECTED,
  MOVEMENT_ENDED,
  OTA_START,
  OTA_PROGRESS,
  OTA_END,
  OTA_ERROR,
  DISPLAY_UPDATE,
  CONFIG_CHANGED,
  CALIBRATION_DONE,
  TEST_STATE_CHANGE,
  MANUAL_OPERATION_START,
  MANUAL_OPERATION_END,
  LOW_MEMORY_WARNING,
  CRITICAL_MEMORY
};

struct Event {
  EventType type;
  uint32_t timestamp;
  union {
    struct {
      float angleX, angleY, temperature;
    } imu;
    struct {
      float pressure[4], masterPressure;
    } pressure;
    struct {
      uint8_t pad;
      bool inflate;
      uint32_t duration;
    } valve;
    struct {
      uint8_t errorCode;
      char message[64];
    } error;
    struct {
      uint8_t fromMode, toMode;
    } modeChange;
    struct {
      uint8_t progress;
      char status[32];
    } ota;
    struct {
      uint8_t testState;
      uint8_t padIndex;
    } test;
    struct {
      uint8_t pad;
      bool inflate;
    } manualOp;
    struct {
      uint32_t freeHeap;
      uint32_t minHeap;
    } memory;
  } data;
};
