#pragma once

#include <Arduino.h>

/* ───── PAD_COUNT ───── */
#define PAD_COUNT 4

/* ───── IMU / Pressure queue payloads ───── */

struct IMUData {
  float angleX;
  float angleY;
  float temperature;
};

struct PressureData {
  float pressure[PAD_COUNT];
  float masterPressure;
};

/* ───── Event types / Event struct ───── */

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

/* ───── Pad ───── */

enum Pad : uint8_t {
  PAD_FRONT_LEFT,
  PAD_FRONT_RIGHT,
  PAD_REAR_LEFT,
  PAD_REAR_RIGHT,
  PAD_COUNT_ENUM
};

/* ───── System / mode / test enums ───── */

enum class SystemState {
  BOOT,
  CALIBRATING,
  RUNNING,
  ERROR,
  OTA_MODE
};

enum class SystemMode : uint8_t {
  MANUAL,
  AUTO,
  MOVEMENT
};

enum class TestState {
  IDLE,
  STARTING,
  TESTING_PAD,
  WAITING_BETWEEN_PHASES,
  COMPLETED
};

enum class Mode : uint8_t {
  MANUAL,
  AUTO
};

enum class TestStep : uint8_t {
  IDLE = 0,
  PREPARE_CHECK_SUPPLY,
  PREPARE_WAIT_PRESSURIZE,
  PREPARE_EQUALIZE_PADS,
  TEST_DEFLATE_VALVE,
  TEST_INFLATE_VALVE,
  TEST_PAD_VALVE_RESET,
  TEST_PAD_VALVE_OPEN,
  TEST_PAD_VALVE_CLOSE,
  COMPLETED
};

/* ───── OTA release list ───── */

constexpr uint8_t OTA_LIST_MAX  = 3;
constexpr uint8_t OTA_FETCH_MAX = 6;

struct OtaRelease {
  char tag[16]     = "";
  char date[11]    = "";      // YYYY-MM-DD
  uint32_t size    = 0;
  char binUrl[176] = "";
  char shaUrl[176] = "";
  char sha256[65]  = "";      // заполняется проверкой sha256-ассета
};
