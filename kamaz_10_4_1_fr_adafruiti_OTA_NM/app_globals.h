#pragma once
/**
 *  app_globals.h — extern declarations for shared FreeRTOS objects
 *  and key volatile flags that multiple modules will reference.
 *
 *  Definitions live in app_globals.cpp; setup() in the .ino creates
 *  the mutexes/queues and assigns them to these externs.
 */

/* ───── Compile-time feature flags ───── */
#ifndef ENABLE_MPU6050
#define ENABLE_MPU6050 1
#endif
#ifndef ENABLE_SIMULATION
#define ENABLE_SIMULATION 0
#endif

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "app_types.h"

/* ───── Mutexes ───── */
extern SemaphoreHandle_t xStateMutex;
extern SemaphoreHandle_t xValveMutex;
extern SemaphoreHandle_t xDisplayMutex;
extern SemaphoreHandle_t xConfigMutex;
extern SemaphoreHandle_t xCalibMutex;
extern SemaphoreHandle_t xTestMutex;
extern SemaphoreHandle_t xCommandMutex;
extern SemaphoreHandle_t xI2CMutex;

/* ───── Queues ───── */
extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xPressureQueue;
extern QueueHandle_t xValveQueue;
extern QueueHandle_t xPressureWakeupQueue;

/* ───── System state ───── */
extern SystemState  currentState;
extern SystemMode   currentSystemMode;
extern SystemMode   previousMode;
extern TestState    currentTestState;

/* ───── Volatile flags ───── */
extern volatile bool displayDirty;
extern volatile bool menuVisible;
extern volatile bool mpuOk;
extern volatile bool otaInProgress;
extern volatile bool otaValveLock;
extern volatile bool calibrationCompleted;
extern volatile bool calibrationValid;

/* ───── IMU angles ───── */
extern float angleX;
extern float angleY;
extern float temperature;

/* ───── OTA list state ───── */
extern OtaRelease   otaReleases[OTA_LIST_MAX];
extern volatile uint8_t otaReleaseCount;
extern volatile int8_t  otaSelectedIndex;
extern char          otaListStatus[64];
extern char          otaLatestTag[16];
extern volatile bool otaUiNeedFullRedraw;

/* ───── Wi-Fi flags ───── */
extern volatile bool wifiSetupActive;
extern volatile bool wifiScanInProgress;
extern volatile bool wifiSetupRequested;
extern volatile bool wifiConnected;

/* ───── Movement state ───── */
extern bool isMoving;
extern bool prolongedMovementDetected;
extern uint32_t movementStartTime;
extern bool movementModeActive;
extern uint32_t movementEndTime;
extern uint32_t movementPressureLastCheck;
extern uint32_t movementStartMs;
extern uint32_t movementLastAdjustFront;
extern uint32_t movementLastAdjustRear;
extern int8_t  movementLastAdjustFrontDir;
extern int8_t  movementLastAdjustRearDir;

/* ───── Pressure state (phase 3) ───── */
extern float pressure[PAD_COUNT];
extern float masterPressure;
extern uint32_t pressureStampMs[PAD_COUNT];
extern uint32_t masterStampMs;
extern bool pressureValid[PAD_COUNT];
extern bool masterValid;
extern bool jhmReady;
extern volatile bool firstPressureMeasurementDone;
extern float g_pressureZeroBar;
extern bool pressureLimitReached;

/* ───── Valve / manual state (phase 3) ───── */

struct ValveCommandMsg {
  union {
    struct {
      Pad pad;
      bool inflate;
      uint32_t durationMs;
      QueueHandle_t ackQueue;
    } sync;
    struct {
      Pad pad;
      bool inflate;
      TickType_t duration;
    } async;
  };
};

struct LastCommand {
  uint32_t startTime = 0;
  uint32_t duration = 0;
  Pad pad;
  bool inflate;
  float pressureBefore = 0;
  bool waitingForCompletion = false;
  bool commandActive = false;
};

struct ValveErrorCounter {
  uint8_t consecutiveFailures = 0;
  uint8_t requiredFailures = 3;
  uint32_t lastFailureTime = 0;
  bool valveErrorActive = false;
};

extern const uint8_t bubPins[PAD_COUNT];
extern const char * const padNames[PAD_COUNT];

extern bool manualControlActive;
extern Pad  manualPadIndex;
extern bool manualInflate;
extern uint32_t manualStartTime;
extern float manualTargetPressure[PAD_COUNT];
extern bool  manualTargetSet[PAD_COUNT];

extern Mode currentMode;
extern bool otaMode;

extern LastCommand lastCmd;
extern ValveErrorCounter valveErrorCounter;
extern volatile uint32_t valveQueueDropCount;
extern volatile uint32_t valveEmergencyStopCount;
extern volatile uint32_t maxValveQueueDepth;
extern volatile bool valveStopRequested;
extern uint32_t valveCycleCount[PAD_COUNT];
extern uint32_t valveOpenAccumMs[PAD_COUNT];

extern volatile bool leakSuspect;
extern char leakSuspectPad[8];

/* ───── Leveling / control timing (phase 3) ───── */
extern uint32_t lastLevelingCheckTime;
extern uint32_t lastLevelingAttemptTime;
extern volatile uint32_t levelingAttemptsThisHour;
extern uint32_t lastHourResetTime;
extern uint32_t lastMasterPressureCheckTime;
extern uint32_t lastManualPressureCheckTime;

/* ───── Valve / pressure constants (phase 3) ───── */
constexpr uint32_t VALVE_OPERATION_TIMEOUT_MS = 15000;
constexpr uint32_t VALVE_MAX_COMMAND_MS       = 15000;
constexpr uint32_t MANUAL_TARGET_CHECK_INTERVAL_MS  = 120000;
constexpr uint32_t MANUAL_ADJUSTMENT_COOLDOWN_MS    = 3000;
constexpr float    MANUAL_PRESSURE_TOLERANCE        = 0.1f;
constexpr uint32_t MANUAL_PRESSURE_CHECK_INTERVAL_MS = 120000;

/* ───── Queue drop counter (IMU) ───── */
extern volatile uint32_t imuQueueDropCount;

/* ───── TaskPool indices (0xFF = not registered) ───── */
extern uint8_t taskIndex_Event;
extern uint8_t taskIndex_Button;
extern uint8_t taskIndex_Display;
extern uint8_t taskIndex_IMU;
extern uint8_t taskIndex_Pressure;
extern uint8_t taskIndex_Control;
extern uint8_t taskIndex_Calib;
extern uint8_t taskIndex_Watchdog;
extern uint8_t taskIndex_OTA;
extern uint8_t taskIndex_ErrRec;
extern uint8_t taskIndex_Valve;
