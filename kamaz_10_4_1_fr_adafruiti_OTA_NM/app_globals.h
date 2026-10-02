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
