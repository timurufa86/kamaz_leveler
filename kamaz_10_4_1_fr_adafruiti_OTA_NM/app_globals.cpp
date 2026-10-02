#include "app_globals.h"
#include "mutex_guard.h"
#include "app_pins.h"

/* ───── Mutexes ───── */
SemaphoreHandle_t xStateMutex   = nullptr;
SemaphoreHandle_t xValveMutex   = nullptr;
SemaphoreHandle_t xDisplayMutex = nullptr;
SemaphoreHandle_t xConfigMutex  = nullptr;
SemaphoreHandle_t xCalibMutex   = nullptr;
SemaphoreHandle_t xTestMutex    = nullptr;
SemaphoreHandle_t xCommandMutex = nullptr;
SemaphoreHandle_t xI2CMutex     = nullptr;

/* ───── Queues ───── */
QueueHandle_t xIMUQueue             = nullptr;
QueueHandle_t xPressureQueue        = nullptr;
QueueHandle_t xValveQueue           = nullptr;
QueueHandle_t xPressureWakeupQueue  = nullptr;

/* ───── System state ───── */
SystemState  currentState      = SystemState::BOOT;
SystemMode   currentSystemMode = SystemMode::MANUAL;
SystemMode   previousMode      = SystemMode::MANUAL;
TestState    currentTestState   = TestState::IDLE;

/* ───── Volatile flags ───── */
volatile bool displayDirty         = true;
volatile bool menuVisible          = false;
volatile bool mpuOk                = false;
volatile bool otaInProgress        = false;
volatile bool otaValveLock         = false;
volatile bool calibrationCompleted = false;
volatile bool calibrationValid     = false;

/* ───── IMU angles ───── */
float angleX       = 0;
float angleY       = 0;
float temperature  = 0;

/* ───── OTA list state ───── */
OtaRelease      otaReleases[OTA_LIST_MAX];
volatile uint8_t otaReleaseCount    = 0;
volatile int8_t  otaSelectedIndex   = -1;
char             otaListStatus[64]  = "нажмите Проверить";
char             otaLatestTag[16]   = "";
volatile bool    otaUiNeedFullRedraw = false;

/* ───── Wi-Fi flags ───── */
volatile bool wifiSetupActive      = false;
volatile bool wifiScanInProgress   = false;
volatile bool wifiSetupRequested   = false;
volatile bool wifiConnected        = false;

/* ───── Movement state ───── */
bool isMoving = false;
bool prolongedMovementDetected = false;
uint32_t movementStartTime = 0;
bool movementModeActive = false;
uint32_t movementEndTime = 0;
uint32_t movementPressureLastCheck = 0;
uint32_t movementStartMs = 0;
uint32_t movementLastAdjustFront = 0;
uint32_t movementLastAdjustRear = 0;
int8_t  movementLastAdjustFrontDir = 0;
int8_t  movementLastAdjustRearDir = 0;

/* ───── Pressure state (phase 3) ───── */
float pressure[PAD_COUNT] = { 0 };
float masterPressure = 0;
uint32_t pressureStampMs[PAD_COUNT] = { 0 };
uint32_t masterStampMs = 0;
bool pressureValid[PAD_COUNT] = { false };
bool masterValid = false;
bool jhmReady = false;
volatile bool firstPressureMeasurementDone = false;
float g_pressureZeroBar = 0.0f;
bool pressureLimitReached = false;

/* ───── Valve / manual state (phase 3) ───── */
const uint8_t bubPins[PAD_COUNT] = { PIN_BUB1, PIN_BUB2, PIN_BUB3, PIN_BUB4 };
const char * const padNames[PAD_COUNT] = { "ПЛ", "ПП", "ЗЛ", "ЗП" };

bool manualControlActive = false;
Pad  manualPadIndex = PAD_FRONT_LEFT;
bool manualInflate = false;
uint32_t manualStartTime = 0;
float manualTargetPressure[PAD_COUNT] = { 3.0f, 3.0f, 3.0f, 3.0f };
bool  manualTargetSet[PAD_COUNT] = { true, true, true, true };

Mode currentMode = Mode::MANUAL;
bool otaMode = false;

LastCommand lastCmd;
ValveErrorCounter valveErrorCounter;
volatile uint32_t valveQueueDropCount = 0;
volatile uint32_t valveEmergencyStopCount = 0;
volatile uint32_t maxValveQueueDepth = 0;
volatile bool valveStopRequested = false;
uint32_t valveCycleCount[PAD_COUNT] = { 0 };
uint32_t valveOpenAccumMs[PAD_COUNT] = { 0 };

volatile bool leakSuspect = false;
char leakSuspectPad[8] = "";

/* ───── Leveling / control timing (phase 3) ───── */
uint32_t lastLevelingCheckTime = 0;
uint32_t lastLevelingAttemptTime = 0;
volatile uint32_t levelingAttemptsThisHour = 0;
uint32_t lastHourResetTime = 0;
uint32_t lastMasterPressureCheckTime = 0;
uint32_t lastManualPressureCheckTime = 0;

/* ───── Queue drop counter (IMU) ───── */
volatile uint32_t imuQueueDropCount = 0;

/* ───── TaskPool indices ───── */
uint8_t taskIndex_Event    = 0xFF;
uint8_t taskIndex_Button   = 0xFF;
uint8_t taskIndex_Display  = 0xFF;
uint8_t taskIndex_IMU      = 0xFF;
uint8_t taskIndex_Pressure = 0xFF;
uint8_t taskIndex_Control  = 0xFF;
uint8_t taskIndex_Calib    = 0xFF;
uint8_t taskIndex_Watchdog = 0xFF;
uint8_t taskIndex_OTA      = 0xFF;
uint8_t taskIndex_ErrRec   = 0xFF;
uint8_t taskIndex_Valve    = 0xFF;

/* ========== БЕЗОПАСНЫЕ ФУНКЦИИ ДЛЯ РАБОТЫ С СОСТОЯНИЕМ ========== */
bool setSystemState(SystemState newState) {
  if (xStateMutex == nullptr) return false;

  if (!takeMutexWithRetry(xStateMutex, pdMS_TO_TICKS(100), 3, "setSystemState")) {
    Serial.println("[STATE] Failed to take state mutex");
    return false;
  }

  SystemState oldState = currentState;
  currentState = newState;
  xSemaphoreGive(xStateMutex);

  Serial.printf("[STATE] Changed from %d to %d\n", (int)oldState, (int)newState);
  displayDirty = true;
  return true;
}

SystemState getSystemState() {
  if (xStateMutex == nullptr) return SystemState::ERROR;

  MutexGuard guard(xStateMutex);
  if (!guard) return SystemState::ERROR;

  return currentState;
}

