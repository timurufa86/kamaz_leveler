#include "app_globals.h"

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
