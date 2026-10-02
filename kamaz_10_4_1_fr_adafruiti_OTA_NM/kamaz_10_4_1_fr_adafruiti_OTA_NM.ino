/****************************************************************************************
 *  Kamaz-Leveler - FreeRTOS + OTA (версия 8.7.0)
 *  Оптимизации: RAII мьютексы, EventBus, TaskPool, улучшенная обработка ошибок
 *  Типографика: U8g2_for_Adafruit_GFX (модули ui_theme.h / ui_fonts.h / ui_text.*)
 *  Меню: КН3+КН4; режим АВТО/РУЧ: КН1+КН2; авария: КН1+КН4. КН5 отключена.
 *****************************************************************************************/

#include <Arduino.h>
#include <new>
#include "mutex_guard.h"
#include "app_version.h"
#include "app_pins.h"
#include "app_types.h"
#include "app_globals.h"
#include "event_bus.h"
#include "task_pool.h"
#include "logger.h"
#include "semver_utils.h"
#include "github_ota_request.h"
#include "ota_net.h"
#include "ota_list.h"
#include "ota_install.h"
#include "ui_marquee.h"
#include "ui_ota_menu.h"
#include "wifi_setup.h"
#include "task_ota.h"
#include "imu_dmp.h"
#include "imu_motion.h"
#include "task_imu.h"
#include "pressure_read.h"
#include "valve_ctrl.h"
#include "auto_level.h"
#include "task_pressure.h"
#include "task_valve.h"
#include "task_control.h"
#include "task_calib.h"
#include "ui_screens.h"
#include "ui_menu_build.h"
#include "task_button.h"
#include "task_display.h"
#include "icon.h"
#include "ui_theme.h"    // палитра и сетка UI
#include "ui_fonts.h"    // шрифтовая сетка U8g2
#include "ui_text.h"     // типографский слой (единственный экземпляр: ui)
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define KAMAZ_OTA_CERT_BUNDLE 0
/** TLS для GitHub OTA. CA-bundle отключён: на фрагментированной куче (maxBlk~34K)
 *  mbedtls падает с «SSL - Memory allocation failed»; целостность bin — SHA-256. */
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"  // MotionApps v6.12 (FIFO 28B)
#include <GyverFilters.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "jhm1200.h"
#include <U8g2_for_Adafruit_GFX.h>   // типографика (также включается из ui_fonts.h/ui_text.h)
#include <SPI.h>
#include <EncButton.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "FontsRus/CourierCyr7.h"   // GFX-шрифты оставлены только для GEM-меню
#include "FontsRus/CourierCyr9.h"

#include <esp_heap_caps.h>
#include <cstring>
#include <cstdlib>
#include <mbedtls/sha256.h>
#include <GEM_adafruit_gfx.h>

// Forward declaration
class ErrorHandler;

/* ====================  РЕЖИМЫ СИМУЛЯЦИИ ==================== */
// ENABLE_MPU6050, ENABLE_SIMULATION — see app_globals.h
#define SIMULATE_AUTO_MODE 0
#define SIMULATE_ERRORS 0
#define SIMULATE_MENU_AUTO_ENTER 0
#define DEBUG_PRINT 0
#define DEBUG_STACK 0

#define EB_NO_CALLBACK
#define EB_NO_COUNTER
#define EB_NO_BUFFER

#define EB_DEB_TIME 50
#define EB_CLICK_TIME 500
#define EB_HOLD_TIME 500
#define EB_STEP_TIME 120

// PAD_COUNT — see app_types.h

// AngleBarState — see ui_screens.cpp

// jhmReady — see app_globals.cpp

/* ====================  КОНСТАНТЫ ==================== */
// SCREEN_WIDTH/HEIGHT — see ui_screens.h

// DISPLAY_UPDATE_INTERVAL_MS — see task_display.h

// menu layout constants — see ui_menu_build.cpp

/* MarqueeId + MarqueeSlot -> ui_marquee.h */

/* ====================  КОМБИНАЦИИ КНОПОК ====================
 *  Меню: удержание КН3+КН4. Режим АВТО/РУЧ: КН1+КН2.
 *  Авария: удержание КН1+КН4 (обрабатывается первой, подавляет меню).
 *  КН5 отключена.
 */
// MENU_COMBO_HOLD_MS / MODE_TOGGLE_HOLD_MS — see task_button.h
constexpr uint32_t IMU_POLL_INTERVAL_MS = 25;      // дефолт периода DMP (см. imuPollMs в меню)
// EMERGENCY_HOLD_MS — see task_button.h
// КН5 (GPIO34) отключена: плавающий вход давал ложные события и перезагрузки.
// MENU_* colors — see ui_menu_build.cpp

/* ====================  НАСТРОЙКИ ТЕСТА ==================== */
constexpr uint32_t TEST_VALVE_OPEN_TIME_MS = 1500;
constexpr uint32_t TEST_STABILIZE_TIME_MS = 500;
constexpr uint32_t TEST_PRESSURE_EQUALIZE_TIME_MS = 3000;
constexpr uint32_t TEST_TIMEOUT_MS = 90000;
constexpr float TEST_MIN_PRESS_FOR_TEST = 0.8f;
constexpr float TEST_ZERO_THRESHOLD = 0.1f;
constexpr float TEST_PRESSURE_CHANGE_THRESHOLD = 0.2f;



/* ====================  ПЕРЕМЕННЫЕ ==================== */
uint32_t uptimeHours = 0;

/* ====================  ПАРАМЕТРЫ ==================== */
// VERSION now in app_version.h



// MvData — see ui_screens.cpp

/* Пины — see app_pins.h */
// CONTRAST_* — see ui_menu_build.h


// ========== JHM1200 (KY-3V3-IIC) 0..10 бар ==========

/* Параметры времени */
constexpr uint32_t CALIB_TIME_MS = 15000;
constexpr uint32_t MANUAL_MAX_TIME = 10000;
constexpr uint32_t MASTER_PRESSURE_CHECK_INTERVAL_MS = 240000;
// MANUAL_PRESSURE_CHECK_INTERVAL_MS — see app_globals.h
constexpr uint32_t LEVELING_ATTEMPT_COOLDOWN_MS = 3600000;
constexpr uint32_t PRESSURE_LIMIT_WARNING_DURATION_MS = 5000;
constexpr uint32_t MOVEMENT_PRESSURE_CHECK_INTERVAL_MS = 120000;
constexpr uint32_t MOVEMENT_DURATION_MS = 30000;
constexpr uint32_t MOVEMENT_SETTLE_TIME_MS = 30000;
constexpr float MOVEMENT_PRESSURE_TOLERANCE = 0.2f;
constexpr int GYRO_MOTION_THRESHOLD = 50;
constexpr int ACCEL_MOTION_THRESHOLD = 1000;

/* Переменные для меню */
int editReleaseDelay;
int editInflateDelay;
float editTiltX;
float editTiltY;
float editPressureMin;
float editPressureMax;
int editMasterLowTenths;      // Низк.МП: 0…40 (= 0.0…4.0 бар), 0 = выкл.
int editNivCount;
int editTimeInterval;
int editContrast;
float editMovementPressureFront;
float editMovementPressureRear;
float editParkingPressure;    // давление стоянки после ДВИЖЕНИЕ→РУЧ (0 = из режима движения)
bool settingsChanged = false;

/* ===== 8.8.0: зеркала новых параметров меню ===== */
int   editMasterCheck;        // период проверки магистрали, с
int   editManualMaxTime;      // максимальное время ручной операции, с
int   editPressStabilizeMs;   // выравнивание МП после клапана, мс
int   editPressIdleMin;       // пауза между полными опросами подушек, мин
float editDeadband;           // зона нечувствительности по давлению, бар
float editCoarseZone;         // грубая зона авторежима (доля порога)
float editFineZone;           // точная зона авторежима (доля порога)
float editWorsening;          // порог «стало хуже» (множитель)
int   editMoveDuration;       // длительность ожидания движения, с
int   editMoveSettle;         // время успокоения после движения, с
int   editMoveCheck;          // период проверки давления после движения, с
float editMoveTolerance;      // допуск давления после движения, бар
int   editBacklightOff;       // гашение подсветки, мин (0 = никогда)
int   editFrameMs;            // интервал кадра дисплея, мс
float editRedrawAngle;        // порог перерисовки по углу, °
float editRedrawPressure;     // порог перерисовки по давлению, бар
int   editImuMotionDet;       // аппаратный порог детектора движения MPU (MOT)
int   editGyroThreshold;      // порог |gyro−EMA| (меню ×8 → порог)
int   editGyroBumpThreshold;  // порог |bump−EMA| — неровности дороги
int   editAccelThreshold;     // порог |linAcc − EMA|
float editZeroAngleX;         // программный нуль углов X, °
float editZeroAngleY;         // программный нуль углов Y, °
float editImuKalmanMea;       // GKalman mea_e (шум измерения)
float editImuKalmanEst;       // GKalman est_e (шум оценки)
float editImuKalmanQ;         // GKalman q (шум процесса)
int   editImuPollMs;          // период опроса DMP, мс
int   editImuFifoAvg;         // сколько FIFO-пакетов усреднять (1..8)
float editImuEmaAlpha;        // EMA после Калмана (^ = быстрее)
float editImuEmaSpikeAlpha;   // EMA при выбросе (v = глуше всплеск)
float editImuEmaSpikeThr;     // порог выброса для spike-EMA, °
float editImuSlewDps;         // макс. скорость изменения угла, °/с
int   editImuPreset;          // 0=Плавно, 1=Быстро, 2=Баланс

/* 8.8.0: обработчик изменения пункта меню со спиннером объявлен НИЖЕ по файлу
   (перед initGEM) — в .ino первое определение функции не должно предшествовать
   объявлению типов/enum'ов и глобальных переменных, иначе автогенерация прототипов
   Arduino вставляет прототипы в начало файла и компиляция падает. */


// GEM spinners — see ui_menu_build.cpp

/* Переменные состояния */
// Pad, SystemState, SystemMode, TestState, Mode, TestStep — see app_types.h

extern const uint16_t COLOR_BG = ST77XX_BLACK;
extern const uint16_t COLOR_TEXT = ST77XX_WHITE;
extern const uint16_t COLOR_HIGHLIGHT = ST77XX_BLUE;
extern const uint16_t COLOR_ERROR = ST77XX_RED;
extern const uint16_t COLOR_SUCCESS = ST77XX_GREEN;
extern const uint16_t COLOR_WARNING = ST77XX_YELLOW;
extern const uint16_t COLOR_OTA = ST77XX_CYAN;
extern const uint16_t COLOR_WHITE = ST77XX_WHITE;

/* Глобальные переменные */
bool errorScreenBlocking = false;

// calibrationCompleted — see app_globals.cpp
// firstPressureMeasurementDone — see app_globals.cpp

bool forceErrorScreenRedraw = false;


#define COLOR_SKY ST77XX_CYAN
#define COLOR_GROUND ST77XX_ORANGE
#define COLOR_HORIZON ST77XX_WHITE
#define COLOR_ROLL ST77XX_WHITE

char wifi_ssid[32] = "Kamaz-OTA-AP";
char wifi_password[64] = "";
char sta_ssid[33] = "";
constexpr char WIFI_STA_PASSWORD[] = "asd12345";
constexpr uint8_t WIFI_SCAN_MAX_NETWORKS = 8;
char wifi_scan_ssids[WIFI_SCAN_MAX_NETWORKS][33] = {};
int8_t wifi_scan_rssi[WIFI_SCAN_MAX_NETWORKS] = {};
uint8_t wifi_scan_count = 0;
uint8_t wifi_scan_selected = 0;
// wifiSetupActive, wifiScanInProgress, wifiSetupRequested, wifiConnected — see app_globals.cpp
volatile bool wifiUiFullRedraw = false;  // только явный полный кадр экрана Wi-Fi
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

constexpr uint16_t OTA_PORT = 8266;
constexpr char OTA_HOSTNAME[] = "Kamaz-leveling-system";
char ota_password[64] = "";
constexpr char GITHUB_API_URL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases/latest";
constexpr char GITHUB_ASSET_NAME[] = "kamaz_leveler.bin";
constexpr char GITHUB_SHA256_ASSET_NAME[] = "kamaz_leveler.bin.sha256";
// VALVE_OPERATION_TIMEOUT_MS, VALVE_MAX_COMMAND_MS — see app_globals.h
// 9.2.0: формат 5 — EMA/slew/FIFO + тюнинг углов в меню «IMU».
// Файлы версий 1–4 читаются, новые поля = значения по умолчанию.
constexpr uint32_t CONFIG_FORMAT_VERSION = 8;

constexpr uint32_t WDT_TIMEOUT_MS = 30000;
constexpr uint32_t TASK_WDT_TIMEOUT_MS = 10000;
constexpr uint8_t TASK_COUNT = 11;  // Event..Valve + ErrorRecovery (индекс 0..10)

// MANUAL_TARGET_CHECK_INTERVAL_MS, MANUAL_ADJUSTMENT_COOLDOWN_MS,
// MANUAL_PRESSURE_TOLERANCE — see app_globals.h

/* ========== МЬЮТЕКС / STATE / displayDirty — see app_globals.cpp ========== */




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

/* otaConfigureTls -> ota_net.h/cpp */

// taskIndex_* — see app_globals.cpp

/* ====================  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==================== */
SPIClass spi(VSPI);
Adafruit_ST7789 tft(&spi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GEM_adafruit_gfx gem(tft);


GEMPage mainPage("Главное меню", menuExitAction);
// Подстраницы создаются сразу с родителем: GEM сам добавляет первым пунктом
// «Назад» (стрелка влево) и корректно обрабатывает возврат по Кн4.
GEMPage systemPage("Система", mainPage);
GEMPage testPage("Тестирование", mainPage);
GEMPage valvePage("Клапаны", mainPage);
GEMPage pressurePage("Давление", mainPage);
GEMPage autoPage("Авторежим", mainPage);
GEMPage displayPage("Дисплей", mainPage);
GEMPage movementPage("Движение", mainPage);
GEMPage infoPage("Информация", mainPage);
GEMPage imuPage("IMU", mainPage);
GEMPage settingsViewPage("Просмотр", mainPage);

// info/settings GEM items — see ui_menu_build.cpp


// mpu, filterX, filterY — see imu_dmp.cpp

Button button0(PIN_BUT1, INPUT_PULLUP, LOW);
Button button1(PIN_BUT2, INPUT_PULLUP, LOW);
Button button2(PIN_BUT3, INPUT_PULLUP, LOW);
Button button3(PIN_BUT4, INPUT_PULLUP, LOW);
// КН5 отключена (GPIO34 плавает) — не тикаем, функции на КН1+КН2 / КН1+КН4.
Button button4(PIN_BUT5, INPUT, LOW);

VirtButton emergencyButton;  // КН1+КН4 авария

// xValveMutex … xPressureWakeupQueue — see app_globals.cpp

// ValveCommandMsg, LastCommand, ValveErrorCounter — see app_globals.h
// lastCmd, valveErrorCounter, valveQueueDropCount, etc. — see app_globals.cpp

// IMUData, PressureData — see app_types.h

// angleX, angleY, temperature — see app_globals.cpp
// pressure[], masterPressure, pressureStampMs[], masterStampMs,
// pressureValid[], masterValid — see app_globals.cpp
// mpuOk, menuVisible — see app_globals.cpp
//bool menuRendered = false;         // < ДОБАВИТЬ!
uint32_t lastMenuInteraction = 0;  // < ДОБАВИТЬ!

/** Инкремент при forceDisplayReset — сбрасывает static-кэши главного экрана
 *  и авиагоризонта (иначе после выхода из меню: чёрный экран + нет гироскопа). */
volatile uint16_t g_displayEpoch = 0;

/** Запрос open/close меню с кнопки/Serial — обрабатывается в DisplayTask
 *  (под xDisplayMutex). Иначе ButtonTask делает tft.fillScreen без мьютекса
 *  параллельно с отрисовкой > подвисание SPI. */
volatile MenuReq g_menuReq = MenuReq::None;
static volatile bool g_testHeartbeat = false;
static volatile bool g_testDebugDump = false;

// g_imu* stream/stats — see task_imu.h / task_imu.cpp
// g_mot* motion state  — see imu_motion.h / imu_motion.cpp
// g_imuStat* STATS     — see task_imu.h / task_imu.cpp

// isMoving, prolongedMovementDetected, movementStartTime — see app_globals.cpp

// manualControlActive, manualPadIndex, manualInflate, manualStartTime — see app_globals.cpp

// currentMode — see app_globals.cpp

// otaMode — see app_globals.cpp
// otaInProgress, otaValveLock — see app_globals.cpp
int otaProgress = 0;
char otaStatus[32] = "";

// bubPins[], padNames[] — see app_globals.cpp

// lastLevelingCheckTime, lastLevelingAttemptTime, levelingAttemptsThisHour, lastHourResetTime — see app_globals.cpp

// lastMasterPressureCheckTime, lastManualPressureCheckTime,
// manualTargetPressure[], manualTargetSet[] — see app_globals.cpp
// pressureLimitReached — see app_globals.cpp

// movementEndTime, movementModeActive, movementPressureLastCheck — see app_globals.cpp

/* ===== 8.9.0: данные для «живого» экрана ДВИЖЕНИЯ ===== */
// movementLastAdjust*, movementStartMs — see app_globals.cpp
bool mvScreenWasActive = false;           // активен ли сейчас экран ДВИЖЕНИЯ

// calibrationValid — see app_globals.cpp
// g_pressureZeroBar — see app_globals.cpp


IMUData lastDisplayedIMU = { 0 };
PressureData lastDisplayedPressure = { 0 };
SystemMode lastDisplayedMode = SystemMode::MANUAL;
bool lastDisplayedMoving = false;

// LastCommand, ValveErrorCounter structs — see app_globals.h
// lastCmd, valveErrorCounter, valveQueueDropCount, etc. instances — see app_globals.cpp
static volatile uint32_t eventQueueDropCount = 0;
// imuQueueDropCount — see app_globals.cpp
// pressureQueueDropCount — see task_pressure.cpp
static volatile uint32_t maxEventQueueDepth = 0;
static volatile uint32_t maxStackLowEvents = 0;

static TestStep currentTestStep = TestStep::IDLE;
static uint32_t testStepStartTime = 0;
static uint8_t testPadIndex = 0;
static float testReferencePressure = 0;
static uint32_t testStartTime = 0;
static bool waitingForUser = false;

struct TestResult {
  bool supplyPressureOk;
  float supplyPressure;
  float equalizedPressure;
  bool inflateValveWorks;
  bool deflateValveWorks;
  bool padValvesWorks[PAD_COUNT];
  float pressureReadings[PAD_COUNT][3];
};

static TestResult testResult;

/* ====================  ПРОТОТИПЫ ==================== */
// setDisplayDirty, emergencyStop, startManualOperation, stopManualOperation,
// closeAllValves, setValve, sendValveCommand — see valve_ctrl.h
// readPressure — see pressure_read.h
void initWatchdog();
void initOTA();
void startOTAMode();
void stopOTAMode();
void handleOTA();
void displayOTAScreen();
// displayWiFiSetupScreen -> wifi_setup.h
//void buildMenu(gm::Builder &b);
bool initFileSystem();
bool loadConfig();
bool saveConfig();
bool loadWiFiConfig();
bool saveWiFiConfig();
// startWiFiSetup -> wifi_setup.h
// connectConfiguredWiFi -> wifi_setup.h
// startFallbackAccessPoint -> wifi_setup.h
// initializeDMP — see imu_dmp.h
// initializeJhm1200 — see pressure_read.h
void resetSystemErrors();
// checkPressureLimits — see pressure_read.h
void saveMenuSettings();
void applyRuntimeSettings();  // 8.8.0: применение настроек к железу без перезагрузки
void requestMenuOpen(const char *via);
void requestMenuClose(const char *via);
static void processSerialTestCommands();
void processTestCommandLine(char *line);
// sendValveCommandSync — see valve_ctrl.h
// maintainMovementPressure — see pressure_read.h
// detectMotionFromIMU — see imu_motion.h
// checkAndAdjustMasterPressure, maintainManualPressure — see pressure_read.h
// setManualTargetPressure, setAllManualTargetsFromCurrent — see pressure_read.h
void setManualTargetsFromParkingPolicy();
void startValveTest();
void runValveTestLogic();
void updateTestDisplay();
// requestPressureMeasurement — see pressure_read.h
void forceDisplayReset(bool force = false);
// initializeDefaultCredentials -> wifi_setup.h
// checkGitHubUpdate -> ota_install.h
// downloadGitHubFirmware -> ota_install.h

// imuTask — see task_imu.h
// pressureTask — see task_pressure.h
// controlTask — see task_control.h
// calibrationTask — see task_calib.h
void watchdogTask(void *pvParameters);
// otaTask -> task_ota.h
void errorRecoveryTask(void *pvParameters);
// valveTask — see task_valve.h
void eventHandlerTask(void *pvParameters);

void drawIconB(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);
void drawIcon(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);
void drawIconL(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);

/* ====================  ConfigManager ==================== */
/* ====================  РАЗДЕЛ «ОБНОВЛЕНИЯ» (GitHub)  ====================
 *  Список последних релизов, карточка релиза и установка выбранной версии.
 *  Сетевые операции выполняет otaTask (см. github_ota_request.h), здесь —
 *  хранение списка, страницы GEM и обработчики пунктов меню.
 */
// OTA_LIST_MAX, OTA_FETCH_MAX, OtaRelease — see app_types.h
// Список: по одному релизу за запрос (полный JSON ~8–12 КБ) — надёжнее на ESP32
constexpr char GITHUB_RELEASES_URL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=1";
constexpr char GITHUB_RELEASES_URL_SMALL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=1&page=1";

// otaReleases, otaReleaseCount, otaSelectedIndex, otaListStatus,
// otaLatestTag, otaUiNeedFullRedraw — see app_globals.cpp



/* OTA net/list/install/menu -> modules */

class ConfigManager {
private:
  struct Config {
    uint32_t formatVersion = CONFIG_FORMAT_VERSION;
    // ===== прежние параметры (8.6.x / 8.7.x) =====
    float pressureMin = 1.0f;
    float pressureMax = 7.0f;
    float masterLowBar = 0.5f;       // LOW_PRESSURE если МП < порога; 0.00 = выкл., бар (0…4)
    float tiltThresholdX = 0.5f;
    float tiltThresholdY = 0.5f;
    int contrast = 50;
    int nivCount = 5;
    int timeInterval = 5;
    int inflateDelay = 4;            // импульс накачки, с (AUTO и ручной)
    int releaseDelay = 7;            // импульс сброса, с (точная зона AUTO = половина)
    float movementPressureFront = 3.5f;
    float movementPressureRear = 4.0f;
    float parkingPressureBar = 1.5f; // после ДВИЖЕНИЕ→РУЧ; 0.00 = взять перед/зад движения

    // ===== 8.8.0: магистраль / клапаны =====
    int masterCheckSec = 240;        // период проверки магистрали, с
    int manualMaxTimeSec = 10;       // максимальное время ручной операции, с
    float pressureDeadband = 0.2f;   // зона нечувствительности по давлению, бар
    int pressureStabilizeMs = 500;   // выравнивание давления после клапана, мс
    int pressureIdleMin = 2;         // пауза между полными опросами подушек в простое, мин (2…30)

    // ===== 8.8.0: авторежим =====
    float coarseZoneRatio = 0.4f;    // грубая зона = доля порога
    float fineZoneRatio = 0.15f;     // точная зона = доля порога
    float worseningRatio = 1.2f;     // порог «стало хуже» (множитель)

    // ===== 8.8.0: движение =====
    int movementDurationSec = 30;    // длительность ожидания движения, с
    int movementSettleSec = 10;      // успокоение; duration > settle → нужен повторный шум
    int movementCheckSec = 120;      // период проверки давления после движения, с
    float movementTolerance = 0.2f;  // допуск давления после движения, бар
    int gyroThreshold = 25;          // порог гироскопа RMS (меню×8) — стол ~p95×margin
    int gyroBumpThreshold = 25;      // порог max(|gx|,|gy|) — неровности (меню×8)
    int accelThreshold = 500;        // порог |linAcc − EMA| (не абсолютный RMS)

    // ===== 8.8.0: дисплей =====
    int backlightOffMin = 0;         // гашение подсветки, мин (0 = никогда)
    int frameMs = 50;                // интервал кадра дисплея, мс
    float redrawAngleThr = 0.06f;    // порог перерисовки по углу, °
    float redrawPressureThr = 0.03f; // порог перерисовки по давлению, бар

    // ===== 8.8.0: IMU / MPU6050 =====
    int imuMotionDet = 40;           // аппаратный порог MOT (LSB=2mg; меньше = чувствительнее)
    // Заводские офсеты из примеров i2cdevlib / MotionApps (не нули).
    int imuGyroOffX = 220;
    int imuGyroOffY = 76;
    int imuGyroOffZ = -85;
    int imuAccelOffX = 0;
    int imuAccelOffY = 0;
    int imuAccelOffZ = 1788;
    float zeroAngleX = 0.0f;         // программный нуль углов (поперечный), °
    float zeroAngleY = 0.0f;         // программный нуль углов (продольный), °
    // ===== 10.1.28: баланс ослаблен (быстрее реакция, меньше залипание) =====
    float imuKalmanMea = 5.5f;       // mea_e — шум измерения (^ = плавнее)
    float imuKalmanEst = 3.5f;       // est_e — шум оценки
    float imuKalmanQ = 0.008f;       // q — шум процесса (v = плавнее)
    int imuPollMs = 20;              // период чтения DMP, мс (v = быстрее)
    int imuFifoAvg = 2;              // пакетов FIFO усреднять (^ = тише, v = быстрее)
    float imuEmaAlpha = 0.28f;       // EMA после Калмана
    float imuEmaSpikeAlpha = 0.10f;  // EMA при выбросе
    float imuEmaSpikeThr = 1.8f;     // порог выброса, °
    float imuSlewDps = 70.0f;        // макс. °/с (шум режет EMA; catch-up догоняет рывки)
  };

  static Config currentConfig;

public:
  static bool load() {
    Serial.println("[CFG] Загрузка конфигурации");

    if (!LittleFS.totalBytes()) {
      Serial.println("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      Serial.println("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    File file = LittleFS.open("/config.txt", "r");
    if (!file) {
      Serial.println("[CFG] Нет /config.txt — пробую /config.bak");
      file = LittleFS.open("/config.bak", "r");
      if (!file) {
        Serial.println("[CFG] Нет конфигурации — значения по умолчанию");
        currentConfig = Config{};
        currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
        dumpToSerial();
        return true;
      }
      Serial.println("[CFG] Загрузка из /config.bak");
    }

    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      Serial.println("[CFG] Ошибка десериализации JSON — пробую /config.bak");
      File bak = LittleFS.open("/config.bak", "r");
      if (!bak) {
        Serial.println("[CFG] Нет /config.bak");
        return false;
      }
      error = deserializeJson(doc, bak);
      bak.close();
      if (error) {
        Serial.println("[CFG] /config.bak тоже повреждён");
        return false;
      }
      Serial.println("[CFG] Восстановлено из /config.bak");
    }

    uint32_t formatVersion = doc["formatVersion"] | 1;
    if (formatVersion > CONFIG_FORMAT_VERSION) {
      Serial.printf("[CFG] Неподдерживаемая версия конфигурации: %u\n", formatVersion);
      return false;
    }

    // 8.8.0: стартуем с дефолтов — поля, которых нет в старом файле (форматы 1 и 2),
    // останутся значениями по умолчанию из структуры Config
    currentConfig = Config{};
    currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
    float val = doc["pressureMin"] | 1.0f;
    currentConfig.pressureMin = constrain(val, 0.1f, 5.0f);

    val = doc["pressureMax"] | 7.0f;
    currentConfig.pressureMax = constrain(val, 1.0f, 8.0f);

    val = doc["masterLowBar"] | 0.5f;
    currentConfig.masterLowBar = constrain(val, 0.0f, 4.0f);

    val = doc["tiltThresholdX"] | 0.5f;
    currentConfig.tiltThresholdX = constrain(val, 0.0f, 5.0f);

    val = doc["tiltThresholdY"] | 0.5f;
    currentConfig.tiltThresholdY = constrain(val, 0.0f, 5.0f);

    currentConfig.contrast = constrain(doc["contrast"] | 50, CONTRAST_MIN, CONTRAST_MAX);
    currentConfig.nivCount = constrain(doc["nivCount"] | 5, 1, 20);
    currentConfig.timeInterval = constrain(doc["timeInterval"] | 5, 1, 60);
    currentConfig.inflateDelay = constrain(doc["inflateDelay"] | 4, 1, 10);
    currentConfig.releaseDelay = constrain(doc["releaseDelay"] | 7, 1, 10);

    val = doc["movementPressureFront"] | 3.5f;
    currentConfig.movementPressureFront = constrain(val, 1.0f, 6.0f);

    val = doc["movementPressureRear"] | 4.0f;
    currentConfig.movementPressureRear = constrain(val, 1.0f, 6.0f);

    val = doc["parkingPressureBar"] | 1.5f;
    currentConfig.parkingPressureBar = constrain(val, 0.0f, 6.0f);

    // ===== 8.8.0: магистраль / клапаны =====
    currentConfig.masterCheckSec = constrain((int)(doc["masterCheckSec"] | 240), 30, 600);
    currentConfig.manualMaxTimeSec = constrain((int)(doc["manualMaxTimeSec"] | 10), 1, 30);
    val = doc["pressureDeadband"] | 0.2f;
    currentConfig.pressureDeadband = constrain(val, 0.05f, 0.5f);
    currentConfig.pressureStabilizeMs = constrain((int)(doc["pressureStabilizeMs"] | 500), 100, 2000);
    // v7: pressureIdleMin (мин). Старое pressureCycleMs было в мс (500…10000) — берём дефолт 2 мин.
    if (doc.containsKey("pressureIdleMin")) {
      currentConfig.pressureIdleMin = constrain((int)(doc["pressureIdleMin"] | 2), 2, 30);
    } else {
      const int legacyCycleMs = (int)(doc["pressureCycleMs"] | 2000);
      currentConfig.pressureIdleMin = (legacyCycleMs > 30) ? 2 : constrain(legacyCycleMs, 2, 30);
    }

    // ===== 8.8.0: авторежим =====
    val = doc["coarseZoneRatio"] | 0.4f;
    currentConfig.coarseZoneRatio = constrain(val, 0.2f, 0.9f);
    val = doc["fineZoneRatio"] | 0.15f;
    currentConfig.fineZoneRatio = constrain(val, 0.05f, 0.3f);
    val = doc["worseningRatio"] | 1.2f;
    currentConfig.worseningRatio = constrain(val, 1.05f, 2.0f);

    // ===== 8.8.0: движение =====
    currentConfig.movementDurationSec = constrain((int)(doc["movementDurationSec"] | 30), 10, 120);
    currentConfig.movementSettleSec = constrain((int)(doc["movementSettleSec"] | 10), 10, 120);
    currentConfig.movementCheckSec = constrain((int)(doc["movementCheckSec"] | 120), 30, 300);
    val = doc["movementTolerance"] | 0.2f;
    currentConfig.movementTolerance = constrain(val, 0.1f, 1.0f);
    currentConfig.gyroThreshold = constrain((int)(doc["gyroThreshold"] | 25), 10, 200);
    currentConfig.gyroBumpThreshold = constrain((int)(doc["gyroBumpThreshold"] | 25), 10, 200);
    currentConfig.accelThreshold = constrain((int)(doc["accelThreshold"] | 500), 100, 3000);

    // ===== 8.8.0: дисплей =====
    currentConfig.backlightOffMin = constrain((int)(doc["backlightOffMin"] | 0), 0, 30);
    currentConfig.frameMs = constrain((int)(doc["frameMs"] | 50), 20, 200);
    val = doc["redrawAngleThr"] | 0.05f;
    currentConfig.redrawAngleThr = constrain(val, 0.01f, 0.5f);
    val = doc["redrawPressureThr"] | 0.03f;
    currentConfig.redrawPressureThr = constrain(val, 0.01f, 0.5f);

    // ===== 8.8.0: IMU / MPU6050 =====
    currentConfig.imuMotionDet = constrain((int)(doc["imuMotionDet"] | 40), 20, 255);
    currentConfig.imuGyroOffX = constrain((int)(doc["imuGyroOffX"] | 220), -32768, 32767);
    currentConfig.imuGyroOffY = constrain((int)(doc["imuGyroOffY"] | 76), -32768, 32767);
    currentConfig.imuGyroOffZ = constrain((int)(doc["imuGyroOffZ"] | -85), -32768, 32767);
    currentConfig.imuAccelOffX = constrain((int)(doc["imuAccelOffX"] | 0), -32768, 32767);
    currentConfig.imuAccelOffY = constrain((int)(doc["imuAccelOffY"] | 0), -32768, 32767);
    currentConfig.imuAccelOffZ = constrain((int)(doc["imuAccelOffZ"] | 1788), -32768, 32767);
    // Старые конфиги с нулевыми офсетами гиро → заводские i2cdevlib
    if (!doc.containsKey("imuGyroOffX") && !doc.containsKey("imuGyroOffY") &&
        !doc.containsKey("imuGyroOffZ")) {
      currentConfig.imuGyroOffX = 220;
      currentConfig.imuGyroOffY = 76;
      currentConfig.imuGyroOffZ = -85;
    } else if (currentConfig.imuGyroOffX == 0 && currentConfig.imuGyroOffY == 0 &&
               currentConfig.imuGyroOffZ == 0) {
      currentConfig.imuGyroOffX = 220;
      currentConfig.imuGyroOffY = 76;
      currentConfig.imuGyroOffZ = -85;
      if (currentConfig.imuAccelOffZ == 0) currentConfig.imuAccelOffZ = 1788;
      Serial.println("[CFG] Офсеты гиро были 0 — применены заводские (220,76,-85)");
    }
    val = doc["zeroAngleX"] | 0.0f;
    currentConfig.zeroAngleX = constrain(val, -45.0f, 45.0f);
    val = doc["zeroAngleY"] | 0.0f;
    currentConfig.zeroAngleY = constrain(val, -45.0f, 45.0f);
    val = doc["imuKalmanMea"] | 7.0f;
    currentConfig.imuKalmanMea = constrain(val, 0.5f, 25.0f);
    val = doc["imuKalmanEst"] | 4.5f;
    currentConfig.imuKalmanEst = constrain(val, 0.5f, 25.0f);
    val = doc["imuKalmanQ"] | 0.005f;
    currentConfig.imuKalmanQ = constrain(val, 0.001f, 0.100f);
    currentConfig.imuPollMs = constrain((int)(doc["imuPollMs"] | 20), 15, 100);
    currentConfig.imuFifoAvg = constrain((int)(doc["imuFifoAvg"] | 3), 1, 8);
    val = doc["imuEmaAlpha"] | 0.20f;
    currentConfig.imuEmaAlpha = constrain(val, 0.05f, 0.50f);
    val = doc["imuEmaSpikeAlpha"] | 0.07f;
    currentConfig.imuEmaSpikeAlpha = constrain(val, 0.05f, 0.50f);
    val = doc["imuEmaSpikeThr"] | 1.4f;
    currentConfig.imuEmaSpikeThr = constrain(val, 0.5f, 5.0f);
    val = doc["imuSlewDps"] | 70.0f;
    currentConfig.imuSlewDps = constrain(val, 5.0f, 120.0f);
    // Миграция старых заводских пресетов > ослабленный баланс 10.1.28
    const bool oldFactoryBal =
        (fabsf(currentConfig.imuKalmanMea - 7.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.005f) < 0.0005f &&
         currentConfig.imuFifoAvg == 3 &&
         fabsf(currentConfig.imuEmaAlpha - 0.20f) < 0.01f);
    const bool oldLegacy =
        (fabsf(currentConfig.imuKalmanMea - 3.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.012f) < 0.0005f) ||
        (fabsf(currentConfig.imuKalmanMea - 8.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.004f) < 0.0005f && formatVersion < 5) ||
        (fabsf(currentConfig.imuKalmanMea - 6.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.006f) < 0.0005f && formatVersion < 5);
    if (oldFactoryBal || oldLegacy) {
      currentConfig.imuKalmanMea = 5.5f;
      currentConfig.imuKalmanEst = 3.5f;
      currentConfig.imuKalmanQ = 0.008f;
      currentConfig.imuPollMs = 20;
      currentConfig.imuFifoAvg = 2;
      currentConfig.imuEmaAlpha = 0.28f;
      currentConfig.imuEmaSpikeAlpha = 0.10f;
      currentConfig.imuEmaSpikeThr = 1.8f;
      currentConfig.imuSlewDps = 70.0f;
      currentConfig.redrawAngleThr = 0.05f;
      Serial.println("[CFG] IMU плавность: миграция > баланс 5.5/3.5/0.008 poll=20");
    }

    Serial.printf("[CFG] Конфигурация загружена успешно (формат файла: %u)\n", formatVersion);
    // Cross-field: min < max, цели движения в пределах.
    if (currentConfig.pressureMin >= currentConfig.pressureMax) {
      Serial.println("[CFG] pressureMin >= pressureMax — исправляю");
      currentConfig.pressureMin = 1.0f;
      currentConfig.pressureMax = 7.0f;
    }
    currentConfig.movementPressureFront =
        constrain(currentConfig.movementPressureFront, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    currentConfig.movementPressureRear =
        constrain(currentConfig.movementPressureRear, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    if (currentConfig.fineZoneRatio >= currentConfig.coarseZoneRatio) {
      currentConfig.fineZoneRatio = currentConfig.coarseZoneRatio * 0.4f;
    }
    dumpToSerial();
    return true;
  }

  static bool save() {
    Serial.println("[CFG] Сохранение конфигурации");

    if (!LittleFS.totalBytes()) {
      Serial.println("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      Serial.println("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    StaticJsonDocument<2048> doc;  // 8.8.0: увеличено под новые параметры
    doc["pressureMin"] = currentConfig.pressureMin;
    doc["formatVersion"] = CONFIG_FORMAT_VERSION;
    doc["pressureMax"] = currentConfig.pressureMax;
    doc["masterLowBar"] = currentConfig.masterLowBar;
    doc["tiltThresholdX"] = currentConfig.tiltThresholdX;
    doc["tiltThresholdY"] = currentConfig.tiltThresholdY;
    doc["contrast"] = currentConfig.contrast;
    doc["nivCount"] = currentConfig.nivCount;
    doc["timeInterval"] = currentConfig.timeInterval;
    doc["inflateDelay"] = currentConfig.inflateDelay;
    doc["releaseDelay"] = currentConfig.releaseDelay;
    doc["movementPressureFront"] = currentConfig.movementPressureFront;
    doc["movementPressureRear"] = currentConfig.movementPressureRear;
    doc["parkingPressureBar"] = currentConfig.parkingPressureBar;

    // ===== 8.8.0 =====
    doc["masterCheckSec"] = currentConfig.masterCheckSec;
    doc["manualMaxTimeSec"] = currentConfig.manualMaxTimeSec;
    doc["pressureDeadband"] = currentConfig.pressureDeadband;
    doc["pressureStabilizeMs"] = currentConfig.pressureStabilizeMs;
    doc["pressureIdleMin"] = currentConfig.pressureIdleMin;
    doc["coarseZoneRatio"] = currentConfig.coarseZoneRatio;
    doc["fineZoneRatio"] = currentConfig.fineZoneRatio;
    doc["worseningRatio"] = currentConfig.worseningRatio;
    doc["movementDurationSec"] = currentConfig.movementDurationSec;
    doc["movementSettleSec"] = currentConfig.movementSettleSec;
    doc["movementCheckSec"] = currentConfig.movementCheckSec;
    doc["movementTolerance"] = currentConfig.movementTolerance;
    doc["gyroThreshold"] = currentConfig.gyroThreshold;
    doc["gyroBumpThreshold"] = currentConfig.gyroBumpThreshold;
    doc["accelThreshold"] = currentConfig.accelThreshold;
    doc["backlightOffMin"] = currentConfig.backlightOffMin;
    doc["frameMs"] = currentConfig.frameMs;
    doc["redrawAngleThr"] = currentConfig.redrawAngleThr;
    doc["redrawPressureThr"] = currentConfig.redrawPressureThr;
    doc["imuMotionDet"] = currentConfig.imuMotionDet;
    doc["imuGyroOffX"] = currentConfig.imuGyroOffX;
    doc["imuGyroOffY"] = currentConfig.imuGyroOffY;
    doc["imuGyroOffZ"] = currentConfig.imuGyroOffZ;
    doc["imuAccelOffX"] = currentConfig.imuAccelOffX;
    doc["imuAccelOffY"] = currentConfig.imuAccelOffY;
    doc["imuAccelOffZ"] = currentConfig.imuAccelOffZ;
    doc["zeroAngleX"] = currentConfig.zeroAngleX;
    doc["zeroAngleY"] = currentConfig.zeroAngleY;
    doc["imuKalmanMea"] = currentConfig.imuKalmanMea;
    doc["imuKalmanEst"] = currentConfig.imuKalmanEst;
    doc["imuKalmanQ"] = currentConfig.imuKalmanQ;
    doc["imuPollMs"] = currentConfig.imuPollMs;
    doc["imuFifoAvg"] = currentConfig.imuFifoAvg;
    doc["imuEmaAlpha"] = currentConfig.imuEmaAlpha;
    doc["imuEmaSpikeAlpha"] = currentConfig.imuEmaSpikeAlpha;
    doc["imuEmaSpikeThr"] = currentConfig.imuEmaSpikeThr;
    doc["imuSlewDps"] = currentConfig.imuSlewDps;

    File file = LittleFS.open("/config.tmp", "w");
    if (!file) {
      // Повтор: иногда FD заняты коротким листингом/OTA-хвостом
      LittleFS.remove("/config.tmp");
      delay(20);
      file = LittleFS.open("/config.tmp", "w");
    }
    if (!file) {
      Serial.println("[CFG] Не удалось создать временный файл (нет FD LittleFS)");
      return false;
    }

    size_t bytesWritten = serializeJson(doc, file);
    file.close();

    if (bytesWritten == 0) {
      Serial.println("[CFG] Ошибка записи JSON");
      LittleFS.remove("/config.tmp");
      return false;
    }

    // 8.8.0: сохраняем предыдущую версию как резервную копию
    if (LittleFS.exists("/config.txt")) {
      LittleFS.remove("/config.bak");
      LittleFS.rename("/config.txt", "/config.bak");
    }
    if (LittleFS.rename("/config.tmp", "/config.txt")) {
      // 8.8.0: проверяем, что записанный файл читается и содержит нужную версию формата
      bool verified = false;
      File check = LittleFS.open("/config.txt", "r");
      if (check) {
        String content = check.readString();
        check.close();
        verified = (content.length() > 0) &&
                   (content.indexOf(String("\"formatVersion\":") + String(CONFIG_FORMAT_VERSION)) >= 0);
      }
      Serial.printf("[CFG] Конфигурация сохранена (%u Б, проверка чтением: %s)\n",
                    (unsigned)bytesWritten, verified ? "OK" : "ОШИБКА");
      if (!verified) {
        Serial.println("[CFG] ВНИМАНИЕ: файл конфигурации не прошёл проверку — оставлена копия /config.bak");
      }
      dumpToSerial();

      Event event;
      event.type = EventType::CONFIG_CHANGED;
      event.timestamp = millis();
      EventBus::publish(event);
      return true;
    } else {
      Serial.println("[CFG] Ошибка переименования файла");
      LittleFS.remove("/config.tmp");
      return false;
    }
  }

  static float getPressureMin() {
    return currentConfig.pressureMin;
  }
  static float getPressureMax() {
    return currentConfig.pressureMax;
  }
  static float getMasterLowBar() {
    return currentConfig.masterLowBar;
  }
  static float getTiltThresholdX() {
    return currentConfig.tiltThresholdX;
  }
  static float getTiltThresholdY() {
    return currentConfig.tiltThresholdY;
  }
  static int getContrast() {
    return currentConfig.contrast;
  }
  static int getNivCount() {
    return currentConfig.nivCount;
  }
  static int getTimeInterval() {
    return currentConfig.timeInterval;
  }
  static int getInflateDelay() {
    return currentConfig.inflateDelay;
  }
  static int getReleaseDelay() {
    return currentConfig.releaseDelay;
  }
  static float getMovementPressureFront() {
    return currentConfig.movementPressureFront;
  }
  static float getMovementPressureRear() {
    return currentConfig.movementPressureRear;
  }
  static float getParkingPressureBar() {
    return currentConfig.parkingPressureBar;
  }

  static float getMovementPressure(int pad) {
    return (pad == PAD_FRONT_LEFT || pad == PAD_FRONT_RIGHT)
             ? currentConfig.movementPressureFront
             : currentConfig.movementPressureRear;
  }

  static void setPressureMin(float v) {
    currentConfig.pressureMin = v;
  }
  static void setPressureMax(float v) {
    currentConfig.pressureMax = v;
  }
  static void setMasterLowBar(float v) {
    if (v < 0.0f) v = 0.0f;
    v = roundf(v * 10.0f) / 10.0f;
    currentConfig.masterLowBar = constrain(v, 0.0f, 4.0f);
  }
  static void setTiltThresholdX(float v) {
    currentConfig.tiltThresholdX = v;
  }
  static void setTiltThresholdY(float v) {
    currentConfig.tiltThresholdY = v;
  }
  static void setContrast(int v) {
    currentConfig.contrast = constrain(v, CONTRAST_MIN, CONTRAST_MAX);
  }
  static void setNivCount(int v) {
    currentConfig.nivCount = v;
  }
  static void setTimeInterval(int v) {
    currentConfig.timeInterval = v;
  }
  static void setInflateDelay(int v) {
    currentConfig.inflateDelay = v;
  }
  static void setReleaseDelay(int v) {
    currentConfig.releaseDelay = v;
  }
  static void setMovementPressureFront(float v) {
    currentConfig.movementPressureFront = v;
  }
  static void setMovementPressureRear(float v) {
    currentConfig.movementPressureRear = v;
  }
  static void setParkingPressureBar(float v) {
    currentConfig.parkingPressureBar = constrain(v, 0.0f, 6.0f);
  }

  /* ===== 8.8.0: аксессоры новых параметров ===== */
#define CFG_INT_ACCESSOR(Name, field)                     \
  static int get##Name() { return currentConfig.field; }  \
  static void set##Name(int v) { currentConfig.field = v; }
#define CFG_FLOAT_ACCESSOR(Name, field)                     \
  static float get##Name() { return currentConfig.field; }  \
  static void set##Name(float v) { currentConfig.field = v; }

  CFG_INT_ACCESSOR(MasterCheckSec, masterCheckSec)
  CFG_INT_ACCESSOR(ManualMaxTimeSec, manualMaxTimeSec)
  CFG_FLOAT_ACCESSOR(PressureDeadband, pressureDeadband)
  CFG_INT_ACCESSOR(PressureStabilizeMs, pressureStabilizeMs)
  CFG_INT_ACCESSOR(PressureIdleMin, pressureIdleMin)
  CFG_FLOAT_ACCESSOR(CoarseZoneRatio, coarseZoneRatio)
  CFG_FLOAT_ACCESSOR(FineZoneRatio, fineZoneRatio)
  CFG_FLOAT_ACCESSOR(WorseningRatio, worseningRatio)
  CFG_INT_ACCESSOR(MovementDurationSec, movementDurationSec)
  CFG_INT_ACCESSOR(MovementSettleSec, movementSettleSec)
  CFG_INT_ACCESSOR(MovementCheckSec, movementCheckSec)
  CFG_FLOAT_ACCESSOR(MovementTolerance, movementTolerance)
  CFG_INT_ACCESSOR(GyroThreshold, gyroThreshold)
  CFG_INT_ACCESSOR(GyroBumpThreshold, gyroBumpThreshold)
  CFG_INT_ACCESSOR(AccelThreshold, accelThreshold)
  CFG_INT_ACCESSOR(BacklightOffMin, backlightOffMin)
  CFG_INT_ACCESSOR(FrameMs, frameMs)
  CFG_FLOAT_ACCESSOR(RedrawAngleThr, redrawAngleThr)
  CFG_FLOAT_ACCESSOR(RedrawPressureThr, redrawPressureThr)
  CFG_INT_ACCESSOR(ImuMotionDet, imuMotionDet)
  CFG_INT_ACCESSOR(ImuGyroOffX, imuGyroOffX)
  CFG_INT_ACCESSOR(ImuGyroOffY, imuGyroOffY)
  CFG_INT_ACCESSOR(ImuGyroOffZ, imuGyroOffZ)
  CFG_INT_ACCESSOR(ImuAccelOffX, imuAccelOffX)
  CFG_INT_ACCESSOR(ImuAccelOffY, imuAccelOffY)
  CFG_INT_ACCESSOR(ImuAccelOffZ, imuAccelOffZ)
  CFG_FLOAT_ACCESSOR(ZeroAngleX, zeroAngleX)
  CFG_FLOAT_ACCESSOR(ZeroAngleY, zeroAngleY)
  CFG_FLOAT_ACCESSOR(ImuKalmanMea, imuKalmanMea)
  CFG_FLOAT_ACCESSOR(ImuKalmanEst, imuKalmanEst)
  CFG_FLOAT_ACCESSOR(ImuKalmanQ, imuKalmanQ)
  CFG_INT_ACCESSOR(ImuPollMs, imuPollMs)
  CFG_INT_ACCESSOR(ImuFifoAvg, imuFifoAvg)
  CFG_FLOAT_ACCESSOR(ImuEmaAlpha, imuEmaAlpha)
  CFG_FLOAT_ACCESSOR(ImuEmaSpikeAlpha, imuEmaSpikeAlpha)
  CFG_FLOAT_ACCESSOR(ImuEmaSpikeThr, imuEmaSpikeThr)
  CFG_FLOAT_ACCESSOR(ImuSlewDps, imuSlewDps)
#undef CFG_INT_ACCESSOR
#undef CFG_FLOAT_ACCESSOR

  /** 8.8.0: разовый перенос офсетов IMU (после калибровки) — без отдельных сеттеров. */
  static void setImuOffsets(int gx, int gy, int gz, int ax, int ay, int az) {
    currentConfig.imuGyroOffX = gx;
    currentConfig.imuGyroOffY = gy;
    currentConfig.imuGyroOffZ = gz;
    currentConfig.imuAccelOffX = ax;
    currentConfig.imuAccelOffY = ay;
    currentConfig.imuAccelOffZ = az;
  }

  /** 8.8.0: печать всех параметров конфигурации в Serial (проверка после загрузки и сохранения). */
  static void dumpToSerial() {
    Serial.printf("[CFG] v%u | Pmin=%.2fбар Pmax=%.2fбар Наклон X=%.2f° Y=%.2f° Ярк=%d%% Попыток=%d Инт=%dмин СБРОС=%dс НАКАЧ=%dс\n",
                  (unsigned)currentConfig.formatVersion, currentConfig.pressureMin, currentConfig.pressureMax,
                  currentConfig.tiltThresholdX, currentConfig.tiltThresholdY, currentConfig.contrast,
                  currentConfig.nivCount, currentConfig.timeInterval,
                  currentConfig.releaseDelay, currentConfig.inflateDelay);
    Serial.printf("[CFG] Движение: перед=%.2f зад=%.2f стоянка=%.2f длит=%dс усп=%dс пров=%dс доп=%.2f гиро=%d акс=%d\n",
                  currentConfig.movementPressureFront, currentConfig.movementPressureRear,
                  currentConfig.parkingPressureBar,
                  currentConfig.movementDurationSec, currentConfig.movementSettleSec, currentConfig.movementCheckSec,
                  currentConfig.movementTolerance, currentConfig.gyroThreshold, currentConfig.accelThreshold);
    Serial.printf("[CFG] Магистраль=%dс | Клапаны: макс=%dс зона=%.2f выравн=%dмс пауза=%dмин | Авто: грубая=%.2f точная=%.2f хуже=%.2f\n",
                  currentConfig.masterCheckSec, currentConfig.manualMaxTimeSec, currentConfig.pressureDeadband,
                  currentConfig.pressureStabilizeMs, currentConfig.pressureIdleMin,
                  currentConfig.coarseZoneRatio, currentConfig.fineZoneRatio, currentConfig.worseningRatio);
    Serial.printf("[CFG] Дисплей: подсветка=%dмин кадр=%dмс углы=%.2f давл=%.2f | IMU: det=%d гиро=%d,%d,%d акс=%d,%d,%d нуль=%.2f,%.2f\n",
                  currentConfig.backlightOffMin, currentConfig.frameMs, currentConfig.redrawAngleThr,
                  currentConfig.redrawPressureThr, currentConfig.imuMotionDet,
                  currentConfig.imuGyroOffX, currentConfig.imuGyroOffY, currentConfig.imuGyroOffZ,
                  currentConfig.imuAccelOffX, currentConfig.imuAccelOffY, currentConfig.imuAccelOffZ,
                  currentConfig.zeroAngleX, currentConfig.zeroAngleY);
    Serial.printf("[CFG] IMU плавность: mea=%.2f est=%.2f q=%.3f poll=%dмс fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f\n",
                  currentConfig.imuKalmanMea, currentConfig.imuKalmanEst, currentConfig.imuKalmanQ,
                  currentConfig.imuPollMs, currentConfig.imuFifoAvg,
                  currentConfig.imuEmaAlpha, currentConfig.imuEmaSpikeAlpha,
                  currentConfig.imuEmaSpikeThr, currentConfig.imuSlewDps);
  }
};

ConfigManager::Config ConfigManager::currentConfig;

/* ── Bridge functions for modules that can't see ConfigManager inline defs ── */
int   cfg_getImuGyroOffX()          { return ConfigManager::getImuGyroOffX(); }
int   cfg_getImuGyroOffY()          { return ConfigManager::getImuGyroOffY(); }
int   cfg_getImuGyroOffZ()          { return ConfigManager::getImuGyroOffZ(); }
int   cfg_getImuAccelOffX()         { return ConfigManager::getImuAccelOffX(); }
int   cfg_getImuAccelOffY()         { return ConfigManager::getImuAccelOffY(); }
int   cfg_getImuAccelOffZ()         { return ConfigManager::getImuAccelOffZ(); }
int   cfg_getImuMotionDet()         { return ConfigManager::getImuMotionDet(); }
int   cfg_getGyroThreshold()        { return ConfigManager::getGyroThreshold(); }
int   cfg_getGyroBumpThreshold()    { return ConfigManager::getGyroBumpThreshold(); }
int   cfg_getAccelThreshold()       { return ConfigManager::getAccelThreshold(); }
int   cfg_getMovementSettleSec()    { return ConfigManager::getMovementSettleSec(); }
int   cfg_getMovementDurationSec()  { return ConfigManager::getMovementDurationSec(); }
int   cfg_getImuPollMs()            { return ConfigManager::getImuPollMs(); }
int   cfg_getImuFifoAvg()           { return ConfigManager::getImuFifoAvg(); }
float cfg_getImuKalmanMea()         { return ConfigManager::getImuKalmanMea(); }
float cfg_getImuKalmanEst()         { return ConfigManager::getImuKalmanEst(); }
float cfg_getImuKalmanQ()           { return ConfigManager::getImuKalmanQ(); }
float cfg_getImuEmaAlpha()          { return ConfigManager::getImuEmaAlpha(); }
float cfg_getImuEmaSpikeAlpha()     { return ConfigManager::getImuEmaSpikeAlpha(); }
float cfg_getImuEmaSpikeThr()       { return ConfigManager::getImuEmaSpikeThr(); }
float cfg_getImuSlewDps()           { return ConfigManager::getImuSlewDps(); }
float cfg_getZeroAngleX()           { return ConfigManager::getZeroAngleX(); }
float cfg_getZeroAngleY()           { return ConfigManager::getZeroAngleY(); }
float cfg_getMovementPressureFront(){ return ConfigManager::getMovementPressureFront(); }
float cfg_getMovementPressureRear() { return ConfigManager::getMovementPressureRear(); }
float cfg_getMovementTolerance()    { return ConfigManager::getMovementTolerance(); }

/* ====================  ErrorHandler ==================== */
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

// ========== ОПРЕДЕЛЕНИЕ СТАТИЧЕСКИХ ЧЛЕНОВ ==========
SemaphoreHandle_t ErrorHandler::mutex_ = nullptr;
ErrorHandler::ActiveError ErrorHandler::activeErrors[8];
uint8_t ErrorHandler::activeCount = 0;
ErrorHandler::PendingClear ErrorHandler::pendingClear[8];
uint8_t ErrorHandler::pendingCount = 0;
uint8_t ErrorHandler::currentDisplayIndex = 0;
uint32_t ErrorHandler::lastDisplaySwitch = 0;
bool ErrorHandler::hasError = false;

/* ── Bridge functions for ErrorHandler / TaskMonitor (modules) ── */
bool cfg_errorIsActive(uint8_t err) { return ErrorHandler::isErrorActive(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorMarkCleared(uint8_t err) { ErrorHandler::markErrorCleared(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorRemove(uint8_t err) { ErrorHandler::removeError(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorHandle(uint8_t err, const char *msg) { ErrorHandler::handleError(static_cast<ErrorHandler::Error>(err), msg); }
void cfg_errorUpdateTime(uint8_t err) { ErrorHandler::updateErrorTime(static_cast<ErrorHandler::Error>(err)); }
bool cfg_errorIsPendingClear(uint8_t err) { return ErrorHandler::isPendingClear(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorCancelClear(uint8_t err) { ErrorHandler::cancelClear(static_cast<ErrorHandler::Error>(err)); }
bool cfg_errorHasActive() { return ErrorHandler::hasActiveErrors(); }
bool cfg_errorHasCriticalPneumatic() { return ErrorHandler::hasCriticalPneumaticErrors(); }

bool cfg_errorHasUiBlocking() { return ErrorHandler::hasUiBlockingErrors(); }
int cfg_errorActiveCount() { return ErrorHandler::getActiveErrorCount(); }
uint8_t cfg_errorCurrent() { return static_cast<uint8_t>(ErrorHandler::getCurrentActiveError()); }
int cfg_errorDisplayIndex() { return ErrorHandler::getCurrentDisplayIndex(); }
const char *cfg_errorMessage(uint8_t err) { return ErrorHandler::getErrorMessage(static_cast<ErrorHandler::Error>(err)); }
void cfg_setMasterLowBar(float v) { ConfigManager::setMasterLowBar(v); }
void cfg_setBacklightOffMin(int v) { ConfigManager::setBacklightOffMin(v); }
// cfg_taskMonitorUpdate — placed after TaskMonitor class definition below

/* ── Bridge functions for ConfigManager (pressure/valve/auto-level modules) ── */
float cfg_getPressureMin()          { return ConfigManager::getPressureMin(); }
float cfg_getPressureMax()          { return ConfigManager::getPressureMax(); }
float cfg_getPressureDeadband()     { return ConfigManager::getPressureDeadband(); }
float cfg_getMasterLowBar()         { return ConfigManager::getMasterLowBar(); }
float cfg_getParkingPressureBar()   { return ConfigManager::getParkingPressureBar(); }
int   cfg_getInflateDelay()         { return ConfigManager::getInflateDelay(); }
int   cfg_getReleaseDelay()         { return ConfigManager::getReleaseDelay(); }
int   cfg_getMasterCheckSec()       { return ConfigManager::getMasterCheckSec(); }
int   cfg_getMovementCheckSec()     { return ConfigManager::getMovementCheckSec(); }
int   cfg_getManualMaxTimeSec()     { return ConfigManager::getManualMaxTimeSec(); }
int   cfg_getPressureStabilizeMs()  { return ConfigManager::getPressureStabilizeMs(); }
int   cfg_getPressureIdleMin()      { return ConfigManager::getPressureIdleMin(); }
float cfg_getRedrawPressureThr()    { return ConfigManager::getRedrawPressureThr(); }
float cfg_getTiltThresholdX()       { return ConfigManager::getTiltThresholdX(); }
float cfg_getTiltThresholdY()       { return ConfigManager::getTiltThresholdY(); }
float cfg_getCoarseZoneRatio()      { return ConfigManager::getCoarseZoneRatio(); }
float cfg_getWorseningRatio()       { return ConfigManager::getWorseningRatio(); }
float cfg_getFineZoneRatio()        { return ConfigManager::getFineZoneRatio(); }
int   cfg_getNivCount()             { return ConfigManager::getNivCount(); }
int cfg_getBacklightOffMin() { return ConfigManager::getBacklightOffMin(); }
int cfg_getContrast() { return ConfigManager::getContrast(); }
int cfg_getFrameMs() { return ConfigManager::getFrameMs(); }
float cfg_getRedrawAngleThr() { return ConfigManager::getRedrawAngleThr(); }
int cfg_getTimeInterval() { return ConfigManager::getTimeInterval(); }

// ZeroCalibrator — moved to task_calib.cpp


/* ====================  ПЕРЕМЕННЫЕ ДЛЯ СИМУЛЯЦИИ ==================== */
// Эти переменные используются даже когда симуляция выключена
float simAngleX = 0;
float simAngleY = 0;
float simPressures[PAD_COUNT] = { 2.0f, 2.5f, 3.0f, 3.5f };
float simMasterPressure = 4.0f;
int simDirectionX = 1;
int simDirectionY = 1;
uint32_t lastSimUpdate = 0;

// ? ДОБАВЛЕНО: Эти переменные теперь всегда объявлены
// чтобы их можно было использовать в forceDisplayReset()
static uint32_t menuAutoEnterTime = 0;
static bool menuAutoEnterDone = false;
static bool errorSimulated = false;
static ErrorHandler::Error simulatedError = ErrorHandler::Error::NONE;

#if ENABLE_SIMULATION
// Эти переменные нужны только когда симуляция включена
static uint32_t lastAutoSimTime = 0;
static uint32_t autoSimPhase = 0;
static uint32_t lastErrorSimTime = 0;
#endif


/* ====================  TaskMonitor ==================== */
class TaskMonitor {
private:
  struct TaskInfo {
    const char *name;
    uint32_t lastRunTime;
    bool isRunning;
  };
  static TaskInfo tasks[TASK_COUNT];

public:
  enum TaskIndex : uint8_t {
    TASK_EVENT = 0,
    TASK_BUTTON,
    TASK_DISPLAY,
    TASK_IMU,
    TASK_PRESSURE,
    TASK_CONTROL,
    TASK_CALIB,
    TASK_WATCHDOG,
    TASK_OTA,
    TASK_VALVE,
    TASK_ERROR_RECOVERY
  };

  static void updateTaskStatus(TaskIndex index) {
    tasks[index].lastRunTime = millis();
    tasks[index].isRunning = true;
  }

  static void checkTasks() {
    uint32_t currentTime = millis();
    for (int i = 0; i < TASK_COUNT; i++) {
      if (tasks[i].lastRunTime == 0) continue;  // ещё не стартовала
      if (currentTime - tasks[i].lastRunTime > TASK_WDT_TIMEOUT_MS) {
        // Pressure при SENSOR / до калибровки / в паузе опроса — не зависание.
        if (i == (int)TASK_PRESSURE &&
            (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) || !calibrationCompleted)) {
          tasks[i].lastRunTime = currentTime;
          continue;
        }
        // TLS OTA: воркеры специально на паузе — не поднимать WATCHDOG.
        if (s_otaWorkersPaused &&
            (i == (int)TASK_BUTTON || i == (int)TASK_DISPLAY || i == (int)TASK_IMU ||
             i == (int)TASK_PRESSURE || i == (int)TASK_CONTROL || i == (int)TASK_VALVE ||
             i == (int)TASK_ERROR_RECOVERY)) {
          tasks[i].lastRunTime = currentTime;
          continue;
        }
        Logger::logf(Logger::ERROR, "WATCHDOG", "Task %s is not responding", tasks[i].name);
        ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG, "Task timeout");
        tasks[i].lastRunTime = currentTime;  // не спамить каждую секунду
      }
    }
  }

  static void init() {
    const uint32_t now = millis();
    tasks[TASK_EVENT] = { "Event", now, false };
    tasks[TASK_BUTTON] = { "Button", now, false };
    tasks[TASK_DISPLAY] = { "Display", now, false };
    tasks[TASK_IMU] = { "IMU", now, false };
    tasks[TASK_PRESSURE] = { "Pressure", now, false };
    tasks[TASK_CONTROL] = { "Control", now, false };
    tasks[TASK_CALIB] = { "Calib", now, false };
    tasks[TASK_WATCHDOG] = { "Watchdog", now, false };
    tasks[TASK_OTA] = { "OTA", now, false };
    tasks[TASK_VALVE] = { "Valve", now, false };
    tasks[TASK_ERROR_RECOVERY] = { "ErrorRecovery", now, false };
  }
};

TaskMonitor::TaskInfo TaskMonitor::tasks[TASK_COUNT];

void cfg_taskMonitorUpdate(uint8_t idx) { TaskMonitor::updateTaskStatus(static_cast<TaskMonitor::TaskIndex>(idx)); }

/* ====================  MemoryMonitor ==================== */
class MemoryMonitor {
private:
  static uint32_t lastCheckTime;
  static uint32_t minFreeHeap;
  static uint32_t lastWarningTime;
  static constexpr uint32_t LOW_MEMORY_THRESHOLD = 8192;
  static constexpr uint32_t CRITICAL_MEMORY_THRESHOLD = 4096;
  static bool lowMemoryReported;
  static bool criticalMemoryReported;

public:
  static void init() {
    lastCheckTime = millis();
    minFreeHeap = ESP.getFreeHeap();
    lastWarningTime = 0;
    lowMemoryReported = false;
    criticalMemoryReported = false;
    Serial.printf("[MEM] Начально свободно: %d байт\n", minFreeHeap);
  }

  static void checkMemory() {
    uint32_t currentFree = ESP.getFreeHeap();
    uint32_t now = millis();

    if (currentFree < minFreeHeap) {
      minFreeHeap = currentFree;
      Serial.printf("[MEM] Новый минимум: %d байт\n", minFreeHeap);
    }

    if (currentFree < CRITICAL_MEMORY_THRESHOLD) {
      if (!criticalMemoryReported) {
        criticalMemoryReported = true;
        Serial.printf("[MEM] ?? КРИТИЧЕСКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

        Event event;
        event.type = EventType::CRITICAL_MEMORY;
        event.timestamp = now;
        event.data.memory.freeHeap = currentFree;
        event.data.memory.minHeap = minFreeHeap;
        EventBus::publish(event);

        emergencyStop();
        closeAllValves();
      }
    } else if (currentFree < LOW_MEMORY_THRESHOLD) {
      if (!lowMemoryReported && (now - lastWarningTime > 60000)) {
        lowMemoryReported = true;
        lastWarningTime = now;
        Serial.printf("[MEM] ?? НИЗКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

        Event event;
        event.type = EventType::LOW_MEMORY_WARNING;
        event.timestamp = now;
        event.data.memory.freeHeap = currentFree;
        event.data.memory.minHeap = minFreeHeap;
        EventBus::publish(event);
      }
    } else {
      if (criticalMemoryReported && currentFree > CRITICAL_MEMORY_THRESHOLD + 2048) {
        criticalMemoryReported = false;
        Serial.printf("[MEM] ? Память восстановлена: %d байт\n", currentFree);
      }
      if (lowMemoryReported && currentFree > LOW_MEMORY_THRESHOLD + 2048) {
        lowMemoryReported = false;
        Serial.printf("[MEM] ? Уровень памяти нормализован: %d байт\n", currentFree);
      }
    }

    if (now - lastCheckTime > 60000) {
      lastCheckTime = now;
      Serial.printf("[MEM] Свободно: %d байт (минимум: %d)\n", currentFree, minFreeHeap);
    }
  }
};

uint32_t MemoryMonitor::lastCheckTime = 0;
uint32_t MemoryMonitor::minFreeHeap = 0;
uint32_t MemoryMonitor::lastWarningTime = 0;
bool MemoryMonitor::lowMemoryReported = false;
bool MemoryMonitor::criticalMemoryReported = false;

/* ====================  РЕАЛИЗАЦИЯ ФУНКЦИЙ ==================== */

void setDisplayDirty() {
  // Меню и экран Wi-Fi сами решают, что перерисовывать — фоновый IMU/давление
  // не должны выставлять dirty (иначе Wi-Fi снова делает fillScreen).
  if (menuVisible || wifiSetupActive || wifiScanInProgress) return;
  displayDirty = true;
  Event event;
  event.type = EventType::DISPLAY_UPDATE;
  event.timestamp = millis();
  EventBus::publish(event, 0);
}

// closeAllValves — moved to valve_ctrl.cpp

// setValve — moved to valve_ctrl.cpp

// emergencyStop — moved to valve_ctrl.cpp

// sendValveCommand — moved to valve_ctrl.cpp

/* ===== Подсветка: рабочая яркость / приглушение по бездействию ===== */
bool backlightDimmed = false;     // true = подсветка на уровне CONTRAST_DIM (не OFF)
uint32_t lastUserActivityMs = 0;

static int contrastToPwm(int level) {
  return map(constrain(level, 1, CONTRAST_MAX), 1, CONTRAST_MAX, 0, 255);
}

void applyBacklightPwm(int level) {
  analogWrite(PIN_TFT_BL, contrastToPwm(level));
}

/** Применение настроек к железу сразу после сохранения (без перезагрузки). */
void applyRuntimeSettings() {
  backlightDimmed = false;
  applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));

  if (ConfigManager::getBacklightOffMin() == 0) {
    backlightDimmed = false;
  }
  lastUserActivityMs = millis();

  if (mpuOk) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(50));
    if (i2c) {
      // Аппаратный MOT после DMP: DHPF + THR + DUR + counter decrement + INT.
      mpu.setDHPFMode(MPU6050_DHPF_1P25);
      mpu.setMotionDetectionThreshold((uint8_t)ConfigManager::getImuMotionDet());
      mpu.setMotionDetectionDuration(g_motDurMs);
      mpu.setAccelerometerPowerOnDelay(3);
      mpu.setMotionDetectionCounterDecrement(1);
      mpu.setIntEnabled(0x52);  // MOT | FIFO_OFLOW | DMP_INT
      (void)mpu.getIntStatus();
      Serial.printf("[IMU] MOT_THR=%d MOT_DUR=%u INT_EN=0x%02X\n",
                    ConfigManager::getImuMotionDet(), (unsigned)g_motDurMs,
                    (unsigned)mpu.getIntEnabled());
    }
  }

  filterX.setParameters(ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                        ConfigManager::getImuKalmanQ());
  filterY.setParameters(ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                        ConfigManager::getImuKalmanQ());
  g_imuFilterResetReq = true;
  Serial.printf("[IMU] Калман mea=%.2f est=%.2f q=%.3f | poll=%dms fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f\n",
                ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                ConfigManager::getImuKalmanQ(), ConfigManager::getImuPollMs(),
                ConfigManager::getImuFifoAvg(), ConfigManager::getImuEmaAlpha(),
                ConfigManager::getImuEmaSpikeAlpha(), ConfigManager::getImuEmaSpikeThr(),
                ConfigManager::getImuSlewDps());

  Serial.printf("[MENU] Применено: кадр=%d мс, приглуш.=%d мин, зона=%.2f бар, допуск=%.2f бар\n",
                ConfigManager::getFrameMs(), ConfigManager::getBacklightOffMin(),
                ConfigManager::getPressureDeadband(), ConfigManager::getMovementTolerance());
  // Не вызываем forceDisplayReset здесь: fillScreen без xDisplayMutex с ButtonTask
  // (и вложенно из close) давал подвисание SPI. Сброс экрана — у вызывающего
  // под мьютексом дисплея или через requestMenuClose().
  displayDirty = true;
}

/* ============================================================================
   8.8.0: СЛУЖЕБНЫЕ ЭКРАНЫ — ручной тест клапанов, IMU (нуль углов, калибровка
   офсетов), диагностика MPU6050.
   Управление: Кн1/Кн2 — выбор, Кн3 — действие/подтверждение, Кн4 — выход.
   Аварийная остановка (Кн4+Кн5) закрывает все клапаны и выходит из экрана.
   ============================================================================ */
// ServiceScreen — see ui_screens.h

volatile ServiceScreen serviceScreen = ServiceScreen::NONE;

// --- ручной тест клапанов: 4 подушечных + накачка + сброс ---
extern const char *const manualValveNames[MANUAL_VALVE_COUNT] = { "ПЛ", "ПП", "ЗЛ", "ЗП", "НАКАЧКА", "СБРОС" };
extern const uint8_t manualValvePins[MANUAL_VALVE_COUNT] = { PIN_BUB1, PIN_BUB2, PIN_BUB3, PIN_BUB4, PIN_INFL, PIN_DEFL };
uint8_t manualValveIndex = 0;
uint8_t manualValveOpenIndex = 0xFF;  // 0xFF — ничего не открыто
uint32_t manualValveOpenSince = 0;
bool valveTestUiFullRedraw = true;
bool imuZeroUiFullRedraw = true;
bool imuCalibUiFullRedraw = true;
bool mpuDiagUiFullRedraw = true;

// --- IMU: калибровка офсетов ---
int8_t imuCalibResult = 0;  // 0 — не запускалась, 1 — успех, -1 — ошибка
uint32_t imuCalibLastRun = 0;

/** Доступ к служебным операциям: только РУЧНОЙ режим, без ошибок и вне OTA. */
static bool serviceOpsAllowed() {
  if (otaMode || otaInProgress || otaValveLock) return false;
  if (ErrorHandler::hasActiveErrors()) return false;
  if (currentSystemMode != SystemMode::MANUAL) return false;
  if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) return false;
  return true;
}

/** Открыть/закрыть один клапан вручную под мьютексом клапанов. */
static void manualValveSet(uint8_t idx, bool open) {
  if (idx >= MANUAL_VALVE_COUNT) return;
  bool locked = (xValveMutex != nullptr) && (xSemaphoreTake(xValveMutex, pdMS_TO_TICKS(150)) == pdTRUE);
  digitalWrite(manualValvePins[idx], open ? HIGH : LOW);
  if (locked) xSemaphoreGive(xValveMutex);
  Serial.printf("[VALVE-TEST] Клапан %s: %s\n", manualValveNames[idx], open ? "ОТКРЫТ" : "ЗАКРЫТ");
}

/** Закрыть все 6 клапанов (выход, ошибка, аварийная остановка). */
void manualValveCloseAll() {
  bool locked = (xValveMutex != nullptr) && (xSemaphoreTake(xValveMutex, pdMS_TO_TICKS(150)) == pdTRUE);
  for (uint8_t i = 0; i < MANUAL_VALVE_COUNT; i++) digitalWrite(manualValvePins[i], LOW);
  if (locked) xSemaphoreGive(xValveMutex);
  manualValveOpenIndex = 0xFF;
  Serial.println("[VALVE-TEST] Все клапаны закрыты");
}

/** Открыть экран ручного теста клапанов. */
void openValveTestScreen() {
  if (!serviceOpsAllowed()) {
    Logger::log(Logger::WARNING, "VALVE-TEST", "Только РУЧНОЙ режим, без ошибок и вне OTA");
    return;
  }
  saveMenuSettings();  // сохраняем возможные изменения меню
  manualValveCloseAll();
  manualValveIndex = 0;
  manualValveOpenIndex = 0xFF;
  serviceScreen = ServiceScreen::VALVE_TEST;
  menuVisible = false;
  valveTestUiFullRedraw = true;
  forceDisplayReset(true);
  displayDirty = true;
  Serial.println("[VALVE-TEST] Кн1/Кн2 — выбор, удерживайте Кн3 для открытия, Кн4 — выход");
}

/** Открыть подтверждение обнуления углов по текущему положению. */
void openImuZeroConfirm() {
  if (!serviceOpsAllowed()) {
    Logger::log(Logger::WARNING, "IMU", "Обнуление углов: только РУЧНОЙ режим без ошибок");
    return;
  }
  serviceScreen = ServiceScreen::IMU_ZERO_CONFIRM;
  menuVisible = false;
  imuZeroUiFullRedraw = true;
  forceDisplayReset(true);
  displayDirty = true;
}

/** Открыть экран калибровки офсетов IMU. */
void openImuCalibScreen() {
  if (!mpuOk) {
    Logger::log(Logger::ERROR, "IMU", "Калибровка недоступна: MPU6050 не отвечает");
    return;
  }
  if (!serviceOpsAllowed()) {
    Logger::log(Logger::WARNING, "IMU", "Калибровка: только РУЧНОЙ режим без ошибок");
    return;
  }
  serviceScreen = ServiceScreen::IMU_CALIB;
  menuVisible = false;
  imuCalibUiFullRedraw = true;
  forceDisplayReset(true);
  displayDirty = true;
}

/** Открыть экран диагностики MPU6050. */
void openMpuDiagScreen() {
  serviceScreen = ServiceScreen::MPU_DIAG;
  menuVisible = false;
  mpuDiagUiFullRedraw = true;
  forceDisplayReset(true);
  displayDirty = true;
}

/** Выход со служебного экрана: всегда закрываем клапаны. */
static void closeServiceScreen() {
  manualValveCloseAll();
  serviceScreen = ServiceScreen::NONE;
  forceDisplayReset(true);
  displayDirty = true;
}

/**
 * 8.8.0: калибровка офсетов IMU по библиотеке MPU6050.
 * Машина должна стоять ровно и неподвижно (10-20 с). DMP на время калибровки
 * отключается, затем перезапускается. Офсеты сохраняются в config.txt и
 * применяются при старте (см. initializeDMP).
 */
static void runImuOffsetCalibration() {
  if (!mpuOk) {
    imuCalibResult = -1;
    return;
  }
  MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(30000));
  if (!i2c) {
    imuCalibResult = -1;
    return;
  }
  Serial.println("[IMU] Калибровка офсетов: НЕ двигайте машину (10-20 с)...");
  imuCalibResult = 0;
  mpu.setDMPEnabled(false);
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);
  ConfigManager::setImuOffsets(mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset(),
                               mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset());
  settingsChanged = true;
  saveMenuSettings();  // офсеты сразу пишутся в config.txt
  mpu.setDMPEnabled(true);
  imuCalibLastRun = millis();
  imuCalibResult = 1;
  Serial.printf("[IMU] Офсеты сохранены: гиро (%d,%d,%d), аксель (%d,%d,%d)\n",
                ConfigManager::getImuGyroOffX(), ConfigManager::getImuGyroOffY(), ConfigManager::getImuGyroOffZ(),
                ConfigManager::getImuAccelOffX(), ConfigManager::getImuAccelOffY(), ConfigManager::getImuAccelOffZ());
}

/** Обработка кнопок на служебных экранах (вызывается из задачи кнопок). */
void handleServiceInput() {
  switch (serviceScreen) {
    case ServiceScreen::VALVE_TEST: {
      if (button0.click()) {
        manualValveIndex = (manualValveIndex + MANUAL_VALVE_COUNT - 1) % MANUAL_VALVE_COUNT;
      }
      if (button1.click()) {
        manualValveIndex = (manualValveIndex + 1) % MANUAL_VALVE_COUNT;
      }
      // Кн3 удерживаем — выбранный клапан открыт; отпустили — закрылся
      if (button2.pressing()) {
        if (manualValveOpenIndex != manualValveIndex) {
          if (manualValveOpenIndex != 0xFF) manualValveSet(manualValveOpenIndex, false);
          manualValveSet(manualValveIndex, true);
          manualValveOpenIndex = manualValveIndex;
          manualValveOpenSince = millis();
        } else if (millis() - manualValveOpenSince > VALVE_MAX_COMMAND_MS) {
          Serial.println("[VALVE-TEST] Страховочный таймаут: клапан закрыт");
          manualValveSet(manualValveOpenIndex, false);
          manualValveOpenIndex = 0xFF;
        }
      } else if (manualValveOpenIndex != 0xFF) {
        manualValveSet(manualValveOpenIndex, false);
        manualValveOpenIndex = 0xFF;
      }
      if (button3.click()) {
        Serial.println("[VALVE-TEST] Выход, все клапаны закрыты");
        closeServiceScreen();
      }
      break;
    }

    case ServiceScreen::IMU_ZERO_CONFIRM: {
      if (button2.click()) {  // подтверждение
        // Берём абсолютный отфильтрованный угол (до вычитания нуля), не angleX.
        float absX = g_imuAbsX;
        float absY = g_imuAbsY;
        if (!isfinite(absX) || !isfinite(absY) || fabsf(absX) > 90.0f || fabsf(absY) > 90.0f) {
          // запасной путь: восстановить из отображаемого + старого нуля
          MutexGuard guard(xStateMutex);
          if (guard) {
            absX = angleX + ConfigManager::getZeroAngleX();
            absY = angleY + ConfigManager::getZeroAngleY();
          }
        }
        if (fabsf(absX) > 45.0f || fabsf(absY) > 45.0f) {
          Serial.printf("[IMU] Наклон вне ±45° (abs %.2f/%.2f) — нуль будет ограничен\n", absX, absY);
        }
        ConfigManager::setZeroAngleX(constrain(absX, -45.0f, 45.0f));
        ConfigManager::setZeroAngleY(constrain(absY, -45.0f, 45.0f));
        editZeroAngleX = ConfigManager::getZeroAngleX();
        editZeroAngleY = ConfigManager::getZeroAngleY();
        // Мгновенный 0 на UI до следующего тика фильтров
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            angleX = 0.0f;
            angleY = 0.0f;
          }
        }
        g_imuOutX = 0.0f;
        g_imuOutY = 0.0f;
        g_imuFilterResetReq = true;
        settingsChanged = true;
        saveMenuSettings();
        Serial.printf("[IMU] Программный нуль углов: X=%.2f Y=%.2f (abs было %.2f/%.2f)\n",
                      ConfigManager::getZeroAngleX(), ConfigManager::getZeroAngleY(), absX, absY);
        closeServiceScreen();
      }
      if (button3.click()) {  // отмена
        Serial.println("[IMU] Обнуление углов отменено");
        closeServiceScreen();
      }
      break;
    }

    case ServiceScreen::IMU_CALIB: {
      if (button2.click()) {
        runImuOffsetCalibration();
      }
      if (button3.click()) {
        closeServiceScreen();
      }
      break;
    }

    case ServiceScreen::MPU_DIAG: {
      if (button3.click()) {
        closeServiceScreen();
      }
      break;
    }

    default:
      break;
  }
}

// ui screens (service, movement, gauges) — see ui_screens.cpp


// displayMovementScreen — see ui_screens.cpp

// 11.6. Сохранение настроек меню
void saveMenuSettings() {
  if (!settingsChanged) return;

  if (editPressureMin >= editPressureMax) {
    Serial.println("[MENU] Отклонено: pressureMin >= pressureMax");
    Logger::log(Logger::WARNING, "MENU", "min >= max — не сохранено");
    return;
  }
  editMovementPressureFront = constrain(editMovementPressureFront, editPressureMin, editPressureMax);
  editMovementPressureRear = constrain(editMovementPressureRear, editPressureMin, editPressureMax);
  if (editFineZone >= editCoarseZone) {
    editFineZone = editCoarseZone * 0.4f;
  }

  ConfigManager::setReleaseDelay(editReleaseDelay);
  ConfigManager::setInflateDelay(editInflateDelay);
  ConfigManager::setTiltThresholdX(editTiltX);
  ConfigManager::setTiltThresholdY(editTiltY);
  ConfigManager::setPressureMin(editPressureMin);
  ConfigManager::setPressureMax(editPressureMax);
  ConfigManager::setMasterLowBar(editMasterLowTenths * 0.1f);
  ConfigManager::setNivCount(editNivCount);
  ConfigManager::setTimeInterval(editTimeInterval);
  ConfigManager::setContrast(editContrast);
  ConfigManager::setMovementPressureFront(editMovementPressureFront);
  ConfigManager::setMovementPressureRear(editMovementPressureRear);
  ConfigManager::setParkingPressureBar(editParkingPressure);

  // ===== 8.8.0: новые параметры =====
  ConfigManager::setMasterCheckSec(editMasterCheck);
  ConfigManager::setManualMaxTimeSec(editManualMaxTime);
  ConfigManager::setPressureDeadband(editDeadband);
  ConfigManager::setPressureStabilizeMs(editPressStabilizeMs);
  ConfigManager::setPressureIdleMin(editPressIdleMin);
  ConfigManager::setCoarseZoneRatio(editCoarseZone);
  ConfigManager::setFineZoneRatio(editFineZone);
  ConfigManager::setWorseningRatio(editWorsening);
  ConfigManager::setMovementDurationSec(editMoveDuration);
  ConfigManager::setMovementSettleSec(editMoveSettle);
  ConfigManager::setMovementCheckSec(editMoveCheck);
  ConfigManager::setMovementTolerance(editMoveTolerance);
  ConfigManager::setGyroThreshold(editGyroThreshold);
  ConfigManager::setGyroBumpThreshold(editGyroBumpThreshold);
  ConfigManager::setAccelThreshold(editAccelThreshold);
  ConfigManager::setBacklightOffMin(editBacklightOff);
  ConfigManager::setFrameMs(editFrameMs);
  ConfigManager::setRedrawAngleThr(editRedrawAngle);
  ConfigManager::setRedrawPressureThr(editRedrawPressure);
  ConfigManager::setImuMotionDet(editImuMotionDet);
  ConfigManager::setZeroAngleX(editZeroAngleX);
  ConfigManager::setZeroAngleY(editZeroAngleY);
  ConfigManager::setImuKalmanMea(editImuKalmanMea);
  ConfigManager::setImuKalmanEst(editImuKalmanEst);
  ConfigManager::setImuKalmanQ(editImuKalmanQ);
  ConfigManager::setImuPollMs(editImuPollMs);
  ConfigManager::setImuFifoAvg(editImuFifoAvg);
  ConfigManager::setImuEmaAlpha(editImuEmaAlpha);
  ConfigManager::setImuEmaSpikeAlpha(editImuEmaSpikeAlpha);
  ConfigManager::setImuEmaSpikeThr(editImuEmaSpikeThr);
  ConfigManager::setImuSlewDps(editImuSlewDps);

  const bool saved = saveConfig();
  if (!saved) {
    Serial.println("[MENU] Сохранение в LittleFS не удалось — settingsChanged оставлен");
    Logger::log(Logger::ERROR, "MENU", "Конфиг не записан (LittleFS FD?)");
    // Параметры уже в RAM — применяем, чтобы пресет MPU не «терялся» до следующего Save
    applyRuntimeSettings();
    return;
  }
  settingsChanged = false;
  applyRuntimeSettings();  // 8.8.0: новые значения действуют сразу, без перезагрузки
  Logger::log(Logger::INFO, "MENU", "Настройки сохранены");
}

// setManualTargetPressure — moved to pressure_read.cpp
// setAllManualTargetsFromCurrent — moved to pressure_read.cpp
// setManualTargetsFromParkingPolicy — moved to pressure_read.cpp
// maintainManualPressure — moved to pressure_read.cpp

// detectMotionFromIMU — moved to imu_motion.cpp

// maintainManualPressure — see pressure_read.cpp

#if ENABLE_SIMULATION
void updateSimulationData() {
  uint32_t now = millis();

  static ErrorHandler::Error lastError = ErrorHandler::Error::NONE;
  ErrorHandler::Error currentErr = ErrorHandler::getCurrentActiveError();
  if (currentErr != lastError) {
    Serial.printf("[SIM] ERROR CHANGED! Old=%d, New=%d\n", (int)lastError, (int)currentErr);
    lastError = currentErr;
  }

  static uint32_t lastCall = 0;
  if (now - lastCall > 100) {
    lastCall = now;
    Serial.printf("[SIM] >>> updateSimulationData called, mode=%d, state=%d, autoMode=%d\n",
                  (int)currentSystemMode, (int)currentState, SIMULATE_AUTO_MODE);
  }

#if SIMULATE_AUTO_MODE
  if (currentState == SystemState::RUNNING) {
    currentSystemMode = SystemMode::AUTO;
    currentMode = Mode::AUTO;
  }

  uint32_t phaseDuration = 10000;
  uint32_t currentPhase = (now / phaseDuration) % 4;

  static uint32_t autoSimPhase = 0;
  static float simAngleXTarget = 0;
  static float simAngleYTarget = 0;

  if (currentPhase != autoSimPhase) {
    autoSimPhase = currentPhase;
    switch (autoSimPhase) {
      case 0:
        simAngleXTarget = 0;
        simAngleYTarget = 4.5f;
        break;
      case 1:
        simAngleXTarget = 0;
        simAngleYTarget = -4.5f;
        break;
      case 2:
        simAngleXTarget = 4.5f;
        simAngleYTarget = 0;
        break;
      case 3:
        simAngleXTarget = -4.5f;
        simAngleYTarget = 0;
        break;
    }
  }

  simAngleX += (simAngleXTarget - simAngleX) * 0.1f;
  simAngleY += (simAngleYTarget - simAngleY) * 0.1f;

#else
  simAngleX += simDirectionX * 0.08f;
  if (simAngleX > 3.0f) {
    simAngleX = 3.0f;
    simDirectionX = -1;
  } else if (simAngleX < -3.0f) {
    simAngleX = -3.0f;
    simDirectionX = 1;
  }

  simAngleY += simDirectionY * 0.08f;
  if (simAngleY > 3.5f) {
    simAngleY = 3.5f;
    simDirectionY = -1;
  } else if (simAngleY < -3.5f) {
    simAngleY = -3.5f;
    simDirectionY = 1;
  }
#endif

  simPressures[PAD_FRONT_LEFT] = 2.5f + (simAngleX < 0 ? -simAngleX * 0.5f : 0) + (simAngleY < 0 ? -simAngleY * 0.3f : 0);
  simPressures[PAD_FRONT_RIGHT] = 2.5f + (simAngleX > 0 ? simAngleX * 0.5f : 0) + (simAngleY < 0 ? -simAngleY * 0.3f : 0);
  simPressures[PAD_REAR_LEFT] = 3.0f + (simAngleX < 0 ? -simAngleX * 0.5f : 0) + (simAngleY > 0 ? simAngleY * 0.3f : 0);
  simPressures[PAD_REAR_RIGHT] = 3.0f + (simAngleX > 0 ? simAngleX * 0.5f : 0) + (simAngleY > 0 ? simAngleY * 0.3f : 0);

  for (int i = 0; i < PAD_COUNT; i++) {
    simPressures[i] = constrain(simPressures[i], 0.5f, 7.0f);
  }

  simMasterPressure = 4.5f + sin(now * 0.001f) * 0.3f;

#if SIMULATE_MENU_AUTO_ENTER
  // Используем переменные только если они объявлены
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    ErrorHandler::removeError(ErrorHandler::Error::LOW_PRESSURE);
    Serial.println("[SIM] Forced reset of LOW_PRESSURE error");
  }
#endif

#if SIMULATE_ERRORS
  static uint32_t lastErrorDebug = 0;
  static uint32_t lastErrorGenTime = 0;
  static uint32_t errorActiveTime = 0;
  static bool firstErrorScheduled = false;

  if (now - lastErrorDebug > 5000) {
    lastErrorDebug = now;
    Serial.printf("[SIM_ERROR] errorSimulated=%d, lastErrorGenTime=%d, currentState=%d, hasError=%d\n",
                  errorSimulated, lastErrorGenTime, (int)currentState,
                  ErrorHandler::hasActiveErrors());
  }

  if (currentState == SystemState::RUNNING && !firstErrorScheduled) {
    lastErrorGenTime = now;
    firstErrorScheduled = true;
    Serial.println("[SIM_ERROR] Ошибки начнутся через 20 секунд");
  }

  if (currentState == SystemState::RUNNING && !errorSimulated && firstErrorScheduled) {
    if (now - lastErrorGenTime > 20000) {
      lastErrorGenTime = now;
      errorSimulated = true;
      errorActiveTime = now;

      int errType = random(0, 3);
      switch (errType) {
        case 0:
          Serial.println("[SIM_ERROR] Симуляция: НИЗКОЕ ДАВЛЕНИЕ!");
          ErrorHandler::handleError(ErrorHandler::Error::LOW_PRESSURE,
                                    "Симуляция низкого давления");
          break;
        case 1:
          Serial.println("[SIM_ERROR] Симуляция: ОШИБКА MPU6050!");
          ErrorHandler::handleError(ErrorHandler::Error::MPU,
                                    "Симуляция ошибки MPU");
          break;
        case 2:
          Serial.println("[SIM_ERROR] Симуляция: WATCHDOG!");
          ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG,
                                    "Симуляция Watchdog");
          break;
      }
    }
  }

  if (errorSimulated && (now - errorActiveTime > 10000)) {
    errorSimulated = false;
    ErrorHandler::forceClearAllErrors();
    displayDirty = true;
    forceDisplayReset(true);
    Serial.println("[SIM_ERROR] Ошибка сброшена, возврат к главному экрану");
    lastErrorGenTime = now;
  }
#endif

  static uint32_t lastDebug = 0;
  if (now - lastDebug > 5000) {
    lastDebug = now;
    Serial.printf("[SIM] X=%.2f, Y=%.2f, Режим=%d, Ошибка=%d\n",
                  simAngleX, simAngleY, (int)currentSystemMode,
                  (int)ErrorHandler::getCurrentActiveError());
  }
}
#endif

// checkAndAdjustMasterPressure — see pressure_read.cpp

// detectMotionFromIMU — moved to imu_motion.cpp
// i2cBusRecover, i2cScanLog — moved to imu_dmp.cpp

// initializeDMP — moved to imu_dmp.cpp
#if 0  // === REMOVED ===
void _removed_initializeDMP() {
#if ENABLE_MPU6050
  MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(1500));
  if (!i2c && xI2CMutex != nullptr) {
    Serial.println("[MPU] I2C busy, откладываю инициализацию");
    return;
  }

  Logger::log(Logger::INFO, "MPU", "Инициализация");

  auto probe = [](uint8_t addr) -> bool {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  };

  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    i2cBusRecover();
    delay(40 * attempt);

    if (attempt == 1 || attempt == 5) {
      i2cScanLog(attempt == 1 ? "до MPU" : "после fail");
    }

    const bool a68 = probe(0x68);
    const bool a69 = probe(0x69);
    Serial.printf("[MPU] attempt %u: ACK 0x68=%d 0x69=%d heap=%u\n", attempt, a68 ? 1 : 0,
                  a69 ? 1 : 0, static_cast<unsigned>(ESP.getFreeHeap()));

    if (!a68 && !a69) {
      continue;
    }

    // AD0=HIGH > 0x69; иначе 0x68
    if (a69 && !a68) {
      new (&mpu) MPU6050(0x69);
    } else {
      new (&mpu) MPU6050(0x68);
    }

    mpu.initialize();
    delay(20);
    if (!mpu.testConnection()) {
      Serial.printf("[MPU] testConnection fail (attempt %u)\n", attempt);
      continue;
    }

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setSleepEnabled(false);
    mpu.setInterruptMode(false);

    // MOT/INT до dmpInitialize() сбрасываются прошивкой DMP — только после setDMPEnabled.
    const uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
      Logger::logf(Logger::ERROR, "MPU", "DMP error %d (attempt %u)", devStatus, attempt);
      Serial.printf("[MPU] DMP error code: %d\n", devStatus);
      continue;
    }

    mpu.setDMPEnabled(true);

    // Офсеты после DMP (dmpInitialize перезаписывает регистры).
    mpu.setXGyroOffset(ConfigManager::getImuGyroOffX());
    mpu.setYGyroOffset(ConfigManager::getImuGyroOffY());
    mpu.setZGyroOffset(ConfigManager::getImuGyroOffZ());
    mpu.setXAccelOffset(ConfigManager::getImuAccelOffX());
    mpu.setYAccelOffset(ConfigManager::getImuAccelOffY());
    mpu.setZAccelOffset(ConfigManager::getImuAccelOffZ());

    // Аппаратный MOT + FIFO + DMP_INT (0x52). HPF нужен, иначе MOT с DMP почти мёртв.
    // MOT_DUR=40 мс, counter decrement=1 — типичный гайд для стабильных импульсов.
    mpu.setDHPFMode(MPU6050_DHPF_1P25);
    mpu.setMotionDetectionThreshold((uint8_t)ConfigManager::getImuMotionDet());
    mpu.setMotionDetectionDuration(g_motDurMs);
    mpu.setAccelerometerPowerOnDelay(3);
    mpu.setMotionDetectionCounterDecrement(1);
    mpu.setZeroMotionDetectionThreshold(156);
    mpu.setZeroMotionDetectionDuration(0);
    mpu.setIntEnabled(0x52);  // MOT | FIFO_OFLOW | DMP_INT
    (void)mpu.getIntStatus();  // очистка latched INT

    Serial.printf("[MPU] DMP OK MotionApps 6.12 packet=%u | MOT_THR=%u MOT_DUR=%u INT_EN=0x%02X gyroOff=(%d,%d,%d)\n",
                  (unsigned)mpu.dmpGetFIFOPacketSize(),
                  (unsigned)ConfigManager::getImuMotionDet(),
                  (unsigned)g_motDurMs,
                  (unsigned)mpu.getIntEnabled(),
                  ConfigManager::getImuGyroOffX(), ConfigManager::getImuGyroOffY(),
                  ConfigManager::getImuGyroOffZ());

    mpuOk = true;
    ErrorHandler::removeError(ErrorHandler::Error::MPU);
    Wire.setClock(400000L);
    Logger::log(Logger::INFO, "MPU", "DMP готов");
    return;
  }

  Logger::log(Logger::ERROR, "MPU", "Ошибка подключения!");
  mpuOk = false;
  Wire.setClock(100000L);  // оставляем 100 кГц — надёжнее для ADS, пока MPU мёртв
  ErrorHandler::removeError(ErrorHandler::Error::MPU);
  displayDirty = true;
  Serial.println("[MPU] IMU отсутствует — углы/движение отключены, давление работает");

#else
  Serial.println("[MPU] MPU6050 отключен (тестовый режим)");
  mpuOk = false;
  angleX = 0;
  angleY = 0;
  temperature = 25.0;
#endif
}
#endif // === END REMOVED initializeDMP ===

// attitude gauges / updateMainTiltLive — see ui_screens.cpp

// maintainMovementPressure — see pressure_read.cpp

// AutoLevelingController — see auto_level.h

// sendValveCommandSync — see valve_ctrl.cpp




bool initFileSystem() {
  Serial.println("[FS] Инициализация LittleFS...");
  // maxOpenFiles=16: при утечке FD (корень каталога и т.п.) save() иначе падает
  // с «Unable to allocate FD» — после смены пресета MPU + СОХРАНИТЬ это давало ребут/обрыв MPU.
  bool mounted = LittleFS.begin(false, "/littlefs", 16, "spiffs");
  if (!mounted) {
    Serial.println("[FS] Раздел повреждён, выполняю однократное форматирование");
    LittleFS.end();
    mounted = LittleFS.begin(true, "/littlefs", 16, "spiffs");
    if (!mounted) {
      Serial.println("[FS] Не удалось восстановить LittleFS");
      return false;
    }
    Serial.println("[FS] LittleFS отформатирована и подключена");
  }
  Serial.println("[FS] LittleFS смонтирован успешно");

  File root = LittleFS.open("/");
  int fileCount = 0;
  if (root) {
    File file = root.openNextFile();
    while (file) {
      fileCount++;
      Serial.printf("[FS] Файл: %s (%d байт)\n", file.name(), file.size());
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }

  if (fileCount == 0 || !LittleFS.exists("/config.txt")) {
    Serial.println("[FS] Конфигурация отсутствует, создаю значения по умолчанию");
    if (!ConfigManager::save()) {
      Serial.println("[FS] Не удалось сохранить конфигурацию по умолчанию");
    }
  }

  return true;
}

bool loadConfig() {
  return ConfigManager::load();
}
bool saveConfig() {
  return ConfigManager::save();
}

// initializeJhm1200 — see pressure_read.cpp

bool loadWiFiConfig() {
  Logger::log(Logger::INFO, "WiFi", "Загрузка");
  File f = LittleFS.open("/wifi_config.txt", "r");
  if (!f) {
    Logger::log(Logger::INFO, "WiFi", "Нет файла – сохраняем дефолты");
    if (!saveWiFiConfig()) {
      Logger::log(Logger::ERROR, "WiFi", "Не удалось сохранить дефолты");
    }
    return false;
  }
  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка JSON");
    return false;
  }

  strlcpy(wifi_ssid, doc["ssid"] | "Kamaz-OTA-AP", sizeof(wifi_ssid));
  strlcpy(wifi_password, doc["password"] | wifi_password, sizeof(wifi_password));
  strlcpy(ota_password, doc["otaPassword"] | ota_password, sizeof(ota_password));
  strlcpy(sta_ssid, doc["staSsid"] | "", sizeof(sta_ssid));
  initializeDefaultCredentials();
  Logger::log(Logger::INFO, "WiFi", "Конфиг загружен");
  return true;
}

bool saveWiFiConfig() {
  Logger::log(Logger::INFO, "WiFi", "Сохранение");
  StaticJsonDocument<384> doc;
  doc["ssid"] = wifi_ssid;
  doc["password"] = wifi_password;
  doc["otaPassword"] = ota_password;
  doc["staSsid"] = sta_ssid;

  File f = LittleFS.open("/wifi_config.txt", "w");
  if (!f) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка записи");
    return false;
  }

  size_t bytesWritten = serializeJson(doc, f);
  f.close();

  if (bytesWritten == 0) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка сериализации JSON");
    LittleFS.remove("/wifi_config.txt");
    return false;
  }

  Logger::log(Logger::INFO, "WiFi", "Сохранено");
  return true;
}


/* WiFi setup -> wifi_setup.h/cpp */

void startValveTest() {
  if (currentTestStep != TestStep::IDLE && currentTestStep != TestStep::COMPLETED) {
    Logger::log(Logger::WARNING, "TEST", "Тест уже выполняется!");
    return;
  }

  if (currentSystemMode != SystemMode::MANUAL) {
    Logger::log(Logger::WARNING, "TEST", "Тест возможен только в РУЧНОМ режиме!");
    return;
  }

  if (ErrorHandler::hasActiveErrors()) {
    Logger::log(Logger::WARNING, "TEST", "Невозможно запустить тест при наличии ошибок!");
    return;
  }

  if (manualControlActive) {
    stopManualOperation();
  }

  forceDisplayReset(true);
  closeAllValves();

  memset(&testResult, 0, sizeof(testResult));
  testStartTime = millis();
  waitingForUser = false;

  currentTestStep = TestStep::PREPARE_CHECK_SUPPLY;
  currentTestState = TestState::STARTING;
  testStepStartTime = millis();

  Serial.println("\nг============================================================¬");
  Serial.println("¦              ТЕСТ КЛАПАНОВ ПНЕВМОСИСТЕМЫ                   ¦");
  Serial.println("L============================================================-");

  Logger::log(Logger::INFO, "TEST", "Запуск теста клапанов");
}

void runValveTestLogic() {
  // ========== ЗАХВАТ МЬЮТЕКСА ==========
  MutexGuard guard(xTestMutex, pdMS_TO_TICKS(100));
  if (!guard) {
    Serial.println("[TEST] Failed to lock test mutex!");
    return;
  }

  if (ErrorHandler::hasActiveErrors()) {
    if (currentTestStep != TestStep::IDLE && currentTestStep != TestStep::COMPLETED) {
      Serial.println("[ТЕСТ] ? Аварийная остановка теста!");
      currentTestStep = TestStep::COMPLETED;
      currentTestState = TestState::COMPLETED;
      closeAllValves();
      waitingForUser = false;
    }
    return;
  }

  if (currentTestStep == TestStep::IDLE || currentTestStep == TestStep::COMPLETED) {
    return;
  }

  uint32_t currentTime = millis();

  if ((currentTime - testStartTime) > TEST_TIMEOUT_MS && !waitingForUser) {
    Serial.println("[ТЕСТ] ? Таймаут теста! Принудительное завершение.");
    currentTestStep = TestStep::COMPLETED;
    currentTestState = TestState::COMPLETED;
    closeAllValves();
    waitingForUser = false;
    return;
  }

  float currentPressure = 0;
  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      currentPressure = masterPressure;
    }
  }

  static bool stepInitialized = false;

  switch (currentTestStep) {
    case TestStep::PREPARE_CHECK_SUPPLY:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.println("\n[ТЕСТ] ШАГ 1/4: ПОДГОТОВКА К ТЕСТУ");
        Serial.println("[ТЕСТ] Проверка давления в магистрали подачи...");
        Serial.println("[ТЕСТ] Открываю клапан НАКАЧКИ для проверки входного давления");
        digitalWrite(PIN_INFL, HIGH);
      }

      if ((currentTime - testStepStartTime) >= TEST_STABILIZE_TIME_MS) {
        testResult.supplyPressure = currentPressure;
        Serial.printf("[ТЕСТ] Давление в магистрали подачи: %.1f бар\n", testResult.supplyPressure);

        if (testResult.supplyPressure >= TEST_MIN_PRESS_FOR_TEST) {
          testResult.supplyPressureOk = true;
          Serial.printf("[ТЕСТ] ? Давление достаточное (>= %.1f бар). Тест возможен.\n",
                        TEST_MIN_PRESS_FOR_TEST);
          digitalWrite(PIN_INFL, LOW);
          Serial.println("[ТЕСТ] Клапан накачки закрыт");

          currentTestStep = TestStep::PREPARE_EQUALIZE_PADS;
          currentTestState = TestState::TESTING_PAD;
          testStepStartTime = currentTime;
          stepInitialized = false;
        } else {
          testResult.supplyPressureOk = false;
          Serial.printf("[ТЕСТ] ? Давление недостаточное (%.1f < %.1f бар)\n",
                        testResult.supplyPressure, TEST_MIN_PRESS_FOR_TEST);
          Serial.println("[ТЕСТ] ?? НЕОБХОДИМО: Включить компрессор или завести двигатель");
          Serial.println("[ТЕСТ] Ожидание повышения давления в магистрали...");

          currentTestStep = TestStep::PREPARE_WAIT_PRESSURIZE;
          testStepStartTime = currentTime;
          waitingForUser = true;
          stepInitialized = false;
        }
      }
      break;

    case TestStep::PREPARE_WAIT_PRESSURIZE:
      {
        static uint32_t lastNotifyTime = 0;

        if ((currentTime - testStepStartTime) >= 2000) {
          if (currentPressure >= TEST_MIN_PRESS_FOR_TEST) {
            Serial.printf("[ТЕСТ] ? Давление поднялось до %.1f бар! Тест возможен.\n", currentPressure);
            testResult.supplyPressure = currentPressure;
            testResult.supplyPressureOk = true;
            digitalWrite(PIN_INFL, LOW);
            Serial.println("[ТЕСТ] Клапан накачки закрыт");

            currentTestStep = TestStep::PREPARE_EQUALIZE_PADS;
            currentTestState = TestState::TESTING_PAD;
            testStepStartTime = currentTime;
            waitingForUser = false;
            lastNotifyTime = 0;
          } else if ((currentTime - lastNotifyTime) >= 5000) {
            lastNotifyTime = currentTime;
            Serial.printf("[ТЕСТ] Ожидание давления... Текущее: %.1f бар (нужно >= %.1f)\n",
                          currentPressure, TEST_MIN_PRESS_FOR_TEST);
          }
          testStepStartTime = currentTime;
        }
      }
      break;

    case TestStep::PREPARE_EQUALIZE_PADS:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.println("\n[ТЕСТ] ШАГ 2/4: ВЫРАВНИВАНИЕ ДАВЛЕНИЯ В ПОДУШКАХ");
        Serial.println("[ТЕСТ] Открываю ВСЕ клапаны подушек для выравнивания давления...");

        for (int i = 0; i < PAD_COUNT; i++) {
          setValve(Pad(i), HIGH);
        }
        Serial.println("[ТЕСТ] Клапаны ПЛ, ПП, ЗЛ, ЗП - ОТКРЫТЫ");
      }

      if ((currentTime - testStepStartTime) >= TEST_PRESSURE_EQUALIZE_TIME_MS) {
        testResult.equalizedPressure = currentPressure;
        Serial.printf("[ТЕСТ] Давление после выравнивания: %.1f бар\n", testResult.equalizedPressure);

        for (int i = 0; i < PAD_COUNT; i++) {
          setValve(Pad(i), LOW);
        }
        Serial.println("[ТЕСТ] Клапаны подушек ЗАКРЫТЫ");

        testReferencePressure = testResult.equalizedPressure;

        currentTestStep = TestStep::TEST_DEFLATE_VALVE;
        testStepStartTime = currentTime;
        stepInitialized = false;
      }
      break;

    case TestStep::TEST_DEFLATE_VALVE:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.println("\n[ТЕСТ] ШАГ 3/4: ТЕСТИРОВАНИЕ КЛАПАНОВ");
        Serial.println("[ТЕСТ] Тест клапана СБРОСА...");
        Serial.println("[ТЕСТ] Открываю клапан сброса");
        digitalWrite(PIN_DEFL, HIGH);
      }

      if ((currentTime - testStepStartTime) >= TEST_VALVE_OPEN_TIME_MS) {
        Serial.printf("[ТЕСТ] Давление после открытия сброса: %.1f бар\n", currentPressure);

        testResult.deflateValveWorks = (currentPressure <= TEST_ZERO_THRESHOLD);
        Serial.printf("[ТЕСТ] %s Клапан СБРОСА %s\n",
                      testResult.deflateValveWorks ? "?" : "?",
                      testResult.deflateValveWorks ? "РАБОТАЕТ" : "НЕ РАБОТАЕТ");

        digitalWrite(PIN_DEFL, LOW);
        Serial.println("[ТЕСТ] Клапан сброса закрыт");

        vTaskDelay(pdMS_TO_TICKS(TEST_STABILIZE_TIME_MS));

        currentTestStep = TestStep::TEST_INFLATE_VALVE;
        testStepStartTime = millis();
        stepInitialized = false;
      }
      break;

    case TestStep::TEST_INFLATE_VALVE:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.println("\n[ТЕСТ] Тест клапана НАКАЧКИ...");
        Serial.println("[ТЕСТ] Открываю клапан накачки");
        digitalWrite(PIN_INFL, HIGH);
      }

      if ((currentTime - testStepStartTime) >= TEST_VALVE_OPEN_TIME_MS) {
        Serial.printf("[ТЕСТ] Давление после открытия накачки: %.1f бар\n", currentPressure);

        testResult.inflateValveWorks = (currentPressure > TEST_ZERO_THRESHOLD);
        Serial.printf("[ТЕСТ] %s Клапан НАКАЧКИ %s\n",
                      testResult.inflateValveWorks ? "?" : "?",
                      testResult.inflateValveWorks ? "РАБОТАЕТ" : "НЕ РАБОТАЕТ");

        digitalWrite(PIN_INFL, LOW);
        Serial.println("[ТЕСТ] Клапан накачки закрыт");

        testReferencePressure = currentPressure;

        currentTestStep = TestStep::TEST_PAD_VALVE_RESET;
        testStepStartTime = currentTime;
        testPadIndex = 0;
        stepInitialized = false;
      }
      break;

    case TestStep::TEST_PAD_VALVE_RESET:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.println("\n[ТЕСТ] ШАГ 4/4: ТЕСТ КЛАПАНОВ ПОДУШЕК");
        Serial.printf("[ТЕСТ] ПОДГОТОВКА К ТЕСТУ ПОДУШКИ %s (%d/4)\n",
                      padNames[testPadIndex], testPadIndex + 1);
        Serial.println("[ТЕСТ] Открываю клапан СБРОСА...");
        digitalWrite(PIN_DEFL, HIGH);
      }

      if ((currentTime - testStepStartTime) >= TEST_VALVE_OPEN_TIME_MS) {
        digitalWrite(PIN_DEFL, LOW);
        Serial.printf("[ТЕСТ] Давление после сброса: %.1f бар\n", currentPressure);

        if (currentPressure <= TEST_ZERO_THRESHOLD) {
          Serial.printf("[ТЕСТ] ? Давление в норме (? %.1f бар)\n", TEST_ZERO_THRESHOLD);
          testReferencePressure = currentPressure;
        } else {
          Serial.printf("[ТЕСТ] ? ОШИБКА: Давление слишком высокое (%.1f > %.1f бар)\n",
                        currentPressure, TEST_ZERO_THRESHOLD);
          Serial.println("[ТЕСТ] Проверьте работу клапана СБРОСА!");
          Serial.println("[ТЕСТ] Тест прерван.");

          currentTestStep = TestStep::COMPLETED;
          currentTestState = TestState::COMPLETED;
          closeAllValves();
          waitingForUser = false;
          return;
        }

        currentTestStep = TestStep::TEST_PAD_VALVE_OPEN;
        testStepStartTime = currentTime;
        stepInitialized = false;
      }
      break;

    case TestStep::TEST_PAD_VALVE_OPEN:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.printf("[ТЕСТ] Открываю клапан ПОДУШКИ %s...\n", padNames[testPadIndex]);
        setValve(Pad(testPadIndex), HIGH);
      }

      if ((currentTime - testStepStartTime) >= TEST_VALVE_OPEN_TIME_MS) {
        float pressureDiff = currentPressure - testReferencePressure;
        Serial.printf("[ТЕСТ] Давление после открытия: %.1f бар (изменение: %+.1f)\n",
                      currentPressure, pressureDiff);

        if (pressureDiff >= TEST_PRESSURE_CHANGE_THRESHOLD) {
          testResult.padValvesWorks[testPadIndex] = true;
          Serial.printf("[ТЕСТ] ? Клапан %s РАБОТАЕТ (давление поднялось на %.1f бар)\n",
                        padNames[testPadIndex], pressureDiff);
        } else {
          testResult.padValvesWorks[testPadIndex] = false;
          Serial.printf("[ТЕСТ] ? Клапан %s НЕ РАБОТАЕТ (изменение %.1f < %.1f бар)\n",
                        padNames[testPadIndex], pressureDiff, TEST_PRESSURE_CHANGE_THRESHOLD);
        }

        testResult.pressureReadings[testPadIndex][0] = testReferencePressure;
        testResult.pressureReadings[testPadIndex][1] = currentPressure;

        currentTestStep = TestStep::TEST_PAD_VALVE_CLOSE;
        testStepStartTime = currentTime;
        stepInitialized = false;
      }
      break;

    case TestStep::TEST_PAD_VALVE_CLOSE:
      if (!stepInitialized) {
        stepInitialized = true;
        Serial.printf("[ТЕСТ] Закрываю клапан %s\n", padNames[testPadIndex]);
        setValve(Pad(testPadIndex), LOW);
      }

      if ((currentTime - testStepStartTime) >= TEST_STABILIZE_TIME_MS) {
        testPadIndex++;

        if (testPadIndex >= PAD_COUNT) {
          currentTestStep = TestStep::COMPLETED;
          currentTestState = TestState::COMPLETED;
          printTestResults();

          Event event;
          event.type = EventType::TEST_STATE_CHANGE;
          event.timestamp = millis();
          event.data.test.testState = static_cast<uint8_t>(TestState::COMPLETED);
          EventBus::publish(event);
        } else {
          currentTestStep = TestStep::TEST_PAD_VALVE_RESET;
          testStepStartTime = currentTime;
          stepInitialized = false;
        }
      }
      break;

    case TestStep::COMPLETED:
      closeAllValves();
      waitingForUser = false;
      if (testResult.inflateValveWorks && testResult.deflateValveWorks &&
          ErrorHandler::isErrorActive(ErrorHandler::Error::VALVE)) {
        ErrorHandler::markErrorCleared(ErrorHandler::Error::VALVE);
        Serial.println("[ТЕСТ] VALVE ошибка будет снята после успешного теста");
      }
      break;

    default:
      break;
  }
}

void printTestResults() {
  Serial.println("\nг============================================================¬");
  Serial.println("¦                 РЕЗУЛЬТАТЫ ТЕСТА КЛАПАНОВ                 ¦");
  Serial.println("L============================================================-");

  Serial.println("\n?? ПОДГОТОВКА:");
  Serial.printf("   Давление в магистрали: %.1f бар %s\n",
                testResult.supplyPressure,
                testResult.supplyPressureOk ? "?" : "?");
  Serial.printf("   Давление после выравнивания: %.1f бар\n", testResult.equalizedPressure);

  Serial.println("\n?? ОБЩИЕ КЛАПАНЫ:");
  Serial.printf("   Клапан НАКАЧКИ: %s\n",
                testResult.inflateValveWorks ? "? РАБОТАЕТ" : "? НЕ РАБОТАЕТ");
  Serial.printf("   Клапан СБРОСА:  %s\n",
                testResult.deflateValveWorks ? "? РАБОТАЕТ" : "? НЕ РАБОТАЕТ");

  Serial.println("\n?? КЛАПАНЫ ПОДУШЕК:");
  for (int i = 0; i < PAD_COUNT; i++) {
    Serial.printf("   %s: %s  (?%+.1f бар)\n",
                  padNames[i],
                  testResult.padValvesWorks[i] ? "? РАБОТАЕТ" : "? НЕ РАБОТАЕТ",
                  testResult.pressureReadings[i][1] - testResult.pressureReadings[i][0]);
  }

  bool allOk = testResult.supplyPressureOk && testResult.inflateValveWorks && testResult.deflateValveWorks;
  for (int i = 0; i < PAD_COUNT; i++) {
    if (!testResult.padValvesWorks[i]) allOk = false;
  }

  Serial.println("\n============================================================");
  if (allOk) {
    Serial.println("? ВСЕ КЛАПАНЫ РАБОТАЮТ КОРРЕКТНО!");
  } else {
    Serial.println("?? ОБНАРУЖЕНЫ НЕИСПРАВНОСТИ КЛАПАНОВ!");
    if (!testResult.supplyPressureOk) {
      Serial.println("   - Нет давления в магистрали подачи");
    }
    if (!testResult.inflateValveWorks) {
      Serial.println("   - Не работает клапан НАКАЧКИ");
    }
    if (!testResult.deflateValveWorks) {
      Serial.println("   - Не работает клапан СБРОСА");
    }
    for (int i = 0; i < PAD_COUNT; i++) {
      if (!testResult.padValvesWorks[i]) {
        Serial.printf("   - Не работает клапан %s\n", padNames[i]);
      }
    }
  }
  Serial.println("============================================================\n");
}

// readPressure — see pressure_read.cpp

// uiInfoRow / uiHeader / uiHintBar — see ui_screens.cpp

void displayOTAScreen() {
  if (errorScreenBlocking || ErrorHandler::hasUiBlockingErrors()) {
    return;
  }

  static bool initialized = false;
  static bool lastInstallMode = false;
  static int lastProgress = -1;
  static uint8_t lastDots = 0;
  static uint32_t lastDotUpdate = 0;
  static char lastOtaStatus[32] = "";
  static uint32_t otaWaitStart = 0;

  if (otaUiNeedFullRedraw) {
    otaUiNeedFullRedraw = false;
    initialized = false;
    lastProgress = -1;
    lastOtaStatus[0] = '\0';
  }

  // Экран обслуживает два случая: режим ArduinoOTA (AP) и установку релиза
  // с GitHub. При смене режима перерисовываем шапку целиком.
  if (lastInstallMode != otaInProgress) {
    lastInstallMode = otaInProgress;
    initialized = false;
    lastProgress = -1;
    lastOtaStatus[0] = '\0';
  }

  if (!otaInProgress && initialized) {
    if (otaWaitStart == 0) otaWaitStart = millis();
    if (millis() - otaWaitStart > 120000) {
      stopOTAMode();
      return;
    }
  } else {
    otaWaitStart = 0;
  }

  constexpr int16_t BAR_X = theme::MARGIN + 4;
  constexpr int16_t BAR_Y = 108;
  constexpr int16_t BAR_W = theme::SCREEN_W - 2 * (theme::MARGIN + 4);
  constexpr int16_t BAR_H = 18;

  if (!initialized) {
    tft.fillScreen(theme::BG);
    uiHeader(otaInProgress ? "ОБНОВЛЕНИЕ" : "РЕЖИМ OTA", theme::ACCENT);

    if (otaInProgress) {
      uiInfoRow(44, "Установлено:", VERSION);
      uiInfoRow(62, "Источник:", "GitHub / ArduinoOTA");
    } else {
      uiInfoRow(44, "Сеть:", wifi_ssid);
      uiInfoRow(62, "Пароль:", "********");
      String apIp = WiFi.softAPIP().toString();
      uiInfoRow(80, "IP:", apIp.c_str());
    }

    drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, 0, theme::OK);
    uiHintBar(otaInProgress ? "Не выключайте питание" : "УДЕРЖ. КН3+КН4 — МЕНЮ", theme::TEXT_DIM);

    initialized = true;
  }

  if (otaInProgress) {
    if (otaProgress != lastProgress) {
      drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, static_cast<uint8_t>(otaProgress), theme::OK);

      ui.setTransparent(true);
      ui.setFont(UiFont::Large);
      ui.setColors(theme::TEXT, theme::BG);
      ui.boxf(0, BAR_Y + BAR_H + 4, theme::SCREEN_W, 34, UiHAlign::Center, UiVAlign::Middle, true,
              "%d%%", static_cast<int>(otaProgress));

      lastProgress = otaProgress;
    }

    if (strcmp(otaStatus, lastOtaStatus) != 0) {
      ui.setTransparent(true);
      ui.setFont(UiFont::Small);
      ui.setColors(theme::TEXT_DIM, theme::BG);
      ui.box(0, 168, theme::SCREEN_W, 18, otaStatus, UiHAlign::Center, UiVAlign::Middle, true);

      strncpy(lastOtaStatus, otaStatus, sizeof(lastOtaStatus) - 1);
      lastOtaStatus[sizeof(lastOtaStatus) - 1] = '\0';
    }
  } else {
    uint32_t now = millis();
    if (now - lastDotUpdate > 500) {
      lastDotUpdate = now;

      lastDots = (lastDots + 1) % 4;
      char dots[4] = "";
      for (uint8_t i = 0; i < lastDots; i++) dots[i] = '.';
      dots[lastDots] = '\0';

      char waitBuf[24];
      snprintf(waitBuf, sizeof(waitBuf), "ОЖИДАНИЕ%s", dots);

      ui.setTransparent(true);
      ui.setFont(UiFont::Small);
      ui.setColors(theme::WARN, theme::BG);
      ui.box(0, 168, theme::SCREEN_W, 18, waitBuf, UiHAlign::Center, UiVAlign::Middle, true);
    }
  }
}

void forceDisplayReset(bool force) {
  // Пока открыто меню или экран Wi-Fi — не трогаем дисплей.
  if (menuVisible || wifiSetupActive || wifiScanInProgress) {
    return;
  }

  static uint32_t lastReset = 0;
  if (millis() - lastReset < 500 && !force) {
    return;
  }
  // Даже с force не чаще 300 мс — защита от спама emergency.release / prune
  if (millis() - lastReset < 300) {
    displayDirty = true;
    return;
  }
  lastReset = millis();

  displayDirty = true;
  g_displayEpoch++;  // инвалидация кэшей displayMainScreen / авиагоризонта
  tft.fillScreen(theme::BG);

  lastDisplayedIMU.angleX = 999;
  lastDisplayedIMU.angleY = 999;
  lastDisplayedIMU.temperature = 999;
  lastDisplayedPressure.masterPressure = 999;
  for (int i = 0; i < PAD_COUNT; i++) {
    lastDisplayedPressure.pressure[i] = 999;
  }
  lastDisplayedMode = SystemMode::MOVEMENT;
  lastDisplayedMoving = !lastDisplayedMoving;

#if ENABLE_SIMULATION
  menuAutoEnterTime = 0;
  menuAutoEnterDone = false;
  errorSimulated = false;
  simulatedError = ErrorHandler::Error::NONE;
#endif

  forceErrorScreenRedraw = true;

  Serial.println("[DISPLAY] Force reset completed");
}

// displayErrorScreen / displayCalibrationScreen — see ui_screens.cpp

void drawProgressBar(int16_t x, int16_t y, int16_t width, int16_t height, uint8_t percent, uint16_t color) {
  if (percent > 100) percent = 100;
  tft.fillRoundRect(x, y, width, height, 4, theme::TRACK);
  int16_t fillWidth = static_cast<int16_t>((width - 4) * percent / 100);
  if (fillWidth > 0) {
    tft.fillRoundRect(x + 2, y + 2, fillWidth, height - 4, 2, color);
  }
  tft.drawRoundRect(x, y, width, height, 4, theme::BORDER);
}

void drawIcon(int16_t x, int16_t y, const unsigned char *icon, uint16_t color) {
  tft.drawBitmap(x, y, icon, 32, 32, color);
}

void drawIconL(int16_t x, int16_t y, const unsigned char *icon, uint16_t color) {
  tft.drawBitmap(x, y, icon, 20, 20, color);
}

void drawIconB(int16_t x, int16_t y, const unsigned char *icon, uint16_t color) {
  tft.drawBitmap(x, y, icon, 45, 45, color);
}

// updateLeakMonitor — see pressure_read.cpp

// main screen — see ui_screens.cpp

void updateTestDisplay() {
  if (errorScreenBlocking || ErrorHandler::hasUiBlockingErrors()) {
    return;
  }

  static bool initialized = false;
  static uint32_t lastUpdate = 0;

  if (millis() - lastUpdate < 100 && currentTestStep != TestStep::COMPLETED) {
    return;
  }
  lastUpdate = millis();

  constexpr int16_t HDR_Y = theme::MARGIN;
  constexpr int16_t HDR_H = 26;
  constexpr int16_t BAR_X = theme::MARGIN;
  constexpr int16_t BAR_Y = 96;
  constexpr int16_t BAR_W = theme::SCREEN_W - 2 * theme::MARGIN;
  constexpr int16_t BAR_H = 16;

  if (!initialized && currentTestStep != TestStep::COMPLETED && currentTestStep != TestStep::IDLE) {
    tft.fillScreen(theme::BG);
    tft.fillRoundRect(theme::MARGIN, HDR_Y, theme::SCREEN_W - 2 * theme::MARGIN, HDR_H,
                      theme::RADIUS, theme::PANEL_ALT);
    ui.setTransparent(true);
    ui.setFont(UiFont::Med);
    ui.setColors(COLOR_OTA, theme::PANEL_ALT);
    ui.box(theme::MARGIN, HDR_Y, theme::SCREEN_W - 2 * theme::MARGIN, HDR_H, "ТЕСТ КЛАПАНОВ",
           UiHAlign::Center, UiVAlign::Middle, false);
    uiHintBar("КН4 — стоп теста | удерж. КН3+КН4 — меню", theme::WARN);
    initialized = true;
  }

  float currentPressure;
  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      currentPressure = masterPressure;
    }
  }

  if (currentTestStep == TestStep::COMPLETED) {
    static bool resultsShown = false;
    if (!resultsShown) {
      resultsShown = true;
      renderTestResults();
    }
    return;
  }

  /* ------------------------ текущий шаг теста ------------------------ */
  static char stepTitlePrev[40] = "";
  static char stepNotePrev[48] = "";
  static char stepPressPrev[24] = "";
  static char stepProgPrev[24] = "";
  static char stepValvePrev[24] = "";
  static TestStep lastStepDrawn = TestStep::IDLE;
  if (lastStepDrawn != currentTestStep) {
    stepTitlePrev[0] = stepNotePrev[0] = '\0';
    lastStepDrawn = currentTestStep;
  }

  const char *stepTitle = "";
  char stepNote[48] = "";

  switch (currentTestStep) {
    case TestStep::PREPARE_CHECK_SUPPLY:
      stepTitle = "ПРОВЕРКА МАГИСТРАЛИ";
      break;
    case TestStep::PREPARE_WAIT_PRESSURIZE:
      stepTitle = "ОЖИДАНИЕ ДАВЛЕНИЯ";
      snprintf(stepNote, sizeof(stepNote), "Включите компрессор!");
      break;
    case TestStep::PREPARE_EQUALIZE_PADS:
      stepTitle = "ВЫРАВНИВАНИЕ ДАВЛЕНИЯ";
      break;
    case TestStep::TEST_DEFLATE_VALVE:
      stepTitle = "ТЕСТ: КЛАПАН СБРОСА";
      break;
    case TestStep::TEST_INFLATE_VALVE:
      stepTitle = "ТЕСТ: КЛАПАН НАКАЧКИ";
      break;
    case TestStep::TEST_PAD_VALVE_RESET:
      stepTitle = "ПОДГОТОВКА: СБРОС";
      snprintf(stepNote, sizeof(stepNote), "Подушка: %s (%d/4)", padNames[testPadIndex], testPadIndex + 1);
      break;
    case TestStep::TEST_PAD_VALVE_OPEN:
      stepTitle = "ОТКРЫТИЕ КЛАПАНА";
      snprintf(stepNote, sizeof(stepNote), "Подушка: %s (%d/4)", padNames[testPadIndex], testPadIndex + 1);
      break;
    case TestStep::TEST_PAD_VALVE_CLOSE:
      stepTitle = "ЗАКРЫТИЕ КЛАПАНА";
      snprintf(stepNote, sizeof(stepNote), "Подушка: %s (%d/4)", padNames[testPadIndex], testPadIndex + 1);
      break;
    default:
      break;
  }

  uiRedrawLabel(theme::MARGIN, 38, theme::SCREEN_W - 2 * theme::MARGIN, 20, stepTitlePrev,
                sizeof(stepTitlePrev), stepTitle, theme::TEXT, theme::BG, UiFont::SmallB,
                UiHAlign::Left);
  uiRedrawLabel(theme::MARGIN, 60, theme::SCREEN_W - 170, 18, stepNotePrev, sizeof(stepNotePrev),
                stepNote, stepNote[0] ? theme::WARN : theme::BG, theme::BG, UiFont::Small,
                UiHAlign::Left);

  char pbuf[24];
  snprintf(pbuf, sizeof(pbuf), "МП: %.1f бар", currentPressure);
  uiRedrawLabel(theme::SCREEN_W - theme::MARGIN - 150, 60, 150, 18, stepPressPrev,
                sizeof(stepPressPrev), pbuf, theme::TEXT, theme::BG, UiFont::Small, UiHAlign::Right);

  /* ----------------------------- прогресс ---------------------------- */
  uint32_t elapsed = millis() - testStartTime;
  uint8_t progress = (elapsed * 100) / TEST_TIMEOUT_MS;
  if (progress > 100) progress = 100;

  drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, progress, theme::ACCENT);

  char progBuf[24];
  snprintf(progBuf, sizeof(progBuf), "Прогресс: %d%%", progress);
  uiRedrawLabel(BAR_X, BAR_Y + BAR_H + 4, BAR_W, 18, stepProgPrev, sizeof(stepProgPrev), progBuf,
                theme::TEXT_DIM, theme::BG, UiFont::Small, UiHAlign::Left);

  /* ------------------------- состояние клапана ----------------------- */
  const uint32_t stepElapsed = millis() - testStepStartTime;
  const bool valveOpen = (stepElapsed < TEST_VALVE_OPEN_TIME_MS) &&
                         (currentTestStep != TestStep::PREPARE_WAIT_PRESSURIZE);
  uiRedrawLabel(theme::MARGIN, 146, theme::SCREEN_W - 2 * theme::MARGIN, 20, stepValvePrev,
                sizeof(stepValvePrev), valveOpen ? "КЛАПАН ОТКРЫТ" : "КЛАПАН ЗАКРЫТ",
                valveOpen ? theme::WARN : theme::TEXT_DIM, theme::BG, UiFont::SmallB,
                UiHAlign::Left);
}

/** Экран результатов теста клапанов (вызывается из updateTestDisplay). */
static void renderTestResults() {
  tft.fillScreen(theme::BG);
  ui.setTransparent(true);

  ui.setFont(UiFont::SmallB);
  ui.setColors(theme::ACCENT, theme::BG);
  ui.box(theme::MARGIN, theme::MARGIN, theme::SCREEN_W - 2 * theme::MARGIN, 20, "РЕЗУЛЬТАТЫ ТЕСТА",
         UiHAlign::Left, UiVAlign::Middle, false);
  tft.drawFastHLine(theme::MARGIN, theme::MARGIN + 22, theme::SCREEN_W - 2 * theme::MARGIN, theme::BORDER);

  int16_t y = 34;

  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN + 2, y, 92, 18, "Магистраль:", UiHAlign::Left, UiVAlign::Middle, false);
  ui.setColors(testResult.supplyPressureOk ? theme::OK : theme::ERR, theme::BG);
  ui.boxf(theme::MARGIN + 96, y, theme::SCREEN_W - 2 * theme::MARGIN - 96, 18, UiHAlign::Left,
          UiVAlign::Middle, false, "%.1f бар   %s", testResult.supplyPressure,
          testResult.supplyPressureOk ? "OK" : "НЕТ");
  y += 20;

  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN + 2, y, 92, 18, "Клапаны:", UiHAlign::Left, UiVAlign::Middle, false);
  ui.setColors(testResult.inflateValveWorks ? theme::OK : theme::ERR, theme::BG);
  ui.boxf(theme::MARGIN + 96, y, 96, 18, UiHAlign::Left, UiVAlign::Middle, false, "НАКАЧ: %s",
          testResult.inflateValveWorks ? "OK" : "НЕТ");
  ui.setColors(testResult.deflateValveWorks ? theme::OK : theme::ERR, theme::BG);
  ui.boxf(theme::MARGIN + 196, y, theme::SCREEN_W - 2 * theme::MARGIN - 196, 18, UiHAlign::Left,
          UiVAlign::Middle, false, "СБРОС: %s", testResult.deflateValveWorks ? "OK" : "НЕТ");
  y += 24;

  for (int i = 0; i < PAD_COUNT; i++) {
    ui.setFont(UiFont::SmallB);
    ui.setColors(theme::TEXT, theme::BG);
    ui.box(theme::MARGIN + 2, y, 34, 18, padNames[i], UiHAlign::Left, UiVAlign::Middle, false);

    ui.setFont(UiFont::Small);
    ui.setColors(testResult.padValvesWorks[i] ? theme::OK : theme::ERR, theme::BG);
    ui.box(theme::MARGIN + 40, y, 122, 18,
           testResult.padValvesWorks[i] ? "РАБОТАЕТ" : "НЕ РАБОТАЕТ",
           UiHAlign::Left, UiVAlign::Middle, false);

    ui.setColors(testResult.padValvesWorks[i] ? theme::TEXT : theme::TEXT_DIM, theme::BG);
    ui.boxf(theme::SCREEN_W - theme::MARGIN - 62, y, 62, 18, UiHAlign::Right, UiVAlign::Middle, false,
            "%+.1f", testResult.pressureReadings[i][1] - testResult.pressureReadings[i][0]);

    y += 18;
  }

  ui.setFont(UiFont::SmallB);
  ui.setColors(theme::WARN, theme::BG);
  ui.box(0, theme::SCREEN_H - 22, theme::SCREEN_W, 20, "УДЕРЖ. КН3+КН4 — МЕНЮ",
         UiHAlign::Center, UiVAlign::Middle, false);
}

void resetSystemErrors() {
    errorScreenBlocking = false;

    int count = ErrorHandler::getActiveErrorCount();

    if (count == 0) {
        forceDisplayReset(true);
        displayDirty = true;
        Serial.println("[SYSTEM] Ошибок нет, обновляем экран");
        return;
    }

    bool anyMarked = false;

    for (int i = 0; i < count; i++) {
        ErrorHandler::Error err = ErrorHandler::getActiveErrorAt(i);
        bool shouldClear = true;

        switch (err) {
            case ErrorHandler::Error::SENSOR:
                {
#if !ENABLE_SIMULATION
                    float bar = 0.0f;
                    bool ok = false;
                    {
                      MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
                      if (i2c) ok = Jhm1200::readBar(bar);
                    }
                    if (!ok) {
                        shouldClear = false;
                        Serial.println("[SYSTEM] SENSOR сохранена (JHM1200 не отвечает)");
                    } else {
                        Serial.printf("[SYSTEM] SENSOR будет удалена (JHM1200 ok, %.2f бар)\n", bar);
                    }
#else
                    shouldClear = true;
                    Serial.println("[SYSTEM] SENSOR будет удалена (режим симуляции)");
#endif
                    break;
                }

            case ErrorHandler::Error::LOW_PRESSURE:
                {
                    float localMaster;
                    {
                        MutexGuard guard(xStateMutex);
                        if (!guard) {
                            shouldClear = false;
                            break;
                        }
                        localMaster = masterPressure;
                    }
                    float masterLow = ConfigManager::getMasterLowBar();

                    // порог 0 = проверка выкл. — ошибку можно снять
                    if (masterLow > 0.0f && localMaster < masterLow) {
                        shouldClear = false;
                        Serial.printf("[SYSTEM] LOW_PRESSURE сохранена (МП %.2f < %.2f)\n",
                                      localMaster, masterLow);
                    } else {
                        Serial.printf("[SYSTEM] LOW_PRESSURE будет удалена (МП %.2f, порог %.2f)\n",
                                      localMaster, masterLow);
                    }
                    break;
                }

            // Остальные ошибки удаляем всегда
            default:
                Serial.printf("[SYSTEM] Ошибка %d будет удалена\n", (int)err);
                break;
        }

        if (shouldClear) {
            ErrorHandler::markErrorCleared(err);
            anyMarked = true;
            Serial.printf("[SYSTEM] Ошибка %d помечена на удаление\n", (int)err);
        }
    }

    if (!anyMarked) {
        Serial.println("[SYSTEM] Ни одна ошибка не может быть удалена");
    }

    forceDisplayReset(true);
    displayDirty = true;
    Serial.println("[SYSTEM] Сброс ошибок инициирован");
}

// checkPressureLimits — see pressure_read.cpp

// startManualOperation / stopManualOperation — see valve_ctrl.cpp

void eventHandlerTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  Event event;

  extern uint8_t taskIndex_Event;

  for (;;) {
    TaskPool::markRun(taskIndex_Event);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_EVENT);
    if (EventBus::receive(event, pdMS_TO_TICKS(10))) {
      switch (event.type) {
        case EventType::ERROR_OCCURRED:
        case EventType::MODE_CHANGE:
        case EventType::DISPLAY_UPDATE:
          break;
        case EventType::CALIBRATION_DONE:
          Logger::log(Logger::INFO, "EVENT", "Калибровка завершена");
          break;
        default:
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// valveTask — see task_valve.cpp

// controlTask — see task_control.cpp

/** Открыть меню. Вызывать из DisplayTask (под xDisplayMutex) или только флаги. */
void openMenu() {
  if (menuVisible) return;
  if (manualControlActive) {
    stopManualOperation();
  }
  // С экрана ошибки тоже можно войти в меню: снимаем блокировку отрисовки.
  errorScreenBlocking = false;
  gem.setMenuPageCurrent(mainPage);
  menuVisible = true;
  displayDirty = true;
  Serial.println("[MENU] OPEN ok");
}

/** Запрос открытия меню — безопасен из ButtonTask / Serial. */
void requestMenuOpen(const char *via) {
  g_menuReq = MenuReq::Open;
  Serial.printf("[MENU] OPEN req via %s\n", via ? via : "?");
}

/** Запрос закрытия с сохранением — безопасен из ButtonTask / Serial / GEM.
 *  Реальная работа (cancel edit, save, fillScreen) в DisplayTask. */
void requestMenuClose(const char *via) {
  g_menuReq = MenuReq::Close;
  Serial.printf("[MENU] CLOSE req via %s\n", via ? via : "?");
}

/** Сброс KN3/KN4 после срабатывания комбо (чтобы не было ложного click/step). */
// buttonTask — see task_button.cpp

// reinitAttempts, imuTask — moved to task_imu.cpp
#if 0  // === REMOVED ===
static uint8_t _removed_reinitAttempts = 0;

void _removed_imuTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_IMU;
  uint8_t fifo[28];  // MotionApps 6.12 default packet
  uint8_t errCnt = 0;
  uint32_t lastIMU = 0;
  uint32_t lastReinitAttempt = 0;
  bool wasMoving = false;
  uint32_t motionStartTime = 0;
  uint32_t motionPulseAccumMs = 0;  // только busy-импульсы → вход в MOVEMENT
  uint32_t lastMotEvalMs = 0;
  bool localProlongedMovement = false;

  uint32_t lastDebugPrint = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_IMU);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_IMU);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    uint32_t currentTime = millis();

    if (currentTime - lastDebugPrint > 10000) {
      lastDebugPrint = currentTime;
      Serial.printf("[IMU] Stats - IMU:%dms, Moving:%d, MPU:%d\n",
                    (currentTime - lastIMU),
                    wasMoving, mpuOk);
    }

#if ENABLE_SIMULATION
    updateSimulationData();
    static uint32_t lastImuDebug = 0;
    if (currentTime - lastImuDebug > 200) {
      lastImuDebug = currentTime;
      Serial.printf("[SIM_IMU] angleX=%.2f, angleY=%.2f\n", simAngleX, simAngleY);
    }

    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        angleX = simAngleX;
        angleY = simAngleY;
        temperature = 25.0f + sin(currentTime * 0.001f) * 0.5f;
      }
    }

    IMUData imuData = { simAngleX, simAngleY, 25.0f };
    if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;

    Event event;
    event.type = EventType::IMU_UPDATE;
    event.timestamp = currentTime;
    event.data.imu.angleX = simAngleX;
    event.data.imu.angleY = simAngleY;
    event.data.imu.temperature = 25.0f;
    EventBus::publish(event, 0);

    if (currentState != SystemState::CALIBRATING) {
      setDisplayDirty();
    }

#elif ENABLE_MPU6050

    // ========== ? ПРОВЕРКА: ЕСЛИ MPU НЕ РАБОТАЕТ - ПРОПУСКАЕМ ==========
    if (!mpuOk) {
      // Пытаемся восстановить раз в 30 секунд
      if (currentTime - lastReinitAttempt > 30000) {
        lastReinitAttempt = currentTime;
        initializeDMP();
        Serial.println("[IMU] Attempting MPU reinitialization");
      }

      // Отправляем нулевые данные
      IMUData imuData = { 0, 0, 25.0f };
      if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;

      // Сбрасываем флаги движения
      wasMoving = false;
      motionPulseAccumMs = 0;
      lastMotEvalMs = 0;
      localProlongedMovement = false;
      if (movementModeActive) {
        movementModeActive = false;
        currentSystemMode = previousMode;
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ========== ДЕТЕКЦИЯ ДВИЖЕНИЯ + УГЛЫ (один getIntStatus за тик) ==========
    if (mpuOk) {
      if (currentTime - lastIMU > (uint32_t)ConfigManager::getImuPollMs()) {
        MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(40));
        if (!i2c) {
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }

        // Один раз за цикл: INT_STATUS (latch) + MOT_DETECT_STATUS (оси).
        const uint8_t intStatus = mpu.getIntStatus();
        const uint8_t motStatus = mpu.getMotionStatus();

        // Сначала FIFO: углы + RMS gyro/linAcc (для непрерывного детекта езды).
        const int fifoMax = ConfigManager::getImuFifoAvg();
        float sumX = 0.0f, sumY = 0.0f;
        int nPkt = 0;
        float gyroRmsMax = -1.0f;
        float gyroBumpMax = -1.0f;
        float linAccRmsMax = -1.0f;
        while (nPkt < fifoMax && mpu.dmpGetCurrentFIFOPacket(fifo)) {
          Quaternion q;
          VectorFloat gravity;
          VectorInt16 aa, aaReal;
          float ypr[3];
          int16_t gRaw[3];
          mpu.dmpGetQuaternion(&q, fifo);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          mpu.dmpGetGyro(gRaw, fifo);
          mpu.dmpGetAccel(&aa, fifo);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          const float gRms = sqrtf((float)gRaw[0] * gRaw[0] + (float)gRaw[1] * gRaw[1] +
                                  (float)gRaw[2] * gRaw[2]);
          if (gRms > gyroRmsMax) gyroRmsMax = gRms;
          // Неровности: кивок (pitch=gy) и крен (roll=gx); yaw (gz) не берём —
          // повороты руля не должны сами по себе держать «неровности».
          const float gxAbs = fabsf((float)gRaw[0]);
          const float gyAbs = fabsf((float)gRaw[1]);
          const float bump = (gxAbs > gyAbs) ? gxAbs : gyAbs;
          if (bump > gyroBumpMax) gyroBumpMax = bump;
          const float aRms = sqrtf((float)aaReal.x * aaReal.x + (float)aaReal.y * aaReal.y +
                                  (float)aaReal.z * aaReal.z);
          if (aRms > linAccRmsMax) linAccRmsMax = aRms;
          const float ax = degrees(ypr[2]);
          const float ay = degrees(ypr[1]);
          if (fabsf(ax) <= 90.0f && fabsf(ay) <= 90.0f && !isnan(ax) && !isnan(ay)) {
            sumX += ax;
            sumY += ay;
            nPkt++;
          }
        }

        const bool currentlyMoving = detectMotionFromIMU(intStatus, motStatus, currentTime,
                                                         gyroRmsMax, linAccRmsMax, gyroBumpMax);

        // Поток отладки движения (TEST MOT STREAM)
        if (g_motStreamUntilMs != 0 && currentTime < g_motStreamUntilMs) {
          static uint32_t lastMotLogMs = 0;
          const uint32_t period = g_motStreamPeriodMs < 50 ? 50 : g_motStreamPeriodMs;
          if (currentTime - lastMotLogMs >= period) {
            lastMotLogMs = currentTime;
            const char *ms = "MANUAL";
            if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
            else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
            float ageSec = 0.0f;
            float enterSec = -1.0f;
            float settleLeftSec = 0.0f;
            const uint32_t needMs = (uint32_t)ConfigManager::getMovementDurationSec() * 1000UL;
            if (currentlyMoving && motionStartTime > 0) {
              ageSec = (currentTime - motionStartTime) * 0.001f;
              if (!movementModeActive) {
                enterSec = (motionPulseAccumMs >= needMs)
                               ? 0.0f
                               : (needMs - motionPulseAccumMs) * 0.001f;
              } else {
                enterSec = 0.0f;
              }
            }
            if (g_motLastActivityMs > 0) {
              const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
              const uint32_t idle = currentTime - g_motLastActivityMs;
              settleLeftSec = (idle >= settleMs) ? 0.0f : (settleMs - idle) * 0.001f;
            }
            float exitSec = -1.0f;
            if (movementModeActive && movementEndTime > 0) {
              const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
              const uint32_t elapsed = currentTime - movementEndTime;
              exitSec = (elapsed >= settleMs) ? 0.0f : (settleMs - elapsed) * 0.001f;
            }
            Serial.printf(
                "[MOT] t=%lu mot=%d motSt=0x%02X gyro=%.0f gdlt=%.0f thr=%.0f busy=%d "
                "bump=%.0f bdlt=%.0f bthr=%.0f bbusy=%d "
                "lin=%.0f ldlt=%.0f lthr=%.0f lbusy=%d hold=%d was=%d mode=%s "
                "age=%.1f enter=%.1f settleLeft=%.1f exit=%.1f int=0x%02X\n",
                (unsigned long)currentTime,
                g_motLastMotPulse ? 1 : 0,
                (unsigned)g_motLastMotStatus,
                g_motLastGyroRms,
                g_motLastGyroDelta,
                g_motLastGyroThr,
                g_motLastGyroBusy ? 1 : 0,
                g_motLastGyroBump,
                g_motLastGyroBumpDelta,
                g_motLastGyroBumpThr,
                g_motLastGyroBumpBusy ? 1 : 0,
                g_motLastLinAccRaw,
                g_motLastLinAccRms,
                g_motLastLinAccThr,
                g_motLastLinAccBusy ? 1 : 0,
                currentlyMoving ? 1 : 0,
                wasMoving ? 1 : 0,
                ms,
                ageSec,
                enterSec,
                settleLeftSec,
                exitSec,
                (unsigned)g_motLastIntStatus);
          }
        } else if (g_motStreamUntilMs != 0 && currentTime >= g_motStreamUntilMs) {
          g_motStreamUntilMs = 0;
          Serial.println("[MOT] STREAM end");
        }

        const bool pulseNow = g_motLastMotPulse || g_motLastGyroBusy || g_motLastGyroBumpBusy ||
                              g_motLastLinAccBusy;

        if (currentlyMoving) {
          if (!wasMoving) {
            wasMoving = true;
            motionStartTime = currentTime;
            motionPulseAccumMs = 0;
            lastMotEvalMs = currentTime;
            movementStartMs = currentTime;  // 8.9.0: отметка начала движения для экрана ДВИЖЕНИЯ
            localProlongedMovement = false;

            Serial.printf("[IMU] >>> ДВИЖЕНИЕ (MOT=%d gyroRms=%.0f bump=%.0f linAcc=%.0f) <<<\n",
                          (intStatus & 0x40) ? 1 : 0, gyroRmsMax, gyroBumpMax, linAccRmsMax);
            Event event;
            event.type = EventType::MOVEMENT_DETECTED;
            event.timestamp = millis();
            EventBus::publish(event);

          } else {
            // Duration: только реальное busy-время (не settle-afterglow).
            const uint32_t dt = (lastMotEvalMs > 0 && currentTime >= lastMotEvalMs)
                                    ? (currentTime - lastMotEvalMs)
                                    : 0;
            lastMotEvalMs = currentTime;
            if (pulseNow && dt > 0 && dt < 500) {
              motionPulseAccumMs += dt;
            }

            if (!movementModeActive &&
                (motionPulseAccumMs > (uint32_t)ConfigManager::getMovementDurationSec() * 1000UL)) {
            localProlongedMovement = true;
            previousMode = currentSystemMode;

            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                currentSystemMode = SystemMode::MOVEMENT;
              }
            }

            movementModeActive = true;
            g_motModeEnterCount++;
            movementPressureLastCheck = currentTime;
            movementLastAdjustFront = currentTime;  // 8.9.0: корректировка при переходе в ДВИЖЕНИЕ
            movementLastAdjustRear = currentTime;
            movementEndTime = 0;

            Serial.printf("[MOVEMENT] Длительное движение! pulseAccum=%lu мс → MOVEMENT\n",
                          (unsigned long)motionPulseAccumMs);

            float targetPressureFront = ConfigManager::getMovementPressureFront();
            float targetPressureRear = ConfigManager::getMovementPressureRear();
            float avgPressureFront, avgPressureRear;
            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                avgPressureFront = (pressure[PAD_FRONT_LEFT] + pressure[PAD_FRONT_RIGHT]) / 2.0f;
                avgPressureRear = (pressure[PAD_REAR_LEFT] + pressure[PAD_REAR_RIGHT]) / 2.0f;
              } else {
                avgPressureFront = avgPressureRear = 3.0f;
              }
            }

            float diffFront = targetPressureFront - avgPressureFront;
            if (abs(diffFront) > ConfigManager::getMovementTolerance()) {  // 8.8.0: допуск из меню
              if (diffFront > 0) {
                sendValveCommand(PAD_FRONT_LEFT, true, 1500);
                sendValveCommand(PAD_FRONT_RIGHT, true, 1500);
              } else {
                sendValveCommand(PAD_FRONT_LEFT, false, 1200);
                sendValveCommand(PAD_FRONT_RIGHT, false, 1200);
              }
            }

            float diffRear = targetPressureRear - avgPressureRear;
            if (abs(diffRear) > ConfigManager::getMovementTolerance()) {  // 8.8.0: допуск из меню
              if (diffRear > 0) {
                sendValveCommand(PAD_REAR_LEFT, true, 1500);
                sendValveCommand(PAD_REAR_RIGHT, true, 1500);
              } else {
                sendValveCommand(PAD_REAR_LEFT, false, 1200);
                sendValveCommand(PAD_REAR_RIGHT, false, 1200);
              }
            }

            if (manualControlActive) {
              stopManualOperation();
            }

            Event event;
            event.type = EventType::MOVEMENT_DETECTED;
            event.timestamp = millis();
            EventBus::publish(event);
            }
          }
        } else {
          if (wasMoving) {
            wasMoving = false;
            motionPulseAccumMs = 0;
            lastMotEvalMs = 0;
            Serial.printf("[IMU] >>> ДВИЖЕНИЕ ОКОНЧЕНО (пауза %d с) <<<\n",
                          ConfigManager::getMovementSettleSec());

            Event event;
            event.type = EventType::MOVEMENT_ENDED;
            event.timestamp = millis();
            EventBus::publish(event);
          }

          if (movementModeActive) {
            if (movementEndTime == 0) {
              movementEndTime = currentTime;
              Serial.println("[MOVEMENT] Ожидание 30 секунд перед возвратом");
            } else if (currentTime - movementEndTime >= (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL) {  // 8.8.0: из меню
              movementModeActive = false;
              movementEndTime = 0;
              currentSystemMode = previousMode;
              localProlongedMovement = false;
              motionStartTime = 0;

              if (previousMode == SystemMode::AUTO) {
                currentMode = Mode::AUTO;
                lastLevelingCheckTime = currentTime;
                lastLevelingAttemptTime = currentTime;
                levelingAttemptsThisHour = 0;
              } else {
                currentMode = Mode::MANUAL;
                setManualTargetsFromParkingPolicy();
              }

              Event event;
              event.type = EventType::MOVEMENT_ENDED;
              event.timestamp = millis();
              EventBus::publish(event);
            }
          } else {
            motionStartTime = 0;
            localProlongedMovement = false;
            movementEndTime = 0;
          }
        }

        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            bool oldIsMoving = isMoving;
            isMoving = wasMoving;
            prolongedMovementDetected = localProlongedMovement;

            if (oldIsMoving != wasMoving) {
              setDisplayDirty();
            }
          }
        }

        // Углы уже прочитаны из FIFO выше (sumX/Y, nPkt).
        (void)intStatus;

        if (nPkt > 0) {
          const float ax = sumX / (float)nPkt;
          const float ay = sumY / (float)nPkt;
          g_imuRawX = ax;
          g_imuRawY = ay;

          // EMA после Калмана. Выброс > thr — тянем слабее (глушим всплеск).
          static float emaX = 0.0f, emaY = 0.0f;
          static bool emaInit = false;
          static float limX = 0.0f, limY = 0.0f;
          static bool limInit = false;
          if (g_imuFilterResetReq) {
            emaInit = false;
            limInit = false;
            // setParameters + «прогрев» — без placement-new (гонка с applyRuntimeSettings).
            const float mea = ConfigManager::getImuKalmanMea();
            const float est = ConfigManager::getImuKalmanEst();
            const float q = ConfigManager::getImuKalmanQ();
            filterX.setParameters(mea, est, q);
            filterY.setParameters(mea, est, q);
            for (int k = 0; k < 8; k++) {
              (void)filterX.filtered(ax);
              (void)filterY.filtered(ay);
            }
            g_imuFilterResetReq = false;
          }

          float newX = filterX.filtered(ax);
          float newY = filterY.filtered(ay);

          const float aNom = ConfigManager::getImuEmaAlpha();
          const float aSpike = ConfigManager::getImuEmaSpikeAlpha();
          const float spikeThr = ConfigManager::getImuEmaSpikeThr();
          if (!emaInit) {
            emaX = newX;
            emaY = newY;
            emaInit = true;
          } else {
            float aX = aNom;
            float aY = aNom;
            if (fabsf(newX - emaX) > spikeThr) aX = aSpike;
            if (fabsf(newY - emaY) > spikeThr) aY = aSpike;
            emaX += aX * (newX - emaX);
            emaY += aY * (newY - emaY);
          }
          newX = emaX;
          newY = emaY;

          // Ограничение °/с + catch-up: при большой ошибке догоняем быстрее
          // (иначе рывок 2°/тик режется до ~0.6° при slew=40).
          const float dt = fmaxf((float)ConfigManager::getImuPollMs() * 0.001f, 0.015f);
          float maxStepX = ConfigManager::getImuSlewDps() * dt;
          float maxStepY = maxStepX;
          if (!limInit) {
            limX = newX;
            limY = newY;
            limInit = true;
          } else {
            const float errX = fabsf(newX - limX);
            const float errY = fabsf(newY - limY);
            if (errX > 1.0f) maxStepX = fmaxf(maxStepX, errX * 0.55f);
            if (errY > 1.0f) maxStepY = fmaxf(maxStepY, errY * 0.55f);
            limX += constrain(newX - limX, -maxStepX, maxStepX);
            limY += constrain(newY - limY, -maxStepY, maxStepY);
          }
          newX = limX;
          newY = limY;

          float newTemp = (mpu.getTemperature() / 340.0f) + 36.53f;

          if (isnan(newX) || isnan(newY) || isnan(newTemp) || fabsf(newX) > 90 ||
              fabsf(newY) > 90 || newTemp < -20 || newTemp > 120) {
            Serial.printf("[IMU] Invalid data: X=%.2f, Y=%.2f, T=%.2f\n", newX, newY, newTemp);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
          }

          // Нуль применяем здесь — один источник истины для UI и control
          g_imuAbsX = newX;
          g_imuAbsY = newY;
          const float outX = newX - ConfigManager::getZeroAngleX();
          const float outY = newY - ConfigManager::getZeroAngleY();
          g_imuOutX = outX;
          g_imuOutY = outY;

          // STREAM: сырой + фильтр на каждый тик
          const uint32_t nowMs = millis();
          if (g_imuStreamUntilMs != 0 && nowMs < g_imuStreamUntilMs) {
            Serial.printf("[IMU] S %lu raw=%.3f,%.3f out=%.3f,%.3f n=%d\n",
                          (unsigned long)nowMs, ax, ay, outX, outY, nPkt);
          } else if (g_imuStreamUntilMs != 0 && nowMs >= g_imuStreamUntilMs) {
            g_imuStreamUntilMs = 0;
            Serial.println("[IMU] STREAM end");
          }

          // STATS: копит метрики шума/всплесков
          if (g_imuStatsActive) {
            if (g_imuStatN == 0) {
              g_imuStatT0 = nowMs;
              g_imuStatPrevX = outX;
              g_imuStatPrevY = outY;
              g_imuStatMinX = g_imuStatMaxX = outX;
              g_imuStatMinY = g_imuStatMaxY = outY;
            } else {
              const float dx = fabsf(outX - g_imuStatPrevX);
              const float dy = fabsf(outY - g_imuStatPrevY);
              const float step = fmaxf(dx, dy);
              if (step > g_imuStatMaxStep) g_imuStatMaxStep = step;
              if (step > spikeThr) g_imuStatSpikes++;
              if (outX < g_imuStatMinX) g_imuStatMinX = outX;
              if (outX > g_imuStatMaxX) g_imuStatMaxX = outX;
              if (outY < g_imuStatMinY) g_imuStatMinY = outY;
              if (outY > g_imuStatMaxY) g_imuStatMaxY = outY;
              g_imuStatPrevX = outX;
              g_imuStatPrevY = outY;
            }
            g_imuStatSumX += outX;
            g_imuStatSumY += outY;
            g_imuStatSumXX += outX * outX;
            g_imuStatSumYY += outY * outY;
            g_imuStatN++;
            if (g_imuStatsUntilMs != 0 && nowMs >= g_imuStatsUntilMs) {
              const float n = (float)g_imuStatN;
              const float meanX = g_imuStatSumX / n;
              const float meanY = g_imuStatSumY / n;
              float varX = g_imuStatSumXX / n - meanX * meanX;
              float varY = g_imuStatSumYY / n - meanY * meanY;
              if (varX < 0) varX = 0;
              if (varY < 0) varY = 0;
              const float dtSec = fmaxf((nowMs - g_imuStatT0) * 0.001f, 0.001f);
              Serial.printf(
                  "[IMU] STATS n=%u hz=%.1f std=%.4f,%.4f pp=%.3f,%.3f spikes=%u maxStep=%.3f "
                  "mean=%.3f,%.3f\n",
                  (unsigned)g_imuStatN, (float)g_imuStatN / dtSec, sqrtf(varX), sqrtf(varY),
                  g_imuStatMaxX - g_imuStatMinX, g_imuStatMaxY - g_imuStatMinY,
                  (unsigned)g_imuStatSpikes, g_imuStatMaxStep, meanX, meanY);
              Serial.println("[TEST] OK IMU_STATS");
              g_imuStatsActive = false;
              g_imuStatsUntilMs = 0;
            }
          }

          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              angleX = outX;
              angleY = outY;
              temperature = newTemp;
            }
          }

          Event event;
          event.type = EventType::IMU_UPDATE;
          event.timestamp = millis();
          event.data.imu.angleX = outX;
          event.data.imu.angleY = outY;
          event.data.imu.temperature = newTemp;
          EventBus::publish(event, 0);

          IMUData imuData = { outX, outY, newTemp };
          // Неблокирующая отправка: иначе при медленном controlTask очередь
          // забивается и imuTask тормозит до ~10 Гц.
          if (xQueueSend(xIMUQueue, &imuData, 0) != pdTRUE) {
            IMUData discard;
            if (xQueueReceive(xIMUQueue, &discard, 0) == pdTRUE) {
              xQueueSend(xIMUQueue, &imuData, 0);
            }
            imuQueueDropCount++;
          }

          lastIMU = currentTime;
          errCnt = 0;
          reinitAttempts = 0;

          if (ErrorHandler::isErrorActive(ErrorHandler::Error::MPU)) {
            ErrorHandler::markErrorCleared(ErrorHandler::Error::MPU);
            Serial.println("[IMU] MPU восстановлен, ошибка будет удалена через 3 сек");
          }

        } else {
          lastIMU = currentTime;  // MOT обработан даже без пакета FIFO
          if (intStatus & 0x10) {
            Serial.println("[IMU] FIFO overflow — resetFIFO");
            mpu.resetFIFO();
            if (++errCnt > 10) {
              mpuOk = false;
              errCnt = 0;
              reinitAttempts++;
              Serial.printf("[IMU] Reinit scheduled after overflow (%d/5)\n", reinitAttempts);
            }
          } else {
            errCnt = 0;
          }
        }
      }
    }

    if (!mpuOk && reinitAttempts > 0 && reinitAttempts <= 5 &&
        currentTime - lastReinitAttempt > 2000) {
      lastReinitAttempt = currentTime;
      initializeDMP();
      (void)reinitAttempts;
    }

#else
    // ========== ТЕСТОВЫЙ РЕЖИМ (MPU отключен) ==========
    static uint32_t lastTestUpdate = 0;
    if (currentTime - lastTestUpdate > 1000) {
      lastTestUpdate = currentTime;

      MutexGuard guard(xStateMutex);
      if (guard) {
        if (angleX != 0 || angleY != 0) {
          angleX = 0;
          angleY = 0;
          setDisplayDirty();
        }
      }

      IMUData imuData = { 0, 0, 25.0f };
      if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;
    }

    wasMoving = false;
    motionPulseAccumMs = 0;
    lastMotEvalMs = 0;
    localProlongedMovement = false;
#endif

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
#endif // === END REMOVED imuTask ===

// pressureTask — see task_pressure.cpp

// calibrationTask — see task_calib.cpp

// displayTask — see task_display.cpp



void watchdogTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_Watchdog;
  esp_task_wdt_add(NULL);
  for (;;) {
    TaskPool::markRun(taskIndex_Watchdog);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_WATCHDOG);
    TaskMonitor::checkTasks();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


/* otaTask -> task_ota.h/cpp */

void errorRecoveryTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_ErrRec;
  const TickType_t period = pdMS_TO_TICKS(2000);

  static SystemMode modeBeforeError = SystemMode::MANUAL;

  for (;;) {
    TaskPool::markRun(taskIndex_ErrRec);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_ERROR_RECOVERY);

    if (!ErrorHandler::hasActiveErrors()) {
      vTaskDelay(period);
      continue;
    }

    static bool modeSaved = false;
    static uint32_t watchdogStableSince = 0;
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::WATCHDOG)) {
      watchdogStableSince = 0;
    }
    if (!modeSaved) {
      MutexGuard guard(xStateMutex, pdMS_TO_TICKS(100));
      if (guard) {
        modeBeforeError = currentSystemMode;
        modeSaved = true;
        Serial.printf("[RECOVERY] Запомнен режим: %d\n", (int)modeBeforeError);
      }
    }

    bool anyCleared = false;
    int count = ErrorHandler::getActiveErrorCount();
    static int goodReads = 0;

    for (int i = 0; i < count; i++) {
      ErrorHandler::Error err = ErrorHandler::getActiveErrorAt(i);
      bool cleared = false;

      switch (err) {
        case ErrorHandler::Error::LOW_PRESSURE:
          checkPressureLimits();
          cleared = !ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE);
          if (cleared) {
            Serial.println("[RECOVERY] LOW_PRESSURE устранена (давление в норме)");
          }
          break;

        case ErrorHandler::Error::MPU:
          {
            MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(200));
            if (i2c) {
              auto probe = [](uint8_t addr) -> bool {
                Wire.beginTransmission(addr);
                return Wire.endTransmission() == 0;
              };
              cleared = probe(0x68) || probe(0x69);
            }
          }
          if (cleared) {
            Serial.println("[RECOVERY] MPU6050 на шине I2C — инициализация DMP");
            initializeDMP();
            cleared = mpuOk;
          }
          break;

        case ErrorHandler::Error::SENSOR:
          {
#if !ENABLE_SIMULATION
            float bar = 0.0f;
            bool ok = false;
            {
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
              if (i2c) {
                if (!Jhm1200::isReady()) {
                  // датчик мог появиться после boot-fail — пробуем begin
                  ok = Jhm1200::begin(Wire);
                  if (ok) jhmReady = true;
                }
                if (Jhm1200::isReady()) ok = Jhm1200::readBar(bar);
              }
            }

            if (ok) {
              goodReads++;
              if (!calibrationCompleted) {
                // Не держим SENSOR вечно: датчик жив > просим CalibTask повторить BOOT.
                if (goodReads >= 2 && getSystemState() == SystemState::RUNNING) {
                  Serial.println("[RECOVERY] JHM1200 отвечает — повтор калибровки");
                  setSystemState(SystemState::BOOT);
                  goodReads = 0;
                }
                cleared = false;
              } else if (goodReads >= 2) {
                cleared = true;
                goodReads = 0;
                Serial.printf("[RECOVERY] JHM1200 восстановлен! %.2f бар\n", bar);
              }
            } else {
              goodReads = 0;
              cleared = false;
              if (!calibrationCompleted) {
                static uint32_t lastKeepLog = 0;
                if (millis() - lastKeepLog > 5000) {
                  lastKeepLog = millis();
                  Serial.println("[RECOVERY] SENSOR: нет ответа JHM1200");
                }
              }
            }
#else
            cleared = true;
            Serial.println("[RECOVERY] SIM: Датчик давления исправен");
#endif
            break;
          }

        case ErrorHandler::Error::VALVE:
          // Не снимаем по таймеру — только после успешного теста / ручного сброса.
          cleared = false;
          break;

        case ErrorHandler::Error::WATCHDOG:
          {
            // Таймер не сбрасываем при каждом тике — иначе pendingClear сразу отменяется.
            if (watchdogStableSince == 0) {
              watchdogStableSince = millis();
            }
            if (millis() - watchdogStableSince > 30000) {
              cleared = true;
              Serial.println("[RECOVERY] WATCHDOG ошибка устранена (система стабильна)");
            }
            break;
          }

        case ErrorHandler::Error::OTA:
          {
            static uint32_t otaErrorTime = 0;
            if (otaErrorTime == 0) {
              otaErrorTime = millis();
            }
            if (millis() - otaErrorTime > 30000) {
              // Не поднимаем AP автоматически — только сброс ошибки OTA.
              if (WiFi.status() != WL_CONNECTED && sta_ssid[0] != '\0') {
                WiFi.mode(WIFI_STA);
                WiFi.begin(sta_ssid, WIFI_STA_PASSWORD);
                Serial.println("[RECOVERY] OTA: повтор STA, без softAP");
              } else {
                Serial.println("[RECOVERY] OTA ошибка снята (без авто-AP)");
              }
              cleared = true;
              otaErrorTime = 0;
            }
            break;
          }

        default:
          cleared = false;
          break;
      }

      // ? ЕСЛИ ОШИБКА УСТРАНЕНА - ПОМЕЧАЕМ НА УДАЛЕНИЕ
      if (cleared) {
        ErrorHandler::markErrorCleared(err);
        anyCleared = true;
      } else {
        // ? ЕСЛИ НЕ УДАЛЯЕМ - ОТМЕНЯЕМ PENDING CLEAR (если была установлена)
        if (ErrorHandler::isPendingClear(err)) {
          ErrorHandler::cancelClear(err);
          Serial.printf("[RECOVERY] Отменено удаление ошибки %d (условия не выполнены)\n", (int)err);
        }
      }
    }

    // ? ЕСЛИ ВСЕ ОШИБКИ УСТРАНЕНЫ - ВОССТАНАВЛИВАЕМ СИСТЕМУ
    if (anyCleared && !ErrorHandler::hasActiveErrors()) {
      Serial.println("[RECOVERY] Все ошибки устранены, восстанавливаем систему");
      vTaskDelay(pdMS_TO_TICKS(500));

      // ? ВОССТАНАВЛИВАЕМ РЕЖИМ
      currentSystemMode = modeBeforeError;
      if (modeBeforeError == SystemMode::AUTO) {
        currentMode = Mode::AUTO;
        lastLevelingCheckTime = millis();
        lastLevelingAttemptTime = millis();
      } else {
        currentMode = Mode::MANUAL;
        setAllManualTargetsFromCurrent();
      }

      errorScreenBlocking = false;
      forceDisplayReset(true);
      modeSaved = false;
    }

    vTaskDelay(period);
  }
}

void initWatchdog() {
  esp_task_wdt_config_t config = {
    .timeout_ms = TASK_WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t result = esp_task_wdt_init(&config);
  if (result == ESP_ERR_INVALID_STATE) {
    Serial.println("[WDT] TWDT уже инициализирован ядром, используем текущую конфигурацию");
  } else if (result != ESP_OK) {
    Serial.printf("[WDT] init failed: %d\n", result);
    return;
  } else {
    Serial.printf("[WDT] initialized: %u ms\n", TASK_WDT_TIMEOUT_MS);
  }
}

/** GitHub OTA через esp_ota_* (без Arduino Update.begin — тот падает под TLS при maxBlk~36K). */

/* ghOta statics + serial -> ota_net, task_ota */


/* downloadGitHubFirmware + checkGitHubUpdate -> ota_install.h/cpp */

void initOTA() {
  Logger::log(Logger::INFO, "OTA", "Инициализация…");
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    if (otaInProgress) {
      Serial.println("[OTA] Обновление уже идет!");
      return;
    }

    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (maxSketchSpace < 1024 * 1024) {
      Serial.printf("[OTA] Недостаточно места! Свободно: %u байт\n", maxSketchSpace);
      ErrorHandler::handleError(ErrorHandler::Error::OTA, "Недостаточно места для OTA");
      otaInProgress = false;
      otaValveLock = false;
      return;
    }

    saveConfig();
    saveWiFiConfig();
    // Keep LittleFS mounted: other tasks may still access it while OTA runs.

    Logger::log(Logger::INFO, "OTA", "Старт обновления");
    otaInProgress = true;
    otaValveLock = true;
    otaProgress = 0;
    strcpy(otaStatus, "Начало");
    emergencyStop();

    Event event;
    event.type = EventType::OTA_START;
    event.timestamp = millis();
    EventBus::publish(event);
  });

  ArduinoOTA.onEnd([]() {
    Logger::log(Logger::INFO, "OTA", "Обновление завершено");
    otaInProgress = false;
    otaValveLock = true;
    otaProgress = 100;
    strcpy(otaStatus, "Готово");

    Event event;
    event.type = EventType::OTA_END;
    event.timestamp = millis();
    EventBus::publish(event);

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    otaProgress = (prog * 100) / total;
    snprintf(otaStatus, sizeof(otaStatus), "Загрузка: %d%%", otaProgress);

    Event event;
    event.type = EventType::OTA_PROGRESS;
    event.timestamp = millis();
    event.data.ota.progress = otaProgress;
    strncpy(event.data.ota.status, otaStatus, sizeof(event.data.ota.status));
    EventBus::publish(event, 0);
  });

  ArduinoOTA.onError([](ota_error_t err) {
    otaInProgress = false;
    otaValveLock = false;

    const char *errMsg;
    switch (err) {
      case OTA_AUTH_ERROR: errMsg = "Ошибка аутентификации"; break;
      case OTA_BEGIN_ERROR: errMsg = "Ошибка начала обновления"; break;
      case OTA_CONNECT_ERROR: errMsg = "Ошибка соединения"; break;
      case OTA_RECEIVE_ERROR: errMsg = "Ошибка приема данных"; break;
      case OTA_END_ERROR: errMsg = "Ошибка завершения"; break;
      default: errMsg = "Неизвестная ошибка";
    }

    char fullMsg[64];
    snprintf(fullMsg, sizeof(fullMsg), "OTA: %s (код: %d)", errMsg, err);
    ErrorHandler::handleError(ErrorHandler::Error::OTA, fullMsg);

    Logger::logf(Logger::ERROR, "OTA", "Ошибка: %s (код: %d)", errMsg, err);

    Event event;
    event.type = EventType::OTA_ERROR;
    event.timestamp = millis();
    EventBus::publish(event);
  });

  ArduinoOTA.begin();
  Logger::log(Logger::INFO, "OTA", "Готово");
}

void startOTAMode() {
  if (otaMode) return;
  Logger::log(Logger::INFO, "OTA", "Запуск режима OTA");
  menuVisible = false;
  displayDirty = true;
  otaValveLock = true;
  emergencyStop();
  otaMode = true;
  setSystemState(SystemState::OTA_MODE);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(wifi_ssid, wifi_password);
  initOTA();
}

void stopOTAMode() {
  if (!otaMode) return;
  otaMode = false;
  otaInProgress = false;
  otaValveLock = false;
  setSystemState(SystemState::RUNNING);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  forceDisplayReset(true);
}

void handleOTA() {
  if (otaMode) ArduinoOTA.handle();
}

// requestPressureMeasurement — see pressure_read.cpp


// GEM menu build / refresh — see ui_menu_build.cpp


/* ====================  SETUP ==================== */
/* ====================  SETUP ==================== */
void setup() {
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  Serial.println("\n\n=== Kamaz?Leveler (FreeRTOS OTA Optimized) ===");
  Serial.printf("Версия: %s\n", VERSION);
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const char *rrs = "OTHER";
    switch (rr) {
      case ESP_RST_POWERON: rrs = "POWERON"; break;
      case ESP_RST_EXT: rrs = "EXT"; break;
      case ESP_RST_SW: rrs = "SW"; break;
      case ESP_RST_PANIC: rrs = "PANIC"; break;
      case ESP_RST_INT_WDT: rrs = "INT_WDT"; break;
      case ESP_RST_TASK_WDT: rrs = "TASK_WDT"; break;
      case ESP_RST_WDT: rrs = "WDT"; break;
      case ESP_RST_DEEPSLEEP: rrs = "DEEPSLEEP"; break;
      case ESP_RST_BROWNOUT: rrs = "BROWNOUT"; break;
      case ESP_RST_SDIO: rrs = "SDIO"; break;
      default: break;
    }
    Serial.printf("[RESET] reason=%s (%d)\n", rrs, static_cast<int>(rr));
  }
  Serial.println("[SETUP] Система в режиме BOOT, калибровка будет запущена автоматически.");

  // ========== 1. ОЧЕРЕДИ ==========
  xIMUQueue = xQueueCreate(10, sizeof(IMUData));
  xPressureQueue = xQueueCreate(10, sizeof(PressureData));
  xValveQueue = xQueueCreate(20, sizeof(ValveCommandMsg));
  xPressureWakeupQueue = xQueueCreate(10, sizeof(uint32_t));
  if (xPressureWakeupQueue == nullptr) {
    Serial.println("[ERROR] Failed to create pressure wakeup queue!");
    xPressureWakeupQueue = xQueueCreate(5, sizeof(uint32_t));
    if (xPressureWakeupQueue == nullptr) {
      ESP.restart();
    }
  }

  if (xIMUQueue == nullptr || xPressureQueue == nullptr || xValveQueue == nullptr) {
    Serial.println("[ERROR] Failed to create queues!");
    while (1) { delay(100); }
  }
  Serial.println("[INIT] Queues created");

  // ========== 2. EVENT BUS ==========
  if (!EventBus::init()) {
    Serial.println("[ERROR] Failed to init EventBus");
    ESP.restart();
  }

  // ========== 3. МЬЮТЕКСЫ (ДО ИХ ИСПОЛЬЗОВАНИЯ!) ==========
  xValveMutex = xSemaphoreCreateMutex();
  xDisplayMutex = xSemaphoreCreateMutex();
  xConfigMutex = xSemaphoreCreateMutex();
  xStateMutex = xSemaphoreCreateMutex();
  xCalibMutex = xSemaphoreCreateMutex();
  xTestMutex = xSemaphoreCreateMutex();
  xCommandMutex = xSemaphoreCreateMutex();
  xI2CMutex = xSemaphoreCreateMutex();

  if (!ErrorHandler::initMutex()) {
    Serial.println("[ERROR] Failed to init ErrorHandler mutex!");
    ESP.restart();
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  if (!xValveMutex || !xDisplayMutex || !xConfigMutex || !xStateMutex ||
      !xCalibMutex || !xTestMutex || !xCommandMutex || !xI2CMutex) {
    Serial.println("[ERROR] Failed to create mutexes");
    ESP.restart();
  }

  // ========== 4. WIRE ==========
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(100000L);  // старт на 100 кГц — надёжнее для MPU; после DMP поднимем до 400
  delay(50);

// ========== JHM1200 – ПРОВЕРКА НАЛИЧИЯ ==========
#if !ENABLE_SIMULATION
    bool jhmOk = initializeJhm1200();
    if (!jhmOk) {
        Serial.println("[JHM1200] Датчик не найден! Калибровка будет пропущена.");
        jhmReady = false;
    } else {
        Serial.println("[JHM1200] Датчик найден на 0x78");
    }
#else
    jhmReady = true;
    Serial.println("[JHM1200] SIM: инициализация пропущена");
#endif

  // ========== 6. TFT ==========
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);
  spi.begin(PIN_TFT_SCLK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  spi.setFrequency(40000000);
  tft.init(240, 320);
  tft.setRotation(1);
  tft.setSPISpeed(40000000);
  errorScreenBlocking = false;

  tft.fillScreen(COLOR_BG);

  // ---------- Инициализация типографского слоя ----------
  ui.begin(tft);
  ui.setColors(theme::TEXT, theme::BG);
  ui.setTransparent(true);

  // ---------- Заголовок "ЗАГРУЗКА" ----------
  ui.setFont(UiFont::Large);
  ui.box(0, 44, theme::SCREEN_W, 30, "ЗАГРУЗКА", UiHAlign::Center, UiVAlign::Middle, false);

  // ---------- Версия ----------
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(0, 78, theme::SCREEN_W, 18, VERSION, UiHAlign::Center, UiVAlign::Middle, false);

  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::ACCENT, theme::BG);
  ui.box(0, 96, theme::SCREEN_W, 16, "FreeRTOS + GitHub OTA",
         UiHAlign::Center, UiVAlign::Middle, false);

  // ---------- Иконки в ряд по центру ----------
  const uint8_t iconSize = 32;
  const uint8_t spacing = 20;
  const uint8_t totalWidth = 3 * iconSize + 2 * spacing;
  const uint8_t startX = (theme::SCREEN_W - totalWidth) / 2;
  const uint8_t iconsY = 140;

  drawIcon(startX, iconsY, iconOK, theme::OK);
  drawIcon(startX + iconSize + spacing, iconsY, iconWiFi, theme::ACCENT);
  drawIcon(startX + 2 * (iconSize + spacing), iconsY, iconSettings, theme::TEXT);

  vTaskDelay(pdMS_TO_TICKS(1000));

  // ========== 7. TASK POOL ==========
  if (!TaskPool::init()) {
    Serial.println("[ERROR] Failed to init TaskPool");
    ESP.restart();
  }

  // ========== 8. ФАЙЛОВАЯ СИСТЕМА ==========
  initializeDefaultCredentials();
  if (initFileSystem()) {
    ConfigManager::load();   // < Теперь загружает config.txt
    loadWiFiConfig();
  }

  // ========== 8b. MPU6050 (до WiFi — меньше шума на I2C при первом probe) ==========
  initializeDMP();

  connectConfiguredWiFi();

  // ========== 9. ПАМЯТЬ ==========
  MemoryMonitor::init();

  // ========== 10. ИНИЦИАЛИЗАЦИЯ ПЕРЕМЕННЫХ МЕНЮ ==========
  editReleaseDelay = constrain(ConfigManager::getReleaseDelay(), 1, 10);
  editInflateDelay = constrain(ConfigManager::getInflateDelay(), 1, 10);
  editTiltX = constrain(ConfigManager::getTiltThresholdX(), 0.0f, 3.0f);
  editTiltY = constrain(ConfigManager::getTiltThresholdY(), 0.0f, 3.0f);
  editPressureMin = constrain(ConfigManager::getPressureMin(), 0.1f, 5.0f);
  editPressureMax = constrain(ConfigManager::getPressureMax(), 1.0f, 8.0f);
  editMasterLowTenths = constrain((int)lroundf(ConfigManager::getMasterLowBar() * 10.0f), 0, 40);
  editNivCount = constrain(ConfigManager::getNivCount(), 1, 20);
  editTimeInterval = constrain(ConfigManager::getTimeInterval(), 1, 60);
  editContrast = constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX);
  editMovementPressureFront = constrain(ConfigManager::getMovementPressureFront(), 1.0f, 6.0f);
  editMovementPressureRear = constrain(ConfigManager::getMovementPressureRear(), 1.0f, 6.0f);
  editParkingPressure = constrain(ConfigManager::getParkingPressureBar(), 0.0f, 6.0f);

  // ===== 8.8.0: зеркала новых параметров =====
  editMasterCheck = constrain(ConfigManager::getMasterCheckSec(), 30, 600);
  editManualMaxTime = constrain(ConfigManager::getManualMaxTimeSec(), 1, 30);
  editPressStabilizeMs = constrain(ConfigManager::getPressureStabilizeMs(), 100, 2000);
  editPressIdleMin = constrain(ConfigManager::getPressureIdleMin(), 2, 30);
  editDeadband = constrain(ConfigManager::getPressureDeadband(), 0.05f, 0.5f);
  editCoarseZone = constrain(ConfigManager::getCoarseZoneRatio(), 0.2f, 0.9f);
  editFineZone = constrain(ConfigManager::getFineZoneRatio(), 0.05f, 0.3f);
  editWorsening = constrain(ConfigManager::getWorseningRatio(), 1.05f, 2.0f);
  editMoveDuration = constrain(ConfigManager::getMovementDurationSec(), 10, 120);
  editMoveSettle = constrain(ConfigManager::getMovementSettleSec(), 10, 120);
  editMoveCheck = constrain(ConfigManager::getMovementCheckSec(), 30, 300);
  editMoveTolerance = constrain(ConfigManager::getMovementTolerance(), 0.1f, 1.0f);
  editBacklightOff = constrain(ConfigManager::getBacklightOffMin(), 0, 30);
  editFrameMs = constrain(ConfigManager::getFrameMs(), 20, 200);
  editRedrawAngle = constrain(ConfigManager::getRedrawAngleThr(), 0.01f, 0.5f);
  editRedrawPressure = constrain(ConfigManager::getRedrawPressureThr(), 0.01f, 0.5f);
  editImuMotionDet = constrain(ConfigManager::getImuMotionDet(), 20, 255);
  editGyroThreshold = constrain(ConfigManager::getGyroThreshold(), 10, 200);
  editGyroBumpThreshold = constrain(ConfigManager::getGyroBumpThreshold(), 10, 200);
  editAccelThreshold = constrain(ConfigManager::getAccelThreshold(), 100, 3000);
  editZeroAngleX = constrain(ConfigManager::getZeroAngleX(), -45.0f, 45.0f);
  editZeroAngleY = constrain(ConfigManager::getZeroAngleY(), -45.0f, 45.0f);
  editImuKalmanMea = constrain(ConfigManager::getImuKalmanMea(), 0.5f, 25.0f);
  editImuKalmanEst = constrain(ConfigManager::getImuKalmanEst(), 0.5f, 25.0f);
  editImuKalmanQ = constrain(ConfigManager::getImuKalmanQ(), 0.001f, 0.100f);
  editImuPollMs = constrain(ConfigManager::getImuPollMs(), 15, 100);
  editImuFifoAvg = constrain(ConfigManager::getImuFifoAvg(), 1, 8);
  editImuEmaAlpha = constrain(ConfigManager::getImuEmaAlpha(), 0.05f, 0.50f);
  editImuEmaSpikeAlpha = constrain(ConfigManager::getImuEmaSpikeAlpha(), 0.05f, 0.50f);
  editImuEmaSpikeThr = constrain(ConfigManager::getImuEmaSpikeThr(), 0.5f, 5.0f);
  editImuSlewDps = constrain(ConfigManager::getImuSlewDps(), 5.0f, 120.0f);
  editImuPreset = 2;  // отображение спиннера: Баланс (параметры уже из config)
  lastUserActivityMs = millis();  // 8.8.0: старт отсчёта гашения подсветки

  applyRuntimeSettings();  // Калман/подсветка из config.txt сразу после загрузки

  // ========== 11. ИНИЦИАЛИЗАЦИЯ GEM ==========
  initGEM();
  gem.hideVersion();       // Скрыть версию на splash-экране
  gem.setSplashDelay(0);   // Отключить splash-экран
  gem.init();

  // ========== 13. ПОДСВЕТКА ==========
  applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));

  // ========== 14. ВЫВОДЫ КЛАПАНОВ ==========
  for (auto pin : bubPins) pinMode(pin, OUTPUT);
  pinMode(PIN_INFL, OUTPUT);
  pinMode(PIN_DEFL, OUTPUT);
  closeAllValves();

  // ========== 15. ИНИЦИАЛИЗАЦИЯ СОСТОЯНИЙ ==========
  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      currentSystemMode = SystemMode::MANUAL;
    }
  }
  currentMode = Mode::MANUAL;
  movementModeActive = false;
  movementEndTime = 0;
  movementPressureLastCheck = 0;

  lastMasterPressureCheckTime = millis();
  lastManualPressureCheckTime = millis();
  lastLevelingCheckTime = millis();
  lastLevelingAttemptTime = millis();
  lastHourResetTime = millis();
  levelingAttemptsThisHour = 0;
  pressureLimitReached = false;

  setAllManualTargetsFromCurrent();

  // ========== 16. TaskMonitor ==========
  TaskMonitor::init();
  initWatchdog();

  // ========== 17. СОЗДАНИЕ ЗАДАЧ ==========
  TaskConfig taskConfigs[] = {
    { "EventTask", eventHandlerTask, 4096, 2, 1, 20, nullptr },
    // Было 16384/14336: лишний стек дробил кучу → TLS BIGNUM alloc fail.
    { "ButtonTask", buttonTask, 8192, 4, 0, 20, nullptr },
    { "DisplayTask", displayTask, 8192, 2, 1, 33, nullptr },
    { "IMUTask", imuTask, 5632, 5, 0, 20, nullptr },
    { "PressTask", pressureTask, 4608, 3, 1, 2000, nullptr },
    { "ControlTask", controlTask, 5120, 3, 0, 500, nullptr },
    { "CalibTask", calibrationTask, 4096, 2, 1, 1000, nullptr },
    { "WatchdogTask", watchdogTask, 2048, 6, 1, 1000, nullptr },
    // TLS в куче; стек нужен под JsonDocument + esp_ota + serial harness
    { "OTATask", otaTask, 16384, 2, 1, 100, nullptr },
    { "ErrRecTask", errorRecoveryTask, 4096, 2, 1, 500, nullptr },
    { "ValveTask", valveTask, 4096, 4, 0, 10, nullptr }
  };

  taskIndex_Event = TaskPool::addTask(taskConfigs[0]);
  taskIndex_Button = TaskPool::addTask(taskConfigs[1]);
  taskIndex_Display = TaskPool::addTask(taskConfigs[2]);
  taskIndex_IMU = TaskPool::addTask(taskConfigs[3]);
  taskIndex_Pressure = TaskPool::addTask(taskConfigs[4]);
  taskIndex_Control = TaskPool::addTask(taskConfigs[5]);
  taskIndex_Calib = TaskPool::addTask(taskConfigs[6]);
  taskIndex_Watchdog = TaskPool::addTask(taskConfigs[7]);
  taskIndex_OTA = TaskPool::addTask(taskConfigs[8]);
  taskIndex_ErrRec = TaskPool::addTask(taskConfigs[9]);
  taskIndex_Valve = TaskPool::addTask(taskConfigs[10]);

  vTaskDelay(pdMS_TO_TICKS(100));

  if (taskIndex_Event == 0xFF || taskIndex_Button == 0xFF || taskIndex_Display == 0xFF || 
      taskIndex_IMU == 0xFF || taskIndex_Pressure == 0xFF || taskIndex_Control == 0xFF || 
      taskIndex_Calib == 0xFF || taskIndex_Watchdog == 0xFF || taskIndex_OTA == 0xFF || 
      taskIndex_ErrRec == 0xFF || taskIndex_Valve == 0xFF) {
    Serial.println("[ERROR] Failed to create some tasks!");
    ESP.restart();
  }

  Serial.println("[SETUP] All tasks created successfully");
  githubOtaReserveTlsHeap("post-tasks");

  uptimeHours = 0;

  emergencyButton.setHoldTimeout(EMERGENCY_HOLD_MS);
  emergencyButton.setDebTimeout(50);

  // ========== 18. ФИНАЛЬНЫЕ НАСТРОЙКИ ==========
  Serial.printf("[SETUP] final state: currentState=%d\n", (int)currentState);

  displayDirty = true;
  forceDisplayReset();
  vTaskDelay(pdMS_TO_TICKS(100));

  Logger::log(Logger::INFO, "SETUP", "Инициализация завершена");
}

void blinkErrorIcon() {
  // Совпадает с displayErrorScreen (iconY +20px); вызывается редко — основной blink там.
  static uint32_t lastBlinkTime = 0;
  static bool blinkState = true;
  const int16_t iconX = (SCREEN_WIDTH - 45) / 2;
  const int16_t iconY = 38;
  uint32_t now = millis();

  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    blinkState = !blinkState;

    if (blinkState) {
      drawIconB(iconX, iconY, err_Big, theme::TEXT);
    } else {
      tft.fillRect(iconX, iconY, 45, 45, COLOR_ERROR);
    }
  }
}

void printTaskInfo() {
  Serial.println("\n=== TASK INFO ===");
  for (uint8_t i = 0; i < TaskPool::getTaskCount(); i++) {
    const char *name = TaskPool::getTaskName(i);
    UBaseType_t stackFree = TaskPool::getTaskMinStack(i);
    bool enabled = TaskPool::isTaskEnabled(i);

    Serial.printf("%s: stack=%d bytes, enabled=%d\n",
                  name ? name : "Unknown",
                  stackFree,
                  enabled ? 1 : 0);

    if (stackFree < 200) {
      Serial.printf("  ?? WARNING: Task '%s' has very low stack!\n",
                    name ? name : "Unknown");
    }
  }

  Serial.printf("Total tasks: %d\n", TaskPool::getTaskCount());
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.println("==================\n");
}

/* ====================  SERIAL SELF-TEST HARNESS ====================
 * Команды (строка + \\n или \\r): TEST <CMD> [args]
 * Ответы: [TEST] ...  / маркеры TEST OK / TEST FAIL / TEST DONE
 * Хост-раннер: kamaz_leveler/_serial_selftest.py
 * ================================================================== */
static uint8_t g_selfTestPhase = 0;
static uint32_t g_selfTestAtMs = 0;
static uint8_t g_selfTestFails = 0;

static void testReplyOk(const char *msg) {
  Serial.printf("[TEST] OK %s\n", msg ? msg : "");
}

static void testReplyFail(const char *msg) {
  Serial.printf("[TEST] FAIL %s\n", msg ? msg : "");
}

static void testPrintHelp() {
  Serial.println("[TEST] HELP commands:");
  Serial.println("  TEST PING | STATUS | HELP | VER | STACK | CFG | HEAP | WDT");
  Serial.println("  TEST MENU OPEN|CLOSE|TOGGLE");
  Serial.println("  TEST BRIGHT <10..100> | DIM ON|OFF|TOGGLE");
  Serial.println("  TEST DIRTY | SAVE | RESETDISP");
  Serial.println("  TEST HEARTBEAT ON|OFF | DEBUG ON|OFF");
  Serial.println("  TEST PRESS | ADS | WIFI | WIFI SCAN | ERR | CALIB | BTN | I2C");
  Serial.println("  TEST IMU | IMU CFG | IMU STREAM [sec] | IMU STATS [sec]");
  Serial.println("  TEST IMU SET <key> <val> | IMU PRESET BAL|SMOOTH|FAST | IMU SAVE");
  Serial.println("  TEST MOT | MOT CFG | MOT STREAM [sec] | MOT STATS | MOT RESET | MOT SAVE");
  Serial.println("  TEST MOT SET gyro|bump|lin|det|dur|duration|settle|period <val>");
  Serial.println("  TEST MODE [MANUAL|AUTO|MOVEMENT]");
  Serial.println("  TEST OTA STATUS | OTA LIST");
  Serial.println("  TEST SELF | FULL   — smoke / multi-mode sequence");
}

static void testPrintPress() {
  float rawBar = -999.0f;
  bool ok = false;
  if (jhmReady) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
    if (i2c) ok = Jhm1200::readBar(rawBar);
  }
  Serial.printf("[TEST] PRESS master=%.2f pads=%.2f,%.2f,%.2f,%.2f zero=%.3f rawBar=%.3f jhm=%d st=0x%02X first=%d calib=%d\n",
                masterPressure, pressure[0], pressure[1], pressure[2], pressure[3],
                g_pressureZeroBar, ok ? rawBar : -1.0f, jhmReady ? 1 : 0, Jhm1200::lastStatus(),
                firstPressureMeasurementDone ? 1 : 0, calibrationCompleted ? 1 : 0);
}

static void testPrintImu() {
  Serial.printf("[TEST] IMU mpu=%d ang=%.3f,%.3f raw=%.3f,%.3f temp=%.1f\n",
                mpuOk ? 1 : 0, angleX, angleY, g_imuRawX, g_imuRawY, temperature);
  Serial.printf("[TEST] IMU_CFG mea=%.2f est=%.2f q=%.3f poll=%d fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f redraw=%.2f det=%d\n",
                ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                ConfigManager::getImuKalmanQ(), ConfigManager::getImuPollMs(),
                ConfigManager::getImuFifoAvg(), ConfigManager::getImuEmaAlpha(),
                ConfigManager::getImuEmaSpikeAlpha(), ConfigManager::getImuEmaSpikeThr(),
                ConfigManager::getImuSlewDps(), ConfigManager::getRedrawAngleThr(),
                ConfigManager::getImuMotionDet());
}

/** Применить пресет фильтров IMU сразу (без меню). */
static void imuApplyPreset(const char *name) {
  if (strcasecmp(name, "BAL") == 0 || strcasecmp(name, "BALANCE") == 0 || strcasecmp(name, "BALANCED") == 0) {
    // 10.1.28: ослаблен (ближе к быстрой реакции)
    ConfigManager::setImuKalmanMea(5.5f);
    ConfigManager::setImuKalmanEst(3.5f);
    ConfigManager::setImuKalmanQ(0.008f);
    ConfigManager::setImuPollMs(20);
    ConfigManager::setImuFifoAvg(2);
    ConfigManager::setImuEmaAlpha(0.28f);
    ConfigManager::setImuEmaSpikeAlpha(0.10f);
    ConfigManager::setImuEmaSpikeThr(1.8f);
    ConfigManager::setImuSlewDps(70.0f);
    ConfigManager::setRedrawAngleThr(0.05f);
  } else if (strcasecmp(name, "SMOOTH") == 0) {
    // 10.1.28: ослаблен (меньше залипание)
    ConfigManager::setImuKalmanMea(8.0f);
    ConfigManager::setImuKalmanEst(5.0f);
    ConfigManager::setImuKalmanQ(0.004f);
    ConfigManager::setImuPollMs(25);
    ConfigManager::setImuFifoAvg(4);
    ConfigManager::setImuEmaAlpha(0.18f);
    ConfigManager::setImuEmaSpikeAlpha(0.07f);
    ConfigManager::setImuEmaSpikeThr(1.4f);
    ConfigManager::setImuSlewDps(25.0f);
    ConfigManager::setRedrawAngleThr(0.07f);
  } else if (strcasecmp(name, "FAST") == 0) {
    ConfigManager::setImuKalmanMea(3.5f);
    ConfigManager::setImuKalmanEst(3.0f);
    ConfigManager::setImuKalmanQ(0.015f);
    ConfigManager::setImuPollMs(15);
    ConfigManager::setImuFifoAvg(1);
    ConfigManager::setImuEmaAlpha(0.50f);
    ConfigManager::setImuEmaSpikeAlpha(0.18f);
    ConfigManager::setImuEmaSpikeThr(2.5f);
    ConfigManager::setImuSlewDps(90.0f);
    ConfigManager::setRedrawAngleThr(0.04f);
  } else {
    return;
  }
  editImuKalmanMea = ConfigManager::getImuKalmanMea();
  editImuKalmanEst = ConfigManager::getImuKalmanEst();
  editImuKalmanQ = ConfigManager::getImuKalmanQ();
  editImuPollMs = ConfigManager::getImuPollMs();
  editImuFifoAvg = ConfigManager::getImuFifoAvg();
  editImuEmaAlpha = ConfigManager::getImuEmaAlpha();
  editImuEmaSpikeAlpha = ConfigManager::getImuEmaSpikeAlpha();
  editImuEmaSpikeThr = ConfigManager::getImuEmaSpikeThr();
  editImuSlewDps = ConfigManager::getImuSlewDps();
  editRedrawAngle = ConfigManager::getRedrawAngleThr();
  applyRuntimeSettings();
}

/** TEST IMU SET key value — live-тюнинг. */
static bool imuSetParam(const char *key, const char *valStr) {
  if (!key || !valStr || !*valStr) return false;
  const float fv = atof(valStr);
  const int iv = atoi(valStr);
  if (strcasecmp(key, "mea") == 0) ConfigManager::setImuKalmanMea(constrain(fv, 0.5f, 25.0f));
  else if (strcasecmp(key, "est") == 0) ConfigManager::setImuKalmanEst(constrain(fv, 0.5f, 25.0f));
  else if (strcasecmp(key, "q") == 0) ConfigManager::setImuKalmanQ(constrain(fv, 0.001f, 0.100f));
  else if (strcasecmp(key, "poll") == 0) ConfigManager::setImuPollMs(constrain(iv, 15, 100));
  else if (strcasecmp(key, "fifo") == 0) ConfigManager::setImuFifoAvg(constrain(iv, 1, 8));
  else if (strcasecmp(key, "ema") == 0) ConfigManager::setImuEmaAlpha(constrain(fv, 0.05f, 0.50f));
  else if (strcasecmp(key, "spike") == 0) ConfigManager::setImuEmaSpikeAlpha(constrain(fv, 0.05f, 0.50f));
  else if (strcasecmp(key, "thr") == 0) ConfigManager::setImuEmaSpikeThr(constrain(fv, 0.5f, 5.0f));
  else if (strcasecmp(key, "slew") == 0) ConfigManager::setImuSlewDps(constrain(fv, 5.0f, 120.0f));
  else if (strcasecmp(key, "redraw") == 0) ConfigManager::setRedrawAngleThr(constrain(fv, 0.01f, 0.5f));
  else return false;
  applyRuntimeSettings();
  return true;
}

static void imuStartStats(uint32_t sec) {
  if (sec < 1) sec = 1;
  if (sec > 30) sec = 30;
  g_imuStatN = 0;
  g_imuStatSpikes = 0;
  g_imuStatSumX = g_imuStatSumY = 0;
  g_imuStatSumXX = g_imuStatSumYY = 0;
  g_imuStatMaxStep = 0;
  g_imuStatsActive = true;
  g_imuStatsUntilMs = millis() + sec * 1000UL;
  Serial.printf("[IMU] STATS start %lus\n", (unsigned long)sec);
}

static void testPrintWifi() {
  const bool sta = (WiFi.status() == WL_CONNECTED);
  Serial.printf("[TEST] WIFI mode=%d sta=%d ssid=%s ip=%s rssi=%d heap=%u maxBlk=%u\n",
                (int)WiFi.getMode(), sta ? 1 : 0,
                sta ? WiFi.SSID().c_str() : sta_ssid,
                sta ? WiFi.localIP().toString().c_str() : "-",
                sta ? (int)WiFi.RSSI() : 0,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

static void testPrintOta() {
  Serial.printf("[TEST] OTA wifi=%d releases=%u sel=%d status=%s hdr=%s latest=%s inProg=%d pct=%d\n",
                (WiFi.status() == WL_CONNECTED) ? 1 : 0,
                (unsigned)otaReleaseCount, (int)otaSelectedIndex,
                otaListStatus, otaListHdrBuf, otaLatestTag,
                otaInProgress ? 1 : 0, otaProgress);
  for (uint8_t i = 0; i < otaReleaseCount && i < OTA_LIST_MAX; i++) {
    Serial.printf("[TEST] OTA_REL %u %s size=%lu\n", (unsigned)i, otaReleases[i].tag,
                  (unsigned long)otaReleases[i].size);
  }
}

static void testPrintBtn() {
  Serial.printf("[TEST] BTN gpio KN1=%d KN2=%d KN3=%d KN4=%d | menuPair=%d modePair=%d emergPair=%d navIgnore=%lu\n",
                digitalRead(PIN_BUT1), digitalRead(PIN_BUT2), digitalRead(PIN_BUT3), digitalRead(PIN_BUT4),
                menuPairPressedNow() ? 1 : 0, modePairPressedNow() ? 1 : 0,
                emergencyPairPressedNow() ? 1 : 0,
                (unsigned long)(g_menuNavIgnoreUntilMs > millis()
                                    ? (g_menuNavIgnoreUntilMs - millis())
                                    : 0));
}

static void testPrintErr() {
  const int n = ErrorHandler::getActiveErrorCount();
  Serial.printf("[TEST] ERR count=%d blocking=%d\n", n, errorScreenBlocking ? 1 : 0);
  for (int i = 0; i < n; i++) {
    ErrorHandler::Error e = ErrorHandler::getActiveErrorAt(i);
    Serial.printf("[TEST] ERR_ITEM %d code=%d %s\n", i, (int)e, ErrorHandler::getErrorMessage(e));
  }
}

static void testPrintCalib() {
  Serial.printf("[TEST] CALIB done=%d state=%d zeroBar=%.3f firstPress=%d\n",
                calibrationCompleted ? 1 : 0, (int)getSystemState(), g_pressureZeroBar,
                firstPressureMeasurementDone ? 1 : 0);
}

static void testScanI2c() {
  Serial.print("[TEST] I2C");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(30));
    if (!i2c) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  Serial.printf(" n=%u\n", (unsigned)found);
}

static void testPrintStatus() {
  Serial.printf("[TEST] STATUS menu=%d dim=%d bright=%d editBright=%d dirty=%d "
                "svc=%d mode=%d mpu=%d heap=%u minHeap=%u maxBlk=%u uptime=%lus req=%u\n",
                menuVisible ? 1 : 0,
                backlightDimmed ? 1 : 0,
                ConfigManager::getContrast(),
                editContrast,
                settingsChanged ? 1 : 0,
                (int)serviceScreen,
                (int)currentSystemMode,
                mpuOk ? 1 : 0,
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMinFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(),
                (unsigned long)(millis() / 1000UL),
                (unsigned)g_menuReq);
  Serial.printf("[TEST] STATUS angles=%.2f,%.2f master=%.2f pads=%.2f/%.2f/%.2f/%.2f frameMs=%d blMin=%d idleMs=%lu\n",
                angleX, angleY, masterPressure,
                pressure[0], pressure[1], pressure[2], pressure[3],
                ConfigManager::getFrameMs(),
                ConfigManager::getBacklightOffMin(),
                (unsigned long)(millis() - lastUserActivityMs));
}

static void testPrintMot() {
  const char *ms = "MANUAL";
  if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
  else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
  const uint32_t now = millis();
  float settleLeft = 0.0f;
  if (g_motLastActivityMs > 0) {
    const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
    const uint32_t idle = now - g_motLastActivityMs;
    settleLeft = (idle >= settleMs) ? 0.0f : (settleMs - idle) * 0.001f;
  }
  Serial.printf(
      "[TEST] MOT hold=%d mot=%d motSt=0x%02X busy=%d gyro=%.0f gdlt=%.0f thr=%.0f "
      "bump=%.0f bdlt=%.0f bthr=%.0f bbusy=%d "
      "lin=%.0f ldlt=%.0f lthr=%.0f lbusy=%d int=0x%02X mode=%s "
      "movActive=%d settleLeft=%.1f\n",
      g_motLastHold ? 1 : 0, g_motLastMotPulse ? 1 : 0, (unsigned)g_motLastMotStatus,
      g_motLastGyroBusy ? 1 : 0,
      g_motLastGyroRms, g_motLastGyroDelta, g_motLastGyroThr,
      g_motLastGyroBump, g_motLastGyroBumpDelta, g_motLastGyroBumpThr,
      g_motLastGyroBumpBusy ? 1 : 0, g_motLastLinAccRaw, g_motLastLinAccRms, g_motLastLinAccThr,
      g_motLastLinAccBusy ? 1 : 0, (unsigned)g_motLastIntStatus, ms,
      movementModeActive ? 1 : 0, settleLeft);
  Serial.printf(
      "[TEST] MOT_CFG duration=%ds settle=%ds gyroMenu=%d gyroThr=%.0f bumpMenu=%d bumpThr=%.0f "
      "linThr=%d det=%d motDur=%ums period=%lums\n",
      ConfigManager::getMovementDurationSec(), ConfigManager::getMovementSettleSec(),
      ConfigManager::getGyroThreshold(), (float)ConfigManager::getGyroThreshold() * 8.0f,
      ConfigManager::getGyroBumpThreshold(), (float)ConfigManager::getGyroBumpThreshold() * 8.0f,
      ConfigManager::getAccelThreshold(),
      ConfigManager::getImuMotionDet(), (unsigned)g_motDurMs,
      (unsigned long)g_motStreamPeriodMs);
}

static void testPrintMotStats() {
  const uint32_t now = millis();
  const uint32_t dt = (g_motStatsSinceMs > 0 && now > g_motStatsSinceMs)
                          ? (now - g_motStatsSinceMs)
                          : 0;
  const float sec = dt > 0 ? dt * 0.001f : 0.0f;
  const float n = g_motSampleCount > 0 ? (float)g_motSampleCount : 1.0f;
  Serial.printf(
      "[TEST] MOT_STATS n=%lu sec=%.1f mot%%=%.1f busy%%=%.1f bump%%=%.1f lin%%=%.1f holdEdges=%lu modeEnter=%lu "
      "lastGyro=%.0f thr=%.0f lastBump=%.0f bthr=%.0f lastLin=%.0f lthr=%.0f\n",
      (unsigned long)g_motSampleCount, sec,
      100.0f * (float)g_motMotPulseCount / n, 100.0f * (float)g_motGyroBusyCount / n,
      100.0f * (float)g_motGyroBumpBusyCount / n,
      100.0f * (float)g_motLinAccBusyCount / n,
      (unsigned long)g_motHoldEdgeCount, (unsigned long)g_motModeEnterCount,
      g_motLastGyroRms, g_motLastGyroThr, g_motLastGyroBump, g_motLastGyroBumpThr,
      g_motLastLinAccRms, g_motLastLinAccThr);
}

static void testMotResetStats() {
  g_motSampleCount = 0;
  g_motMotPulseCount = 0;
  g_motGyroBusyCount = 0;
  g_motGyroBumpBusyCount = 0;
  g_motLinAccBusyCount = 0;
  g_motHoldEdgeCount = 0;
  g_motModeEnterCount = 0;
  g_motStatsSinceMs = millis();
}

void processTestCommandLine(char *line) {
  if (!line || line[0] == '\0') return;
  if (strncmp(line, "TEST", 4) != 0 && strncmp(line, "test", 4) != 0) return;
  char *cmd = line + 4;
  while (*cmd == ' ') cmd++;

      if (strcasecmp(cmd, "PING") == 0) {
        testReplyOk("PONG");
      } else if (strcasecmp(cmd, "HELP") == 0) {
        testPrintHelp();
        testReplyOk("HELP");
      } else if (strcasecmp(cmd, "VER") == 0) {
        Serial.printf("[TEST] VER %s\n", VERSION);
        testReplyOk("VER");
      } else if (strcasecmp(cmd, "STATUS") == 0) {
        testPrintStatus();
        testReplyOk("STATUS");
      } else if (strcasecmp(cmd, "HEAP") == 0) {
        Serial.printf("[TEST] HEAP free=%u min=%u\n",
                      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
        testReplyOk("HEAP");
      } else if (strcasecmp(cmd, "STACK") == 0) {
        printTaskInfo();
        testReplyOk("STACK");
      } else if (strcasecmp(cmd, "CFG") == 0) {
        ConfigManager::dumpToSerial();
        testReplyOk("CFG");
      } else if (strcasecmp(cmd, "MENU OPEN") == 0) {
        requestMenuOpen("TEST");
        testReplyOk("MENU_OPEN_REQ");
      } else if (strcasecmp(cmd, "MENU CLOSE") == 0) {
        requestMenuClose("TEST");
        testReplyOk("MENU_CLOSE_REQ");
      } else if (strcasecmp(cmd, "MENU TOGGLE") == 0) {
        if (menuVisible) requestMenuClose("TEST");
        else requestMenuOpen("TEST");
        testReplyOk("MENU_TOGGLE_REQ");
      } else if (strncasecmp(cmd, "BRIGHT ", 7) == 0) {
        int v = atoi(cmd + 7);
        v = constrain(v, CONTRAST_MIN, CONTRAST_MAX);
        editContrast = v;
        ConfigManager::setContrast(v);
        applyBacklightPwm(v);
        backlightDimmed = false;
        settingsChanged = true;
        lastUserActivityMs = millis();
        Serial.printf("[TEST] BRIGHT %d\n", v);
        testReplyOk("BRIGHT");
      } else if (strcasecmp(cmd, "DIM ON") == 0) {
        backlightDimmed = true;
        applyBacklightPwm(CONTRAST_DIM);
        testReplyOk("DIM_ON");
      } else if (strcasecmp(cmd, "DIM OFF") == 0) {
        backlightDimmed = false;
        applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        lastUserActivityMs = millis();
        testReplyOk("DIM_OFF");
      } else if (strcasecmp(cmd, "DIM TOGGLE") == 0) {
        if (backlightDimmed) {
          backlightDimmed = false;
          applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
          lastUserActivityMs = millis();
          testReplyOk("DIM_OFF");
        } else {
          backlightDimmed = true;
          applyBacklightPwm(CONTRAST_DIM);
          testReplyOk("DIM_ON");
        }
      } else if (strcasecmp(cmd, "DIRTY") == 0) {
        settingsChanged = true;
        testReplyOk("DIRTY");
      } else if (strcasecmp(cmd, "SAVE") == 0) {
        saveMenuSettings();
        testReplyOk("SAVE");
      } else if (strcasecmp(cmd, "RESETDISP") == 0) {
        // Безопасно: только флаг — DisplayTask подхватит через dirty;
        // forceDisplayReset без мьютекса не вызываем.
        if (!menuVisible) {
          displayDirty = true;
          // Запрос полного сброса через close-path нельзя — меню закрыто.
          // Ставим dirty; для hard reset — краткий take.
          MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(200));
          if (guard) {
            forceDisplayReset(true);
            testReplyOk("RESETDISP");
          } else {
            testReplyFail("RESETDISP_NO_MUTEX");
          }
        } else {
          testReplyFail("RESETDISP_MENU_OPEN");
        }
      } else if (strcasecmp(cmd, "HEARTBEAT ON") == 0) {
        g_testHeartbeat = true;
        testReplyOk("HEARTBEAT_ON");
      } else if (strcasecmp(cmd, "HEARTBEAT OFF") == 0) {
        g_testHeartbeat = false;
        testReplyOk("HEARTBEAT_OFF");
      } else if (strcasecmp(cmd, "DEBUG ON") == 0) {
        g_testDebugDump = true;
        testReplyOk("DEBUG_ON");
      } else if (strcasecmp(cmd, "DEBUG OFF") == 0) {
        g_testDebugDump = false;
        testReplyOk("DEBUG_OFF");
      } else if (strcasecmp(cmd, "WDT") == 0) {
        Serial.printf("[TEST] WDT alive t=%lu\n", (unsigned long)millis());
        testReplyOk("WDT");
      } else if (strcasecmp(cmd, "PRESS") == 0) {
        testPrintPress();
        testReplyOk("PRESS");
      } else if (strcasecmp(cmd, "ADS") == 0) {
        testPrintPress();
        testReplyOk("ADS");
      } else if (strncasecmp(cmd, "MOT", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        const char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg == '\0' || strcasecmp(arg, "CFG") == 0) {
          testPrintMot();
          testReplyOk(*arg ? "MOT_CFG" : "MOT");
        } else if (strncasecmp(arg, "STREAM", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 30;
          if (sec < 1) sec = 1;
          if (sec > 600) sec = 600;  // до 10 мин — тест в машине
          if (g_motStatsSinceMs == 0) testMotResetStats();
          g_motStreamUntilMs = millis() + (uint32_t)sec * 1000UL;
          Serial.printf("[MOT] STREAM start %ds period=%lums (gyro thr=%.0f = menu%d×8, linThr=%d)\n",
                        sec, (unsigned long)g_motStreamPeriodMs, g_motLastGyroThr > 0
                            ? g_motLastGyroThr
                            : (float)ConfigManager::getGyroThreshold() * 8.0f,
                        ConfigManager::getGyroThreshold(),
                        ConfigManager::getAccelThreshold());
          testReplyOk("MOT_STREAM");
        } else if (strcasecmp(arg, "STATS") == 0) {
          testPrintMotStats();
          testReplyOk("MOT_STATS");
        } else if (strcasecmp(arg, "RESET") == 0) {
          testMotResetStats();
          Serial.println("[MOT] stats reset");
          testReplyOk("MOT_RESET");
        } else if (strcasecmp(arg, "SAVE") == 0) {
          // Пишем live ConfigManager в LittleFS (не через устаревшие edit*).
          editGyroThreshold = ConfigManager::getGyroThreshold();
          editGyroBumpThreshold = ConfigManager::getGyroBumpThreshold();
          editAccelThreshold = ConfigManager::getAccelThreshold();
          editMoveSettle = ConfigManager::getMovementSettleSec();
          editMoveDuration = ConfigManager::getMovementDurationSec();
          const bool saved = saveConfig();
          if (saved) {
            settingsChanged = false;
            Serial.println("[MOT] config.txt сохранён");
            testPrintMot();
            testReplyOk("MOT_SAVE");
          } else {
            testReplyFail("MOT_SAVE");
          }
        } else if (strncasecmp(arg, "SET", 3) == 0) {
          char key[16] = {0};
          char val[24] = {0};
          if (sscanf(arg + 3, " %15s %23s", key, val) == 2) {
            const int iv = atoi(val);
            bool ok = true;
            if (strcasecmp(key, "gyro") == 0) {
              ConfigManager::setGyroThreshold(constrain(iv, 10, 200));
              editGyroThreshold = ConfigManager::getGyroThreshold();
            } else if (strcasecmp(key, "bump") == 0 || strcasecmp(key, "pitch") == 0) {
              ConfigManager::setGyroBumpThreshold(constrain(iv, 10, 200));
              editGyroBumpThreshold = ConfigManager::getGyroBumpThreshold();
            } else if (strcasecmp(key, "lin") == 0 || strcasecmp(key, "accel") == 0) {
              ConfigManager::setAccelThreshold(constrain(iv, 100, 3000));
              editAccelThreshold = ConfigManager::getAccelThreshold();
            } else if (strcasecmp(key, "det") == 0) {
              ConfigManager::setImuMotionDet(constrain(iv, 20, 255));
              editImuMotionDet = ConfigManager::getImuMotionDet();
              if (mpuOk) {
                MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(80));
                if (i2c) {
                  mpu.setMotionDetectionThreshold((uint8_t)ConfigManager::getImuMotionDet());
                }
              }
            } else if (strcasecmp(key, "dur") == 0) {
              // MOT_DUR (мс), не путать с durationSec входа в MOVEMENT.
              g_motDurMs = (uint8_t)constrain(iv, 1, 255);
              if (mpuOk) {
                MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(80));
                if (i2c) {
                  mpu.setMotionDetectionDuration(g_motDurMs);
                }
              }
            } else if (strcasecmp(key, "duration") == 0) {
              ConfigManager::setMovementDurationSec(constrain(iv, 10, 120));
              editMoveDuration = ConfigManager::getMovementDurationSec();
            } else if (strcasecmp(key, "settle") == 0) {
              ConfigManager::setMovementSettleSec(constrain(iv, 10, 120));
              editMoveSettle = ConfigManager::getMovementSettleSec();
            } else if (strcasecmp(key, "period") == 0) {
              g_motStreamPeriodMs = (uint32_t)constrain(iv, 50, 1000);
            } else {
              ok = false;
            }
            if (ok) {
              settingsChanged = true;
              testPrintMot();
              testReplyOk("MOT_SET");
            } else {
              testReplyFail("MOT_SET");
            }
          } else {
            testReplyFail("MOT_SET");
          }
        } else {
          testReplyFail("MOT_UNKNOWN");
        }
      } else if (strncasecmp(cmd, "IMU", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        const char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg == '\0' || strcasecmp(arg, "CFG") == 0) {
          testPrintImu();
          testReplyOk(*arg ? "IMU_CFG" : "IMU");
        } else if (strncasecmp(arg, "STREAM", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 3;
          if (sec < 1) sec = 1;
          if (sec > 20) sec = 20;
          g_imuStreamUntilMs = millis() + (uint32_t)sec * 1000UL;
          Serial.printf("[IMU] STREAM start %ds\n", sec);
          testReplyOk("IMU_STREAM");
        } else if (strncasecmp(arg, "STATS", 5) == 0) {
          const char *p = arg + 5;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 5;
          imuStartStats((uint32_t)sec);
          // OK придёт из imuTask по завершении; здесь ACK старта
          testReplyOk("IMU_STATS_START");
        } else if (strncasecmp(arg, "PRESET", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          if (*p == '\0') {
            testReplyFail("IMU_PRESET_ARG");
          } else {
            imuApplyPreset(p);
            testPrintImu();
            testReplyOk("IMU_PRESET");
          }
        } else if (strncasecmp(arg, "SET", 3) == 0) {
          char key[16] = {0};
          char val[24] = {0};
          if (sscanf(arg + 3, " %15s %23s", key, val) == 2 && imuSetParam(key, val)) {
            testPrintImu();
            testReplyOk("IMU_SET");
          } else {
            testReplyFail("IMU_SET");
          }
        } else if (strcasecmp(arg, "SAVE") == 0) {
          // синхронизируем edit* из live-конфига и пишем в LittleFS
          editImuKalmanMea = ConfigManager::getImuKalmanMea();
          editImuKalmanEst = ConfigManager::getImuKalmanEst();
          editImuKalmanQ = ConfigManager::getImuKalmanQ();
          editImuPollMs = ConfigManager::getImuPollMs();
          editImuFifoAvg = ConfigManager::getImuFifoAvg();
          editImuEmaAlpha = ConfigManager::getImuEmaAlpha();
          editImuEmaSpikeAlpha = ConfigManager::getImuEmaSpikeAlpha();
          editImuEmaSpikeThr = ConfigManager::getImuEmaSpikeThr();
          editImuSlewDps = ConfigManager::getImuSlewDps();
          editRedrawAngle = ConfigManager::getRedrawAngleThr();
          saveMenuSettings();
          testReplyOk("IMU_SAVE");
        } else {
          testReplyFail("IMU_UNKNOWN");
        }
      } else if (strcasecmp(cmd, "WIFI") == 0) {
        testPrintWifi();
        testReplyOk("WIFI");
      } else if (strcasecmp(cmd, "WIFI SCAN") == 0) {
        requestWiFiSetup();
        testReplyOk("WIFI_SCAN");
      } else if (strcasecmp(cmd, "BTN") == 0) {
        testPrintBtn();
        testReplyOk("BTN");
      } else if (strcasecmp(cmd, "ERR") == 0) {
        testPrintErr();
        testReplyOk("ERR");
      } else if (strcasecmp(cmd, "CALIB") == 0) {
        testPrintCalib();
        testReplyOk("CALIB");
      } else if (strcasecmp(cmd, "I2C") == 0) {
        testScanI2c();
        testReplyOk("I2C");
      } else if (strcasecmp(cmd, "OTA STATUS") == 0) {
        testPrintOta();
        testReplyOk("OTA_STATUS");
      } else if (strcasecmp(cmd, "OTA LIST") == 0) {
        if (WiFi.status() != WL_CONNECTED) {
          testReplyFail("OTA_LIST_NO_WIFI");
        } else {
          strlcpy(otaListStatus, "загрузка списка…", sizeof(otaListStatus));
          requestGitHubOtaFetchList();
          Serial.println("[TEST] OTA LIST req");
          testReplyOk("OTA_LIST_REQ");
        }
      } else if (strncasecmp(cmd, "MODE", 4) == 0) {
        const char *arg = cmd + 4;
        while (*arg == ' ') arg++;
        if (*arg == '\0') {
          const char *ms = "MANUAL";
          if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
          else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
          Serial.printf("[TEST] MODE %s\n", ms);
          testReplyOk("MODE");
        } else if (strcasecmp(arg, "MANUAL") == 0) {
          {
            MutexGuard guard(xStateMutex);
            if (guard) currentSystemMode = SystemMode::MANUAL;
          }
          currentMode = Mode::MANUAL;
          setAllManualTargetsFromCurrent();
          forceDisplayReset(true);
          setDisplayDirty();
          Serial.println("[TEST] MODE > MANUAL");
          testReplyOk("MODE_MANUAL");
        } else if (strcasecmp(arg, "AUTO") == 0) {
          currentSystemMode = SystemMode::AUTO;
          currentMode = Mode::AUTO;
          lastLevelingCheckTime = millis();
          forceDisplayReset(true);
          setDisplayDirty();
          Serial.println("[TEST] MODE > AUTO");
          testReplyOk("MODE_AUTO");
        } else if (strcasecmp(arg, "MOVEMENT") == 0) {
          if (!mpuOk) {
            testReplyFail("MODE_MOVEMENT_NO_MPU");
          } else {
            previousMode = currentSystemMode;
            currentSystemMode = SystemMode::MOVEMENT;
            movementModeActive = true;
            movementStartTime = millis();
            movementStartMs = movementStartTime;
            forceDisplayReset(true);
            setDisplayDirty();
            Serial.println("[TEST] MODE > MOVEMENT");
            testReplyOk("MODE_MOVEMENT");
          }
        } else {
          testReplyFail("MODE_BAD_ARG");
        }
      } else if (strcasecmp(cmd, "SELF") == 0) {
        if (g_selfTestPhase != 0) {
          testReplyFail("SELF_BUSY");
        } else {
          g_selfTestPhase = 1;
          g_selfTestAtMs = millis();
          g_selfTestFails = 0;
          Serial.println("[TEST] SELF begin");
          testReplyOk("SELF_START");
        }
      } else if (strcasecmp(cmd, "FULL") == 0) {
        if (g_selfTestPhase != 0) {
          testReplyFail("FULL_BUSY");
        } else {
          // Расширенная последовательность: SELF + режимы + датчики
          g_selfTestPhase = 10;
          g_selfTestAtMs = millis();
          g_selfTestFails = 0;
          Serial.println("[TEST] FULL begin");
          testReplyOk("FULL_START");
        }
      } else {
        Serial.printf("[TEST] UNKNOWN '%s'\n", cmd);
        testReplyFail("UNKNOWN");
      }
}

static void processSerialTestCommands() {
  // Serial линии обычно забирает OTA-task → processTestCommandLine.
  // Здесь — запасной путь + фоновый SELF/FULL.
  static char line[96];
  static uint8_t len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      len = 0;
      if (line[0] == 0) continue;
      processTestCommandLine(line);
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;
    }
  }

  // Фоновый smoke: OPEN > wait > CLOSE > bright/dim > STATUS
  // FULL (phase>=10): + PRESS/IMU/WIFI + MODE AUTO/MANUAL/MOVEMENT
  if (g_selfTestPhase != 0 && (int32_t)(millis() - g_selfTestAtMs) >= 0) {
    switch (g_selfTestPhase) {
      case 1:
        requestMenuOpen("SELF");
        g_selfTestPhase = 2;
        g_selfTestAtMs = millis() + 400;
        break;
      case 2:
        if (!menuVisible) {
          testReplyFail("SELF_OPEN");
          g_selfTestFails++;
        } else {
          testReplyOk("SELF_OPEN");
        }
        requestMenuClose("SELF");
        g_selfTestPhase = 3;
        g_selfTestAtMs = millis() + 600;
        break;
      case 3:
        if (menuVisible) {
          testReplyFail("SELF_CLOSE");
          g_selfTestFails++;
        } else {
          testReplyOk("SELF_CLOSE");
        }
        editContrast = 40;
        ConfigManager::setContrast(40);
        applyBacklightPwm(40);
        g_selfTestPhase = 4;
        g_selfTestAtMs = millis() + 200;
        break;
      case 4:
        backlightDimmed = true;
        applyBacklightPwm(CONTRAST_DIM);
        g_selfTestPhase = 5;
        g_selfTestAtMs = millis() + 200;
        break;
      case 5:
        backlightDimmed = false;
        applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        testPrintStatus();
        printTaskInfo();
        if (g_selfTestFails == 0) {
          Serial.println("[TEST] SELF DONE PASS");
          testReplyOk("SELF_PASS");
        } else {
          Serial.printf("[TEST] SELF DONE FAIL fails=%u\n", (unsigned)g_selfTestFails);
          testReplyFail("SELF_FAIL");
        }
        g_selfTestPhase = 0;
        break;

      // ----- FULL sequence -----
      case 10:
        testPrintPress();
        testPrintImu();
        testPrintWifi();
        testPrintCalib();
        testPrintErr();
        testPrintBtn();
        testScanI2c();
        testReplyOk("FULL_SENSORS");
        g_selfTestPhase = 11;
        g_selfTestAtMs = millis() + 100;
        break;
      case 11:
        requestMenuOpen("FULL");
        g_selfTestPhase = 12;
        g_selfTestAtMs = millis() + 500;
        break;
      case 12:
        if (!menuVisible) { testReplyFail("FULL_MENU_OPEN"); g_selfTestFails++; }
        else testReplyOk("FULL_MENU_OPEN");
        requestMenuClose("FULL");
        g_selfTestPhase = 13;
        g_selfTestAtMs = millis() + 700;
        break;
      case 13:
        if (menuVisible) { testReplyFail("FULL_MENU_CLOSE"); g_selfTestFails++; }
        else testReplyOk("FULL_MENU_CLOSE");
        currentSystemMode = SystemMode::AUTO;
        currentMode = Mode::AUTO;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 14;
        g_selfTestAtMs = millis() + 400;
        break;
      case 14:
        if (currentSystemMode != SystemMode::AUTO) { testReplyFail("FULL_AUTO"); g_selfTestFails++; }
        else testReplyOk("FULL_AUTO");
        {
          MutexGuard guard(xStateMutex);
          if (guard) currentSystemMode = SystemMode::MANUAL;
        }
        currentMode = Mode::MANUAL;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 15;
        g_selfTestAtMs = millis() + 400;
        break;
      case 15:
        if (currentSystemMode != SystemMode::MANUAL) { testReplyFail("FULL_MANUAL"); g_selfTestFails++; }
        else testReplyOk("FULL_MANUAL");
        if (mpuOk) {
          previousMode = SystemMode::MANUAL;
          currentSystemMode = SystemMode::MOVEMENT;
          movementModeActive = true;
          movementStartTime = millis();
          movementStartMs = movementStartTime;
          forceDisplayReset(true);
          setDisplayDirty();
          g_selfTestPhase = 16;
          g_selfTestAtMs = millis() + 500;
        } else {
          testReplyOk("FULL_MOVEMENT_SKIP");
          g_selfTestPhase = 17;
          g_selfTestAtMs = millis() + 100;
        }
        break;
      case 16:
        if (currentSystemMode != SystemMode::MOVEMENT) { testReplyFail("FULL_MOVEMENT"); g_selfTestFails++; }
        else testReplyOk("FULL_MOVEMENT");
        movementModeActive = false;
        currentSystemMode = SystemMode::MANUAL;
        currentMode = Mode::MANUAL;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 17;
        g_selfTestAtMs = millis() + 400;
        break;
      case 17:
        testPrintStatus();
        testPrintOta();
        testPrintPress();
        if (g_selfTestFails == 0) {
          Serial.println("[TEST] FULL DONE PASS");
          testReplyOk("FULL_PASS");
        } else {
          Serial.printf("[TEST] FULL DONE FAIL fails=%u\n", (unsigned)g_selfTestFails);
          testReplyFail("FULL_FAIL");
        }
        g_selfTestPhase = 0;
        break;
      default:
        g_selfTestPhase = 0;
        break;
    }
  }
}

/* ====================  LOOP ==================== */
void loop() {
  static uint32_t lastHourCounter = 0;
  static uint32_t lastMemoryCheck = 0;
  static uint32_t lastStackCheck = 0;
  static uint32_t lastTaskInfoPrint = 0;

  processSerialTestCommands();

  if (g_testHeartbeat) {
    static uint32_t lastHb = 0;
    if (millis() - lastHb >= 1000) {
      lastHb = millis();
      Serial.printf("[TEST] HB t=%lu heap=%u maxBlk=%u menu=%d mode=%d\n",
                    (unsigned long)millis(), (unsigned)ESP.getFreeHeap(),
                    (unsigned)ESP.getMaxAllocHeap(),
                    menuVisible ? 1 : 0, (int)currentSystemMode);
    }
  }

  if (g_testDebugDump) {
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg >= 2000) {
      lastDbg = millis();
      testPrintStatus();
      testPrintPress();
      testPrintImu();
    }
  }

  if (millis() - lastHourCounter >= 3600000) {
    uptimeHours++;
    lastHourCounter = millis();
  }

  if (millis() - lastMemoryCheck >= 5000) {
    MemoryMonitor::checkMemory();
    lastMemoryCheck = millis();
  }

  if (millis() - lastStackCheck > 30000) {
    lastStackCheck = millis();
    for (uint8_t i = 0; i < TaskPool::getTaskCount(); i++) {
      UBaseType_t stackFree = TaskPool::getTaskMinStack(i);
      if (stackFree < 200) {
        const char *name = TaskPool::getTaskName(i);
        Serial.printf("[STACK] ?? Task '%s' has only %d bytes free!\n",
                      name ? name : "Unknown", stackFree);
      }
    }
  }

  // ? Добавлена диагностика задач раз в минуту
  if (millis() - lastTaskInfoPrint > 60000) {
    lastTaskInfoPrint = millis();
    printTaskInfo();
  }

  static uint32_t lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 60000) {
    Serial.printf("[MEM] Free heap: %d bytes\n", ESP.getFreeHeap());
    lastHeapCheck = millis();
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}

