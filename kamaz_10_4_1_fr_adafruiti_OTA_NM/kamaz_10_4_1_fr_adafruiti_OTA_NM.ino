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
#include "config_manager.h"
#include "error_handler.h"
#include "task_monitor.h"
#include "memory_monitor.h"
#include "task_watchdog.h"
#include "task_recovery.h"
#include "task_event.h"
#include "test_harness.h"
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

constexpr uint32_t WDT_TIMEOUT_MS = 30000;

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
void initOTA();
void startOTAMode();
void stopOTAMode();
void handleOTA();
void displayOTAScreen();
// displayWiFiSetupScreen -> wifi_setup.h
//void buildMenu(gm::Builder &b);
// startWiFiSetup -> wifi_setup.h
// connectConfiguredWiFi -> wifi_setup.h
// startFallbackAccessPoint -> wifi_setup.h
// initializeDMP — see imu_dmp.h
// initializeJhm1200 — see pressure_read.h
// checkPressureLimits — see pressure_read.h
void requestMenuOpen(const char *via);
void requestMenuClose(const char *via);
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
// otaTask -> task_ota.h

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

// attitude gauges / updateMainTiltLive — see ui_screens.cpp

// maintainMovementPressure — see pressure_read.cpp

// AutoLevelingController — see auto_level.h

// sendValveCommandSync — see valve_ctrl.cpp






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


// checkPressureLimits — see pressure_read.cpp

// startManualOperation / stopManualOperation — see valve_ctrl.cpp


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

// pressureTask — see task_pressure.cpp

// calibrationTask — see task_calib.cpp

// displayTask — see task_display.cpp





/* otaTask -> task_ota.h/cpp */



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



/* ====================  LOOP ==================== */
void loop() {
  testHarnessPoll();
  systemMaintenanceTick();
  vTaskDelay(pdMS_TO_TICKS(100));
}
