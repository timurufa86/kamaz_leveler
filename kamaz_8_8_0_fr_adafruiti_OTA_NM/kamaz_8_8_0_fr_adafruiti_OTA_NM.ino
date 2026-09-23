/****************************************************************************************
 *  Kamaz-Leveler - FreeRTOS + OTA (версия 8.7.0)
 *  Оптимизации: RAII мьютексы, EventBus, TaskPool, улучшенная обработка ошибок
 *  Типографика: U8g2_for_Adafruit_GFX (модули ui_theme.h / ui_fonts.h / ui_text.*)
 *  Меню: вход — удержание пары КН3+КН4 (GPIO15+GPIO17); аварийный стоп — КН4+КН5
 *****************************************************************************************/

#include <Arduino.h>
#include "mutex_guard.h"
#include "semver_utils.h"
#include "github_ota_request.h"
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
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <GyverFilters.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_ADS1X15.h>
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
#define ENABLE_MPU6050 1
#define ENABLE_SIMULATION 0
#define SIMULATE_AUTO_MODE 0
#define SIMULATE_ERRORS 0
#define SIMULATE_MENU_AUTO_ENTER 0
#define DEBUG_PRINT 0
#define DEBUG_STACK 0

#define EB_NO_FOR
#define EB_NO_CALLBACK
#define EB_NO_COUNTER
#define EB_NO_BUFFER

#define EB_DEB_TIME 50
#define EB_CLICK_TIME 500
#define EB_HOLD_TIME 600
#define EB_STEP_TIME 200

//const uint8_t PAD_COUNT = 4;
#define PAD_COUNT 4

bool adsInitialized = false;  // флаг готовности ADS1015

/* ====================  КОНСТАНТЫ ==================== */
constexpr int16_t SCREEN_WIDTH = 320;
constexpr int16_t SCREEN_HEIGHT = 240;

constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 50;

/* ====================  НАСТРОЙКИ МЕНЮ ====================  ST77XX_WHITE */
constexpr uint8_t MENU_COLS = 60;          // символов в строке
constexpr uint8_t MENU_ROWS = 9;           // строк на экране (240/24 ≈ 10)
constexpr uint8_t MENU_TOP_OFFSET = 14;    // отступ сверху
constexpr uint8_t MENU_LEFT_PADDING = 15;  // отступ слева (для внешних панелей)
constexpr uint8_t ROW_HEIGHT = 22;         // высота строки (важно!)
constexpr uint8_t CURSOR_WIDTH = 2;
// Колонка значений в меню GEM. Раньше сюда ошибочно ставился MENU_LEFT_PADDING (15),
// из-за чего длина заголовка считалась как (15-5)/6 = 1 символ и подписи пунктов
// обрезались до одной буквы. 150 px даёт 24 символа заголовка и 27 символов значения.
constexpr uint8_t MENU_VALUES_LEFT_OFFSET = 150;

/* ====================  КОМБИНАЦИИ КНОПОК ====================
 *  Меню открывается удержанием ПАРЫ кнопок «КН3 + КН4» (GPIO15 + GPIO17)
 *  из любого режима и экрана. Аварийная остановка — удержание пары
 *  «КН4 + КН5» (GPIO17 + GPIO34); она обрабатывается первой и подавляет меню.
 */
constexpr uint32_t MENU_COMBO_HOLD_MS = 2000;      // удержание пары для меню
constexpr uint32_t MENU_COMBO_DEBOUNCE_MS = 50;    // антидребезг пары
constexpr uint32_t EMERGENCY_HOLD_MS = 2000;       // удержание пары для аварийного стопа
constexpr uint16_t MENU_BG_COLOR = ST77XX_BLACK;
constexpr uint16_t MENU_TEXT_COLOR = ST77XX_WHITE;
constexpr uint16_t MENU_SEL_COLOR = ST77XX_BLUE;

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
constexpr const char *VERSION = "V 8.8.0 FreeRTOS OTA";

/* Пины */
constexpr uint8_t PIN_OLED_SDA = 21;
constexpr uint8_t PIN_OLED_SCL = 22;
constexpr uint8_t PIN_TFT_MOSI = 23;
constexpr uint8_t PIN_TFT_SCLK = 18;
constexpr uint8_t PIN_TFT_CS = 5;
constexpr uint8_t PIN_TFT_DC = 16;
constexpr uint8_t PIN_TFT_RST = 19;
constexpr uint8_t PIN_TFT_BL = 4;

constexpr uint8_t PIN_BUB1 = 32;
constexpr uint8_t PIN_BUB2 = 33;
constexpr uint8_t PIN_BUB3 = 25;
constexpr uint8_t PIN_BUB4 = 26;
constexpr uint8_t PIN_INFL = 27;
constexpr uint8_t PIN_DEFL = 14;

constexpr uint8_t PIN_BUT1 = 13;
constexpr uint8_t PIN_BUT2 = 12;
constexpr uint8_t PIN_BUT3 = 15;
constexpr uint8_t PIN_BUT4 = 17;
// GPIO34 is input-only and has no internal pull-up. Use an external pull-up.
constexpr uint8_t PIN_BUT5 = 34;


// ========== ADS1015 НАСТРОЙКИ ==========
constexpr uint8_t ADS1015_ADDRESS = 0x48;        // Адрес по умолчанию
uint8_t adsDetectedAddress = ADS1015_ADDRESS;
constexpr uint8_t ADS1015_PRESSURE_CHANNEL = 0;  // Канал 0 для датчика давления
constexpr float ADS1015_MAX_VOLTAGE = 3.3f;      // Опорное напряжение
constexpr uint16_t ADS1015_MAX_VALUE = 2047;     // 11-битный АЦП (2048 шагов)

/* Параметры времени */
constexpr uint32_t CALIB_TIME_MS = 15000;
constexpr uint32_t MANUAL_MAX_TIME = 10000;
constexpr uint32_t MASTER_PRESSURE_CHECK_INTERVAL_MS = 240000;
constexpr uint32_t MANUAL_PRESSURE_CHECK_INTERVAL_MS = 120000;
constexpr uint32_t LEVELING_ATTEMPT_COOLDOWN_MS = 3600000;
constexpr uint32_t PRESSURE_LIMIT_WARNING_DURATION_MS = 5000;
constexpr uint32_t MOVEMENT_PRESSURE_CHECK_INTERVAL_MS = 120000;
constexpr uint32_t MOVEMENT_DURATION_MS = 30000;
constexpr uint32_t MOVEMENT_SETTLE_TIME_MS = 30000;
constexpr float MOVEMENT_PRESSURE_TOLERANCE = 0.2f;
constexpr int GYRO_MOTION_THRESHOLD = 50;
constexpr int ACCEL_MOTION_THRESHOLD = 1000;

/* Переменные для меню */
static int editReleaseDelay;
static int editInflateDelay;
static float editTiltX;
static float editTiltY;
static float editPressureMin;
static float editPressureMax;
static int editNivCount;
static int editTimeInterval;
static int editContrast;
static float editMovementPressureFront;
static float editMovementPressureRear;
static bool settingsChanged = false;

/* ===== 8.8.0: зеркала новых параметров меню ===== */
static int   editMasterCheck;        // период проверки магистрали, с
static int   editManualMaxTime;      // максимальное время ручной операции, с
static float editDeadband;           // зона нечувствительности по давлению, бар
static float editCoarseZone;         // грубая зона авторежима (доля порога)
static float editFineZone;           // точная зона авторежима (доля порога)
static float editWorsening;          // порог «стало хуже» (множитель)
static int   editMoveDuration;       // длительность ожидания движения, с
static int   editMoveSettle;         // время успокоения после движения, с
static int   editMoveCheck;          // период проверки давления после движения, с
static float editMoveTolerance;      // допуск давления после движения, бар
static int   editGyroThreshold;      // порог гироскопа детектора движения
static int   editAccelThreshold;     // порог акселерометра детектора движения
static int   editBacklightOff;       // гашение подсветки, мин (0 = никогда)
static int   editFrameMs;            // интервал кадра дисплея, мс
static float editRedrawAngle;        // порог перерисовки по углу, °
static float editRedrawPressure;     // порог перерисовки по давлению, бар
static int   editImuMotionDet;       // аппаратный порог детектора движения MPU
static float editZeroAngleX;         // программный нуль углов X, °
static float editZeroAngleY;         // программный нуль углов Y, °

/* 8.8.0: обработчик изменения пункта меню со спиннером объявлен НИЖЕ по файлу
   (перед initGEM) — в .ino первое определение функции не должно предшествовать
   объявлению типов/enum'ов и глобальных переменных, иначе автогенерация прототипов
   Arduino вставляет прототипы в начало файла и компиляция падает. */


/* ====================  SPINNER ДЛЯ МЕНЮ ==================== */

// --- Spinner для целых чисел (1-10, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_10 = { .step = 1, .min = 1, .max = 10 };
GEMSpinner spinnerRelease(spinnerInt1_10);  // Вр.СБРОС
GEMSpinner spinnerInflate(spinnerInt1_10);  // Вр.НАКАЧ

// --- Spinner для целых чисел (1-20, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_20 = { .step = 1, .min = 1, .max = 20 };
GEMSpinner spinnerNivCount(spinnerInt1_20);  // Попыток в час

// --- Spinner для целых чисел (1-60, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_60 = { .step = 1, .min = 1, .max = 60 };
GEMSpinner spinnerTimeInterval(spinnerInt1_60);  // Интервал

// --- Spinner для целых чисел (1-100, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_100 = { .step = 1, .min = 1, .max = 100 };
GEMSpinner spinnerContrast(spinnerInt1_100);  // Яркость

// --- Spinner для float (0.0-3.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat0_3 = { .step = 0.1f, .min = 0.0f, .max = 3.0f };
GEMSpinner spinnerTiltX(spinnerFloat0_3);  // Поперечный
GEMSpinner spinnerTiltY(spinnerFloat0_3);  // Продольный

// --- Spinner для float (0.1-5.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat0_1_5 = { .step = 0.1f, .min = 0.1f, .max = 5.0f };
GEMSpinner spinnerPressureMin(spinnerFloat0_1_5);  // P.МИНИМУМ

// --- Spinner для float (1.0-8.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat1_8 = { .step = 0.1f, .min = 1.0f, .max = 8.0f };
GEMSpinner spinnerPressureMax(spinnerFloat1_8);  // P.МАКСИМУМ

// --- Spinner для float (1.0-6.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat1_6 = { .step = 0.1f, .min = 1.0f, .max = 6.0f };
GEMSpinner spinnerMovementFront(spinnerFloat1_6);  // ПЕРЕДНИЕ
GEMSpinner spinnerMovementRear(spinnerFloat1_6);   // ЗАДНИЕ

/* ===== 8.8.0: спиннеры новых параметров ===== */
// --- целочисленные ---
GEMSpinnerBoundariesInt spinnerInt30_600 = { .step = 30, .min = 30, .max = 600 };     // Проверка МП
GEMSpinnerBoundariesInt spinnerInt1_30 = { .step = 1, .min = 1, .max = 30 };          // Макс. время операции
GEMSpinnerBoundariesInt spinnerInt10_120 = { .step = 5, .min = 10, .max = 120 };      // Длительность/успокоение
GEMSpinnerBoundariesInt spinnerInt30_300 = { .step = 10, .min = 30, .max = 300 };     // Проверка давления
GEMSpinnerBoundariesInt spinnerInt10_200 = { .step = 5, .min = 10, .max = 200 };      // Порог гироскопа
GEMSpinnerBoundariesInt spinnerInt200_3000 = { .step = 100, .min = 200, .max = 3000 }; // Порог акселерометра
GEMSpinnerBoundariesInt spinnerInt0_30 = { .step = 1, .min = 0, .max = 30 };          // Гасить подсветку
GEMSpinnerBoundariesInt spinnerInt20_200 = { .step = 5, .min = 20, .max = 200 };      // Интервал кадра
GEMSpinnerBoundariesInt spinnerInt20_255 = { .step = 5, .min = 20, .max = 255 };      // Порог детектора MPU

// --- float ---
GEMSpinnerBoundariesFloat spinnerFloat005_05 = { .step = 0.05f, .min = 0.05f, .max = 0.5f };  // Зона нечувствительности
GEMSpinnerBoundariesFloat spinnerFloat02_09 = { .step = 0.05f, .min = 0.2f, .max = 0.9f };    // Грубая зона
GEMSpinnerBoundariesFloat spinnerFloat005_03 = { .step = 0.01f, .min = 0.05f, .max = 0.3f };  // Точная зона
GEMSpinnerBoundariesFloat spinnerFloat105_2 = { .step = 0.05f, .min = 1.05f, .max = 2.0f };   // «Стало хуже»
GEMSpinnerBoundariesFloat spinnerFloat01_1 = { .step = 0.05f, .min = 0.1f, .max = 1.0f };     // Допуск давления
GEMSpinnerBoundariesFloat spinnerFloat001_05 = { .step = 0.01f, .min = 0.01f, .max = 0.5f };  // Пороги перерисовки
GEMSpinnerBoundariesFloat spinnerFloatNeg3_3 = { .step = 0.01f, .min = -3.0f, .max = 3.0f };  // Нуль углов

GEMSpinner spinnerMasterCheck(spinnerInt30_600);        // Проверка МП
GEMSpinner spinnerManualMaxTime(spinnerInt1_30);        // Макс. время операции
GEMSpinner spinnerDeadband(spinnerFloat005_05);         // Зона нечувствительности
GEMSpinner spinnerCoarseZone(spinnerFloat02_09);        // Грубая зона
GEMSpinner spinnerFineZone(spinnerFloat005_03);         // Точная зона
GEMSpinner spinnerWorsening(spinnerFloat105_2);         // Порог «стало хуже»
GEMSpinner spinnerMoveDuration(spinnerInt10_120);       // Длительность движения
GEMSpinner spinnerMoveSettle(spinnerInt10_120);         // Время успокоения
GEMSpinner spinnerMoveCheck(spinnerInt30_300);          // Проверка давления после движения
GEMSpinner spinnerMoveTolerance(spinnerFloat01_1);      // Допуск давления
GEMSpinner spinnerGyroThr(spinnerInt10_200);            // Порог гироскопа
GEMSpinner spinnerAccelThr(spinnerInt200_3000);         // Порог акселерометра
GEMSpinner spinnerBacklightOff(spinnerInt0_30);         // Гасить подсветку, мин
GEMSpinner spinnerFrameMs(spinnerInt20_200);            // Интервал кадра
GEMSpinner spinnerRedrawAngle(spinnerFloat001_05);      // Порог перерисовки углов
GEMSpinner spinnerRedrawPressure(spinnerFloat001_05);   // Порог перерисовки давления
GEMSpinner spinnerImuMotionDet(spinnerInt20_255);       // Порог детектора движения MPU
GEMSpinner spinnerZeroAngleX(spinnerFloatNeg3_3);       // Нуль углов X
GEMSpinner spinnerZeroAngleY(spinnerFloatNeg3_3);       // Нуль углов Y

/* Переменные состояния */
enum Pad : uint8_t {
  PAD_FRONT_LEFT,
  PAD_FRONT_RIGHT,
  PAD_REAR_LEFT,
  PAD_REAR_RIGHT,
  PAD_COUNT_ENUM
};

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

constexpr uint16_t COLOR_BG = ST77XX_BLACK;
constexpr uint16_t COLOR_TEXT = ST77XX_WHITE;
constexpr uint16_t COLOR_HIGHLIGHT = ST77XX_BLUE;
constexpr uint16_t COLOR_ERROR = ST77XX_RED;
constexpr uint16_t COLOR_SUCCESS = ST77XX_GREEN;
constexpr uint16_t COLOR_WARNING = ST77XX_YELLOW;
constexpr uint16_t COLOR_OTA = ST77XX_CYAN;
constexpr uint16_t COLOR_WHITE = ST77XX_WHITE;

/* Глобальные переменные */
bool errorScreenBlocking = false;

volatile bool calibrationCompleted = false;          // флаг калибровки
volatile bool firstPressureMeasurementDone = false;  // ← ДОБАВИТЬ

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
volatile bool wifiSetupActive = false;
volatile bool wifiScanInProgress = false;
volatile bool wifiConnected = false;
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
constexpr uint32_t VALVE_OPERATION_TIMEOUT_MS = 15000;
constexpr uint32_t VALVE_MAX_COMMAND_MS = 15000;
// 8.8.0: формат конфигурации 3 — добавлены параметры меню (магистраль/клапаны/авторежим/
// движение/дисплей) и калибровка IMU. Файлы версий 1 и 2 читаются, новые поля = значения по умолчанию.
constexpr uint32_t CONFIG_FORMAT_VERSION = 3;

constexpr uint32_t WDT_TIMEOUT_MS = 30000;
constexpr uint32_t TASK_WDT_TIMEOUT_MS = 10000;
constexpr uint8_t TASK_COUNT = 10;

constexpr uint32_t MANUAL_TARGET_CHECK_INTERVAL_MS = 120000;
constexpr uint32_t MANUAL_ADJUSTMENT_COOLDOWN_MS = 3000;
constexpr float MANUAL_PRESSURE_TOLERANCE = 0.1f;

/* ========== МЬЮТЕКС ДЛЯ ЗАЩИТЫ currentState ========== */
SemaphoreHandle_t xStateMutex = nullptr;

SystemState currentState = SystemState::BOOT;
SystemMode currentSystemMode = SystemMode::MANUAL;
SystemMode previousMode = SystemMode::MANUAL;
TestState currentTestState = TestState::IDLE;

// ========== ГЛОБАЛЬНАЯ ПЕРЕМЕННАЯ displayDirty (ОБЪЯВЛЕНА) ==========
volatile bool displayDirty = true;




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

/* ====================  EventBus ==================== */
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

class EventBus {
private:
  static QueueHandle_t eventQueue_;
  static constexpr size_t QUEUE_SIZE = 128;
  static SemaphoreHandle_t mutex_;
  static uint32_t droppedEvents_;
  static uint32_t maxDepth_;

public:
  static bool init() {
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) return false;

    eventQueue_ = xQueueCreate(QUEUE_SIZE, sizeof(Event));
    if (eventQueue_ == nullptr) return false;

    Serial.println("[EVENTBUS] Queue created");
    return true;
  }

  static bool publish(const Event &event, TickType_t timeout = pdMS_TO_TICKS(100)) {
    if (eventQueue_ == nullptr) return false;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (uxQueueSpacesAvailable(eventQueue_) == 0) {
        droppedEvents_++;
        if (droppedEvents_ % 10 == 1) {
          Serial.printf("[WARN] EventBus queue full, dropped %d events\n", droppedEvents_);
        }
        xSemaphoreGive(mutex_);
        return false;
      }
      xSemaphoreGive(mutex_);
    }

    Event eventCopy = event;
    BaseType_t result = xQueueSend(eventQueue_, &eventCopy, timeout);
    size_t depth = uxQueueMessagesWaiting(eventQueue_);
    if (depth > maxDepth_) maxDepth_ = depth;
    if (result != pdTRUE) droppedEvents_++;
    return result == pdTRUE;
  }

  static bool receive(Event &event, TickType_t timeout = portMAX_DELAY) {
    if (eventQueue_ == nullptr) return false;
    return xQueueReceive(eventQueue_, &event, timeout) == pdTRUE;
  }

  static size_t available() {
    if (eventQueue_ == nullptr) return 0;
    return uxQueueMessagesWaiting(eventQueue_);
  }

  static void flush() {
    if (eventQueue_ == nullptr) return;
    xQueueReset(eventQueue_);
    droppedEvents_ = 0;
  }

  static uint32_t getDroppedCount() {
    return droppedEvents_;
  }

  static uint32_t dropped() { return droppedEvents_; }
  static uint32_t maxDepth() { return maxDepth_; }
};

QueueHandle_t EventBus::eventQueue_ = nullptr;
SemaphoreHandle_t EventBus::mutex_ = nullptr;
uint32_t EventBus::droppedEvents_ = 0;
uint32_t EventBus::maxDepth_ = 0;

/* ====================  TaskPool ==================== */
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
  static bool init() {
    poolMutex_ = xSemaphoreCreateMutex();
    return poolMutex_ != nullptr;
  }

  static uint8_t addTask(const TaskConfig &config) {
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

  static bool removeTask(uint8_t index) {
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

  static void enableTask(uint8_t index) {
    if (poolMutex_ == nullptr) return;
    if (index >= taskCount_) return;

    if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (index < taskCount_) tasks_[index].enabled = true;
      xSemaphoreGive(poolMutex_);
    }
  }

  static void disableTask(uint8_t index) {
    if (poolMutex_ == nullptr) return;
    if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (index < taskCount_) tasks_[index].enabled = false;
      xSemaphoreGive(poolMutex_);
    }
  }

  static bool isTaskEnabled(uint8_t index) {
    if (poolMutex_ == nullptr) return false;
    bool result = false;
    if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (index < taskCount_) result = tasks_[index].enabled;
      xSemaphoreGive(poolMutex_);
    }
    return result;
  }

  static const char *getTaskName(uint8_t index) {
    if (poolMutex_ == nullptr) return nullptr;
    const char *name = nullptr;
    if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (index < taskCount_) name = tasks_[index].name;
      xSemaphoreGive(poolMutex_);
    }
    return name;
  }

  static void markRun(uint8_t index) {
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
              Serial.printf("[TASKPOOL] ⚠️ Task '%s' stack critical: %d bytes free\n",
                            tasks_[index].name, currentStack);
            }
          }
        }
      }
      xSemaphoreGive(poolMutex_);
    }
  }

  static uint32_t getTimeUntilNextRun(uint8_t index) {
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

  static uint8_t getTaskCount() {
    return taskCount_;
  }

  static UBaseType_t getTaskMinStack(uint8_t index) {
    if (poolMutex_ == nullptr) return 0;
    UBaseType_t result = 0;
    if (xSemaphoreTake(poolMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (index < taskCount_) result = tasks_[index].minStackFree;
      xSemaphoreGive(poolMutex_);
    }
    return result;
  }
};

TaskPool::TaskHandle TaskPool::tasks_[16];
uint8_t TaskPool::taskCount_ = 0;
SemaphoreHandle_t TaskPool::poolMutex_ = nullptr;

uint8_t taskIndex_Event = 0xFF;
uint8_t taskIndex_Button = 0xFF;
uint8_t taskIndex_Display = 0xFF;
uint8_t taskIndex_IMU = 0xFF;
uint8_t taskIndex_Pressure = 0xFF;
uint8_t taskIndex_Control = 0xFF;
uint8_t taskIndex_Calib = 0xFF;
uint8_t taskIndex_Watchdog = 0xFF;
uint8_t taskIndex_OTA = 0xFF;
uint8_t taskIndex_ErrRec = 0xFF;
uint8_t taskIndex_Valve = 0xFF;

/* ====================  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==================== */
SPIClass spi(VSPI);
Adafruit_ST7789 tft(&spi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GEM_adafruit_gfx gem(tft);  // передаём ссылку на TFT

void menuExitAction();   // Кн4 на главной странице меню: сохранить и закрыть

GEMPage mainPage("Главное меню", menuExitAction);
// Подстраницы создаются сразу с родителем: GEM сам добавляет первым пунктом
// «Назад» (стрелка влево) и корректно обрабатывает возврат по Кн4.
GEMPage systemPage("Система", mainPage);
GEMPage testPage("Тестирование", mainPage);
GEMPage valvePage("Клапаны", mainPage);
GEMPage tiltPage("Наклон", mainPage);
GEMPage pressurePage("Давление", mainPage);
GEMPage autoPage("Авторежим", mainPage);
GEMPage displayPage("Дисплей", mainPage);
GEMPage movementPage("Движение", mainPage);
GEMPage infoPage("Информация", mainPage);
// 8.8.0: новые страницы — ручной тест клапанов, IMU/MPU6050 и просмотр настроек
GEMPage imuPage("IMU / MPU6050", mainPage);
GEMPage settingsViewPage("Настройки", mainPage);

/* --- Страница «Информация»: строки обновляются динамически (refreshInfoPage) ---
 * Важно: LABEL-пункт создаётся конструктором с ОДНИМ аргументом (GEMItem.cpp:43
 * выставляет type = GEM_ITEM_LABEL). Вариант ("текст", nullptr, GEM_ITEM_LABEL)
 * даёт readonly-ссылку, а не метку.
 */
static GEMItem itemInfoVersion("Версия: …");
// 8.8.0: строки страницы «Настройки» (обновляются в refreshSettingsView)
static GEMItem itemSet1("Давление: …");
static GEMItem itemSet2("Наклон: …");
static GEMItem itemSet3("Клапаны: …");
static GEMItem itemSet4("Авторежим: …");
static GEMItem itemSet5("Движение: …");
static GEMItem itemSet6("Дисплей: …");
static GEMItem itemSet7("IMU: …");
static GEMItem itemInfoMode("Режим: …");
static GEMItem itemInfoMpu("MPU: …");
static GEMItem itemInfoMaster("МП: …");
static GEMItem itemInfoWiFi("Wi-Fi: …");
static GEMItem itemInfoSystem("Аптайм: …");
static GEMItem itemInfoErrors("Ошибки: …");


MPU6050 mpu;
GKalman filterX(40, 40, 0.5f);
GKalman filterY(40, 40, 0.5f);
GMedian<5, int> pressureFilter;

// ========== ADS1015 ==========
Adafruit_ADS1015 ads;  // Объект для работы с ADS1015

Button button0(PIN_BUT1, INPUT_PULLUP, LOW);
Button button1(PIN_BUT2, INPUT_PULLUP, LOW);
Button button2(PIN_BUT3, INPUT_PULLUP, LOW);
Button button3(PIN_BUT4, INPUT_PULLUP, LOW);
// GPIO34 has no internal pull-up; install an external pull-up resistor.
Button button4(PIN_BUT5, INPUT, LOW);

VirtButton emergencyButton;
VirtButton menuCombo;   // пара КН3+КН4 (GPIO15+GPIO17) — открытие меню

SemaphoreHandle_t xValveMutex = nullptr;
SemaphoreHandle_t xDisplayMutex = nullptr;
SemaphoreHandle_t xConfigMutex = nullptr;
SemaphoreHandle_t xCalibMutex = nullptr;
SemaphoreHandle_t xTestMutex = nullptr;
SemaphoreHandle_t xCommandMutex = nullptr;

QueueHandle_t xIMUQueue = nullptr;
QueueHandle_t xPressureQueue = nullptr;
QueueHandle_t xValveQueue = nullptr;
QueueHandle_t xPressureWakeupQueue = nullptr;

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

struct IMUData {
  float angleX;
  float angleY;
  float temperature;
};

struct PressureData {
  float pressure[PAD_COUNT];
  float masterPressure;
};

float angleX = 0, angleY = 0, temperature = 0;
float pressure[PAD_COUNT] = { 0 };
float masterPressure = 0;
volatile bool mpuOk = false;
volatile bool menuVisible = false;
//bool menuRendered = false;         // ← ДОБАВИТЬ!
uint32_t lastMenuInteraction = 0;  // ← ДОБАВИТЬ!

bool isMoving = false;
bool prolongedMovementDetected = false;
//uint32_t lastMovementTime = 0;
uint32_t movementStartTime = 0;

bool manualControlActive = false;
Pad manualPadIndex = PAD_FRONT_LEFT;
bool manualInflate = false;
uint32_t manualStartTime = 0;

Mode currentMode = Mode::MANUAL;

bool otaMode = false;
volatile bool otaInProgress = false;
volatile bool otaValveLock = false;
int otaProgress = 0;
char otaStatus[32] = "";

constexpr uint8_t bubPins[PAD_COUNT] = { PIN_BUB1, PIN_BUB2, PIN_BUB3, PIN_BUB4 };
constexpr const char *padNames[PAD_COUNT] = { "ПЛ", "ПП", "ЗЛ", "ЗП" };

uint32_t lastLevelingCheckTime = 0;
uint32_t lastLevelingAttemptTime = 0;
volatile uint32_t levelingAttemptsThisHour = 0;
uint32_t lastHourResetTime = 0;

uint32_t lastMasterPressureCheckTime = 0;
uint32_t lastManualPressureCheckTime = 0;
float manualTargetPressure[PAD_COUNT] = { 3.0f, 3.0f, 3.0f, 3.0f };
bool manualTargetSet[PAD_COUNT] = { true, true, true, true };
bool pressureLimitReached = false;

uint32_t movementEndTime = 0;
bool movementModeActive = false;
uint32_t movementPressureLastCheck = 0;

volatile bool calibrationValid = false;
int g_pressureZeroRaw = 0;


IMUData lastDisplayedIMU = { 0 };
PressureData lastDisplayedPressure = { 0 };
SystemMode lastDisplayedMode = SystemMode::MANUAL;
bool lastDisplayedMoving = false;

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

static LastCommand lastCmd;
static ValveErrorCounter valveErrorCounter;
static volatile uint32_t valveQueueDropCount = 0;
static volatile uint32_t eventQueueDropCount = 0;
static volatile uint32_t imuQueueDropCount = 0;
static volatile uint32_t pressureQueueDropCount = 0;
static volatile uint32_t valveEmergencyStopCount = 0;
static volatile uint32_t maxValveQueueDepth = 0;
static volatile uint32_t maxEventQueueDepth = 0;
static volatile uint32_t maxStackLowEvents = 0;
static volatile bool valveStopRequested = false;

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
void setDisplayDirty();
void emergencyStop();
void startManualOperation(Pad padIdx, bool inflate);
void stopManualOperation();
void closeAllValves();
void setValve(Pad pad, bool state);
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs);
float readPressure();
void initWatchdog();
void initOTA();
void startOTAMode();
void stopOTAMode();
void handleOTA();
void displayOTAScreen();
void displayWiFiSetupScreen();
void displayErrorScreen();
void displayMainScreen();
void displayCalibrationScreen();
//void buildMenu(gm::Builder &b);
bool initFileSystem();
bool loadConfig();
bool saveConfig();
bool loadWiFiConfig();
bool saveWiFiConfig();
void startWiFiSetup();
void connectConfiguredWiFi();
void startFallbackAccessPoint();
void initializeDMP();
void resetSystemErrors();
bool checkPressureLimits();
void saveMenuSettings();
void applyRuntimeSettings();  // 8.8.0: применение настроек к железу без перезагрузки
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs);
void maintainMovementPressure();
bool detectMotionFromIMU();
void checkAndAdjustMasterPressure();
void maintainManualPressure();
void setManualTargetPressure(Pad pad);
void setAllManualTargetsFromCurrent();
void startValveTest();
void runValveTestLogic();
void updateTestDisplay();
void requestPressureMeasurement();
void forceDisplayReset(bool force = false);
void initializeDefaultCredentials();
bool checkGitHubUpdate(bool install);
bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag);

void buttonTask(void *pvParameters);
void displayTask(void *pvParameters);
void imuTask(void *pvParameters);
void pressureTask(void *pvParameters);
void controlTask(void *pvParameters);
void calibrationTask(void *pvParameters);
void watchdogTask(void *pvParameters);
void otaTask(void *pvParameters);
void errorRecoveryTask(void *pvParameters);
void valveTask(void *pvParameters);
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
constexpr uint8_t OTA_LIST_MAX = 5;
constexpr char GITHUB_RELEASES_URL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=5";

struct OtaRelease {
  char tag[16] = "";
  char date[11] = "";   // YYYY-MM-DD
  uint32_t size = 0;
  char binUrl[176] = "";
  char shaUrl[176] = "";
  char sha256[65] = "";  // заполняется проверкой sha256-ассета
};

OtaRelease otaReleases[OTA_LIST_MAX];
volatile uint8_t otaReleaseCount = 0;
volatile int8_t otaSelectedIndex = -1;
char otaListStatus[64] = "список не загружен";

// --- Страницы раздела ---
GEMPage otaPage("Обновления", mainPage);
GEMPage otaListPage("Прошивки (GitHub)", otaPage);
GEMPage otaCardPage("Релиз", otaListPage);

// --- Прототипы обработчиков пунктов (реализация ниже) ---
static void otaFetchList();
static void otaInstallLast();
static void otaSelectRelease(uint8_t idx);
static void otaCardInstall();
static void otaCardCheckSha();
static void otaArduinoMode();
static void otaExitMode();
static void otaListRefresh();
static void refreshOtaCard();

// --- Пункты страницы «Обновления» ---
static GEMItem itemOtaCurrent("Версия: …");
static GEMItem itemOtaStatus("Состояние: …");
static GEMItem itemOtaFetch("Проверить обновления", []() { otaFetchList(); });
static GEMItem itemOtaList("Список прошивок", otaListPage);
static GEMItem itemOtaInstallLast("Установить последнюю", []() { otaInstallLast(); });
static GEMItem itemOtaArduino("Режим ArduinoOTA (Wi-Fi)", []() { otaArduinoMode(); });
static GEMItem itemOtaExit("Выход из режима OTA", []() { otaExitMode(); });

// --- Пункты страницы «Прошивки (GitHub)» ---
static char otaItemTitle[OTA_LIST_MAX][34];
static GEMItem itemOtaRel0("— нет данных —", []() { otaSelectRelease(0); });
static GEMItem itemOtaRel1("— нет данных —", []() { otaSelectRelease(1); });
static GEMItem itemOtaRel2("— нет данных —", []() { otaSelectRelease(2); });
static GEMItem itemOtaRel3("— нет данных —", []() { otaSelectRelease(3); });
static GEMItem itemOtaRel4("— нет данных —", []() { otaSelectRelease(4); });
static GEMItem *const otaRelItems[OTA_LIST_MAX] = {
  &itemOtaRel0, &itemOtaRel1, &itemOtaRel2, &itemOtaRel3, &itemOtaRel4
};
static GEMItem itemOtaListRefresh("Обновить список", []() { otaListRefresh(); });

// --- Пункты карточки релиза ---
static GEMItem itemCardTag("Релиз: —");
static GEMItem itemCardInfo("данных нет");
static GEMItem itemCardStatus("обновите список");
static GEMItem itemCardSha("SHA-256: не проверен");
static GEMItem itemCardInstall("УСТАНОВИТЬ", []() { otaCardInstall(); });
static GEMItem itemCardCheckSha("Проверить SHA-256", []() { otaCardCheckSha(); });

/* --- Сетевые операции (вызываются из otaTask) --- */

/** Загрузить список последних релизов. true, если получен хотя бы один. */
static bool fetchGitHubReleaseList() {
  if (WiFi.status() != WL_CONNECTED) {
    strlcpy(otaListStatus, "нет Wi-Fi", sizeof(otaListStatus));
    Serial.println("[GH-OTA] Список: нет Wi-Fi");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, GITHUB_RELEASES_URL)) {
    strlcpy(otaListStatus, "ошибка соединения", sizeof(otaListStatus));
    return false;
  }
  http.addHeader("User-Agent", "kamaz-leveler");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const int code = http.GET();
  if (code != HTTP_CODE_OK) {
    snprintf(otaListStatus, sizeof(otaListStatus), "GitHub HTTP %d", code);
    Serial.printf("[GH-OTA] Список: HTTP %d\n", code);
    http.end();
    return false;
  }

  JsonDocument filter;
  filter[0]["tag_name"] = true;
  filter[0]["published_at"] = true;
  filter[0]["assets"][0]["name"] = true;
  filter[0]["assets"][0]["size"] = true;
  filter[0]["assets"][0]["browser_download_url"] = true;

  JsonDocument doc;
  const DeserializationError err =
      deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    strlcpy(otaListStatus, "ошибка JSON", sizeof(otaListStatus));
    Serial.printf("[GH-OTA] Список: JSON %s\n", err.c_str());
    return false;
  }

  uint8_t n = 0;
  for (JsonObject rel : doc.as<JsonArray>()) {
    if (n >= OTA_LIST_MAX) break;
    OtaRelease &r = otaReleases[n];
    strlcpy(r.tag, rel["tag_name"] | "", sizeof(r.tag));
    const char *published = rel["published_at"] | "";
    if (strlen(published) >= 10) {
      memcpy(r.date, published, 10);
      r.date[10] = '\0';
    } else {
      r.date[0] = '\0';
    }
    r.size = 0;
    r.binUrl[0] = '\0';
    r.shaUrl[0] = '\0';
    r.sha256[0] = '\0';

    for (JsonObject asset : rel["assets"].as<JsonArray>()) {
      const char *name = asset["name"] | "";
      const char *url = asset["browser_download_url"] | "";
      if (strcmp(name, GITHUB_ASSET_NAME) == 0) {
        strlcpy(r.binUrl, url, sizeof(r.binUrl));
        r.size = asset["size"] | 0UL;
      } else if (strcmp(name, GITHUB_SHA256_ASSET_NAME) == 0) {
        strlcpy(r.shaUrl, url, sizeof(r.shaUrl));
      }
    }
    if (r.binUrl[0] != '\0' && r.shaUrl[0] != '\0') n++;
  }

  otaReleaseCount = n;
  snprintf(otaListStatus, sizeof(otaListStatus), "релизов: %u", n);
  Serial.printf("[GH-OTA] Получено релизов: %u\n", n);
  return n > 0;
}

/** Скачать sha256-ассет релиза (только проверка, без установки). */
static bool fetchReleaseSha256(uint8_t idx) {
  if (idx >= otaReleaseCount) return false;
  OtaRelease &r = otaReleases[idx];
  if (r.shaUrl[0] == '\0' || WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, r.shaUrl)) return false;
  http.addHeader("User-Agent", "kamaz-leveler");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const int code = http.GET();
  String body = (code == HTTP_CODE_OK) ? http.getString() : String();
  http.end();
  body.trim();
  if (body.length() < 64) {
    strlcpy(otaListStatus, "sha256 недоступен", sizeof(otaListStatus));
    return false;
  }
  strlcpy(r.sha256, body.substring(0, 64).c_str(), sizeof(r.sha256));
  strlcpy(otaListStatus, "sha256 получен", sizeof(otaListStatus));
  Serial.printf("[GH-OTA] %s sha256: %s\n", r.tag, r.sha256);
  return true;
}

/** Установить релиз из списка (вызывается из otaTask). */
static bool installGitHubReleaseIndex(int8_t idx) {
  if (idx < 0 || idx >= static_cast<int8_t>(otaReleaseCount)) return false;
  const OtaRelease &r = otaReleases[idx];
  if (r.binUrl[0] == '\0' || r.shaUrl[0] == '\0') return false;

  Serial.printf("[GH-OTA] Установка релиза %s\n", r.tag);
  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(r.binUrl, r.shaUrl, r.tag)) {
    otaValveLock = false;
    return false;
  }
  ESP.restart();
  return true;
}

/* --- Обработчики пунктов меню --- */

static void otaFetchList() {
  requestGitHubOtaFetchList();
  strlcpy(otaListStatus, "запрос отправлен…", sizeof(otaListStatus));
  displayDirty = true;
  Serial.println("[GH-OTA] Запрос списка релизов отправлен в otaTask");
}

static void otaListRefresh() { otaFetchList(); }

static void otaInstallLast() {
  requestGitHubOtaCheckAndInstall();
  menuVisible = false;   // показываем экран прогресса установки
  displayDirty = true;
  Serial.println("[GH-OTA] Запрос на установку последнего релиза");
}

static void otaSelectRelease(uint8_t idx) {
  if (idx >= otaReleaseCount) {
    strlcpy(otaListStatus, "сначала обновите список", sizeof(otaListStatus));
    displayDirty = true;
    return;
  }
  otaSelectedIndex = static_cast<int8_t>(idx);
  gem.setMenuPageCurrent(otaCardPage);
  displayDirty = true;
}

static void otaCardInstall() {
  if (otaSelectedIndex < 0) return;
  requestGitHubOtaInstallIndex(otaSelectedIndex);
  menuVisible = false;   // показываем экран прогресса установки
  displayDirty = true;
  Serial.printf("[GH-OTA] Запрос на установку релиза #%d\n", (int)otaSelectedIndex);
}

static void otaCardCheckSha() {
  if (otaSelectedIndex < 0 || otaSelectedIndex >= static_cast<int8_t>(otaReleaseCount)) return;
  // Ассет 64 байта — качаем синхронно (короткая операция), затем обновляем карточку.
  fetchReleaseSha256(static_cast<uint8_t>(otaSelectedIndex));
  refreshOtaCard();
  displayDirty = true;
}

static void otaArduinoMode() {
  menuVisible = false;
  displayDirty = true;
  startOTAMode();
}

static void otaExitMode() {
  stopOTAMode();
  displayDirty = true;
}

class ConfigManager {
private:
  struct Config {
    uint32_t formatVersion = CONFIG_FORMAT_VERSION;
    // ===== прежние параметры (8.6.x / 8.7.x) =====
    float pressureMin = 1.0f;
    float pressureMax = 7.0f;
    float tiltThresholdX = 0.5f;
    float tiltThresholdY = 0.5f;
    int contrast = 50;
    int nivCount = 5;
    int timeInterval = 5;
    int inflateDelay = 4;
    int releaseDelay = 7;
    float movementPressureFront = 3.5f;
    float movementPressureRear = 4.0f;

    // ===== 8.8.0: магистраль / клапаны =====
    int masterCheckSec = 240;        // период проверки магистрали, с
    int manualMaxTimeSec = 10;       // максимальное время ручной операции, с
    float pressureDeadband = 0.2f;   // зона нечувствительности по давлению, бар

    // ===== 8.8.0: авторежим =====
    float coarseZoneRatio = 0.4f;    // грубая зона = доля порога
    float fineZoneRatio = 0.15f;     // точная зона = доля порога
    float worseningRatio = 1.2f;     // порог «стало хуже» (множитель)

    // ===== 8.8.0: движение =====
    int movementDurationSec = 30;    // длительность ожидания движения, с
    int movementSettleSec = 30;      // время успокоения после движения, с
    int movementCheckSec = 120;      // период проверки давления после движения, с
    float movementTolerance = 0.2f;  // допуск давления после движения, бар
    int gyroThreshold = 50;          // порог гироскопа для детектора движения
    int accelThreshold = 1000;       // порог акселерометра для детектора движения

    // ===== 8.8.0: дисплей =====
    int backlightOffMin = 0;         // гашение подсветки, мин (0 = никогда)
    int frameMs = 50;                // интервал кадра дисплея, мс
    float redrawAngleThr = 0.05f;    // порог перерисовки по углу, °
    float redrawPressureThr = 0.03f; // порог перерисовки по давлению, бар

    // ===== 8.8.0: IMU / MPU6050 =====
    int imuMotionDet = 100;          // аппаратный порог детектора движения MPU
    int imuGyroOffX = 0;             // офсеты гироскопа (калибровка)
    int imuGyroOffY = 0;
    int imuGyroOffZ = 0;
    int imuAccelOffX = 0;            // офсеты акселерометра (калибровка)
    int imuAccelOffY = 0;
    int imuAccelOffZ = 0;
    float zeroAngleX = 0.0f;         // программный нуль углов (поперечный), °
    float zeroAngleY = 0.0f;         // программный нуль углов (продольный), °
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
      Serial.println("[CFG] Нет файла конфигурации — применяю значения по умолчанию");
      currentConfig = Config{};  // 8.8.0: все дефолты заданы в структуре Config
      currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
      dumpToSerial();
      return true;
    }

    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      Serial.println("[CFG] Ошибка десериализации JSON");
      return false;
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

    val = doc["tiltThresholdX"] | 0.5f;
    currentConfig.tiltThresholdX = constrain(val, 0.0f, 5.0f);

    val = doc["tiltThresholdY"] | 0.5f;
    currentConfig.tiltThresholdY = constrain(val, 0.0f, 5.0f);

    currentConfig.contrast = constrain(doc["contrast"] | 50, 1, 100);
    currentConfig.nivCount = constrain(doc["nivCount"] | 5, 1, 20);
    currentConfig.timeInterval = constrain(doc["timeInterval"] | 5, 1, 60);
    currentConfig.inflateDelay = constrain(doc["inflateDelay"] | 4, 1, 10);
    currentConfig.releaseDelay = constrain(doc["releaseDelay"] | 7, 1, 10);

    val = doc["movementPressureFront"] | 3.5f;
    currentConfig.movementPressureFront = constrain(val, 1.0f, 6.0f);

    val = doc["movementPressureRear"] | 4.0f;
    currentConfig.movementPressureRear = constrain(val, 1.0f, 6.0f);

    // ===== 8.8.0: магистраль / клапаны =====
    currentConfig.masterCheckSec = constrain((int)(doc["masterCheckSec"] | 240), 30, 600);
    currentConfig.manualMaxTimeSec = constrain((int)(doc["manualMaxTimeSec"] | 10), 1, 30);
    val = doc["pressureDeadband"] | 0.2f;
    currentConfig.pressureDeadband = constrain(val, 0.05f, 0.5f);

    // ===== 8.8.0: авторежим =====
    val = doc["coarseZoneRatio"] | 0.4f;
    currentConfig.coarseZoneRatio = constrain(val, 0.2f, 0.9f);
    val = doc["fineZoneRatio"] | 0.15f;
    currentConfig.fineZoneRatio = constrain(val, 0.05f, 0.3f);
    val = doc["worseningRatio"] | 1.2f;
    currentConfig.worseningRatio = constrain(val, 1.05f, 2.0f);

    // ===== 8.8.0: движение =====
    currentConfig.movementDurationSec = constrain((int)(doc["movementDurationSec"] | 30), 10, 120);
    currentConfig.movementSettleSec = constrain((int)(doc["movementSettleSec"] | 30), 10, 120);
    currentConfig.movementCheckSec = constrain((int)(doc["movementCheckSec"] | 120), 30, 300);
    val = doc["movementTolerance"] | 0.2f;
    currentConfig.movementTolerance = constrain(val, 0.1f, 1.0f);
    currentConfig.gyroThreshold = constrain((int)(doc["gyroThreshold"] | 50), 10, 200);
    currentConfig.accelThreshold = constrain((int)(doc["accelThreshold"] | 1000), 200, 3000);

    // ===== 8.8.0: дисплей =====
    currentConfig.backlightOffMin = constrain((int)(doc["backlightOffMin"] | 0), 0, 30);
    currentConfig.frameMs = constrain((int)(doc["frameMs"] | 50), 20, 200);
    val = doc["redrawAngleThr"] | 0.05f;
    currentConfig.redrawAngleThr = constrain(val, 0.01f, 0.5f);
    val = doc["redrawPressureThr"] | 0.03f;
    currentConfig.redrawPressureThr = constrain(val, 0.01f, 0.5f);

    // ===== 8.8.0: IMU / MPU6050 =====
    currentConfig.imuMotionDet = constrain((int)(doc["imuMotionDet"] | 100), 20, 255);
    currentConfig.imuGyroOffX = constrain((int)(doc["imuGyroOffX"] | 0), -32768, 32767);
    currentConfig.imuGyroOffY = constrain((int)(doc["imuGyroOffY"] | 0), -32768, 32767);
    currentConfig.imuGyroOffZ = constrain((int)(doc["imuGyroOffZ"] | 0), -32768, 32767);
    currentConfig.imuAccelOffX = constrain((int)(doc["imuAccelOffX"] | 0), -32768, 32767);
    currentConfig.imuAccelOffY = constrain((int)(doc["imuAccelOffY"] | 0), -32768, 32767);
    currentConfig.imuAccelOffZ = constrain((int)(doc["imuAccelOffZ"] | 0), -32768, 32767);
    val = doc["zeroAngleX"] | 0.0f;
    currentConfig.zeroAngleX = constrain(val, -3.0f, 3.0f);
    val = doc["zeroAngleY"] | 0.0f;
    currentConfig.zeroAngleY = constrain(val, -3.0f, 3.0f);

    Serial.printf("[CFG] Конфигурация загружена успешно (формат файла: %u)\n", formatVersion);
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
    doc["tiltThresholdX"] = currentConfig.tiltThresholdX;
    doc["tiltThresholdY"] = currentConfig.tiltThresholdY;
    doc["contrast"] = currentConfig.contrast;
    doc["nivCount"] = currentConfig.nivCount;
    doc["timeInterval"] = currentConfig.timeInterval;
    doc["inflateDelay"] = currentConfig.inflateDelay;
    doc["releaseDelay"] = currentConfig.releaseDelay;
    doc["movementPressureFront"] = currentConfig.movementPressureFront;
    doc["movementPressureRear"] = currentConfig.movementPressureRear;

    // ===== 8.8.0 =====
    doc["masterCheckSec"] = currentConfig.masterCheckSec;
    doc["manualMaxTimeSec"] = currentConfig.manualMaxTimeSec;
    doc["pressureDeadband"] = currentConfig.pressureDeadband;
    doc["coarseZoneRatio"] = currentConfig.coarseZoneRatio;
    doc["fineZoneRatio"] = currentConfig.fineZoneRatio;
    doc["worseningRatio"] = currentConfig.worseningRatio;
    doc["movementDurationSec"] = currentConfig.movementDurationSec;
    doc["movementSettleSec"] = currentConfig.movementSettleSec;
    doc["movementCheckSec"] = currentConfig.movementCheckSec;
    doc["movementTolerance"] = currentConfig.movementTolerance;
    doc["gyroThreshold"] = currentConfig.gyroThreshold;
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

    File file = LittleFS.open("/config.tmp", "w");
    if (!file) {
      Serial.println("[CFG] Не удалось создать временный файл");
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
  static void setTiltThresholdX(float v) {
    currentConfig.tiltThresholdX = v;
  }
  static void setTiltThresholdY(float v) {
    currentConfig.tiltThresholdY = v;
  }
  static void setContrast(int v) {
    currentConfig.contrast = v;
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
  CFG_FLOAT_ACCESSOR(CoarseZoneRatio, coarseZoneRatio)
  CFG_FLOAT_ACCESSOR(FineZoneRatio, fineZoneRatio)
  CFG_FLOAT_ACCESSOR(WorseningRatio, worseningRatio)
  CFG_INT_ACCESSOR(MovementDurationSec, movementDurationSec)
  CFG_INT_ACCESSOR(MovementSettleSec, movementSettleSec)
  CFG_INT_ACCESSOR(MovementCheckSec, movementCheckSec)
  CFG_FLOAT_ACCESSOR(MovementTolerance, movementTolerance)
  CFG_INT_ACCESSOR(GyroThreshold, gyroThreshold)
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
    Serial.printf("[CFG] v%u | Pmin=%.2f Pmax=%.2f Наклон X=%.2f Y=%.2f Ярк=%d Попыток=%d Инт=%d СБРОС=%d НАКАЧ=%d\n",
                  (unsigned)currentConfig.formatVersion, currentConfig.pressureMin, currentConfig.pressureMax,
                  currentConfig.tiltThresholdX, currentConfig.tiltThresholdY, currentConfig.contrast,
                  currentConfig.nivCount, currentConfig.timeInterval,
                  currentConfig.releaseDelay, currentConfig.inflateDelay);
    Serial.printf("[CFG] Движение: перед=%.2f зад=%.2f длит=%dс усп=%dс пров=%dс доп=%.2f гиро=%d акс=%d\n",
                  currentConfig.movementPressureFront, currentConfig.movementPressureRear,
                  currentConfig.movementDurationSec, currentConfig.movementSettleSec, currentConfig.movementCheckSec,
                  currentConfig.movementTolerance, currentConfig.gyroThreshold, currentConfig.accelThreshold);
    Serial.printf("[CFG] Магистраль=%dс | Клапаны: макс=%dс зона=%.2f | Авто: грубая=%.2f точная=%.2f хуже=%.2f\n",
                  currentConfig.masterCheckSec, currentConfig.manualMaxTimeSec, currentConfig.pressureDeadband,
                  currentConfig.coarseZoneRatio, currentConfig.fineZoneRatio, currentConfig.worseningRatio);
    Serial.printf("[CFG] Дисплей: подсветка=%dмин кадр=%dмс углы=%.2f давл=%.2f | IMU: det=%d гиро=%d,%d,%d акс=%d,%d,%d нуль=%.2f,%.2f\n",
                  currentConfig.backlightOffMin, currentConfig.frameMs, currentConfig.redrawAngleThr,
                  currentConfig.redrawPressureThr, currentConfig.imuMotionDet,
                  currentConfig.imuGyroOffX, currentConfig.imuGyroOffY, currentConfig.imuGyroOffZ,
                  currentConfig.imuAccelOffX, currentConfig.imuAccelOffY, currentConfig.imuAccelOffZ,
                  currentConfig.zeroAngleX, currentConfig.zeroAngleY);
  }
};

ConfigManager::Config ConfigManager::currentConfig;



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
    return error == Error::MPU || error == Error::VALVE || error == Error::WATCHDOG;
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
    forceDisplayReset(true);
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
      forceDisplayReset(true);
    }
    displayDirty = true;
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
            Serial.printf("[ERROR] Удалена ошибка %d\n", (int)pendingClear[i].error);
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
      displayDirty = true;
    }

    if (activeCount == 0 && hasError) {
      hasError = false;
      errorScreenBlocking = false;
      forceDisplayReset(true);
      Serial.println("[ERROR] Все ошибки удалены");
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



class ZeroCalibrator {
private:
  bool calibrationDone_ = false;
  uint32_t stepStartTime_ = 0;
  enum CalibStep : uint8_t {
    STEP_IDLE,
    STEP_OPEN_VALVE,
    STEP_WAIT_STABILIZE,
    STEP_MEASURE,
    STEP_CLOSE_VALVE,
    STEP_DONE
  } currentStep_ = STEP_IDLE;
  int measurements_[20] = { 0 };
  uint8_t measureCount_ = 0;
  int finalZeroValue_ = 0;
  
  // Константы калибровки
  static constexpr int ZERO_MIN = 80;      // Минимальное значение АЦП при 0 давлении
  static constexpr int ZERO_MAX = 140;     // Максимальное значение АЦП при 0 давлении
  static constexpr int ZERO_ERROR_THRESHOLD_LOW = 50;   // Ниже этого - обрыв цепи
  static constexpr int ZERO_ERROR_THRESHOLD_HIGH = 180;  // Выше этого - завышение

public:
  void start() {
    if (calibrationDone_) {
      Serial.println("[CALIB] start() ignored - already done");
      return;
    }
    if (currentStep_ != STEP_IDLE) {
      Serial.println("[CALIB] start() ignored - already started");
      return;
    }

#if ENABLE_SIMULATION
    Serial.println("[CALIB] SIM: Запуск калибровки пропущен");
    calibrationDone_ = true;
    return;
#else
    calibrationDone_ = false;
    currentStep_ = STEP_OPEN_VALVE;
    stepStartTime_ = millis();
    measureCount_ = 0;
    
    int testRaw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
    Serial.printf("[CALIB] start(): testRaw=%d\n", testRaw);
    
    for (int i = 0; i < 5; i++) {
        int raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
        if (raw > 10 && raw < 2040) {
            pressureFilter.filtered(raw);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    Serial.println("[CALIB] Фильтр инициализирован");
    
    Serial.println("\n========================================");
    Serial.println("     АВТОМАТИЧЕСКАЯ КАЛИБРОВКА НУЛЯ");
    Serial.println("========================================");
#endif
  }

  void process() {
    if (calibrationDone_) return;

#if ENABLE_SIMULATION
    calibrationDone_ = true;
    Serial.println("[CALIB] SIM: Калибровка пропущена");
    return;
#endif

    uint32_t now = millis();

    switch (currentStep_) {
      case STEP_OPEN_VALVE:
        {
          MutexGuard guard(xValveMutex);
          if (guard) {
            closeAllValves();
            digitalWrite(PIN_DEFL, HIGH);
            Serial.println("[CALIB] Клапан сброса ОТКРЫТ");
            currentStep_ = STEP_WAIT_STABILIZE;
            stepStartTime_ = now;
          }
          break;
        }

      case STEP_WAIT_STABILIZE:
        {
          uint32_t elapsed = now - stepStartTime_;
          if (elapsed >= 3000) {
            Serial.printf("[CALIB] Ожидание стабилизации завершено (прошло %d мс)\n", elapsed);
            currentStep_ = STEP_MEASURE;
            stepStartTime_ = now;
            measureCount_ = 0;
          }
          break;
        }

      case STEP_MEASURE:
        if (measureCount_ < 20) {
          static uint32_t lastMeasureTime = 0;
          if (now - lastMeasureTime >= 200) {
            int raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);

#if ENABLE_SIMULATION
            if (raw == 0) raw = 512;
#endif

            if (raw == 0) {
              static uint32_t lastWarn = 0;
              if (now - lastWarn > 3000) {
                lastWarn = now;
                Serial.printf("[CALIB] ⚠️ Ошибка чтения ADS1015! raw=%d (пропуск)\n", raw);
              }
              lastMeasureTime = now;  // не копим одно и то же, ждём следующее окно
              break;
            }

            int filtered = pressureFilter.filtered(raw);
            measurements_[measureCount_++] = filtered;
            lastMeasureTime = now;

            Serial.printf("[CALIB] Измерение %d: raw=%d, filtered=%d\n",
                          measureCount_, raw, filtered);
          }
        } else {
          // Вычисляем среднее
          long sum = 0;
          int minVal = measurements_[5];
          int maxVal = measurements_[5];

          for (int i = 5; i < 20; i++) {
            sum += measurements_[i];
            if (measurements_[i] < minVal) minVal = measurements_[i];
            if (measurements_[i] > maxVal) maxVal = measurements_[i];
          }

          int range = maxVal - minVal;
          finalZeroValue_ = sum / 15;

          Serial.printf("[CALIB] Среднее (последние 15): %d, Разброс: %d\n",
                        finalZeroValue_, range);

          // ============================================================
          // ✅ ПРОВЕРКА ДАТЧИКА
          // ============================================================
          
          // 1. Проверка на обрыв цепи (слишком низкое значение)
          if (finalZeroValue_ < ZERO_ERROR_THRESHOLD_LOW) {
            Serial.printf("[CALIB] ❌ ОБРЫВ ЦЕПИ! АЦП=%d (норма: %d-%d)\n",
                          finalZeroValue_, ZERO_MIN, ZERO_MAX);
            if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
              ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                        "Датчик давления - обрыв цепи");
            } else {
              ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
            }
            // Сохраняем значение, но помечаем как невалидное
            calibrationValid = false;
          }
          // 2. Проверка на КЗ (слишком высокое значение)
          else if (finalZeroValue_ > ZERO_ERROR_THRESHOLD_HIGH) {
            Serial.printf("[CALIB] ❌ ДАТЧИК ЗАВЫШАЕТ ПОКАЗАНИЯ! АЦП=%d (норма: %d-%d)\n",
                          finalZeroValue_, ZERO_MIN, ZERO_MAX);
            Serial.println("[CALIB] При открытом клапане сброса давление должно быть 0");
            Serial.println("[CALIB] Датчик завышает показания! Требуется проверка.");
            
            if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
              ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                        "Датчик давления завышает показания");
            } else {
              ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
            }
            // Сохраняем значение, но помечаем как невалидное
            calibrationValid = false;
          }
          // 3. Датчик исправен
          else if (finalZeroValue_ >= ZERO_MIN && finalZeroValue_ <= ZERO_MAX) {
            Serial.printf("[CALIB] ✅ Датчик исправен (АЦП=%d в норме %d-%d)\n",
                          finalZeroValue_, ZERO_MIN, ZERO_MAX);
            
            // Удаляем ошибку SENSOR, если она была
            if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
              ErrorHandler::removeError(ErrorHandler::Error::SENSOR);
              Serial.println("[CALIB] Ошибка SENSOR удалена (датчик исправен)");
            }
            calibrationValid = true;
          }
          // 4. Пограничное значение - предупреждение
          else {
            Serial.printf("[CALIB] ⚠️ НЕОБЫЧНОЕ ЗНАЧЕНИЕ НУЛЯ: %d! (норма: %d-%d)\n",
                          finalZeroValue_, ZERO_MIN, ZERO_MAX);
            Serial.println("[CALIB] Датчик работает, но возможны проблемы.");
            // Не создаём ошибку, но помечаем как сомнительное
            calibrationValid = true;
          }

          // Проверка стабильности
          if (range > 15) {
            Serial.printf("[CALIB] ⚠️ Нестабильные измерения! Разброс: %d\n", range);
            Serial.println("[CALIB] Возможны проблемы с питанием или датчиком.");
          }

          currentStep_ = STEP_CLOSE_VALVE;
          stepStartTime_ = now;
        }
        break;

      case STEP_CLOSE_VALVE:
        {
          MutexGuard guard(xValveMutex);
          if (guard) {
            digitalWrite(PIN_DEFL, LOW);
            Serial.println("[CALIB] Клапан сброса ЗАКРЫТ");
            currentStep_ = STEP_DONE;
          }
          break;
        }

      case STEP_DONE:
        {
          MutexGuard guard(xCalibMutex);
          if (guard) {
            int correctedZero = finalZeroValue_;
            if (correctedZero < 0) correctedZero = 0;

            // Сохраняем значение даже если датчик неисправен
            g_pressureZeroRaw = correctedZero;
            calibrationDone_ = true;
            
            // Если датчик исправен - устанавливаем флаг
            if (calibrationValid) {
              calibrationValid = true;
            }

            Serial.printf("[CALIB] Исходное среднее: %d\n", finalZeroValue_);
            Serial.printf("[CALIB] Новая калибровка: filtered -= %d\n", g_pressureZeroRaw);
            Serial.println("========================================");
            Serial.println("     КАЛИБРОВКА НУЛЯ ЗАВЕРШЕНА");
            Serial.println("========================================\n");

            Event event;
            event.type = EventType::CALIBRATION_DONE;
            event.timestamp = millis();
            EventBus::publish(event);
          }
          break;
        }

      default: break;
    }
  }

  bool isDone() const {
    return calibrationDone_;
  }
  int getZeroValue() const {
    return finalZeroValue_;
  }
};


/* ====================  ПЕРЕМЕННЫЕ ДЛЯ СИМУЛЯЦИИ ==================== */
// Эти переменные используются даже когда симуляция выключена
float simAngleX = 0;
float simAngleY = 0;
float simPressures[PAD_COUNT] = { 2.0f, 2.5f, 3.0f, 3.5f };
float simMasterPressure = 4.0f;
int simDirectionX = 1;
int simDirectionY = 1;
uint32_t lastSimUpdate = 0;

// ✅ ДОБАВЛЕНО: Эти переменные теперь всегда объявлены
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





/* ====================  Logger ==================== */
class Logger {
public:
  enum Level { DEBUG = 0,
               INFO = 1,
               WARNING = 2,
               ERROR = 3 };

  static void log(Level level, const char *tag, const char *message) {
    const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
    Serial.printf("[%s][%s] %s\n", levelStr[level], tag, message);
  }

  static void logf(Level level, const char *tag, const char *format, ...) {
    const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
    Serial.printf("[%s][%s] ", levelStr[level], tag);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    Serial.println();
  }
};

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
      if (currentTime - tasks[i].lastRunTime > TASK_WDT_TIMEOUT_MS) {
        Logger::logf(Logger::ERROR, "WATCHDOG", "Task %s is not responding", tasks[i].name);
        ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG, "Task timeout");
      }
    }
  }

  static void init() {
    tasks[TASK_EVENT] = { "Event", 0, false };
    tasks[TASK_BUTTON] = { "Button", 0, false };
    tasks[TASK_DISPLAY] = { "Display", 0, false };
    tasks[TASK_IMU] = { "IMU", 0, false };
    tasks[TASK_PRESSURE] = { "Pressure", 0, false };
    tasks[TASK_CONTROL] = { "Control", 0, false };
    tasks[TASK_CALIB] = { "Calib", 0, false };
    tasks[TASK_WATCHDOG] = { "Watchdog", 0, false };
    tasks[TASK_OTA] = { "OTA", 0, false };
    tasks[TASK_VALVE] = { "Valve", 0, false };
    tasks[TASK_ERROR_RECOVERY] = { "ErrorRecovery", 0, false };
  }
};

TaskMonitor::TaskInfo TaskMonitor::tasks[TASK_COUNT];

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
        Serial.printf("[MEM] ⚠️ КРИТИЧЕСКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

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
        Serial.printf("[MEM] ⚠️ НИЗКИЙ УРОВЕНЬ ПАМЯТИ! %d байт\n", currentFree);

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
        Serial.printf("[MEM] ✅ Память восстановлена: %d байт\n", currentFree);
      }
      if (lowMemoryReported && currentFree > LOW_MEMORY_THRESHOLD + 2048) {
        lowMemoryReported = false;
        Serial.printf("[MEM] ✅ Уровень памяти нормализован: %d байт\n", currentFree);
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
  displayDirty = true;
  Event event;
  event.type = EventType::DISPLAY_UPDATE;
  event.timestamp = millis();
  EventBus::publish(event, 0);
}

void closeAllValves() {
  for (auto pin : bubPins) digitalWrite(pin, LOW);
  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
}

void setValve(Pad pad, bool state) {
  if (pad >= PAD_COUNT) return;
  digitalWrite(bubPins[pad], state);
}

void emergencyStop() {
  static SemaphoreHandle_t mutex = nullptr;
  if (mutex == nullptr) mutex = xSemaphoreCreateMutex();

  if (!takeMutexWithRetry(mutex, pdMS_TO_TICKS(100), 3, "emergencyStop/outer")) {
    Serial.println("[EMERGENCY] outer mutex failed — forcing valve pins LOW");
    for (auto pin : bubPins) digitalWrite(pin, LOW);
    digitalWrite(PIN_INFL, LOW);
    digitalWrite(PIN_DEFL, LOW);
    valveStopRequested = true;
    return;
  }

  if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(150), 3, "emergencyStop/valve")) {
    Serial.println("[EMERGENCY] valve mutex failed — forcing valve pins LOW");
    closeAllValves();
    valveStopRequested = true;
    xSemaphoreGive(mutex);
    return;
  }

  Logger::log(Logger::WARNING, "EMERGENCY", "Остановка всех клапанов!");
  closeAllValves();
  manualControlActive = false;
  for (int i = 0; i < PAD_COUNT; i++) {
    manualTargetSet[i] = false;
  }
  valveStopRequested = true;
  valveEmergencyStopCount++;
  xSemaphoreGive(xValveMutex);
  xSemaphoreGive(mutex);
}

void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs) {
  for (int attempt = 0; attempt < 3; attempt++) {
    MutexGuard commandGuard(xCommandMutex, pdMS_TO_TICKS(100));
    if (!commandGuard) {
      if (attempt == 2) {
        Serial.println("[VALVE] Command state mutex unavailable after retries");
      }
      continue;
    }

    if (xValveQueue == nullptr) {
      Serial.println("[ERROR] Valve queue not created!");
      return;
    }

    if (otaMode || otaInProgress || otaValveLock) {
      Serial.println("[VALVE] Command rejected while OTA mode is active");
      lastCmd.waitingForCompletion = false;
      lastCmd.commandActive = false;
      return;
    }

    if (pad >= PAD_COUNT) {
      Serial.printf("[ERROR] Invalid pad: %d\n", (int)pad);
      return;
    }

    if (durationMs > 0) {
      float currentMaster;
      {
        MutexGuard guard(xStateMutex);
        if (!guard) return;
        currentMaster = masterPressure;
      }

      float minPressure = ConfigManager::getPressureMin();
      if (currentMaster < minPressure - ConfigManager::getPressureDeadband()) {
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 5000) {
          lastLog = millis();
          Serial.printf("[VALVE] ❌ Команда заблокирована! Давление: %.1f бар (мин: %.1f)\n",
                        currentMaster, minPressure);
        }

        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        return;
      }
    }
    uint32_t safeDuration;
    if (durationMs == 0 || durationMs == UINT32_MAX) {
      safeDuration = 0;
    } else {
      safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
    }

    if (safeDuration > 0 && durationMs != UINT32_MAX && !ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) && !ErrorHandler::isErrorActive(ErrorHandler::Error::MPU)) {

      if (lastCmd.waitingForCompletion) {
        Serial.println("[VALVE] Новая команда до анализа предыдущей, предыдущая отменена");
        lastCmd.waitingForCompletion = false;
      }

      lastCmd.startTime = millis();
      lastCmd.duration = safeDuration;
      lastCmd.pad = pad;
      lastCmd.inflate = inflate;
      lastCmd.waitingForCompletion = true;
      lastCmd.commandActive = true;

      MutexGuard guard(xStateMutex);
      if (guard) {
        lastCmd.pressureBefore = pressure[pad];
      }
    }

    ValveCommandMsg cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.async.pad = pad;
    cmd.async.inflate = inflate;
    cmd.async.duration = (safeDuration == 0) ? 0 : pdMS_TO_TICKS(safeDuration);

    BaseType_t result = xQueueSend(xValveQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
      valveQueueDropCount++;
      Serial.println("[ERROR] Failed to send to valve queue!");
      return;
    }
    uint32_t depth = uxQueueMessagesWaiting(xValveQueue);
    if (depth > maxValveQueueDepth) maxValveQueueDepth = depth;

    if (safeDuration > 0 || durationMs == UINT32_MAX) {
      Event event;
      event.type = EventType::VALVE_COMMAND;
      event.timestamp = millis();
      event.data.valve.pad = static_cast<uint8_t>(pad);
      event.data.valve.inflate = inflate;
      event.data.valve.duration = durationMs;
      EventBus::publish(event, pdMS_TO_TICKS(100));
    }
    return;
  }
}

/* ===== 8.8.0: состояние подсветки и «служебных экранов» ===== */
bool backlightOff = false;        // подсветка погашена по таймауту бездействия
uint32_t lastUserActivityMs = 0;  // время последнего нажатия любой кнопки

/** 8.8.0: применение настроек к железу сразу после сохранения (без перезагрузки). */
void applyRuntimeSettings() {
  // Яркость подсветки (пункт «Дисплей → Яркость»)
  analogWrite(PIN_TFT_BL, map(constrain(ConfigManager::getContrast(), 1, 100), 1, 100, 0, 255));

  // Политика гашения подсветки: 0 = никогда
  if (ConfigManager::getBacklightOffMin() == 0) {
    backlightOff = false;
  }
  lastUserActivityMs = millis();

  // Аппаратный порог детектора движения MPU применяем на ходу
  if (mpuOk) {
    mpu.setMotionDetectionThreshold(ConfigManager::getImuMotionDet());
    Serial.printf("[IMU] Порог детектора движения MPU: %d\n", ConfigManager::getImuMotionDet());
  }

  Serial.printf("[MENU] Применено: кадр=%d мс, подсветка=%d мин, зона=%.2f бар, допуск=%.2f бар\n",
                ConfigManager::getFrameMs(), ConfigManager::getBacklightOffMin(),
                ConfigManager::getPressureDeadband(), ConfigManager::getMovementTolerance());
  displayDirty = true;
  forceDisplayReset(true);
}

/* ============================================================================
   8.8.0: СЛУЖЕБНЫЕ ЭКРАНЫ — ручной тест клапанов, IMU (нуль углов, калибровка
   офсетов), диагностика MPU6050.
   Управление: Кн1/Кн2 — выбор, Кн3 — действие/подтверждение, Кн4 — выход.
   Аварийная остановка (Кн4+Кн5) закрывает все клапаны и выходит из экрана.
   ============================================================================ */
enum class ServiceScreen : uint8_t {
  NONE = 0,
  VALVE_TEST,        // ручной тест клапанов (6 выходов, «пока держишь Кн3»)
  IMU_ZERO_CONFIRM,  // подтверждение обнуления углов по текущему положению
  IMU_CALIB,         // калибровка офсетов IMU
  MPU_DIAG           // диагностика MPU6050
};

volatile ServiceScreen serviceScreen = ServiceScreen::NONE;

// --- ручной тест клапанов: 4 подушечных + накачка + сброс ---
constexpr uint8_t MANUAL_VALVE_COUNT = 6;
const char *const manualValveNames[MANUAL_VALVE_COUNT] = { "ПЛ", "ПП", "ЗЛ", "ЗП", "НАКАЧКА", "СБРОС" };
const uint8_t manualValvePins[MANUAL_VALVE_COUNT] = { PIN_BUB1, PIN_BUB2, PIN_BUB3, PIN_BUB4, PIN_INFL, PIN_DEFL };
uint8_t manualValveIndex = 0;
uint8_t manualValveOpenIndex = 0xFF;  // 0xFF — ничего не открыто
uint32_t manualValveOpenSince = 0;

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
static void manualValveCloseAll() {
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
  forceDisplayReset(true);
  displayDirty = true;
}

/** Открыть экран диагностики MPU6050. */
void openMpuDiagScreen() {
  serviceScreen = ServiceScreen::MPU_DIAG;
  menuVisible = false;
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
static void handleServiceInput() {
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
        ConfigManager::setZeroAngleX(constrain(angleX, -3.0f, 3.0f));
        ConfigManager::setZeroAngleY(constrain(angleY, -3.0f, 3.0f));
        editZeroAngleX = ConfigManager::getZeroAngleX();
        editZeroAngleY = ConfigManager::getZeroAngleY();
        settingsChanged = true;
        saveMenuSettings();
        Serial.printf("[IMU] Программный нуль углов: X=%.2f Y=%.2f\n",
                      ConfigManager::getZeroAngleX(), ConfigManager::getZeroAngleY());
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

/* ===== 8.8.0: отрисовка служебных экранов ===== */
static void drawValveTestScreen() {
  tft.fillScreen(theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  ui.box(theme::MARGIN, 2, theme::SCREEN_W - 2 * theme::MARGIN, 18, "РУЧНОЙ ТЕСТ КЛАПАНОВ",
         UiHAlign::Center, UiVAlign::Middle, false);

  const int16_t rowH = 20;
  int16_t y = 24;
  for (uint8_t i = 0; i < MANUAL_VALVE_COUNT; i++) {
    const bool selected = (i == manualValveIndex);
    const bool isOpen = (manualValveOpenIndex == i);
    if (selected) {
      tft.fillRoundRect(theme::MARGIN - 2, y - 1, theme::SCREEN_W - 2 * theme::MARGIN + 4, rowH, 3,
                        isOpen ? theme::OK : theme::PANEL);
    }
    ui.boxf(theme::MARGIN + 4, y, 150, rowH - 2, UiHAlign::Left, UiVAlign::Middle, false,
            "%s %s", selected ? ">" : " ", manualValveNames[i]);
    ui.box(theme::SCREEN_W - theme::MARGIN - 110, y, 110, rowH - 2, isOpen ? "ОТКРЫТ" : "закрыт",
           UiHAlign::Right, UiVAlign::Middle, false);
    y += rowH;
  }

  const uint32_t holdSec = (manualValveOpenIndex == 0xFF) ? 0 : (millis() - manualValveOpenSince) / 1000;
  ui.boxf(theme::MARGIN, y + 2, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "МП: %.2f бар   открыт: %lu с / 15 с", masterPressure, (unsigned long)holdSec);
  ui.box(theme::MARGIN, y + 20, theme::SCREEN_W - 2 * theme::MARGIN, 16,
         "КН1/КН2-ВЫБОР  КН3-УДЕРЖ=ОТКРЫТ", UiHAlign::Left, UiVAlign::Middle, true);
  ui.box(theme::MARGIN, y + 36, theme::SCREEN_W - 2 * theme::MARGIN, 16,
         "КН4-ВЫХОД (все закрыть)  АВАРИЯ КН4+КН5", UiHAlign::Left, UiVAlign::Middle, true);
}

static void drawImuZeroConfirmScreen() {
  tft.fillScreen(theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  ui.box(theme::MARGIN, 6, theme::SCREEN_W - 2 * theme::MARGIN, 20, "ОБНУЛИТЬ УГЛЫ ПО ТЕКУЩЕМУ ПОЛОЖЕНИЮ",
         UiHAlign::Center, UiVAlign::Middle, false);
  ui.box(theme::MARGIN, 40, theme::SCREEN_W - 2 * theme::MARGIN, 16,
         "Текущее положение станет 0.00 / 0.00", UiHAlign::Center, UiVAlign::Middle, true);
  ui.boxf(theme::MARGIN, 70, theme::SCREEN_W - 2 * theme::MARGIN, 18, UiHAlign::Center, UiVAlign::Middle, true,
          "Поперечный: %.2f   Продольный: %.2f", angleX, angleY);
  ui.boxf(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, UiHAlign::Center, UiVAlign::Middle, true,
          "Текущий нуль: %.2f / %.2f", ConfigManager::getZeroAngleX(), ConfigManager::getZeroAngleY());
  ui.box(theme::MARGIN, 150, theme::SCREEN_W - 2 * theme::MARGIN, 18,
         "Машина должна стоять на месте!", UiHAlign::Center, UiVAlign::Middle, true);
  ui.box(theme::MARGIN, 190, theme::SCREEN_W - 2 * theme::MARGIN, 18,
         "КН3 - ПОДТВЕРДИТЬ     КН4 - ОТМЕНА", UiHAlign::Center, UiVAlign::Middle, true);
}

static void drawImuCalibScreen() {
  tft.fillScreen(theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  ui.box(theme::MARGIN, 6, theme::SCREEN_W - 2 * theme::MARGIN, 20, "КАЛИБРОВКА ОФСЕТОВ IMU",
         UiHAlign::Center, UiVAlign::Middle, false);
  ui.box(theme::MARGIN, 40, theme::SCREEN_W - 2 * theme::MARGIN, 16,
         "Машина ровно, НЕПОДВИЖНО 10-20 с", UiHAlign::Center, UiVAlign::Middle, true);
  ui.box(theme::MARGIN, 62, theme::SCREEN_W - 2 * theme::MARGIN, 16,
         "DMP на время калибровки отключается", UiHAlign::Center, UiVAlign::Middle, true);

  if (imuCalibResult == 1) {
    ui.box(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, "ОФСЕТЫ СОХРАНЕНЫ В КОНФИГ",
           UiHAlign::Center, UiVAlign::Middle, true);
  } else if (imuCalibResult < 0) {
    ui.box(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, "ОШИБКА: MPU6050 недоступен",
           UiHAlign::Center, UiVAlign::Middle, true);
  } else {
    ui.box(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, "Готово к запуску",
           UiHAlign::Center, UiVAlign::Middle, true);
  }

  ui.boxf(theme::MARGIN, 126, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Center, UiVAlign::Middle, true,
          "гиро: %d %d %d", ConfigManager::getImuGyroOffX(), ConfigManager::getImuGyroOffY(),
          ConfigManager::getImuGyroOffZ());
  ui.boxf(theme::MARGIN, 144, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Center, UiVAlign::Middle, true,
          "аксель: %d %d %d", ConfigManager::getImuAccelOffX(), ConfigManager::getImuAccelOffY(),
          ConfigManager::getImuAccelOffZ());
  ui.boxf(theme::MARGIN, 170, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Center, UiVAlign::Middle, true,
          "Последняя калибровка: %lu с назад",
          (unsigned long)(imuCalibLastRun ? (millis() - imuCalibLastRun) / 1000 : 0));
  ui.box(theme::MARGIN, 200, theme::SCREEN_W - 2 * theme::MARGIN, 18,
         "КН3 - КАЛИБРОВАТЬ     КН4 - ВЫХОД", UiHAlign::Center, UiVAlign::Middle, true);
}

static void drawMpuDiagScreen() {
  tft.fillScreen(theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  ui.box(theme::MARGIN, 4, theme::SCREEN_W - 2 * theme::MARGIN, 18, "ДИАГНОСТИКА MPU6050",
         UiHAlign::Center, UiVAlign::Middle, false);

  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  int deviceId = -1;
  float temp = 0.0f;
  if (mpuOk) {
    deviceId = mpu.getDeviceID();
    temp = mpu.getTemperature() / 340.0f + 36.53f;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }

  ui.boxf(theme::MARGIN, 26, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "ID: 0x%02X   MPU: %s   DMP: %s", deviceId, mpuOk ? "OK" : "ОШИБКА",
          (mpuOk && mpu.getDMPEnabled()) ? "готов" : "нет");
  ui.boxf(theme::MARGIN, 44, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Температура: %.1f C   Порог детектора: %d", temp, ConfigManager::getImuMotionDet());
  ui.boxf(theme::MARGIN, 62, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Углы (с нулём): X=%.2f  Y=%.2f", angleX, angleY);
  ui.boxf(theme::MARGIN, 80, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Нуль конфига: X=%.2f  Y=%.2f", ConfigManager::getZeroAngleX(), ConfigManager::getZeroAngleY());
  ui.box(theme::MARGIN, 104, theme::SCREEN_W - 2 * theme::MARGIN, 16, "Сырые данные getMotion6:",
         UiHAlign::Left, UiVAlign::Middle, true);
  ui.boxf(theme::MARGIN, 122, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "ax=%d ay=%d az=%d", ax, ay, az);
  ui.boxf(theme::MARGIN, 140, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "gx=%d gy=%d gz=%d", gx, gy, gz);
  ui.boxf(theme::MARGIN, 158, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Офсеты гиро: %d %d %d", ConfigManager::getImuGyroOffX(), ConfigManager::getImuGyroOffY(),
          ConfigManager::getImuGyroOffZ());
  ui.boxf(theme::MARGIN, 176, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Офсеты аксель: %d %d %d", ConfigManager::getImuAccelOffX(), ConfigManager::getImuAccelOffY(),
          ConfigManager::getImuAccelOffZ());
  ui.boxf(theme::MARGIN, 194, theme::SCREEN_W - 2 * theme::MARGIN, 16, UiHAlign::Left, UiVAlign::Middle, true,
          "Ошибка MPU активна: %s", ErrorHandler::isErrorActive(ErrorHandler::Error::MPU) ? "ДА" : "нет");
  ui.box(theme::MARGIN, 214, theme::SCREEN_W - 2 * theme::MARGIN, 16, "КН4 - ВЫХОД",
         UiHAlign::Center, UiVAlign::Middle, true);
}

/** Отрисовка активного служебного экрана (вызывается из задачи дисплея). */
static void drawServiceScreen() {
  switch (serviceScreen) {
    case ServiceScreen::VALVE_TEST: drawValveTestScreen(); break;
    case ServiceScreen::IMU_ZERO_CONFIRM: drawImuZeroConfirmScreen(); break;
    case ServiceScreen::IMU_CALIB: drawImuCalibScreen(); break;
    case ServiceScreen::MPU_DIAG: drawMpuDiagScreen(); break;
    default: break;
  }
}

// 11.6. Сохранение настроек меню
void saveMenuSettings() {
  if (!settingsChanged) return;

  ConfigManager::setReleaseDelay(editReleaseDelay);
  ConfigManager::setInflateDelay(editInflateDelay);
  ConfigManager::setTiltThresholdX(editTiltX);
  ConfigManager::setTiltThresholdY(editTiltY);
  ConfigManager::setPressureMin(editPressureMin);
  ConfigManager::setPressureMax(editPressureMax);
  ConfigManager::setNivCount(editNivCount);
  ConfigManager::setTimeInterval(editTimeInterval);
  ConfigManager::setContrast(editContrast);
  ConfigManager::setMovementPressureFront(editMovementPressureFront);
  ConfigManager::setMovementPressureRear(editMovementPressureRear);

  // ===== 8.8.0: новые параметры =====
  ConfigManager::setMasterCheckSec(editMasterCheck);
  ConfigManager::setManualMaxTimeSec(editManualMaxTime);
  ConfigManager::setPressureDeadband(editDeadband);
  ConfigManager::setCoarseZoneRatio(editCoarseZone);
  ConfigManager::setFineZoneRatio(editFineZone);
  ConfigManager::setWorseningRatio(editWorsening);
  ConfigManager::setMovementDurationSec(editMoveDuration);
  ConfigManager::setMovementSettleSec(editMoveSettle);
  ConfigManager::setMovementCheckSec(editMoveCheck);
  ConfigManager::setMovementTolerance(editMoveTolerance);
  ConfigManager::setGyroThreshold(editGyroThreshold);
  ConfigManager::setAccelThreshold(editAccelThreshold);
  ConfigManager::setBacklightOffMin(editBacklightOff);
  ConfigManager::setFrameMs(editFrameMs);
  ConfigManager::setRedrawAngleThr(editRedrawAngle);
  ConfigManager::setRedrawPressureThr(editRedrawPressure);
  ConfigManager::setImuMotionDet(editImuMotionDet);
  ConfigManager::setZeroAngleX(editZeroAngleX);
  ConfigManager::setZeroAngleY(editZeroAngleY);

  saveConfig();
  settingsChanged = false;
  applyRuntimeSettings();  // 8.8.0: новые значения действуют сразу, без перезагрузки
  Logger::log(Logger::INFO, "MENU", "Настройки сохранены");
}

void setManualTargetPressure(Pad pad) {
  MutexGuard guard(xStateMutex);
  if (guard) {
    manualTargetPressure[pad] = pressure[pad];
    manualTargetSet[pad] = true;
  }
  Serial.printf("[MANUAL] Установлено целевое давление для %s: %.1f бар\n",
                padNames[pad], manualTargetPressure[pad]);
  setDisplayDirty();
}

void setAllManualTargetsFromCurrent() {
  MutexGuard guard(xStateMutex);
  if (guard) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      manualTargetPressure[i] = pressure[i];
      manualTargetSet[i] = true;
    }
  }
  Serial.println("[MANUAL] Установлены целевые давления для всех подушек из текущих значений");
  setDisplayDirty();
}

void maintainManualPressure() {
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[MANUAL] Датчик давления неисправен, поддержание давления приостановлено");
    }
    return;
  }

  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[MANUAL] Поддержание давления приостановлено: LOW_PRESSURE");
    }
    return;
  }

  static uint32_t lastAdjustmentTime[PAD_COUNT] = { 0 };
  static const uint32_t ADJUSTMENT_COOLDOWN_MS = 3000;
  static uint32_t lastPressureRequest = 0;

  uint32_t currentTime = millis();

  if (currentTime - lastPressureRequest >= MANUAL_TARGET_CHECK_INTERVAL_MS) {
    requestPressureMeasurement();
    lastPressureRequest = currentTime;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    float currentPressure, targetPressure;
    bool targetIsSet;
    float minPressure = ConfigManager::getPressureMin();
    float maxPressure = ConfigManager::getPressureMax();

    {
      MutexGuard guard(xStateMutex);
      if (!guard) continue;

      targetIsSet = manualTargetSet[i];
      if (!targetIsSet) {
        manualTargetPressure[i] = pressure[i];
        manualTargetSet[i] = true;
        continue;
      }
      currentPressure = pressure[i];
      targetPressure = manualTargetPressure[i];
    }

    // Если давление ниже минимума - подкачать
    if (currentPressure < minPressure - 0.1f) {
      if (currentTime - lastAdjustmentTime[i] >= ADJUSTMENT_COOLDOWN_MS) {
        int duration = ConfigManager::getInflateDelay() * 1000;
        sendValveCommand(Pad(i), true, duration);
        lastAdjustmentTime[i] = currentTime;
        setDisplayDirty();
        requestPressureMeasurement();
      }
      continue;
    }

    // Если давление выше максимума - стравить
    if (currentPressure > maxPressure + 0.1f) {
      if (currentTime - lastAdjustmentTime[i] >= ADJUSTMENT_COOLDOWN_MS) {
        int duration = ConfigManager::getReleaseDelay() * 1000;
        sendValveCommand(Pad(i), false, duration);
        lastAdjustmentTime[i] = currentTime;
        setDisplayDirty();
        requestPressureMeasurement();
      }
      continue;
    }

    if (currentPressure < 0) continue;

    float pressureDiff = targetPressure - currentPressure;

    if (abs(pressureDiff) > MANUAL_PRESSURE_TOLERANCE && (currentTime - lastAdjustmentTime[i] >= MANUAL_ADJUSTMENT_COOLDOWN_MS)) {

      if (pressureDiff > 0) {
        if (currentPressure >= maxPressure - ConfigManager::getPressureDeadband()) continue;
        int duration = ConfigManager::getInflateDelay() * 1000;
        sendValveCommand(Pad(i), true, duration);
      } else {
        if (currentPressure <= minPressure + ConfigManager::getPressureDeadband()) continue;
        int duration = ConfigManager::getReleaseDelay() * 1000;
        sendValveCommand(Pad(i), false, duration);
      }
      lastAdjustmentTime[i] = currentTime;
      setDisplayDirty();
      requestPressureMeasurement();
    }
  }
}

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

void checkAndAdjustMasterPressure() {
  if (!calibrationCompleted || !firstPressureMeasurementDone) return;
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) return;
  if (manualControlActive || otaValveLock || otaMode || otaInProgress) return;

  uint32_t now = millis();
  if (now - lastMasterPressureCheckTime < (uint32_t)ConfigManager::getMasterCheckSec() * 1000UL) {  // 8.8.0: период из меню
    return;
  }
  lastMasterPressureCheckTime = now;

  requestPressureMeasurement();

  float localMaster = 0.0f;
  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(200));
    if (!guard) {
      Serial.println("[MASTER] state mutex timeout — skip adjust");
      return;
    }
    localMaster = masterPressure;
  }

  float minP = ConfigManager::getPressureMin();
  float maxP = ConfigManager::getPressureMax();
  Serial.printf("[MASTER] periodic check: %.2f bar (min=%.1f max=%.1f)\n",
                localMaster, minP, maxP);

  // Обновить LOW_PRESSURE по текущему значению магистрали
  checkPressureLimits();

  // Избыток в магистрали: краткий сброс через DEFL (подушки закрыты)
  if (localMaster > maxP + ConfigManager::getPressureDeadband()) {
    if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(200), 3, "MASTER/relieve")) {
      Serial.println("[MASTER] cannot relieve — valve mutex busy");
      return;
    }
    closeAllValves();
    digitalWrite(PIN_DEFL, HIGH);
    xSemaphoreGive(xValveMutex);

    vTaskDelay(pdMS_TO_TICKS(400));

    if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(200), 3, "MASTER/relieve-off")) {
      digitalWrite(PIN_DEFL, LOW);
      xSemaphoreGive(xValveMutex);
    } else {
      digitalWrite(PIN_DEFL, LOW);  // fail-safe без мьютекса
    }
    requestPressureMeasurement();
    Serial.println("[MASTER] relieved excess pressure on supply line");
  }
}

bool detectMotionFromIMU() {
#if ENABLE_MPU6050
  // ============================================================
  // МЕТОД 1: Аппаратное прерывание (быстрое обнаружение)
  // ============================================================
  if (mpu.getIntMotionStatus()) {

    mpu.getIntStatus();
    return true;
  }

  // ============================================================
  // МЕТОД 2: Программный анализ
  // ============================================================
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Фильтры для гироскопа
  static GKalman gyroFilterX(1, 1, 0.1f);
  static GKalman gyroFilterY(1, 1, 0.1f);
  static GKalman gyroFilterZ(1, 1, 0.1f);

  float filteredGx = gyroFilterX.filtered(gx);
  float filteredGy = gyroFilterY.filtered(gy);
  float filteredGz = gyroFilterZ.filtered(gz);

  // RMS гироскопа
  float gyroRms = sqrt((filteredGx * filteredGx + filteredGy * filteredGy + filteredGz * filteredGz) / 3.0f);

  // ============================================================
  // МЕТОД 3: Анализ акселерометра (дополнительный)
  // ============================================================
  // Фильтр для акселерометра
  static GKalman accelFilterX(1, 1, 0.1f);
  static GKalman accelFilterY(1, 1, 0.1f);
  static GKalman accelFilterZ(1, 1, 0.1f);

  float filteredAx = accelFilterX.filtered(ax);
  float filteredAy = accelFilterY.filtered(ay);
  float filteredAz = accelFilterZ.filtered(az);

  // Вычисляем изменение ускорения (детектируем рывки)
  static float prevAx = 0, prevAy = 0, prevAz = 0;
  float accelDelta = sqrt(pow(filteredAx - prevAx, 2) + pow(filteredAy - prevAy, 2) + pow(filteredAz - prevAz, 2));
  prevAx = filteredAx;
  prevAy = filteredAy;
  prevAz = filteredAz;

  // ============================================================
  // АДАПТИВНЫЙ ПОРОГ С ИСТОРИЕЙ
  // ============================================================
  static float historyBuffer[10] = { 0 };
  static uint8_t historyIndex = 0;
  static bool historyFilled = false;

  // Сохраняем текущее значение в историю
  historyBuffer[historyIndex] = gyroRms;
  historyIndex = (historyIndex + 1) % 10;
  if (historyIndex == 0) historyFilled = true;

  // Вычисляем среднее и стандартное отклонение
  float sum = 0, sumSq = 0;
  uint8_t count = historyFilled ? 10 : historyIndex;
  for (uint8_t i = 0; i < count; i++) {
    sum += historyBuffer[i];
    sumSq += historyBuffer[i] * historyBuffer[i];
  }
  float mean = sum / count;
  float stdDev = sqrt((sumSq / count) - (mean * mean));

  // Динамический порог: среднее + 3 * стандартное отклонение
  float dynamicThreshold = mean + 3.0f * stdDev;

  // Минимальный порог для защиты от шума
  float minThreshold = ConfigManager::getGyroThreshold() * 0.8f;  // 8.8.0: порог из меню (Движение)
  if (dynamicThreshold < minThreshold) {
    dynamicThreshold = minThreshold;
  }

  // ============================================================
  // КОМБИНИРОВАННОЕ ОБНАРУЖЕНИЕ
  // ============================================================
  bool gyroMotion = gyroRms > dynamicThreshold;
  bool accelMotion = accelDelta > ConfigManager::getAccelThreshold();  // 8.8.0: порог из меню (Движение)

  // Если есть движение по гироскопу ИЛИ резкое изменение ускорения
  return gyroMotion || accelMotion;

#else
  return false;
#endif
}

void initializeDMP() {
#if ENABLE_MPU6050
  Logger::log(Logger::INFO, "MPU", "Инициализация");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Logger::log(Logger::ERROR, "MPU", "Ошибка подключения!");
    ErrorHandler::handleError(ErrorHandler::Error::MPU, "MPU6050 не отвечает");
    mpuOk = false;
    return;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setSleepEnabled(false);

  mpu.setMotionDetectionThreshold(ConfigManager::getImuMotionDet());  // 8.8.0: порог из меню (IMU / MPU6050)
  mpu.setMotionDetectionDuration(50);
  mpu.setIntMotionEnabled(true);
  mpu.setInterruptMode(false);

  // 8.8.0: применяем сохранённые в конфиге офсеты калибровки IMU (Меню → IMU / MPU6050)
  mpu.setXGyroOffset(ConfigManager::getImuGyroOffX());
  mpu.setYGyroOffset(ConfigManager::getImuGyroOffY());
  mpu.setZGyroOffset(ConfigManager::getImuGyroOffZ());
  mpu.setXAccelOffset(ConfigManager::getImuAccelOffX());
  mpu.setYAccelOffset(ConfigManager::getImuAccelOffY());
  mpu.setZAccelOffset(ConfigManager::getImuAccelOffZ());

  // ========== ✅ ПРОВЕРКА DMP ==========
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Logger::logf(Logger::ERROR, "MPU", "DMP error %d", devStatus);
    mpuOk = false;
    ErrorHandler::handleError(ErrorHandler::Error::MPU, "MPU6050 DMP ошибка");

    // ✅ ВАЖНО: ВЫХОДИМ ИЗ ФУНКЦИИ, ЧТОБЫ НЕ ИСПОЛЬЗОВАТЬ DMP
    Serial.printf("[MPU] DMP error code: %d - DMP отключен\n", devStatus);
    return;  // ← ВЫХОДИМ ЗДЕСЬ!
  }

  // ========== ✅ ВКЛЮЧАЕМ DMP ТОЛЬКО ЕСЛИ ИНИЦИАЛИЗАЦИЯ УСПЕШНА ==========
  mpu.setDMPEnabled(true);
  mpuOk = true;
  ErrorHandler::removeError(ErrorHandler::Error::MPU);

  Logger::log(Logger::INFO, "MPU", "Детектор движения настроен (порог=100, длительность=50ms)");

#else
  Serial.println("[MPU] MPU6050 отключен (тестовый режим)");
  mpuOk = false;
  angleX = 0;
  angleY = 0;
  temperature = 25.0;
#endif
}

void drawTiltIndicatorAdvanced(int16_t x, int16_t y, int16_t size, float angleX, float angleY) {
  static float lastAngleX = 0, lastAngleY = 0;

  // 8.8.0: порог перерисовки по углам из настроек (Дисплей → Порог углов)
  if (abs(angleX - lastAngleX) < ConfigManager::getRedrawAngleThr() && abs(angleY - lastAngleY) < ConfigManager::getRedrawAngleThr()) {
    return;
  }

  lastAngleX = angleX;
  lastAngleY = angleY;

  const int16_t centerX = x + size / 2;
  const int16_t centerY = y + size / 2;
  const int16_t halfSize = size / 2;

  int16_t horizonShift = constrain(angleY * 10.0f, -60, 60);

  tft.fillRect(x, y, size, halfSize + horizonShift, theme::SKY);
  tft.fillRect(x, y + halfSize + horizonShift, size, halfSize - horizonShift, theme::GROUND);

  // «Трава» на земле
  for (int i = 0; i < 5; i++) {
    int16_t lineY = y + halfSize + horizonShift + 8 + i * 15;
    if (lineY < y + size - 8) {
      tft.drawFastHLine(x + 12, lineY, size - 24, ST77XX_YELLOW);
    }
  }

  // Шкала тангажа
  int minMark = -9;
  int maxMark = 9;

  if (angleY > 0) {
    int extraMarks = constrain(angleY * 2, 0, 6);
    minMark = -9 - extraMarks;
    if (minMark < -15) minMark = -15;
  } else if (angleY < 0) {
    int extraMarks = constrain(abs(angleY) * 2, 0, 6);
    maxMark = 9 + extraMarks;
    if (maxMark > 15) maxMark = 15;
  }

  ui.setTransparent(true);
  ui.setColors(ST77XX_BLACK, theme::SKY);
  ui.setFont(UiFont::Tiny);

  for (int i = minMark; i <= maxMark; i++) {
    if (i == 0 || i % 2 == 0) continue;

    int16_t markY = centerY + i * 7 + horizonShift;
    if (markY >= y + 4 && markY <= y + size - 4) {
      tft.drawFastHLine(x + 6, markY, 6, theme::TEXT);
      tft.drawFastHLine(x + size - 12, markY, 6, theme::TEXT);

      ui.drawf(x + 16, markY + 2, "%d", abs(i));
    }
  }

  int16_t centerMarkY = centerY + horizonShift;
  if (centerMarkY >= y + 4 && centerMarkY <= y + size - 4) {
    tft.drawFastHLine(x + 6, centerMarkY, 10, theme::TEXT);
    tft.drawFastHLine(x + size - 16, centerMarkY, 10, theme::TEXT);
  }

  int16_t horizonLineY = centerY + horizonShift;
  tft.drawFastHLine(x + 6, horizonLineY, size - 12, theme::TEXT);

  // Метка крена
  int16_t lineLength = size - 16;
  float visualAngle = constrain(angleX, -5.0f, 5.0f) * 11.0f;
  float rollRad = -visualAngle * 3.14159f / 180.0f;

  int16_t lineX1 = centerX - (lineLength / 2) * cos(rollRad);
  int16_t lineY1 = centerY - (lineLength / 2) * sin(rollRad);
  int16_t lineX2 = centerX + (lineLength / 2) * cos(rollRad);
  int16_t lineY2 = centerY + (lineLength / 2) * sin(rollRad);

  for (int dx = -2; dx <= 2; dx++) {
    for (int dy = -2; dy <= 2; dy++) {
      if (dx == 0 && dy == 0) continue;
      tft.drawLine(lineX1 + dx, lineY1 + dy, lineX2 + dx, lineY2 + dy, ST77XX_BLACK);
    }
  }
  tft.drawLine(lineX1, lineY1, lineX2, lineY2, ST77XX_BLACK);

  tft.fillCircle(lineX1, lineY1, 4, ST77XX_RED);
  tft.fillCircle(lineX2, lineY2, 4, ST77XX_RED);

  tft.fillTriangle(centerX - 8, centerY - 5,
                   centerX + 8, centerY - 5,
                   centerX, centerY + 9, ST77XX_RED);
  tft.fillTriangle(centerX - 5, centerY + 5,
                   centerX + 5, centerY + 5,
                   centerX, centerY - 9, ST77XX_RED);

  tft.fillCircle(centerX, centerY, 6, theme::TEXT);
  tft.fillCircle(centerX, centerY, 2, ST77XX_BLACK);
  tft.drawCircle(centerX, centerY, 7, theme::TEXT);

  // Рамка — последней, чтобы заливки неба/земли её не перекрыли
  tft.drawRect(x, y, size, size, theme::ACCENT);
}

void maintainMovementPressure() {
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    static uint32_t lastWaitLog = 0;
    if (millis() - lastWaitLog > 30000) {
      lastWaitLog = millis();
      Serial.println("[MOVEMENT] Ожидание нормализации давления в магистрали...");
    }
    return;
  }

  static uint32_t lastAdjustmentTimeFront = 0;
  static uint32_t lastAdjustmentTimeRear = 0;
  static const uint32_t ADJUSTMENT_COOLDOWN_MS = 5000;
  static uint32_t lastPressureCheck = 0;

  uint32_t currentTime = millis();

  if (currentTime - lastPressureCheck >= 120000) {
    requestPressureMeasurement();
    lastPressureCheck = currentTime;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  float targetPressureFront = ConfigManager::getMovementPressureFront();
  float targetPressureRear = ConfigManager::getMovementPressureRear();
  float minPressure = ConfigManager::getPressureMin();

  float avgPressureFront, avgPressureRear;
  float currentPressure[PAD_COUNT];
  float currentMaster;

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    memcpy(currentPressure, pressure, sizeof(pressure));
    currentMaster = masterPressure;
    avgPressureFront = (currentPressure[PAD_FRONT_LEFT] + currentPressure[PAD_FRONT_RIGHT]) / 2.0f;
    avgPressureRear = (currentPressure[PAD_REAR_LEFT] + currentPressure[PAD_REAR_RIGHT]) / 2.0f;
  }

  if (currentMaster < minPressure) {
    Serial.printf("[MOVEMENT] Низкое давление в магистрали: %.1f бар < %.1f\n",
                  currentMaster, minPressure);

    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::handleError(ErrorHandler::Error::LOW_PRESSURE,
                                "Низкое давление в магистрали");
      errorScreenBlocking = true;
      emergencyStop();
    }
    return;
  }

  float diffFront = targetPressureFront - avgPressureFront;
  float maxPressure = ConfigManager::getPressureMax();

  bool canAdjustFront = true;
  if (diffFront > 0) {
    if (currentPressure[PAD_FRONT_LEFT] >= maxPressure - ConfigManager::getPressureDeadband() || currentPressure[PAD_FRONT_RIGHT] >= maxPressure - ConfigManager::getPressureDeadband()) {
      canAdjustFront = false;
      Serial.println("[MOVEMENT] Передние подушки достигли MAX давления!");
    }
  } else if (diffFront < 0) {
    if (currentPressure[PAD_FRONT_LEFT] <= minPressure + ConfigManager::getPressureDeadband() || currentPressure[PAD_FRONT_RIGHT] <= minPressure + ConfigManager::getPressureDeadband()) {
      canAdjustFront = false;
      Serial.println("[MOVEMENT] Передние подушки достигли MIN давления!");
    }
  }

  if (canAdjustFront && abs(diffFront) > ConfigManager::getMovementTolerance() && (currentTime - lastAdjustmentTimeFront >= ADJUSTMENT_COOLDOWN_MS)) {

    if (diffFront > 0) {
      Serial.printf("[MOVEMENT] Накачка передних: %.1f -> %.1f\n", avgPressureFront, targetPressureFront);
      sendValveCommand(PAD_FRONT_LEFT, true, 800);
      sendValveCommand(PAD_FRONT_RIGHT, true, 800);
    } else {
      Serial.printf("[MOVEMENT] Стравливание передних: %.1f -> %.1f\n", avgPressureFront, targetPressureFront);
      sendValveCommand(PAD_FRONT_LEFT, false, 600);
      sendValveCommand(PAD_FRONT_RIGHT, false, 600);
    }
    lastAdjustmentTimeFront = currentTime;
    setDisplayDirty();
    requestPressureMeasurement();
  }

  float diffRear = targetPressureRear - avgPressureRear;

  bool canAdjustRear = true;
  if (diffRear > 0) {
    if (currentPressure[PAD_REAR_LEFT] >= maxPressure - ConfigManager::getPressureDeadband() || currentPressure[PAD_REAR_RIGHT] >= maxPressure - ConfigManager::getPressureDeadband()) {
      canAdjustRear = false;
      Serial.println("[MOVEMENT] Задние подушки достигли MAX давления!");
    }
  } else if (diffRear < 0) {
    if (currentPressure[PAD_REAR_LEFT] <= minPressure + ConfigManager::getPressureDeadband() || currentPressure[PAD_REAR_RIGHT] <= minPressure + ConfigManager::getPressureDeadband()) {
      canAdjustRear = false;
      Serial.println("[MOVEMENT] Задние подушки достигли MIN давления!");
    }
  }

  if (canAdjustRear && abs(diffRear) > ConfigManager::getMovementTolerance() && (currentTime - lastAdjustmentTimeRear >= ADJUSTMENT_COOLDOWN_MS)) {

    if (diffRear > 0) {
      Serial.printf("[MOVEMENT] Накачка задних: %.1f -> %.1f\n", avgPressureRear, targetPressureRear);
      sendValveCommand(PAD_REAR_LEFT, true, 2000);
      sendValveCommand(PAD_REAR_RIGHT, true, 2000);
    } else {
      Serial.printf("[MOVEMENT] Стравливание задних: %.1f -> %.1f\n", avgPressureRear, targetPressureRear);
      sendValveCommand(PAD_REAR_LEFT, false, 1500);
      sendValveCommand(PAD_REAR_RIGHT, false, 1500);
    }
    lastAdjustmentTimeRear = currentTime;
    setDisplayDirty();
    requestPressureMeasurement();
  }
}

class AutoLevelingController {
private:
  // ✅ ВЫНЕСЕНЫ В КОНСТАНТЫ КЛАССА
  static constexpr float STABLE_THRESHOLD = 0.1f;
  static constexpr uint32_t STABLE_DURATION_MS = 1000;
  static constexpr uint32_t READ_TIMEOUT_MS = 100;
  static constexpr float FINE_TUNING_SCALE = 1.0f;
  static constexpr uint8_t MAX_FINE_TUNING_ITERATIONS = 10;

  enum class LevelingStage {
    IDLE,
    COARSE_ROLL,
    COARSE_PITCH,
    FINE_TUNING,
    WAITING_STABLE,
    COMPLETED
  };

  LevelingStage currentStage = LevelingStage::IDLE;
  uint8_t coarseStep = 0;
  uint8_t fineTuningIterations = 0;
  uint32_t stageStartTime = 0;

  bool waitForStability(float targetX, float targetY, uint32_t timeoutMs = 5000) {
    uint32_t startTime = millis();
    uint32_t stableStartTime = 0;
    uint32_t lastReadTime = 0;

    while (millis() - startTime < timeoutMs) {
      if (millis() - lastReadTime < READ_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      lastReadTime = millis();

      float currentX, currentY;
      {
        MutexGuard guard(xStateMutex);
        if (!guard) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
        currentX = angleX;
        currentY = angleY;
      }

      float deltaX = abs(currentX - targetX);
      float deltaY = abs(currentY - targetY);

      if (deltaX <= STABLE_THRESHOLD && deltaY <= STABLE_THRESHOLD) {
        if (stableStartTime == 0) {
          stableStartTime = millis();
        } else if (millis() - stableStartTime >= STABLE_DURATION_MS) {
          Serial.println("[AUTO] Система стабилизировалась");
          return true;
        }
      } else {
        stableStartTime = 0;
      }

      vTaskDelay(pdMS_TO_TICKS(50));
    }

    Serial.println("[AUTO] Таймаут ожидания стабилизации");
    return false;
  }

  bool executeValveCommand(Pad pad, bool inflate, uint32_t durationMs, uint32_t stabilizeMs = 500) {
    Serial.printf("[AUTO] Выполнение: %s %s на %d мс\n",
                  padNames[pad], inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ", durationMs);

    float pressureBefore;
    {
      MutexGuard guard(xStateMutex);
      if (!guard) return false;
      pressureBefore = pressure[pad];
    }

    if (!sendValveCommandSync(pad, inflate, durationMs, stabilizeMs)) {
      Serial.printf("[AUTO] Ошибка выполнения команды для %s\n", padNames[pad]);
      return false;
    }

    requestPressureMeasurement();
    vTaskDelay(pdMS_TO_TICKS(500));

    float pressureAfter;
    {
      MutexGuard guard(xStateMutex);
      if (!guard) return false;
      pressureAfter = pressure[pad];
    }

    float pressureDelta = pressureAfter - pressureBefore;

    if (abs(pressureDelta) < 0.1f) {
      Serial.printf("[AUTO] ⚠️ Давление не изменилось! Было: %.1f, Стало: %.1f\n",
                    pressureBefore, pressureAfter);
      return false;
    }

    return true;
  }

public:
  void process() {
    // Базовые проверки
    if (currentSystemMode != SystemMode::AUTO) return;
    if (currentState != SystemState::RUNNING) return;

    // Проверка ошибок
    if (ErrorHandler::hasActiveErrors()) {
      static uint32_t lastErrorLog = 0;
      uint32_t now = millis();
      if (currentStage != LevelingStage::IDLE) {
        if (now - lastErrorLog > 5000) {
          lastErrorLog = now;
          Serial.println("[AUTO] Выравнивание прервано из-за ошибок");
        }
        currentStage = LevelingStage::IDLE;
        coarseStep = 0;
        fineTuningIterations = 0;
      }
      return;
    }

    // Проверка MPU
    if (!mpuOk) {
      static uint32_t lastMpuLog = 0;
      uint32_t now = millis();
      if (currentStage != LevelingStage::IDLE) {
        if (now - lastMpuLog > 5000) {
          lastMpuLog = now;
          Serial.println("[AUTO] Выравнивание прервано: MPU не исправен");
        }
        currentStage = LevelingStage::IDLE;
        coarseStep = 0;
        fineTuningIterations = 0;
      }
      return;
    }

    // Проверка движения
    if (isMoving) {
      static uint32_t lastMoveLog = 0;
      uint32_t now = millis();
      if (currentStage != LevelingStage::IDLE) {
        if (now - lastMoveLog > 5000) {
          lastMoveLog = now;
          Serial.println("[AUTO] Выравнивание прервано из-за движения");
        }
        currentStage = LevelingStage::IDLE;
        coarseStep = 0;
        fineTuningIterations = 0;
      }
      return;
    }

    // Проверка давления
    float currentMaster;
    {
      MutexGuard guard(xStateMutex);
      if (!guard) return;
      currentMaster = masterPressure;
    }

    float minPressure = ConfigManager::getPressureMin();
    if (currentMaster < minPressure) {
      static uint32_t lastPressureLog = 0;
      uint32_t now = millis();
      if (now - lastPressureLog > 10000) {
        lastPressureLog = now;
        Serial.printf("[AUTO] Низкое давление в магистрали (%.1f < %.1f), выравнивание невозможно\n",
                      currentMaster, minPressure);
      }
      if (currentStage != LevelingStage::IDLE) {
        currentStage = LevelingStage::IDLE;
        coarseStep = 0;
        fineTuningIterations = 0;
      }
      return;
    }

    // Ограничение частоты
    static uint32_t lastProcessTime = 0;
    uint32_t now = millis();

    if (now - lastProcessTime < 500) return;
    lastProcessTime = now;

    // Получение данных
    float currentX, currentY;
    float thX = ConfigManager::getTiltThresholdX();
    float thY = ConfigManager::getTiltThresholdY();

    {
      MutexGuard guard(xStateMutex);
      if (!guard) return;
      currentX = angleX;
      currentY = angleY;
    }

    bool needLeveling = (abs(currentX) > thX) || (abs(currentY) > thY);

    // Основной автомат
    switch (currentStage) {
      case LevelingStage::IDLE:
        if (needLeveling) {
          if (levelingAttemptsThisHour >= ConfigManager::getNivCount()) {
            static uint32_t lastLimitLog = 0;
            if (now - lastLimitLog > 60000) {
              lastLimitLog = now;
              Serial.printf("[AUTO] Лимит попыток: %d/%d\n",
                            levelingAttemptsThisHour, ConfigManager::getNivCount());
            }
            return;
          }

          if (now - lastLevelingAttemptTime < 30000) {
            return;
          }

          Serial.printf("[AUTO] Начало выравнивания: X=%.2f° (порог=%.1f°), Y=%.2f° (порог=%.1f°)\n",
                        currentX, thX, currentY, thY);

          levelingAttemptsThisHour++;
          lastLevelingAttemptTime = now;
          currentStage = LevelingStage::COARSE_ROLL;
          coarseStep = 0;
          stageStartTime = now;
        }
        break;

      case LevelingStage::COARSE_ROLL:
        if (abs(currentX) > thX * ConfigManager::getCoarseZoneRatio()) {
          float prevX = currentX;

          if (coarseStep == 0) {
            if (currentX > thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВЛЕВО - стравливание правых");
              if (executeValveCommand(PAD_FRONT_RIGHT, false,
                                      ConfigManager::getReleaseDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 100, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentX < -thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - стравливание левых");
              if (executeValveCommand(PAD_FRONT_LEFT, false,
                                      ConfigManager::getReleaseDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_LEFT, false,
                                         ConfigManager::getReleaseDelay() * 100, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;

          } else if (coarseStep == 1) {
            if (currentX > thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВЛЕВО - накачка левых");
              if (executeValveCommand(PAD_FRONT_LEFT, true,
                                      ConfigManager::getInflateDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_LEFT, true,
                                         ConfigManager::getInflateDelay() * 100, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentX < -thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - накачка правых");
              if (executeValveCommand(PAD_FRONT_RIGHT, true,
                                      ConfigManager::getInflateDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 100, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;
          }

          vTaskDelay(pdMS_TO_TICKS(500));
          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              currentX = angleX;
            }
          }
          if (abs(currentX) > abs(prevX) * ConfigManager::getWorseningRatio() && abs(prevX) > 0.1f) {
            Serial.printf("[AUTO] ⚠️ Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
                          prevX, currentX);
            if (coarseStep > 0) {
              coarseStep--;
              Serial.printf("[AUTO] Возврат к шагу %d\n", coarseStep);
            }
          }
        } else {
          Serial.println("[AUTO] Крен в норме, переход к тангажу");
          currentStage = LevelingStage::COARSE_PITCH;
          coarseStep = 0;
          stageStartTime = now;
        }
        break;

      case LevelingStage::COARSE_PITCH:
        if (abs(currentY) > thY * ConfigManager::getCoarseZoneRatio()) {
          float prevY = currentY;

          if (coarseStep == 0) {
            if (currentY > thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВВЕРХ - стравливание передних");
              if (executeValveCommand(PAD_FRONT_LEFT, false,
                                      ConfigManager::getReleaseDelay() * 100, 500)
                  && executeValveCommand(PAD_FRONT_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 100, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentY < -thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВНИЗ - стравливание задних");
              if (executeValveCommand(PAD_REAR_LEFT, false,
                                      ConfigManager::getReleaseDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 100, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;

          } else if (coarseStep == 1) {
            if (currentY > thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВВЕРХ - накачка задних");
              if (executeValveCommand(PAD_REAR_LEFT, true,
                                      ConfigManager::getInflateDelay() * 100, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 100, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentY < -thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВНИЗ - накачка передних");
              if (executeValveCommand(PAD_FRONT_LEFT, true,
                                      ConfigManager::getInflateDelay() * 100, 500)
                  && executeValveCommand(PAD_FRONT_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 100, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ❌ Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;
          }

          vTaskDelay(pdMS_TO_TICKS(500));
          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              currentY = angleY;
            }
          }
          if (abs(currentY) > abs(prevY) * ConfigManager::getWorseningRatio() && abs(prevY) > 0.1f) {
            Serial.printf("[AUTO] ⚠️ Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
                          prevY, currentY);
            if (coarseStep > 0) {
              coarseStep--;
              Serial.printf("[AUTO] Возврат к шагу %d\n", coarseStep);
            }
          }
        } else {
          if (abs(currentX) <= thX && abs(currentY) <= thY) {
            Serial.println("[AUTO] Выравнивание завершено успешно");
            currentStage = LevelingStage::COMPLETED;
          } else {
            Serial.println("[AUTO] Тангаж в норме, переход к точной настройке");
            currentStage = LevelingStage::FINE_TUNING;
            fineTuningIterations = 0;
            stageStartTime = now;
          }
        }
        break;

      case LevelingStage::FINE_TUNING:
        if (fineTuningIterations >= MAX_FINE_TUNING_ITERATIONS) {
          Serial.println("[AUTO] Точная настройка: лимит итераций");
          currentStage = LevelingStage::COMPLETED;
          break;
        }

        {
          float cornerHeights[4];
          cornerHeights[PAD_FRONT_LEFT] = (-currentY - currentX) * FINE_TUNING_SCALE;
          cornerHeights[PAD_FRONT_RIGHT] = (-currentY + currentX) * FINE_TUNING_SCALE;
          cornerHeights[PAD_REAR_LEFT] = (+currentY - currentX) * FINE_TUNING_SCALE;
          cornerHeights[PAD_REAR_RIGHT] = (+currentY + currentX) * FINE_TUNING_SCALE;

          uint8_t highCorner = 0;
          uint8_t lowCorner = 0;
          float maxHeight = cornerHeights[0];
          float minHeight = cornerHeights[0];

          for (int i = 1; i < 4; i++) {
            if (cornerHeights[i] > maxHeight) {
              maxHeight = cornerHeights[i];
              highCorner = i;
            }
            if (cornerHeights[i] < minHeight) {
              minHeight = cornerHeights[i];
              lowCorner = i;
            }
          }

          float heightDifference = maxHeight - minHeight;
          float fineThreshold = max(thX, thY) * ConfigManager::getFineZoneRatio();

          if (heightDifference >= fineThreshold) {
            Serial.printf("[AUTO] Точная: стравливание угла %d (высота %.2f), разница %.2f\n",
                          highCorner, maxHeight, heightDifference);

            if (executeValveCommand(Pad(highCorner), false,
                                    ConfigManager::getReleaseDelay() * 50, 300)) {
              vTaskDelay(pdMS_TO_TICKS(500));

              {
                MutexGuard guard(xStateMutex);
                if (guard) {
                  currentX = angleX;
                  currentY = angleY;
                }
              }

              cornerHeights[PAD_FRONT_LEFT] = (-currentY - currentX) * FINE_TUNING_SCALE;
              cornerHeights[PAD_FRONT_RIGHT] = (-currentY + currentX) * FINE_TUNING_SCALE;
              cornerHeights[PAD_REAR_LEFT] = (+currentY - currentX) * FINE_TUNING_SCALE;
              cornerHeights[PAD_REAR_RIGHT] = (+currentY + currentX) * FINE_TUNING_SCALE;

              float newMinHeight = cornerHeights[0];
              uint8_t newLowCorner = 0;
              for (int i = 1; i < 4; i++) {
                if (cornerHeights[i] < newMinHeight) {
                  newMinHeight = cornerHeights[i];
                  newLowCorner = i;
                }
              }

              Serial.printf("[AUTO] Точная: накачка угла %d\n", newLowCorner);
              executeValveCommand(Pad(newLowCorner), true,
                                  ConfigManager::getInflateDelay() * 50, 300);
            }
            fineTuningIterations++;
            currentStage = LevelingStage::WAITING_STABLE;
            stageStartTime = now;
          } else {
            Serial.printf("[AUTO] Точная настройка завершена (разница %.2f < %.2f)\n",
                          heightDifference, fineThreshold);
            currentStage = LevelingStage::COMPLETED;
          }
        }
        break;

      case LevelingStage::WAITING_STABLE:
        if (now - stageStartTime >= 1500) {
          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              currentX = angleX;
              currentY = angleY;
            }
          }

          if (abs(currentX) <= thX && abs(currentY) <= thY) {
            Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - ОТЛИЧНО!\n", currentX, currentY);
            currentStage = LevelingStage::COMPLETED;

          } else if (abs(currentX) <= thX * ConfigManager::getWorseningRatio() && abs(currentY) <= thY * ConfigManager::getWorseningRatio()) {
            Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - близко к цели\n", currentX, currentY);

            if (abs(currentX) > thX * ConfigManager::getCoarseZoneRatio() || abs(currentY) > thY * ConfigManager::getCoarseZoneRatio()) {
              currentStage = LevelingStage::COARSE_ROLL;
              coarseStep = 0;
              Serial.println("[AUTO] Возврат к грубой настройке крена");
            } else {
              currentStage = LevelingStage::FINE_TUNING;
              fineTuningIterations = 0;
              Serial.println("[AUTO] Возврат к точной настройке");
            }

          } else {
            Serial.printf("[AUTO] Стабилизация: X=%.2f°, Y=%.2f° - требуется продолжение\n", currentX, currentY);

            if (abs(currentX) > thX || abs(currentY) > thY) {
              currentStage = LevelingStage::COARSE_ROLL;
              coarseStep = 0;
              Serial.println("[AUTO] Возврат к грубой настройке крена");
            }
          }
          stageStartTime = now;
        }
        break;

      case LevelingStage::COMPLETED:
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            float finalX = angleX;
            float finalY = angleY;
            if (abs(finalX) > thX || abs(finalY) > thY) {
              Serial.printf("[AUTO] ⚠️ Выравнивание не завершено! X=%.2f, Y=%.2f (пороги: %.1f, %.1f)\n",
                            finalX, finalY, thX, thY);
              currentStage = LevelingStage::COARSE_ROLL;
              coarseStep = 0;
              fineTuningIterations = 0;
              stageStartTime = now;
              break;
            }
          }
        }

        currentStage = LevelingStage::IDLE;
        coarseStep = 0;
        fineTuningIterations = 0;
        lastLevelingCheckTime = now;
        Serial.println("[AUTO] ✅ Выравнивание успешно завершено!");
        break;
    }
  }
};

AutoLevelingController autoLevelingController;

bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs) {
  if (pad >= PAD_COUNT) {
    Serial.printf("[SYNC] Invalid pad: %d\n", pad);
    return false;
  }

  uint32_t safeDuration = (durationMs > 30000) ? 30000 : durationMs;

  QueueHandle_t ackQueue = xQueueCreate(1, sizeof(bool));
  if (ackQueue == nullptr) {
    Serial.println("[SYNC] Failed to create ack queue!");
    return false;
  }

  ValveCommandMsg cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.sync.pad = pad;
  cmd.sync.inflate = inflate;
  cmd.sync.durationMs = safeDuration;
  cmd.sync.ackQueue = ackQueue;

  if (xQueueSend(xValveQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[SYNC] Failed to send to valve queue!");
    vQueueDelete(ackQueue);
    return false;
  }

  bool success = false;
  uint32_t timeoutMs = safeDuration + waitAfterMs + 2000;
  BaseType_t result = xQueueReceive(ackQueue, &success, pdMS_TO_TICKS(timeoutMs));

  vQueueDelete(ackQueue);

  if (result != pdTRUE) {
    Serial.println("[SYNC] Command timeout!");
    return false;
  }

  if (!success) {
    Serial.println("[SYNC] Command failed!");
    return false;
  }

  if (waitAfterMs > 0) {
    vTaskDelay(pdMS_TO_TICKS(waitAfterMs));
  }

  return true;
}




bool initFileSystem() {
  Serial.println("[FS] Инициализация LittleFS...");
  bool mounted = LittleFS.begin(false);
  if (!mounted) {
    Serial.println("[FS] Раздел повреждён, выполняю однократное форматирование");
    LittleFS.end();
    mounted = LittleFS.begin(true);
    if (!mounted) {
      Serial.println("[FS] Не удалось восстановить LittleFS");
      return false;
    }
    Serial.println("[FS] LittleFS отформатирована и подключена");
  }
  Serial.println("[FS] LittleFS смонтирован успешно");

  File root = LittleFS.open("/");
  File file = root.openNextFile();
  int fileCount = 0;
  while (file) {
    fileCount++;
    Serial.printf("[FS] Файл: %s (%d байт)\n", file.name(), file.size());
    file = root.openNextFile();
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

bool initializeAds1015() {
  for (uint8_t address = 0x48; address <= 0x4B; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error != 0) continue;

    Serial.printf("[ADS1015] I2C устройство найдено по адресу 0x%02X\n", address);
    if (ads.begin(address)) {
      adsDetectedAddress = address;
      adsInitialized = true;
      ads.setGain(GAIN_ONE);
      ads.setDataRate(RATE_ADS1015_1600SPS);
      return true;
    }
  }

  adsInitialized = false;
  Serial.println("[ADS1015] Не найдено устройство по адресам 0x48-0x4B");
  return false;
}

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

void startFallbackAccessPoint() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  if (!WiFi.softAP(wifi_ssid, wifi_password)) {
    Serial.println("[WiFi] Не удалось запустить резервную точку доступа");
    wifiConnected = false;
    return;
  }
  wifiConnected = false;
  Serial.printf("[WiFi] AP fallback: %s, IP %s\n",
                wifi_ssid, WiFi.softAPIP().toString().c_str());
}

void connectConfiguredWiFi() {
  if (sta_ssid[0] == '\0') {
    Serial.println("[WiFi] SSID роутера не выбран, запущен fallback AP");
    startFallbackAccessPoint();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid, WIFI_STA_PASSWORD);
  Serial.printf("[WiFi] Подключение к \"%s\"\n", sta_ssid);
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("[WiFi] STA подключен, IP %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WiFi] STA не подключен, переход в fallback AP");
    startFallbackAccessPoint();
  }
}

void startWiFiSetup() {
  if (wifiScanInProgress) return;
  wifiScanInProgress = true;
  wifiSetupActive = false;
  menuVisible = false;
  displayDirty = true;
  Serial.println("[WiFi] Сканирование сетей...");

  WiFi.mode(WIFI_STA);
  int found = WiFi.scanNetworks(false, true);
  wifi_scan_count = 0;
  for (int i = 0; i < found && wifi_scan_count < WIFI_SCAN_MAX_NETWORKS; i++) {
    String name = WiFi.SSID(i);
    if (name.length() == 0) continue;
    bool duplicate = false;
    for (uint8_t j = 0; j < wifi_scan_count; j++) {
      if (name.equals(wifi_scan_ssids[j])) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) continue;
    strlcpy(wifi_scan_ssids[wifi_scan_count], name.c_str(),
            sizeof(wifi_scan_ssids[wifi_scan_count]));
    wifi_scan_rssi[wifi_scan_count] = static_cast<int8_t>(WiFi.RSSI(i));
    wifi_scan_count++;
  }
  WiFi.scanDelete();
  wifiScanInProgress = false;
  wifi_scan_selected = 0;
  wifiSetupActive = wifi_scan_count > 0;
  displayDirty = true;
  Serial.printf("[WiFi] Найдено сетей: %u\n", wifi_scan_count);
  if (wifi_scan_count == 0) {
    Serial.println("[WiFi] Сети не найдены, возвращаюсь к fallback AP");
    startFallbackAccessPoint();
    menuVisible = true;
    displayDirty = true;
  }
}

void displayWiFiSetupScreen() {
  tft.fillScreen(theme::BG);

  // Шапка
  tft.fillRoundRect(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24,
                    theme::RADIUS, theme::PANEL_ALT);
  ui.setTransparent(true);
  ui.setFont(UiFont::Med);
  ui.setColors(theme::ACCENT, theme::PANEL_ALT);
  ui.box(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24, "ВЫБОР WI-FI",
         UiHAlign::Center, UiVAlign::Middle, false);

  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN, 32, 250, 16, "КН1/КН2 — выбор,  КН3 — подключить",
         UiHAlign::Left, UiVAlign::Middle, false);
  ui.box(theme::MARGIN, 46, 250, 16, "КН4 — отмена, удерж. КН3+КН4 — меню",
         UiHAlign::Left, UiVAlign::Middle, false);

  if (wifiScanInProgress) {
    ui.setFont(UiFont::Small);
    ui.setColors(theme::WARN, theme::BG);
    ui.box(theme::MARGIN, 90, 250, 18, "Сканирование...", UiHAlign::Left, UiVAlign::Middle, true);
    return;
  }

  constexpr int16_t LIST_Y = 66;
  constexpr int16_t ROW = 20;

  for (uint8_t i = 0; i < wifi_scan_count; i++) {
    const int16_t y = LIST_Y + i * ROW;
    const bool selected = (i == wifi_scan_selected);
    const int rssi = wifi_scan_rssi[i];

    if (selected) {
      tft.fillRoundRect(theme::MARGIN - 2, y, theme::SCREEN_W - 2 * theme::MARGIN + 4, ROW, 3, theme::PANEL);
    } else {
      tft.fillRect(theme::MARGIN, y, theme::SCREEN_W - 2 * theme::MARGIN, ROW, theme::BG);
    }

    // Уровень сигнала: 4 полоски
    const uint8_t bars = (rssi > -55) ? 4 : (rssi > -70) ? 3 : (rssi > -85) ? 2 : 1;
    for (uint8_t b = 0; b < 4; b++) {
      const int16_t bx = static_cast<int16_t>(theme::SCREEN_W - theme::MARGIN - 66 + b * 6);
      const int16_t bh = static_cast<int16_t>(4 + b * 3);
      const uint16_t barColor = (b < bars) ? ((bars >= 3) ? theme::OK : theme::WARN) : theme::TRACK;
      tft.fillRect(bx, static_cast<int16_t>(y + ROW - 5 - bh), 4, bh, barColor);
    }

    ui.setFont(selected ? UiFont::SmallB : UiFont::Small);
    ui.setColors(selected ? theme::ACCENT : theme::TEXT, selected ? theme::PANEL : theme::BG);
    ui.box(theme::MARGIN + 2, y, 186, ROW, ui.ellipsize(wifi_scan_ssids[i], 180),
           UiHAlign::Left, UiVAlign::Middle, true);

    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, selected ? theme::PANEL : theme::BG);
    ui.boxf(theme::SCREEN_W - theme::MARGIN - 26, y, 26, ROW, UiHAlign::Right, UiVAlign::Middle, true,
            "%d", rssi);
  }
}

void initializeDefaultCredentials() {
  uint64_t chipId = ESP.getEfuseMac();
  uint32_t suffix = static_cast<uint32_t>(chipId & 0xFFFFFF);
  if (wifi_password[0] == '\0' || strcmp(wifi_password, "12345678") == 0) {
    snprintf(wifi_password, sizeof(wifi_password), "KzWiFi-%06lX", suffix);
  }
  if (ota_password[0] == '\0' || strcmp(ota_password, "12345678") == 0) {
    snprintf(ota_password, sizeof(ota_password), "KzOTA-%06lX", suffix);
  }
}

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
  testStepStartTime = millis();

  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║              ТЕСТ КЛАПАНОВ ПНЕВМОСИСТЕМЫ                   ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝");

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
      Serial.println("[ТЕСТ] ❌ Аварийная остановка теста!");
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
    Serial.println("[ТЕСТ] ⏰ Таймаут теста! Принудительное завершение.");
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
          Serial.printf("[ТЕСТ] ✅ Давление достаточное (>= %.1f бар). Тест возможен.\n",
                        TEST_MIN_PRESS_FOR_TEST);
          digitalWrite(PIN_INFL, LOW);
          Serial.println("[ТЕСТ] Клапан накачки закрыт");

          currentTestStep = TestStep::PREPARE_EQUALIZE_PADS;
          currentTestState = TestState::TESTING_PAD;
          testStepStartTime = currentTime;
          stepInitialized = false;
        } else {
          testResult.supplyPressureOk = false;
          Serial.printf("[ТЕСТ] ❌ Давление недостаточное (%.1f < %.1f бар)\n",
                        testResult.supplyPressure, TEST_MIN_PRESS_FOR_TEST);
          Serial.println("[ТЕСТ] ⚠️ НЕОБХОДИМО: Включить компрессор или завести двигатель");
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
            Serial.printf("[ТЕСТ] ✅ Давление поднялось до %.1f бар! Тест возможен.\n", currentPressure);
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
                      testResult.deflateValveWorks ? "✅" : "❌",
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
                      testResult.inflateValveWorks ? "✅" : "❌",
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
          Serial.printf("[ТЕСТ] ✅ Давление в норме (≤ %.1f бар)\n", TEST_ZERO_THRESHOLD);
          testReferencePressure = currentPressure;
        } else {
          Serial.printf("[ТЕСТ] ❌ ОШИБКА: Давление слишком высокое (%.1f > %.1f бар)\n",
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
          Serial.printf("[ТЕСТ] ✅ Клапан %s РАБОТАЕТ (давление поднялось на %.1f бар)\n",
                        padNames[testPadIndex], pressureDiff);
        } else {
          testResult.padValvesWorks[testPadIndex] = false;
          Serial.printf("[ТЕСТ] ❌ Клапан %s НЕ РАБОТАЕТ (изменение %.1f < %.1f бар)\n",
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
      break;

    default:
      break;
  }
}

void printTestResults() {
  Serial.println("\n╔════════════════════════════════════════════════════════════╗");
  Serial.println("║                 РЕЗУЛЬТАТЫ ТЕСТА КЛАПАНОВ                 ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝");

  Serial.println("\n📋 ПОДГОТОВКА:");
  Serial.printf("   Давление в магистрали: %.1f бар %s\n",
                testResult.supplyPressure,
                testResult.supplyPressureOk ? "✅" : "❌");
  Serial.printf("   Давление после выравнивания: %.1f бар\n", testResult.equalizedPressure);

  Serial.println("\n🔧 ОБЩИЕ КЛАПАНЫ:");
  Serial.printf("   Клапан НАКАЧКИ: %s\n",
                testResult.inflateValveWorks ? "✅ РАБОТАЕТ" : "❌ НЕ РАБОТАЕТ");
  Serial.printf("   Клапан СБРОСА:  %s\n",
                testResult.deflateValveWorks ? "✅ РАБОТАЕТ" : "❌ НЕ РАБОТАЕТ");

  Serial.println("\n🛞 КЛАПАНЫ ПОДУШЕК:");
  for (int i = 0; i < PAD_COUNT; i++) {
    Serial.printf("   %s: %s  (Δ%+.1f бар)\n",
                  padNames[i],
                  testResult.padValvesWorks[i] ? "✅ РАБОТАЕТ" : "❌ НЕ РАБОТАЕТ",
                  testResult.pressureReadings[i][1] - testResult.pressureReadings[i][0]);
  }

  bool allOk = testResult.supplyPressureOk && testResult.inflateValveWorks && testResult.deflateValveWorks;
  for (int i = 0; i < PAD_COUNT; i++) {
    if (!testResult.padValvesWorks[i]) allOk = false;
  }

  Serial.println("\n════════════════════════════════════════════════════════════");
  if (allOk) {
    Serial.println("✅ ВСЕ КЛАПАНЫ РАБОТАЮТ КОРРЕКТНО!");
  } else {
    Serial.println("⚠️ ОБНАРУЖЕНЫ НЕИСПРАВНОСТИ КЛАПАНОВ!");
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
  Serial.println("════════════════════════════════════════════════════════════\n");
}

/* ====================  Чтение давления через ADS1015 ==================== */
/* ====================  Чтение давления через ADS1015 ==================== */
float readPressure() {
#if ENABLE_SIMULATION
  static float simPressure = 4.5f;
  simPressure += ((float)random(-10, 10) / 100.0f);
  simPressure = constrain(simPressure, 3.0f, 6.0f);

  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
    ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
  }
  return simPressure;
#endif

  // ========== ЧТЕНИЕ ЧЕРЕЗ ADS1015 ==========
  // Проверка, инициализирован ли ADS1015
  if (!adsInitialized) {
    // Если датчик не инициализирован, возвращаем 0, ошибка уже установлена в setup
    return 0.0f;
  }

  static uint32_t lastReadTime = 0;
  uint32_t now = millis();

  // Не читаем чаще чем раз в 50 мс (стабилизация)
  if (now - lastReadTime < 50) {
    return -1.0f;
  }
  lastReadTime = now;

  // Читаем значение с ADS1015
  int16_t raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);

  // raw==0 часто бывает при кратком сбое I2C — не считаем ошибкой с первого раза
  static uint8_t consecutiveZeroReads = 0;
  if (raw == 0) {
    consecutiveZeroReads++;
    if (consecutiveZeroReads < 3) {
      Serial.printf("[WARN] ADS1015: raw=0 (transient %u/3)\n", consecutiveZeroReads);
      return -1.0f;
    }
    consecutiveZeroReads = 0;
    static uint32_t lastErrorTime = 0;
    if (millis() - lastErrorTime > 5000) {
      lastErrorTime = millis();
      Serial.println("[WARN] ADS1015: Ошибка чтения (raw=0 x3)!");
    }
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                "ADS1015 - ошибка чтения");
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
    }
    return 0.0f;
  }
  consecutiveZeroReads = 0;

  // Проверка на обрыв цепи (очень низкое значение)
  if (raw < 10) {
    static uint32_t lastErrorTime = 0;
    if (millis() - lastErrorTime > 5000) {
      lastErrorTime = millis();
      Serial.printf("[WARN] ADS1015: Обрыв цепи! raw=%d\n", raw);
    }
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                "Датчик давления - обрыв цепи");
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
    }
    return 0.0f;
  }

  // Проверка на короткое замыкание (очень высокое значение)
  if (raw >= 2040) {
    static uint32_t lastErrorTime = 0;
    if (millis() - lastErrorTime > 5000) {
      lastErrorTime = millis();
      Serial.printf("[WARN] ADS1015: Короткое замыкание! raw=%d\n", raw);
    }
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                "Датчик давления - КЗ на питание");
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
    }
    return 0.0f;
  }

  // Датчик исправен - помечаем ошибку для удаления (если она была)
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
    ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
    // Serial.println("[SENSOR] Датчик давления исправен, ошибка будет удалена через 3 сек");
  }

  // Применяем медианный фильтр (сглаживание)
  int filtered = pressureFilter.filtered(raw);

  // ============================================================
  // КАЛИБРОВКА - с проверкой валидности
  // ============================================================
  int localZeroRaw;
  bool calibrationValid = false;

  {
    MutexGuard guard(xCalibMutex);
    if (guard) {
      localZeroRaw = g_pressureZeroRaw;
      calibrationValid = true;  // Предполагаем, что калибровка есть
    } else {
      localZeroRaw = 122;
      Serial.println("[WARN] Не удалось получить калибровку, использую default=122");
    }
  }

  // Если калибровка невалидна - используем значение по умолчанию
  if (!calibrationValid) {
    localZeroRaw = 122;
    Serial.println("[WARN] Калибровка не выполнена, использую default=122");
  }

  // Коррекция нуля
  int correctedValue = filtered - localZeroRaw;

  // Защита от отрицательных значений (шум)
  if (correctedValue < 0) correctedValue = 0;

  // Если correctedValue слишком маленький - это 0 давление
  if (correctedValue < 3) {
    return 0.0f;
  }

  // Преобразуем в напряжение (0-3.3V)
  float voltage = ads.computeVolts(correctedValue);
  voltage = constrain(voltage, 0.0f, 3.3f);

  // ============================================================
  // КАЛИБРОВОЧНАЯ ТАБЛИЦА
  // ============================================================
  static const float calib[20][2] = {
    { 0.00f, 0.0f },  // 0 бар
    { 0.06f, 0.5f },  // 0.5 бар
    { 0.12f, 1.0f },  // 1.0 бар
    { 0.16f, 1.5f },  // 1.5 бар
    { 0.21f, 2.0f },  // 2.0 бар
    { 0.24f, 2.5f },  // 2.5 бар
    { 0.28f, 3.0f },  // 3.0 бар
    { 0.31f, 3.5f },  // 3.5 бар
    { 0.35f, 4.0f },  // 4.0 бар
    { 0.39f, 4.5f },  // 4.5 бар
    { 0.43f, 5.0f },  // 5.0 бар
    { 0.47f, 5.5f },  // 5.5 бар
    { 0.51f, 6.0f },  // 6.0 бар
    { 0.56f, 6.5f },  // 6.5 бар
    { 0.61f, 7.0f },  // 7.0 бар
    { 0.66f, 7.5f },  // 7.5 бар
    { 0.72f, 8.0f },  // 8.0 бар
    { 0.78f, 8.5f },  // 8.5 бар
    { 0.85f, 9.0f },  // 9.0 бар (запас)
    { 0.92f, 9.5f }   // 9.5 бар (запас)
  };

  const uint8_t CALIB_SIZE = 20;

  // Интерполяция с проверкой границ
  if (voltage <= calib[0][0]) return 0.0f;
  if (voltage >= calib[CALIB_SIZE - 1][0]) return 9.5f;

  // Находим нужный интервал
  uint8_t i = 0;
  while (i < CALIB_SIZE - 1 && voltage > calib[i + 1][0]) {
    i++;
  }

  // Линейная интерполяция
  float pressure = calib[i][1] + (voltage - calib[i][0]) * (calib[i + 1][1] - calib[i][1]) / (calib[i + 1][0] - calib[i][0]);

  // Защита от выхода за пределы
  if (pressure < 0.0f) pressure = 0.0f;
  if (pressure > 9.5f) pressure = 9.5f;

  // Дополнительная фильтрация (сглаживание скачков)
  static float lastPressure = 0.0f;
  float delta = pressure - lastPressure;
  if (abs(delta) > 0.5f) {
    // Резкий скачок - ограничиваем
    pressure = lastPressure + (delta > 0 ? 0.5f : -0.5f);
  }
  lastPressure = pressure;

  return pressure;
}

/* ====================  UI-ХЕЛПЕРЫ  ==================== */

/** Строка «подпись: значение» для служебных экранов. */
static void uiInfoRow(int16_t y, const char *label, const char *value) {
  ui.setTransparent(true);
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN + 2, y, 80, 18, label, UiHAlign::Left, UiVAlign::Middle, false);
  ui.setColors(theme::TEXT, theme::BG);
  ui.box(theme::MARGIN + 84, y, theme::SCREEN_W - 2 * theme::MARGIN - 84, 18, value,
         UiHAlign::Left, UiVAlign::Middle, false);
}

/** Шапка экрана: скруглённая панель с заголовком по центру. */
static void uiHeader(const char *title, uint16_t titleColor) {
  constexpr int16_t HDR_Y = theme::MARGIN;
  constexpr int16_t HDR_H = 26;
  tft.fillRoundRect(theme::MARGIN, HDR_Y, theme::SCREEN_W - 2 * theme::MARGIN, HDR_H, theme::RADIUS, theme::PANEL_ALT);
  ui.setTransparent(true);
  ui.setFont(UiFont::Med);
  ui.setColors(titleColor, theme::PANEL_ALT);
  ui.box(theme::MARGIN, HDR_Y, theme::SCREEN_W - 2 * theme::MARGIN, HDR_H, title,
         UiHAlign::Center, UiVAlign::Middle, false);
}

/** Нижняя строка-подсказка по центру. */
static void uiHintBar(const char *text, uint16_t color) {
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(color, theme::BG);
  ui.box(0, theme::SCREEN_H - 20, theme::SCREEN_W, 20, text, UiHAlign::Center, UiVAlign::Middle, true);
}

void displayOTAScreen() {
  if (errorScreenBlocking || ErrorHandler::hasActiveErrors()) {
    return;
  }

  static bool initialized = false;
  static bool lastInstallMode = false;
  static int lastProgress = -1;
  static uint8_t lastDots = 0;
  static uint32_t lastDotUpdate = 0;
  static char lastOtaStatus[32] = "";
  static uint32_t otaWaitStart = 0;

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
  static uint32_t lastReset = 0;
  if (millis() - lastReset < 500 && !force) {
    return;
  }
  lastReset = millis();


  displayDirty = true;
  tft.fillScreen(COLOR_BG);

  lastDisplayedIMU.angleX = 999;
  lastDisplayedIMU.angleY = 999;
  lastDisplayedIMU.temperature = 999;
  lastDisplayedPressure.masterPressure = 999;
  for (int i = 0; i < PAD_COUNT; i++) {
    lastDisplayedPressure.pressure[i] = 999;
  }
  lastDisplayedMode = SystemMode::MOVEMENT;
  lastDisplayedMoving = !lastDisplayedMoving;

// ✅ Сброс переменных симуляции с проверкой
#if ENABLE_SIMULATION
  menuAutoEnterTime = 0;
  menuAutoEnterDone = false;
  errorSimulated = false;
  simulatedError = ErrorHandler::Error::NONE;
#endif

  forceErrorScreenRedraw = true;

  Serial.println("[DISPLAY] Force reset completed");
}

void displayErrorScreen() {
  uint32_t now = millis();

  int totalActiveErrors = ErrorHandler::getActiveErrorCount();
  if (totalActiveErrors == 0) {
    errorScreenBlocking = false;
    forceDisplayReset(true);
    return;
  }

  errorScreenBlocking = true;

  ErrorHandler::Error currentErr = ErrorHandler::getCurrentActiveError();

  static bool firstRender = true;
  static ErrorHandler::Error lastDisplayedError = ErrorHandler::Error::NONE;

  // ✅ ПРИНУДИТЕЛЬНАЯ ПЕРЕРИСОВКА ПОСЛЕ СБРОСА
  if (forceErrorScreenRedraw) {
    forceErrorScreenRedraw = false;
    firstRender = true;
    Serial.println("[DISPLAY] Принудительная перерисовка после сброса");
  }

  bool needRefresh = firstRender || (lastDisplayedError != currentErr);

  if (needRefresh) {
    firstRender = false;
    lastDisplayedError = currentErr;

    Serial.printf("[DISPLAY] Перерисовка экрана ошибки. Код: %d\n", (int)currentErr);

    tft.fillScreen(COLOR_ERROR);
    tft.drawRect(5, 5, SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, theme::TEXT);

    const int16_t iconX = (SCREEN_WIDTH - 45) / 2;
    const int16_t iconY = 8;
    drawIconB(iconX, iconY, err_Big, theme::TEXT);

    // Текст ошибки: одна или две центрированные строки
    const char *line1 = nullptr;
    const char *line2 = nullptr;
    bool customText = true;

    switch (currentErr) {
      case ErrorHandler::Error::LOW_PRESSURE:
        line1 = "НИЗКОЕ ДАВЛЕНИЕ";
        line2 = "В МАГИСТРАЛИ";
        break;
      case ErrorHandler::Error::MPU:
        line1 = "ОШИБКА MPU6050";
        break;
      case ErrorHandler::Error::SENSOR:
        line1 = "НЕИСПРАВЕН ДАТЧИК";
        line2 = "ДАВЛЕНИЯ";
        break;
      case ErrorHandler::Error::VALVE:
        line1 = "НЕИСПРАВЕН КЛАПАН";
        line2 = "ПРОВЕРЬТЕ ЦЕПИ";
        break;
      case ErrorHandler::Error::WATCHDOG:
        line1 = "СБОЙ WATCHDOG";
        line2 = "ПЕРЕЗАГРУЗИТЕ";
        break;
      case ErrorHandler::Error::OTA:
        line1 = "ОШИБКА OTA";
        line2 = "ПРОВЕРЬТЕ WiFi";
        break;
      default:
        line1 = ErrorHandler::getErrorMessage(currentErr);
        customText = false;
        break;
    }

    ui.setTransparent(true);
    ui.setColors(theme::TEXT, COLOR_ERROR);
    ui.setFont(customText ? UiFont::SmallB : UiFont::Small);

    const int16_t textY = (line2 != nullptr) ? 60 : 72;
    ui.box(6, textY, SCREEN_WIDTH - 12, 20, line1, UiHAlign::Center, UiVAlign::Middle, true);
    if (line2 != nullptr) {
      ui.box(6, textY + 22, SCREEN_WIDTH - 12, 20, line2, UiHAlign::Center, UiVAlign::Middle, true);
    }

    // Индикатор номера ошибки (если ошибок несколько)
    if (totalActiveErrors > 1) {
      char indicatorStr[12];
      int currentIndex = ErrorHandler::getCurrentDisplayIndex();
      snprintf(indicatorStr, sizeof(indicatorStr), "%d/%d", currentIndex + 1, totalActiveErrors);

      ui.setFont(UiFont::Small);
      ui.setColors(theme::ACCENT, COLOR_ERROR);
      ui.box(SCREEN_WIDTH - 72, 26, 64, 18, indicatorStr, UiHAlign::Right, UiVAlign::Middle, true);
    }

    // Подсказки внизу
    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT, COLOR_ERROR);
    ui.box(theme::MARGIN, 212, 130, 18, "КН3 — СБРОС", UiHAlign::Left, UiVAlign::Middle, true);
    ui.box(theme::SCREEN_W - theme::MARGIN - 180, 212, 180, 18, "УДЕРЖ. КН3+КН4 — МЕНЮ", UiHAlign::Right, UiVAlign::Middle, true);

    Serial.println("[DISPLAY] Экран ошибки нарисован");
  }

  // ✅ МИГАНИЕ ИКОНКИ
  static uint32_t lastBlinkTime = 0;
  static bool blinkState = true;
  const int16_t iconX = (SCREEN_WIDTH - 45) / 2;
  const int16_t iconY = 8;

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

void displayCalibrationScreen() {
  // ✅ ЕСЛИ КАЛИБРОВКА ЗАВЕРШЕНА - ВЫХОДИМ
  if (calibrationCompleted) {
    return;
  }

  if (errorScreenBlocking || ErrorHandler::hasActiveErrors()) {
    return;
  }

  static uint32_t startTime = 0;
  static uint8_t lastProgress = 0;
  static uint32_t lastRemaining = 0;
  static bool initialized = false;
  static uint32_t lastTitleRedraw = 0;

  if (xDisplayMutex != nullptr && xSemaphoreGetMutexHolder(xDisplayMutex) != xTaskGetCurrentTaskHandle()) {
    Serial.println("[ERROR] displayCalibrationScreen called without mutex!");
    return;
  }

  if (currentState != SystemState::CALIBRATING) {
    if (initialized) {
      initialized = false;
      startTime = 0;
      lastProgress = 0;
      lastRemaining = 0;
      lastTitleRedraw = 0;
    }
    return;
  }

  if (!initialized) {
    tft.fillScreen(theme::BG);
    uiHeader("СТАБИЛИЗАЦИЯ", theme::TEXT);
    uiInfoRow(44, "Процесс:", "выравнивание датчиков");
    lastProgress = 0;
    lastRemaining = 0;
    initialized = true;
    lastTitleRedraw = 0;
  }

  uint32_t elapsed = millis() - startTime;
  uint8_t progress = (elapsed * 100) / CALIB_TIME_MS;
  if (progress > 100) progress = 100;
  uint32_t remaining = (CALIB_TIME_MS - elapsed) / 1000;

  uint32_t now = millis();
  if (now - lastTitleRedraw > 500) {
    lastTitleRedraw = now;
    uiHintBar("Не отключайте питание", theme::WARN);
  }

  if (progress != lastProgress) {
    drawProgressBar(theme::MARGIN, 76, theme::SCREEN_W - 2 * theme::MARGIN, 18, progress, theme::OK);

    ui.setTransparent(true);
    ui.setFont(UiFont::Large);
    ui.setColors(theme::TEXT, theme::BG);
    ui.boxf(0, 102, theme::SCREEN_W, 34, UiHAlign::Center, UiVAlign::Middle, true, "%d%%", progress);

    lastProgress = progress;
  }

  if (remaining != lastRemaining) {
    ui.setTransparent(true);
    ui.setFont(UiFont::Small);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.boxf(0, 146, theme::SCREEN_W, 18, UiHAlign::Center, UiVAlign::Middle, true,
            "Осталось: %lu с", static_cast<unsigned long>(remaining));

    lastRemaining = remaining;
  }
}

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

void displayMainScreen() {
  static uint32_t lastRenderTime = 0;
  uint32_t now = millis();

  if (errorScreenBlocking) return;

  static bool firstRun = true;

  if (now - lastRenderTime < 50 && !firstRun) return;
  lastRenderTime = now;

  float localAngleX, localAngleY, localTemp;
  float localPressure[PAD_COUNT];
  float localMasterPressure;
  bool localIsMoving;
  bool localProlongedMovement;

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    localAngleX = angleX;
    localAngleY = angleY;
    localTemp = temperature;
    memcpy(localPressure, pressure, sizeof(pressure));
    localMasterPressure = masterPressure;
    localIsMoving = isMoving;
    localProlongedMovement = prolongedMovementDetected;
  }
  (void)localProlongedMovement;

  static struct {
    float angleX, angleY, temp;
    float pressure[PAD_COUNT];
    float masterPressure;
    bool isMoving;
    SystemMode mode;
  } lastState = { 0 };

  static SystemMode lastSystemMode = SystemMode::MANUAL;

  bool needRedraw = firstRun;

  if (abs(localAngleX - lastState.angleX) > ConfigManager::getRedrawAngleThr() || abs(localAngleY - lastState.angleY) > ConfigManager::getRedrawAngleThr() ||
      abs(localTemp - lastState.temp) > 0.5f || abs(localMasterPressure - lastState.masterPressure) > ConfigManager::getRedrawPressureThr() ||
      localIsMoving != lastState.isMoving || currentSystemMode != lastSystemMode) {
    needRedraw = true;
  }

  if (!needRedraw) {
    for (int i = 0; i < PAD_COUNT; i++) {
      if (abs(localPressure[i] - lastState.pressure[i]) > ConfigManager::getRedrawPressureThr()) {
        needRedraw = true;
        break;
      }
    }
  }

  if (!needRedraw) return;

  lastState.angleX = localAngleX;
  lastState.angleY = localAngleY;
  lastState.temp = localTemp;
  lastState.masterPressure = localMasterPressure;
  lastState.isMoving = localIsMoving;
  memcpy(lastState.pressure, localPressure, sizeof(localPressure));
  lastSystemMode = currentSystemMode;

  /* ------------------------- геометрия экрана ------------------------- */
  constexpr int16_t CARD_X = theme::MARGIN;           // 6
  constexpr int16_t CARD_W = 146;
  constexpr int16_t CARD_H = theme::PAD_ROW_H;        // 30
  constexpr int16_t CARD_DY = CARD_H + 2;
  constexpr int16_t CARD_Y0 = 42;
  constexpr int16_t ANGLES_Y = 22;
  constexpr int16_t MASTER_Y = 172;
  constexpr int16_t MASTER_H = 24;
  constexpr int16_t INFO_Y = 200;
  constexpr int16_t HINT_Y = 220;
  constexpr int16_t TILT_X = 160;
  constexpr int16_t TILT_Y = 42;
  constexpr int16_t TILT_SIZE = 150;

  const float pMin = ConfigManager::getPressureMin();
  const float pMaxAbs = ConfigManager::getPressureMax();
  const float pScale = (pMaxAbs > 0.1f) ? pMaxAbs : 8.0f;

  ui.setTransparent(true);

  /* ------------------ каркас: рисуем один раз при входе ---------------- */
  if (firstRun) {
    tft.fillScreen(theme::BG);
    tft.drawFastHLine(0, theme::STATUS_H, theme::SCREEN_W, theme::BORDER);

    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      int16_t y = CARD_Y0 + i * CARD_DY;
      tft.fillRoundRect(CARD_X, y, CARD_W, CARD_H, theme::RADIUS, theme::PANEL);
      tft.drawRoundRect(CARD_X, y, CARD_W, CARD_H, theme::RADIUS, theme::BORDER);
    }

    tft.fillRoundRect(CARD_X, MASTER_Y, CARD_W, MASTER_H, theme::RADIUS, theme::PANEL_ALT);
    tft.drawRoundRect(CARD_X, MASTER_Y, CARD_W, MASTER_H, theme::RADIUS, theme::BORDER);

    firstRun = false;
  }

  /* --------------------------- статус-строка -------------------------- */
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT, theme::BG);
  ui.boxf(theme::MARGIN, 0, 70, theme::STATUS_H, UiHAlign::Left, UiVAlign::Middle, true,
          "T:%4.1f", localTemp);

  const bool showMoving = (currentSystemMode == SystemMode::MOVEMENT);
  ui.setColors(showMoving ? theme::WARN : theme::OK, theme::BG);
  ui.box(92, 0, 60, theme::STATUS_H, showMoving ? "ДВИЖ" : "СТАТ",
         UiHAlign::Center, UiVAlign::Middle, true);

  const char *modeStr = "РУЧ";
  uint16_t modeColor = theme::ACCENT;
  if (currentSystemMode == SystemMode::AUTO) {
    modeStr = "АВТО";
    modeColor = theme::OK;
  } else if (currentSystemMode == SystemMode::MOVEMENT) {
    modeStr = "ДВИЖ";
    modeColor = theme::WARN;
  }
  ui.setColors(modeColor, theme::BG);
  ui.box(200, 0, theme::SCREEN_W - 200 - theme::MARGIN, theme::STATUS_H, modeStr,
         UiHAlign::Right, UiVAlign::Middle, true);

  /* ------------------------------ углы IMU ----------------------------- */
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT, theme::BG);
  ui.boxf(CARD_X, ANGLES_Y, CARD_W, 18, UiHAlign::Left, UiVAlign::Middle, true,
          "X:%+5.1f  Y:%+5.1f", localAngleX, localAngleY);

  /* ----------------------- давление в подушках ------------------------- */
  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    const int16_t y = CARD_Y0 + i * CARD_DY;
    const float v = localPressure[i];

    ui.setFont(UiFont::SmallB);
    ui.setColors(theme::TEXT_DIM, theme::PANEL);
    ui.box(CARD_X + 6, y + 2, 34, 18, padNames[i], UiHAlign::Left, UiVAlign::Middle, false);

    ui.setFont(UiFont::Med);
    ui.setColors(theme::TEXT, theme::PANEL);
    ui.boxf(CARD_X + CARD_W - 106, y + 1, 100, 22, UiHAlign::Right, UiVAlign::Middle, true,
            "%.1f", v);

    const int16_t barX = CARD_X + 6;
    const int16_t barY = y + CARD_H - 7;
    const int16_t barW = CARD_W - 12;
    tft.fillRect(barX, barY, barW, 4, theme::TRACK);

    const float ratio = constrain(v / pScale, 0.0f, 1.0f);
    const int16_t fillW = static_cast<int16_t>(barW * ratio);
    uint16_t barColor = theme::OK;
    if (v > pMaxAbs) {
      barColor = theme::ERR;
    } else if (v < pMin) {
      barColor = theme::WARN;
    }
    if (fillW > 0) {
      tft.fillRect(barX, barY, fillW, 4, barColor);
    }
  }

  /* ---------------------- магистральное давление ----------------------- */
  ui.setFont(UiFont::SmallB);
  ui.setColors(theme::TEXT_DIM, theme::PANEL_ALT);
  ui.box(CARD_X + 6, MASTER_Y + 3, 34, 18, "МП", UiHAlign::Left, UiVAlign::Middle, true);

  ui.setFont(UiFont::Med);
  ui.setColors(theme::TEXT, theme::PANEL_ALT);
  ui.boxf(CARD_X + CARD_W - 106, MASTER_Y + 3, 100, 18, UiHAlign::Right, UiVAlign::Middle, true,
          "%.1f", localMasterPressure);

  /* ---------------------------- авиагоризонт --------------------------- */
  drawTiltIndicatorAdvanced(TILT_X, TILT_Y, TILT_SIZE, localAngleX, localAngleY);

  /* ------------------------- строка информации ------------------------- */
  char infoMsg[48] = "";

  if (manualControlActive && currentSystemMode != SystemMode::MOVEMENT) {
    snprintf(infoMsg, sizeof(infoMsg), "РУЧ: %s %s", padNames[manualPadIndex],
             manualInflate ? "НАКАЧ" : "СБРОС");
  } else if (currentSystemMode == SystemMode::MOVEMENT && movementEndTime > 0) {
    const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;  // 8.8.0: из меню
    uint32_t remaining = settleMs - (millis() - movementEndTime);
    if (remaining < settleMs) {
      snprintf(infoMsg, sizeof(infoMsg), "ВОЗВРАТ: %lu с", (unsigned long)(remaining / 1000));
    }
  } else if (currentSystemMode == SystemMode::AUTO && !localIsMoving) {
    const uint32_t nextCheck =
        (lastLevelingCheckTime + ConfigManager::getTimeInterval() * 60000UL - millis()) / 1000;
    if (nextCheck < 600) {
      snprintf(infoMsg, sizeof(infoMsg), "ПОП: %d/%d  СЛЕД: %luс",
               levelingAttemptsThisHour, ConfigManager::getNivCount(), (unsigned long)nextCheck);
    } else {
      snprintf(infoMsg, sizeof(infoMsg), "ПОП: %d/%d",
               levelingAttemptsThisHour, ConfigManager::getNivCount());
    }
  } else if (currentSystemMode == SystemMode::MANUAL && !manualControlActive) {
    const uint32_t secLeft =
        (MANUAL_PRESSURE_CHECK_INTERVAL_MS - (millis() % MANUAL_PRESSURE_CHECK_INTERVAL_MS)) / 1000;
    snprintf(infoMsg, sizeof(infoMsg), "ПОДДЕРЖАНИЕ: %lu с", (unsigned long)secLeft);
  }

  ui.setFont(UiFont::Small);
  ui.setColors(infoMsg[0] != '\0' ? theme::ACCENT : theme::BG, theme::BG);
  ui.box(CARD_X, INFO_Y, theme::SCREEN_W - 2 * theme::MARGIN, 16, infoMsg,
         UiHAlign::Left, UiVAlign::Middle, true);

  /* ------------------------------ подсказка ---------------------------- */
  const char *hint = "МЕНЮ: удерж. КН3+КН4";
  if (currentSystemMode == SystemMode::MANUAL) {
    hint = "МЕНЮ: удерж. КН3+КН4 | СТОП: КН4+КН5";
  } else if (currentSystemMode == SystemMode::AUTO) {
    hint = "МЕНЮ: удерж. КН3+КН4 | РУЧН/АВТО: КН5";
  }

  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::WARN, theme::BG);
  ui.box(0, HINT_Y, theme::SCREEN_W, 20, hint, UiHAlign::Center, UiVAlign::Middle, true);
}

void updateTestDisplay() {
  if (errorScreenBlocking || ErrorHandler::hasActiveErrors()) {
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
  ui.clear(theme::MARGIN, 38, theme::SCREEN_W - 2 * theme::MARGIN, 52, theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::SmallB);
  ui.setColors(theme::TEXT, theme::BG);

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

  ui.box(theme::MARGIN, 38, theme::SCREEN_W - 2 * theme::MARGIN, 20, stepTitle,
         UiHAlign::Left, UiVAlign::Middle, false);

  ui.setFont(UiFont::Small);
  ui.setColors(stepNote[0] != '\0' ? theme::WARN : theme::BG, theme::BG);
  ui.box(theme::MARGIN, 60, theme::SCREEN_W - 170, 18, stepNote, UiHAlign::Left, UiVAlign::Middle, true);

  ui.setColors(theme::TEXT, theme::BG);
  ui.boxf(theme::SCREEN_W - theme::MARGIN - 150, 60, 150, 18, UiHAlign::Right, UiVAlign::Middle, true,
          "МП: %.1f бар", currentPressure);

  /* ----------------------------- прогресс ---------------------------- */
  uint32_t elapsed = millis() - testStartTime;
  uint8_t progress = (elapsed * 100) / TEST_TIMEOUT_MS;
  if (progress > 100) progress = 100;

  drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, progress, theme::ACCENT);

  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.boxf(BAR_X, BAR_Y + BAR_H + 4, BAR_W, 18, UiHAlign::Left, UiVAlign::Middle, true,
          "Прогресс: %d%%", progress);

  /* ------------------------- состояние клапана ----------------------- */
  const uint32_t stepElapsed = millis() - testStepStartTime;
  const bool valveOpen = (stepElapsed < TEST_VALVE_OPEN_TIME_MS) &&
                         (currentTestStep != TestStep::PREPARE_WAIT_PRESSURIZE);

  ui.setFont(UiFont::SmallB);
  ui.setColors(valveOpen ? theme::WARN : theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN, 146, theme::SCREEN_W - 2 * theme::MARGIN, 20,
         valveOpen ? "КЛАПАН ОТКРЫТ" : "КЛАПАН ЗАКРЫТ",
         UiHAlign::Left, UiVAlign::Middle, true);
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
                    int16_t raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
                    
                    const int SENSOR_ZERO_MIN = 80;
                    const int SENSOR_ZERO_MAX = 180;
                    
                    bool sensorOk = (raw > 10 && raw < 2040 && raw != 0);
                    bool sensorInRange = (raw >= SENSOR_ZERO_MIN && raw <= SENSOR_ZERO_MAX);
                    
                    if (!sensorOk || !sensorInRange) {
                        shouldClear = false;
                        Serial.printf("[SYSTEM] SENSOR сохранена (датчик неисправен, raw=%d, норма: %d-%d)\n", 
                                      raw, SENSOR_ZERO_MIN, SENSOR_ZERO_MAX);
                    } else {
                        Serial.println("[SYSTEM] SENSOR будет удалена (датчик исправен)");
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
                    float minPressure = ConfigManager::getPressureMin();

                    if (localMaster < minPressure) {
                        shouldClear = false;
                        Serial.printf("[SYSTEM] LOW_PRESSURE сохранена (давление %.2f < %.2f)\n",
                                      localMaster, minPressure);
                    } else {
                        Serial.printf("[SYSTEM] LOW_PRESSURE будет удалена (давление %.2f >= %.2f)\n",
                                      localMaster, minPressure);
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

bool checkPressureLimits() {
#if ENABLE_SIMULATION
  return false;
#endif

  if (!calibrationCompleted) {
    return false;
  }

  if (!firstPressureMeasurementDone) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[CHECK] Ожидание первого замера давления...");
    }
    return false;
  }

  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
    return false;
  }

  float localMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return false;
    localMaster = masterPressure;
  }

    // ✅ ДОБАВЛЯЕМ ПРОВЕРКУ: если давление = 0, значит данные ещё не пришли
    if (localMaster < 0.01f) {
        static uint32_t lastZeroLog = 0;
        if (millis() - lastZeroLog > 5000) {
            lastZeroLog = millis();
            Serial.println("[CHECK] Давление = 0, ожидание данных...");
        }
        return false;
    }

  float minPressure = ConfigManager::getPressureMin();

  // ✅ ДАВЛЕНИЕ НИЖЕ МИНИМУМА - СОЗДАЕМ ОШИБКУ
  if (localMaster < minPressure) {
    // ✅ Если ошибка в pendingClear - отменяем удаление
    if (ErrorHandler::isPendingClear(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::cancelClear(ErrorHandler::Error::LOW_PRESSURE);
      Serial.println("[CHECK] Отменено удаление LOW_PRESSURE (давление снова упало)");
    }

    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::handleError(ErrorHandler::Error::LOW_PRESSURE,
                                "Низкое давление в магистрали");
      Serial.printf("[CHECK] LOW_PRESSURE создана! Давление: %.2f бар (мин: %.2f)\n",
                    localMaster, minPressure);
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::LOW_PRESSURE);
    }
    return true;
  }

  // ✅ ДАВЛЕНИЕ В НОРМЕ - УДАЛЯЕМ ОШИБКУ
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    // ✅ ПРОВЕРЯЕМ, НЕ НАХОДИТСЯ ЛИ УЖЕ ОШИБКА В pendingClear
    if (!ErrorHandler::isPendingClear(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::markErrorCleared(ErrorHandler::Error::LOW_PRESSURE);
      Serial.printf("[CHECK] LOW_PRESSURE будет удалена через 3 сек. Давление: %.2f бар\n",
                    localMaster);
    } else {
      // ✅ ОШИБКА УЖЕ В ОЧЕРЕДИ НА УДАЛЕНИЕ - НЕ ТРОГАЕМ ТАЙМЕР
      static uint32_t lastPendingLog = 0;
      if (millis() - lastPendingLog > 5000) {
        lastPendingLog = millis();
        Serial.printf("[CHECK] LOW_PRESSURE уже в очереди на удаление. Давление: %.2f бар\n",
                      localMaster);
      }
    }
  }

  return false;
}

void startManualOperation(Pad padIdx, bool inflate) {
  if (padIdx >= PAD_COUNT) return;
  if (manualControlActive) return;
  if (currentSystemMode == SystemMode::MOVEMENT) return;

  // ✅ Проверка давления в магистрали
  float currentMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentMaster = masterPressure;
  }


  float minPressure = ConfigManager::getPressureMin();
  if (currentMaster < minPressure) {
    Serial.printf("[MANUAL] Низкое давление в магистрали (%.1f < %.1f), операция запрещена\n",
                  currentMaster, minPressure);
    return;
  }

  float currentPressure;
  float maxPressure = ConfigManager::getPressureMax();

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentPressure = pressure[padIdx];
  }

  if (inflate && currentPressure >= maxPressure - ConfigManager::getPressureDeadband()) {
    Serial.printf("[MANUAL] %s уже на максимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return;
  }
  if (!inflate && currentPressure <= minPressure + ConfigManager::getPressureDeadband()) {
    Serial.printf("[MANUAL] %s уже на минимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return;
  }

  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[padIdx] = currentPressure;
      manualTargetSet[padIdx] = true;
    }
  }

  manualControlActive = true;
  manualPadIndex = padIdx;
  manualInflate = inflate;
  manualStartTime = millis();

  sendValveCommand(padIdx, inflate, UINT32_MAX);

  Logger::logf(Logger::INFO, "MANUAL", "%s %s (давление %.1f)",
               inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ",
               padNames[padIdx], currentPressure);

  //manualTargetSet[padIdx] = false;

  Event event;
  event.type = EventType::MANUAL_OPERATION_START;
  event.timestamp = millis();
  event.data.manualOp.pad = static_cast<uint8_t>(padIdx);
  event.data.manualOp.inflate = inflate;
  EventBus::publish(event);

  setDisplayDirty();
}

void stopManualOperation() {
  if (!manualControlActive) return;

  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[manualPadIndex] = pressure[manualPadIndex];
      manualTargetSet[manualPadIndex] = true;
    }
  }

  sendValveCommand(manualPadIndex, manualInflate, 0);
  manualControlActive = false;

  Event event;
  event.type = EventType::MANUAL_OPERATION_END;
  event.timestamp = millis();
  event.data.manualOp.pad = static_cast<uint8_t>(manualPadIndex);
  event.data.manualOp.inflate = manualInflate;
  EventBus::publish(event);

  Logger::log(Logger::INFO, "MANUAL", "Операция завершена");
  setDisplayDirty();
}

void eventHandlerTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
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

void valveTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  Serial.println("[DEBUG] Valve Task started");

  extern uint8_t taskIndex_Valve;

  ValveCommandMsg cmd;
  ValveCommandMsg activeCmd;
  memset(&cmd, 0, sizeof(cmd));
  memset(&activeCmd, 0, sizeof(activeCmd));
  bool cmdActive = false;
  TickType_t cmdStartTime = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_Valve);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_VALVE);

    if (valveStopRequested) {
      if (cmdActive) {
        if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(100), 3, "valve/stop")) {
          closeAllValves();
          xSemaphoreGive(xValveMutex);
        } else {
          closeAllValves();  // fail-safe
        }
        if (activeCmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(activeCmd.sync.ackQueue, &success, 0);
        }
        cmdActive = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
      while (xQueueReceive(xValveQueue, &cmd, 0) == pdTRUE) {
        if (cmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(cmd.sync.ackQueue, &success, 0);
        }
      }
      closeAllValves();
      valveStopRequested = false;
    }

    if (otaValveLock) {
      if (cmdActive) {
        if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(100), 3, "valve/ota-lock")) {
          closeAllValves();
          xSemaphoreGive(xValveMutex);
        } else {
          closeAllValves();
        }
        cmdActive = false;
        lastCmd.commandActive = false;
        lastCmd.waitingForCompletion = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
      while (xQueueReceive(xValveQueue, &cmd, 0) == pdTRUE) {
        if (cmd.sync.ackQueue != nullptr) {
          bool success = false;
          xQueueSend(cmd.sync.ackQueue, &success, 0);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (cmdActive) {
      TickType_t now = xTaskGetTickCount();
      TickType_t elapsed = now - cmdStartTime;

      if (elapsed >= activeCmd.async.duration ||
          elapsed >= pdMS_TO_TICKS(VALVE_OPERATION_TIMEOUT_MS) ||
          activeCmd.async.duration == 0) {
        MutexGuard guard(xValveMutex);
        if (guard) {
          setValve(activeCmd.async.pad, LOW);
          digitalWrite(PIN_INFL, LOW);
          digitalWrite(PIN_DEFL, LOW);
        }



        if (activeCmd.sync.ackQueue != nullptr) {
          bool success = true;
          xQueueSend(activeCmd.sync.ackQueue, &success, 0);
        }

        cmdActive = false;
        lastCmd.commandActive = false;
        memset(&activeCmd, 0, sizeof(activeCmd));
      }
    }

    if (!cmdActive) {
      if (xQueueReceive(xValveQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {

        if (cmd.async.duration == 0) {
          MutexGuard guard(xValveMutex);
          if (guard) {
            setValve(cmd.async.pad, LOW);
            if (cmd.async.inflate) {
              digitalWrite(PIN_INFL, LOW);
            } else {
              digitalWrite(PIN_DEFL, LOW);
            }
          }

          if (cmd.sync.ackQueue != nullptr && uxQueueSpacesAvailable(cmd.sync.ackQueue) > 0) {
            bool success = true;
            xQueueSend(cmd.sync.ackQueue, &success, 0);
          }

          if (lastCmd.commandActive) {
            lastCmd.commandActive = false;
            lastCmd.waitingForCompletion = false;
          }

          continue;
        }

        if (cmd.async.pad >= PAD_COUNT) {
          Serial.printf("[VALVE] Invalid pad: %d\n", (int)cmd.async.pad);
          if (cmd.sync.ackQueue != nullptr) {
            bool success = false;
            xQueueSend(cmd.sync.ackQueue, &success, 0);
          }
          continue;
        }

        memcpy(&activeCmd, &cmd, sizeof(ValveCommandMsg));

        MutexGuard guard(xValveMutex);
        if (guard) {
          setValve(activeCmd.async.pad, HIGH);
          digitalWrite(activeCmd.async.inflate ? PIN_DEFL : PIN_INFL, LOW);
          digitalWrite(activeCmd.async.inflate ? PIN_INFL : PIN_DEFL, HIGH);

          cmdActive = true;
          cmdStartTime = xTaskGetTickCount();

          Serial.printf("[VALVE] Started: pad=%d, inflate=%d, duration=%d ticks\n",
                        (int)activeCmd.async.pad,
                        activeCmd.async.inflate,
                        (int)activeCmd.async.duration);
        } else {
          Serial.println("[VALVE] Failed to get mutex!");
          if (activeCmd.sync.ackQueue != nullptr) {
            bool success = false;
            xQueueSend(activeCmd.sync.ackQueue, &success, 0);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void controlTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Control;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  IMUData imu = { 0 };
  PressureData press = { 0 };

  // ========== ДОБАВЛЕНО: ПЕРЕМЕННЫЕ ДЛЯ ADS1015 ==========
  static uint32_t lastAdsCheck = 0;
  static bool adsErrorReported = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Control);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CONTROL);

    uint32_t currentTime = millis();

    // ========== ✅ ДОБАВЛЕНО: СБРОС lastCmd ПРИ ОШИБКЕ SENSOR ==========
    if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      if (lastCmd.waitingForCompletion || lastCmd.commandActive) {
        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        Serial.println("[CONTROL] lastCmd сброшен из-за ошибки SENSOR");
      }
    }


// ========== ДОБАВЛЕНО: ПРОВЕРКА ADS1015 КАЖДЫЕ 10 СЕКУНД ==========
#if !ENABLE_SIMULATION

    // В controlTask(), в цикле:
    static uint32_t lastHourReset = 0;
    uint32_t now = millis();

    // Сброс счетчика каждый час
    if (now - lastHourReset >= 3600000) {
      levelingAttemptsThisHour = 0;
      lastHourReset = now;
      Serial.println("[AUTO] Сброс счетчика попыток (прошел час)");
    }

    if (currentTime - lastAdsCheck > 100000) {
      lastAdsCheck = currentTime;

      // Проверяем, доступен ли ADS1015
      int16_t testRead = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);

      if (testRead == 0) {
        // Ошибка чтения - ADS1015 не отвечает
        if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) && !adsErrorReported) {
          adsErrorReported = true;
          ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                    "ADS1015 не отвечает");
          Serial.println("[ADS1015] ❌ Ошибка чтения! Проверьте подключение.");
        }
      } else {
        // ADS1015 отвечает
        if (adsErrorReported) {
          adsErrorReported = false;
          // Если была ошибка - отмечаем, что она устранена
          if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
            // Проверяем, что датчик действительно исправен (не обрыв и не КЗ)
            if (testRead > 10 && testRead < 2040) {
              ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
              Serial.println("[ADS1015] ✅ Датчик восстановлен");
            }
          }
        }
      }
    }

#endif
    if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
      if (xQueueReceive(xIMUQueue, &imu, pdMS_TO_TICKS(10)) == pdTRUE) {
        MutexGuard guard(xStateMutex);
        if (guard) {
          // 8.8.0: программный нуль углов из настроек
          angleX = imu.angleX - ConfigManager::getZeroAngleX();
          angleY = imu.angleY - ConfigManager::getZeroAngleY();
          temperature = imu.temperature;
        }
      }
      if (xQueueReceive(xPressureQueue, &press, pdMS_TO_TICKS(10)) == pdTRUE) {
        MutexGuard guard(xStateMutex);
        if (guard) {
          memcpy(pressure, press.pressure, sizeof(pressure));
          masterPressure = press.masterPressure;
        }
      }
      runValveTestLogic();
      vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
      continue;
    }

    if (xQueueReceive(xIMUQueue, &imu, pdMS_TO_TICKS(10)) == pdTRUE) {
      MutexGuard guard(xStateMutex);
      if (guard) {
        // 8.8.0: программный нуль углов из настроек
        angleX = imu.angleX - ConfigManager::getZeroAngleX();
        angleY = imu.angleY - ConfigManager::getZeroAngleY();
        temperature = imu.temperature;
      }
    }
    if (xQueueReceive(xPressureQueue, &press, pdMS_TO_TICKS(10)) == pdTRUE) {
      MutexGuard guard(xStateMutex);
      if (guard) {
        memcpy(pressure, press.pressure, sizeof(pressure));
        masterPressure = press.masterPressure;
      }
    }



#if !ENABLE_SIMULATION
    if (currentSystemMode == SystemMode::MOVEMENT) {
      if (!mpuOk) {
        if (movementModeActive) {
          movementModeActive = false;
          currentSystemMode = previousMode;
          Serial.println("[CONTROL] MOVEMENT mode disabled (MPU not working)");
        }
      } else if (!ErrorHandler::hasActiveErrors() || ErrorHandler::getCurrentActiveError() != ErrorHandler::Error::LOW_PRESSURE) {
        static uint32_t lastMovementCheck = 0;
        if (currentTime - lastMovementCheck >= (uint32_t)ConfigManager::getMovementCheckSec() * 1000UL) {  // 8.8.0: период из меню
          maintainMovementPressure();
          lastMovementCheck = currentTime;
        }
      }
    } else if (currentSystemMode == SystemMode::AUTO && currentState == SystemState::RUNNING) {
      // ✅ Проверка ошибок перед выравниванием
      if (!ErrorHandler::hasActiveErrors() && mpuOk) {
        autoLevelingController.process();
      } else {
        static uint32_t lastAutoErrorLog = 0;
        if (millis() - lastAutoErrorLog > 10000) {
          lastAutoErrorLog = millis();
          if (ErrorHandler::hasActiveErrors()) {
            Serial.println("[AUTO] Выравнивание приостановлено из-за ошибок");
          }
          if (!mpuOk) {
            Serial.println("[AUTO] Выравнивание приостановлено: MPU не исправен");
          }
        }
      }
      checkAndAdjustMasterPressure();
    } else if (currentSystemMode == SystemMode::MANUAL && currentState == SystemState::RUNNING) {
      checkAndAdjustMasterPressure();
      static uint32_t lastManualMaintainTime = 0;
      if (currentTime - lastManualMaintainTime >= MANUAL_PRESSURE_CHECK_INTERVAL_MS) {
        maintainManualPressure();
        lastManualMaintainTime = currentTime;
      }
    }
#endif
    if (manualControlActive && (millis() - manualStartTime > (uint32_t)ConfigManager::getManualMaxTimeSec() * 1000UL)) {  // 8.8.0: время из меню
      stopManualOperation();
    }

    bool sensorActive = ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR);
    bool mpuActive = ErrorHandler::isErrorActive(ErrorHandler::Error::MPU);


    if (!sensorActive) {
      // ДАТЧИК РАБОТАЕТ - проверяем давление
      checkPressureLimits();
    } else {
      // ДАТЧИК НЕИСПРАВЕН - LOW_PRESSURE НЕ ИМЕЕТ СМЫСЛА
      if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
        // ✅ Ждем 3 секунды перед удалением
        ErrorHandler::markErrorCleared(ErrorHandler::Error::LOW_PRESSURE);
        static uint32_t lastLogTime = 0;
        if (millis() - lastLogTime > 10000) {
          lastLogTime = millis();
          Serial.println("[CONTROL] LOW_PRESSURE будет удалена через 3 сек (датчик неисправен)");
        }
      }
    }
    static uint32_t stabilizeStart = 0;

    if (lastCmd.waitingForCompletion && !lastCmd.commandActive && !ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {

      if (stabilizeStart == 0) {
        stabilizeStart = millis();
      }

      if (stabilizeStart != 0 && (millis() - stabilizeStart) >= 500) {
        bool pressureChanged = false;

        MutexGuard guard(xStateMutex);
        if (guard) {
          float currentPressure = pressure[lastCmd.pad];
          float pressureDelta = abs(currentPressure - lastCmd.pressureBefore);

          if (pressureDelta > 0.3f) {
            pressureChanged = true;
          }
        }

        if (!pressureChanged) {
          valveErrorCounter.consecutiveFailures++;
          valveErrorCounter.lastFailureTime = millis();

          Serial.printf("[VALVE] Сбой #%d/%d при команде %s %s\n",
                        valveErrorCounter.consecutiveFailures,
                        valveErrorCounter.requiredFailures,
                        padNames[lastCmd.pad],
                        lastCmd.inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ");

          if (valveErrorCounter.consecutiveFailures >= valveErrorCounter.requiredFailures && !valveErrorCounter.valveErrorActive) {

            valveErrorCounter.valveErrorActive = true;
            if (!ErrorHandler::isErrorActive(ErrorHandler::Error::VALVE)) {
              ErrorHandler::handleError(ErrorHandler::Error::VALVE,
                                        "Клапанный блок не реагирует на команды");
            } else {
              ErrorHandler::updateErrorTime(ErrorHandler::Error::VALVE);  // ✅
            }
          }
        } else {
          if (valveErrorCounter.consecutiveFailures > 0) {
            Serial.printf("[VALVE] Команда успешна, сброс счётчика (было %d сбоев)\n",
                          valveErrorCounter.consecutiveFailures);
            valveErrorCounter.consecutiveFailures = 0;
          }

          if (valveErrorCounter.valveErrorActive) {
            valveErrorCounter.valveErrorActive = false;
            if (ErrorHandler::isErrorActive(ErrorHandler::Error::VALVE)) {
              ErrorHandler::removeError(ErrorHandler::Error::VALVE);
            }
            Serial.println("[VALVE] Ошибка VALVE удалена (система восстановилась)");
          }
        }

        lastCmd.waitingForCompletion = false;
        stabilizeStart = 0;
      }
    } else {
      stabilizeStart = 0;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

/** Открыть меню удержанием пары КН3+КН4. Безопасно останавливает ручное управление. */
void openMenu() {
  if (menuVisible) return;
  if (manualControlActive) {
    stopManualOperation();
  }
  gem.setMenuPageCurrent(mainPage);
  menuVisible = true;
  displayDirty = true;
  Serial.println("[MENU] Открыто удержанием пары КН3+КН4");
}

void buttonTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Button;
  bool lastState[4] = { false };
  uint32_t pressTime[4] = { 0 };
  bool emergencyProcessed = false;
  uint32_t lastEmergencyTime = 0;

  uint32_t lastButtonCheck = 0;
  const uint32_t BUTTON_CHECK_INTERVAL_MS = 50;

  for (;;) {
    TaskPool::markRun(taskIndex_Button);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_BUTTON);

    uint32_t now = millis();

    if (now - lastButtonCheck < BUTTON_CHECK_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    lastButtonCheck = now;

    // ============================================================
    // 1. ЧТЕНИЕ КНОПОК
    // ============================================================
    button0.tick();
    button1.tick();
    button2.tick();
    button3.tick();
    button4.tick();
    emergencyButton.tick(button3, button4);
    menuCombo.tick(button2, button3);

    // ============================================================
    // 1.05. 8.8.0: ПРОБУЖДЕНИЕ ПОДСВЕТКИ
    //       Первое нажатие только включает подсветку и НЕ выполняет действие
    //       (кнопки сбрасываются, чтобы нажатие не сработало как команда).
    // ============================================================
    if (backlightOff &&
        (button0.pressing() || button1.pressing() || button2.pressing() || button3.pressing() || button4.pressing())) {
      backlightOff = false;
      lastUserActivityMs = millis();
      analogWrite(PIN_TFT_BL, map(constrain(ConfigManager::getContrast(), 1, 100), 1, 100, 0, 255));
      button0.reset();
      button1.reset();
      button2.reset();
      button3.reset();
      button4.reset();
      Serial.println("[DISP] Подсветка включена (нажатие не выполняет действие)");
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (button0.pressing() || button1.pressing() || button2.pressing() || button3.pressing() || button4.pressing()) {
      lastUserActivityMs = millis();  // 8.8.0: активность для таймера подсветки
    }

    // ============================================================
    // 1.1. 8.8.0: СЛУЖЕБНЫЕ ЭКРАНЫ — своя обработка кнопок
    // ============================================================
    if (serviceScreen != ServiceScreen::NONE) {
      if (emergencyButton.pressing()) {
        manualValveCloseAll();
        serviceScreen = ServiceScreen::NONE;
        forceDisplayReset(true);
        displayDirty = true;
        Serial.println("[SERVICE] АВАРИЙНАЯ ОСТАНОВКА (КН4+КН5): экран закрыт, клапаны закрыты");
      } else {
        handleServiceInput();
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 1.1. МЕНЮ ПО УДЕРЖАНИЮ ПАРЫ КН3+КН4 (GPIO15+GPIO17)
    //      Приоритет НИЖЕ аварийной остановки (КН4+КН5): если удерживается
    //      аварийная пара, меню не открывается. Работает из любого режима.
    // ============================================================
    if (!menuVisible && !emergencyButton.pressing() && menuCombo.hold()) {
      openMenu();
    }

    // ============================================================
    // 2. АВАРИЙНАЯ ОСТАНОВКА (кн3 + кн4)
    // ============================================================
    if (emergencyButton.hold()) {
      if (!emergencyProcessed) {
        emergencyStop();

        if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
          currentTestState = TestState::COMPLETED;
          closeAllValves();
          Serial.println("[TEST] Тест принудительно остановлен аварийной кнопкой!");
        }

        emergencyProcessed = true;
        lastEmergencyTime = millis();
        Serial.println("[EMERGENCY] АВАРИЙНАЯ ОСТАНОВКА! (кн.3+кн.4)");
        displayDirty = true;
      }
    } else if (emergencyButton.release()) {
      emergencyProcessed = false;
      forceDisplayReset(true);
    }

    if (wifiSetupActive) {
      if (button0.click() && wifi_scan_count > 0) {
        wifi_scan_selected = wifi_scan_selected == 0
                                 ? wifi_scan_count - 1
                                 : wifi_scan_selected - 1;
        displayDirty = true;
      }
      if (button1.click() && wifi_scan_count > 0) {
        wifi_scan_selected = (wifi_scan_selected + 1) % wifi_scan_count;
        displayDirty = true;
      }
      if (button2.click() && wifi_scan_count > 0) {
        strlcpy(sta_ssid, wifi_scan_ssids[wifi_scan_selected], sizeof(sta_ssid));
        saveWiFiConfig();
        wifiSetupActive = false;
        Serial.printf("[WiFi] Выбрана сеть \"%s\"\n", sta_ssid);
        connectConfiguredWiFi();
        menuVisible = true;
        displayDirty = true;
      }
      if (button3.click()) {
        wifiSetupActive = false;
        menuVisible = true;
        displayDirty = true;
        Serial.println("[WiFi] Настройка отменена");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ============================================================
    // 3. РЕЖИМ ОШИБОК
    // ============================================================
    if (ErrorHandler::hasActiveErrors() && !menuVisible) {
      // КНОПКА 3 - СБРОС ОШИБОК
      if (button2.click()) {
        resetSystemErrors();
        forceDisplayReset(true);
        menuVisible = false;
        errorScreenBlocking = false;
        displayDirty = true;
        Serial.println("[BUTTON] Ошибки сброшены (кнопка 3)");
      }

      // Меню открывается удержанием пары КН3+КН4 (см. п. 1.1 в начале buttonTask)

      // КНОПКА 5 - ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ
      if (button4.click() && currentSystemMode != SystemMode::MOVEMENT) {
        if (currentSystemMode == SystemMode::AUTO) {
          currentSystemMode = SystemMode::MANUAL;
          currentMode = Mode::MANUAL;
          setAllManualTargetsFromCurrent();
          requestPressureMeasurement();
          Logger::log(Logger::INFO, "MODE", "Переключено в РУЧНОЙ режим");
          forceDisplayReset(true);
          setDisplayDirty();
        } else {
          currentSystemMode = SystemMode::AUTO;
          currentMode = Mode::AUTO;
          lastLevelingCheckTime = millis();
          lastLevelingAttemptTime = millis();
          levelingAttemptsThisHour = 0;
          lastHourResetTime = millis();
          Logger::log(Logger::INFO, "MODE", "Переключено в АВТО режим");
          forceDisplayReset(true);
          setDisplayDirty();
        }
        Serial.println("[BUTTON] Переключение режимов (кнопка 5)");
      }

      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 4. РЕЖИМ МЕНЮ
    // ============================================================
    if (menuVisible) {
      static uint32_t lastMenuButtonTime = 0;
      constexpr uint32_t MENU_DEBOUNCE_MS = 50;

      if (millis() - lastMenuButtonTime >= MENU_DEBOUNCE_MS) {
        // Навигация по меню. Отрисовку GEM выполняет сам внутри
        // registerKeyPress(), поэтому вызываем его под xDisplayMutex —
        // иначе кадр из displayTask накладывается на кадр меню.
        if (button0.click()) {  // Кн1 - ВВЕРХ
          {
            MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
            gem.registerKeyPress(GEM_KEY_UP);
          }
          lastMenuButtonTime = millis();
          Serial.println("[MENU] UP");
        }
        if (button1.click()) {  // Кн2 - ВНИЗ
          {
            MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
            gem.registerKeyPress(GEM_KEY_DOWN);
          }
          lastMenuButtonTime = millis();
          Serial.println("[MENU] DOWN");
        }
        if (button2.click()) {  // Кн3 - ВЫБОР/OK
          {
            MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
            gem.registerKeyPress(GEM_KEY_OK);
          }
          lastMenuButtonTime = millis();
          Serial.println("[MENU] SELECT");
        }
        if (button3.click()) {  // Кн4 - НАЗАД/CANCEL
          {
            MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
            gem.registerKeyPress(GEM_KEY_CANCEL);
          }
          lastMenuButtonTime = millis();
          Serial.println("[MENU] BACK");
        }
        if (button4.click()) {  // Кн5 - ВЫХОД с сохранением
          saveMenuSettings();
          menuVisible = false;
          forceDisplayReset(true);
          displayDirty = true;
          lastMenuButtonTime = millis();
          Serial.println("[MENU] Выход с сохранением");
        }
      }

      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ============================================================
    // 5. РЕЖИМ ТЕСТА
    // ============================================================
    if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
      if (button3.click()) {
        MutexGuard guard(xTestMutex, pdMS_TO_TICKS(100));
        if (guard) {
          currentTestState = TestState::COMPLETED;
          closeAllValves();
          Serial.println("[TEST] Принудительная остановка теста");
          displayDirty = true;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ============================================================
    // 6. РЕЖИМ OTA
    // ============================================================
    if (currentState == SystemState::OTA_MODE) {
      if (button3.click()) {  // Кн3 - выход из режима OTA (меню — удержанием КН3+КН4)
        stopOTAMode();
        displayDirty = true;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ============================================================
    // 7. ОСНОВНОЙ РЕЖИМ (RUNNING)
    // ============================================================
    if (currentState == SystemState::RUNNING) {

      // ============================================================
      // 7.1. ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ (кн5)
      // ============================================================
      if (button4.click() && currentSystemMode != SystemMode::MOVEMENT) {
        if (currentSystemMode == SystemMode::AUTO) {
          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              currentSystemMode = SystemMode::MANUAL;
            }
          }
          currentMode = Mode::MANUAL;
          setAllManualTargetsFromCurrent();
          requestPressureMeasurement();

          Logger::log(Logger::INFO, "MODE", "Переключено в РУЧНОЙ режим");
          forceDisplayReset(true);
          setDisplayDirty();
        } else {
          currentSystemMode = SystemMode::AUTO;
          currentMode = Mode::AUTO;
          lastLevelingCheckTime = millis();
          lastLevelingAttemptTime = millis();
          levelingAttemptsThisHour = 0;
          lastHourResetTime = millis();
          Logger::log(Logger::INFO, "MODE", "Переключено в АВТО режим");
          forceDisplayReset(true);
          setDisplayDirty();
        }
      }

      // ============================================================
      // 7.2. РУЧНОЕ УПРАВЛЕНИЕ ПОДУШКАМИ (кн0-кн3)
      // ============================================================
      if (currentSystemMode == SystemMode::MANUAL && !movementModeActive) {
        float localX = 0, localY = 0;
        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            localX = angleX;
            localY = angleY;
          }
        }

        Button *buttons[] = { &button0, &button1, &button2, &button3 };
        const bool menuPairPressed = menuCombo.pressing();

        for (uint8_t i = 0; i < 4; i++) {
          // Пока удерживается пара КН3+КН4 (открытие меню) — подушки не трогаем
          if (menuPairPressed && (i == 2 || i == 3)) {
            continue;
          }
          if (i == 3 && emergencyButton.pressing()) {
            if (manualControlActive && manualPadIndex == i) {
              stopManualOperation();
            }
            continue;
          }

          Button *btn = buttons[i];
          bool held = btn->hold();
          bool pressed = btn->press();

          bool inflateDefault = false;
          switch (i) {
            case PAD_FRONT_LEFT:
              inflateDefault = (localX < -ConfigManager::getTiltThresholdX()) || (localY > ConfigManager::getTiltThresholdY());
              break;
            case PAD_FRONT_RIGHT:
              inflateDefault = (localX > ConfigManager::getTiltThresholdX()) || (localY > ConfigManager::getTiltThresholdY());
              break;
            case PAD_REAR_LEFT:
              inflateDefault = (localX < -ConfigManager::getTiltThresholdX()) || (localY < -ConfigManager::getTiltThresholdY());
              break;
            case PAD_REAR_RIGHT:
              inflateDefault = (localX > ConfigManager::getTiltThresholdX()) || (localY < -ConfigManager::getTiltThresholdY());
              break;
          }

          if (held && !lastState[i]) {
            if (!lastCmd.commandActive) {
              startManualOperation(Pad(i), inflateDefault);
              pressTime[i] = millis();
            }
          } else if (!held && lastState[i]) {
            if (manualControlActive && manualPadIndex == i) {
              stopManualOperation();
            }
          } else if (pressed && !held) {
            startManualOperation(Pad(i), inflateDefault);
          }
          lastState[i] = held;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void imuTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_IMU;
  uint8_t fifo[42];
  uint8_t errCnt = 0;
  uint32_t lastIMU = 0;
  uint32_t lastReinitAttempt = 0;
  bool wasMoving = false;
  uint32_t motionStartTime = 0;
  bool localProlongedMovement = false;

  uint32_t lastMotionCheck = 0;
  const uint32_t MOTION_CHECK_INTERVAL_MS = 100;

  uint32_t lastDebugPrint = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_IMU);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_IMU);
    uint32_t currentTime = millis();

    if (currentTime - lastDebugPrint > 10000) {
      lastDebugPrint = currentTime;
      Serial.printf("[IMU] Stats - Motion:%dms, IMU:%dms, Moving:%d, MPU:%d\n",
                    MOTION_CHECK_INTERVAL_MS, (currentTime - lastIMU),
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

    // ========== ✅ ПРОВЕРКА: ЕСЛИ MPU НЕ РАБОТАЕТ - ПРОПУСКАЕМ ==========
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
      localProlongedMovement = false;
      if (movementModeActive) {
        movementModeActive = false;
        currentSystemMode = previousMode;
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ========== ДЕТЕКЦИЯ ДВИЖЕНИЯ ==========
    if (currentTime - lastMotionCheck >= MOTION_CHECK_INTERVAL_MS) {
      lastMotionCheck = currentTime;

      bool currentlyMoving = false;
      if (mpuOk) {
        currentlyMoving = detectMotionFromIMU();
      }

      if (currentlyMoving) {
        if (!wasMoving) {
          wasMoving = true;
          motionStartTime = currentTime;
          localProlongedMovement = false;

          // ✅ Отправляем событие начала движения
          Event event;
          event.type = EventType::MOVEMENT_DETECTED;
          event.timestamp = millis();
          EventBus::publish(event);

        } else if (!movementModeActive && (currentTime - motionStartTime > (uint32_t)ConfigManager::getMovementDurationSec() * 1000UL)) {  // 8.8.0: из меню
          localProlongedMovement = true;
          previousMode = currentSystemMode;

          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              currentSystemMode = SystemMode::MOVEMENT;
            }
          }

          movementModeActive = true;
          movementPressureLastCheck = currentTime;
          movementEndTime = 0;

          Serial.println("[MOVEMENT] Длительное движение! Переход в режим MOVEMENT");

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
      } else {
        if (wasMoving) {
          wasMoving = false;

          // ✅ Отправляем событие окончания движения
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
              setAllManualTargetsFromCurrent();
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
    }

    // ========== ✅ ЧТЕНИЕ УГЛОВ - ТОЛЬКО ЕСЛИ MPU РАБОТАЕТ ==========
    if (mpuOk) {
      if (currentTime - lastIMU > 100) {
        // ✅ ПРОВЕРЯЕМ, ЧТО FIFO НЕ ПУСТ
        if (mpu.dmpGetCurrentFIFOPacket(fifo)) {
          Quaternion q;
          VectorFloat gravity;
          float ypr[3];
          mpu.dmpGetQuaternion(&q, fifo);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          float ax = degrees(ypr[2]);
          float ay = degrees(ypr[1]);

          if (abs(ax) <= 90 && abs(ay) <= 90) {
            float newX = filterX.filtered(ax);
            float newY = filterY.filtered(ay);
            float newTemp = (mpu.getTemperature() / 340.0f) + 36.53f;

            // ✅ ДОБАВЛЕНА ПРОВЕРКА ДАННЫХ
            if (isnan(newX) || isnan(newY) || isnan(newTemp) || abs(newX) > 90 || abs(newY) > 90 || newTemp < -20 || newTemp > 120) {
              Serial.printf("[IMU] Invalid data: X=%.2f, Y=%.2f, T=%.2f\n",
                            newX, newY, newTemp);
              vTaskDelay(pdMS_TO_TICKS(10));
              continue;
            }

            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                angleX = newX;
                angleY = newY;
                temperature = newTemp;
              }
            }

            Event event;
            event.type = EventType::IMU_UPDATE;
            event.timestamp = millis();
            event.data.imu.angleX = newX;
            event.data.imu.angleY = newY;
            event.data.imu.temperature = newTemp;
            EventBus::publish(event, 0);

            IMUData imuData = { newX, newY, newTemp };
            if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) {
              Serial.println("[ERROR] Failed to send to IMU queue");
            }

            lastIMU = currentTime;
          }
          errCnt = 0;

          if (ErrorHandler::isErrorActive(ErrorHandler::Error::MPU)) {
            ErrorHandler::markErrorCleared(ErrorHandler::Error::MPU);
            Serial.println("[IMU] MPU восстановлен, ошибка будет удалена через 3 сек");
          }

        } else {
          // ✅ ОШИБКА ЧТЕНИЯ FIFO
          if (++errCnt > 10) {
            static uint8_t reinitAttempts = 0;
            if (reinitAttempts < 5) {
              mpuOk = false;
              initializeDMP();
              errCnt = 0;
              reinitAttempts++;
              Serial.printf("[IMU] Reinit attempt %d/5\n", reinitAttempts);
            } else {
              static uint32_t lastMpuErrorUpdate = 0;
              if (currentTime - lastMpuErrorUpdate > 1000) {
                lastMpuErrorUpdate = currentTime;
                if (!ErrorHandler::isErrorActive(ErrorHandler::Error::MPU)) {
                  ErrorHandler::handleError(ErrorHandler::Error::MPU, "MPU failed permanently!");
                } else {
                  ErrorHandler::updateErrorTime(ErrorHandler::Error::MPU);
                }
              }
            }
          }
        }
      }
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
    localProlongedMovement = false;

    if (movementModeActive) {
      movementModeActive = false;
      {
        MutexGuard guard(xStateMutex);
        if (guard) {
          currentSystemMode = SystemMode::MANUAL;
        }
      }
      Serial.println("[TEST] Movement mode disabled in test mode");
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void pressureTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Pressure;
  PressureData pd = { 0 };
  uint8_t curPad = 0;
  TickType_t last = xTaskGetTickCount();
  const uint32_t STABILIZE_MS = 500;
  Event event;

  // ✅ ФЛАГ ПЕРВОГО ЗАМЕРА
  static bool firstMeasurementDone = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Pressure);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_PRESSURE);

    // ============================================================
    // ОПРЕДЕЛЕНИЕ ТАЙМАУТА
    // ============================================================
    uint32_t timeout;
    if (ErrorHandler::hasActiveErrors()) {
      timeout = 60000;
    } else if (currentSystemMode == SystemMode::AUTO) {
      timeout = portMAX_DELAY;
    } else if (currentSystemMode == SystemMode::MANUAL) {
      // ✅ В РУЧНОМ РЕЖИМЕ - 5 СЕКУНД ДЛЯ БЫСТРОГО ПЕРВОГО ЗАМЕРА
      timeout = 5000;
    } else if (currentSystemMode == SystemMode::MOVEMENT) {
      timeout = 120000;
    } else {
      timeout = 300000;
    }

    // ============================================================
    // ОЖИДАНИЕ ПРОБУЖДЕНИЯ
    // ============================================================
    uint32_t dummy;
    if (xQueueReceive(xPressureWakeupQueue, &dummy, pdMS_TO_TICKS(timeout)) == pdTRUE) {
      Serial.println("[PRESS] Wakeup by request");
    } else {
      Serial.println("[PRESS] Periodic measurement");
    }

    // Во время калибровки нуля не трогаем клапаны — иначе искажается zero
    {
      SystemState st = getSystemState();
      if (st == SystemState::BOOT || st == SystemState::CALIBRATING || !calibrationCompleted) {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
    }

#if ENABLE_SIMULATION
    // ============================================================
    // РЕЖИМ СИМУЛЯЦИИ
    // ============================================================
    updateSimulationData();

    for (int i = 0; i < PAD_COUNT; i++) {
      pd.pressure[i] = simPressures[i];
    }
    pd.masterPressure = simMasterPressure;

    if (xQueueSend(xPressureQueue, &pd, pdMS_TO_TICKS(100)) != pdTRUE) pressureQueueDropCount++;

    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        memcpy(pressure, simPressures, sizeof(pressure));
        masterPressure = simMasterPressure;
      }
    }

    event.type = EventType::PRESSURE_UPDATE;
    event.timestamp = millis();
    memcpy(event.data.pressure.pressure, simPressures, sizeof(simPressures));
    event.data.pressure.masterPressure = simMasterPressure;
    EventBus::publish(event, 0);

    if (currentState != SystemState::CALIBRATING) {
      setDisplayDirty();
    }

#else
    // ============================================================
    // РЕАЛЬНЫЙ РЕЖИМ
    // ============================================================

    // --- Измерение подушки ---
    {
      MutexGuard guard(xValveMutex, pdMS_TO_TICKS(500));
      if (guard) {
        setValve(Pad(curPad), HIGH);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(STABILIZE_MS));
    float p = readPressure();
    {
      MutexGuard guard(xValveMutex);
      if (guard) {
        setValve(Pad(curPad), LOW);
      }
    }
    pd.pressure[curPad] = p;

    // --- Измерение магистрали ---
    curPad = (curPad + 1) % PAD_COUNT;
    if (curPad == 0) {
      {
        MutexGuard guard(xValveMutex);
        if (guard) {
          digitalWrite(PIN_INFL, HIGH);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(STABILIZE_MS));
      pd.masterPressure = readPressure();
      {
        MutexGuard guard(xValveMutex);
        if (guard) {
          digitalWrite(PIN_INFL, LOW);
        }
      }

      // ============================================================
      // ✅ ПЕРВЫЙ ЗАМЕР ВЫПОЛНЕН!
      // ============================================================
      if (!firstMeasurementDone) {
        firstMeasurementDone = true;
        firstPressureMeasurementDone = true;
        Serial.printf("[PRESS] Первый замер выполнен! Давление: %.2f бар\n", pd.masterPressure);

        // ✅ ПРОВЕРЯЕМ ДАВЛЕНИЕ СРАЗУ ПОСЛЕ ПЕРВОГО ЗАМЕРА
        // Если давление < minPressure → LOW_PRESSURE
        // Если давление >= minPressure → НЕТ ошибки
        checkPressureLimits();
      }
    }
#endif

    // ============================================================
    // ОТПРАВКА ДАННЫХ В ОЧЕРЕДЬ
    // ============================================================
    if (xQueueSend(xPressureQueue, &pd, pdMS_TO_TICKS(100)) != pdTRUE) {
      Serial.println("[ERROR] Failed to send to pressure queue");
    }

    // ============================================================
    // ОБНОВЛЕНИЕ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ
    // ============================================================
    bool pressureChanged = false;
    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        for (int i = 0; i < PAD_COUNT; i++) {
          if (abs(pressure[i] - pd.pressure[i]) > ConfigManager::getRedrawPressureThr()) {
            pressureChanged = true;
            break;
          }
        }
        if (abs(masterPressure - pd.masterPressure) > ConfigManager::getRedrawPressureThr()) {
          pressureChanged = true;
        }
        memcpy(pressure, pd.pressure, sizeof(pressure));
        masterPressure = pd.masterPressure;
      }
    }

    // ============================================================
    // ПУБЛИКАЦИЯ СОБЫТИЯ
    // ============================================================
    event.type = EventType::PRESSURE_UPDATE;
    event.timestamp = millis();
    memcpy(event.data.pressure.pressure, pd.pressure, sizeof(pd.pressure));
    event.data.pressure.masterPressure = pd.masterPressure;
    EventBus::publish(event, 0);

    // ============================================================
    // ОБНОВЛЕНИЕ ЭКРАНА
    // ============================================================
    if (pressureChanged) {
      if (currentState != SystemState::CALIBRATING) {
        setDisplayDirty();
      }
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
  }
}

void calibrationTask(void *pvParameters) {
    extern uint8_t taskIndex_Calib;

    for (;;) {
        TaskPool::markRun(taskIndex_Calib);
        TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CALIB);

        if (getSystemState() == SystemState::BOOT) {

#if ENABLE_SIMULATION
            Serial.println("[CALIB] SIM: Пропускаем калибровку (режим симуляции)");
            setSystemState(SystemState::RUNNING);
            calibrationCompleted = true;
            displayDirty = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
#else

            // ============================================================
            // ШАГ 1: ИНИЦИАЛИЗАЦИЯ ADS1015
            // ============================================================
            Serial.println("[CALIB] ШАГ 1/3: Инициализация ADS1015...");

            bool adsOk = initializeAds1015();
            if (!adsOk) {
                Serial.println("[CALIB] ❌ ADS1015 НЕ НАЙДЕН! Калибровка невозможна.");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "ADS1015 не найден");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            Serial.printf("[CALIB] ✅ ADS1015 инициализирован, адрес 0x%02X\n",
                          adsDetectedAddress);

            // ============================================================
            // ШАГ 2: ПРОВЕРКА ДАТЧИКА ДАВЛЕНИЯ
            // ============================================================
            Serial.println("[CALIB] ШАГ 2/3: Проверка датчика давления...");

            int16_t raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
            // Повтор при одиночном raw==0 (краткий сбой шины)
            if (raw == 0) {
              vTaskDelay(pdMS_TO_TICKS(50));
              raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
            }
            if (raw == 0) {
              vTaskDelay(pdMS_TO_TICKS(50));
              raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);
            }

            if (raw == 0) {
                Serial.println("[CALIB] ❌ ОШИБКА ЧТЕНИЯ ADS1015! Калибровка невозможна.");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "ADS1015 - ошибка чтения");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (raw < 10) {
                Serial.println("[CALIB] ❌ ОБРЫВ ЦЕПИ! Калибровка невозможна.");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "Датчик давления - обрыв цепи");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (raw >= 2040) {
                Serial.println("[CALIB] ❌ КОРОТКОЕ ЗАМЫКАНИЕ! Калибровка невозможна.");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "Датчик давления - КЗ на питание");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            Serial.printf("[CALIB] ✅ Датчик давления исправен (raw=%d)\n", raw);

            // Если датчик исправен – убираем ошибку SENSOR (если была)
            if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
                ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
                Serial.println("[CALIB] Ошибка SENSOR помечена для удаления");
            }

            // ============================================================
            // ШАГ 3: ПРОВЕРКА MPU (если включен)
            // ============================================================
#if ENABLE_MPU6050
            Serial.println("[CALIB] ШАГ 3/4: Проверка MPU6050...");

            if (!mpu.testConnection()) {
                Serial.println("[CALIB] ❌ MPU6050 НЕ ОТВЕЧАЕТ!");
                ErrorHandler::handleError(ErrorHandler::Error::MPU, "MPU6050 не отвечает");
                mpuOk = false;
            } else {
                mpu.initialize();
                mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
                mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
                mpu.setSleepEnabled(false);

                uint8_t devStatus = mpu.dmpInitialize();
                if (devStatus != 0) {
                    Serial.printf("[CALIB] ❌ DMP ошибка %d\n", devStatus);
                    ErrorHandler::handleError(ErrorHandler::Error::MPU, "MPU6050 DMP ошибка");
                    mpuOk = false;
                } else {
                    mpu.setDMPEnabled(true);
                    mpuOk = true;
                    Serial.println("[CALIB] ✅ MPU6050 инициализирован успешно");
                }
            }
#else
            Serial.println("[CALIB] ШАГ 3/4: MPU6050 отключен (тестовый режим)");
            mpuOk = false;
#endif

            // ============================================================
            // ШАГ 4: КАЛИБРОВКА НУЛЯ ДАТЧИКА ДАВЛЕНИЯ
            // ============================================================
            Serial.println("[CALIB] ШАГ 4/4: Калибровка нуля давления...");

            setSystemState(SystemState::CALIBRATING);
            Serial.println("[CALIB] Entering CALIBRATING state");

            displayDirty = true;

            // ✅ СОЗДАЁМ ЛОКАЛЬНЫЙ ОБЪЕКТ КАЛИБРАТОРА
            ZeroCalibrator localCalibrator;
            bool calibratorStarted = false;
            bool sensorErrorDuringCalib = false;

            uint32_t start = millis();

            while (millis() - start < CALIB_TIME_MS) {
                TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CALIB);

                // Проверяем, не появилась ли ошибка SENSOR во время калибровки
                if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
                    sensorErrorDuringCalib = true;
                    Serial.println("[CALIB] ❌ Ошибка SENSOR во время калибровки!");
                    break;
                }

                if (!calibratorStarted && (millis() - start >= 2000)) {
                    localCalibrator.start();  // ← ИСПОЛЬЗУЕМ ЛОКАЛЬНЫЙ
                    calibratorStarted = true;
                }

                if (calibratorStarted && !localCalibrator.isDone()) {
                    localCalibrator.process();  // ← ИСПОЛЬЗУЕМ ЛОКАЛЬНЫЙ
                }

                static uint32_t lastDirtySet = 0;
                if (millis() - lastDirtySet > 100) {
                    displayDirty = true;
                    lastDirtySet = millis();
                }

                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // ✅ ПОСЛЕ КАЛИБРОВКИ - СОХРАНЯЕМ РЕЗУЛЬТАТ
            if (sensorErrorDuringCalib) {
                Serial.println("[CALIB] ❌ Калибровка прервана из-за ошибки SENSOR!");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (calibratorStarted && !localCalibrator.isDone()) {
                Serial.println("[CALIB] Таймаут калибровки, использую последнее значение");
                if (localCalibrator.getZeroValue() > 0) {
                    g_pressureZeroRaw = localCalibrator.getZeroValue();
                }
            } else if (localCalibrator.isDone()) {
                g_pressureZeroRaw = localCalibrator.getZeroValue();
            }

            Serial.printf("[CALIB] ✅ Калибровка завершена: filtered -= %d\n", g_pressureZeroRaw);
            Logger::log(Logger::INFO, "CALIB", "Прогрев и калибровка завершены");

            // ============================================================
            // ПЕРЕХОД В РЕЖИМ RUNNING
            // ============================================================
            setSystemState(SystemState::RUNNING);
            calibrationCompleted = true;
            calibrationValid = true;
            displayDirty = true;
            forceDisplayReset(true);

            Serial.println("[CALIB] ✅ Система готова к работе!");
            Serial.println("[CALIB] Ожидание первого замера давления...");
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void displayTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Display;
  TickType_t framePeriod = pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL_MS);  // 8.8.0: обновляется в цикле из настроек

  static bool lastHasError = false;
  static uint32_t errorHoldUntil = 0;
  const uint32_t ERROR_HOLD_MS = 3000;

  for (;;) {
    TaskPool::markRun(taskIndex_Display);
    // 8.8.0: интервал кадра из настроек (Меню → Дисплей → Интервал,мс)
    framePeriod = pdMS_TO_TICKS(ConfigManager::getFrameMs());

    // 8.8.0: гашение подсветки по бездействию (0 = никогда, Меню → Дисплей → Гасить,мин)
    {
      const int blMin = ConfigManager::getBacklightOffMin();
      if (blMin > 0 && !backlightOff && (millis() - lastUserActivityMs > (uint32_t)blMin * 60000UL)) {
        backlightOff = true;
        analogWrite(PIN_TFT_BL, 0);
        Serial.printf("[DISP] Подсветка погашена (бездействие %d мин)\n", blMin);
      }
    }
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_DISPLAY);

    // ============================================================
    // ЗАХВАТ МЬЮТЕКСА ДИСПЛЕЯ
    // ============================================================
    if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(500)) == pdTRUE) {

      // ============================================================
      // 1. ОТОБРАЖЕНИЕ МЕНЮ (САМЫЙ ПРИОРИТЕТНЫЙ!)
      // ============================================================
      if (menuVisible) {
        // Рисуем меню ТОЛЬКО при необходимости (открытие/возврат), а не каждый
        // кадр. GEM сам перерисовывает элементы меню внутри registerKeyPress()
        // при обработке нажатий, поэтому постоянный вызов drawMenu() каждые
        // 50 мс приводил к мерцанию экрана.
        if (displayDirty) {
          refreshDynamicMenu();   // живые строки страниц «Информация» и «Обновления»
          gem.drawMenu();
          displayDirty = false;
        }
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

      if (wifiSetupActive || wifiScanInProgress) {
        displayWiFiSetupScreen();
        displayDirty = false;
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

      // ============================================================
      // 1.5. 8.8.0: СЛУЖЕБНЫЕ ЭКРАНЫ (тест клапанов, IMU, диагностика MPU)
      //      Рисуются поверх всего и обновляются каждый кадр (состояние меняется).
      // ============================================================
      if (serviceScreen != ServiceScreen::NONE) {
        drawServiceScreen();
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

      // ============================================================
      // 2. ОБРАБОТКА ОШИБОК
      // ============================================================
      bool hasErrorNow = ErrorHandler::hasActiveErrors();
      uint32_t now = millis();

      if (hasErrorNow) {
        if (!lastHasError) {
          lastHasError = true;
          displayDirty = true;
        }
        errorHoldUntil = now + ERROR_HOLD_MS;
      } else {
        if (lastHasError && now > errorHoldUntil) {
          lastHasError = false;
          displayDirty = true;
          forceDisplayReset(true);
        }
      }

      // ============================================================
      // 3. ОТОБРАЖЕНИЕ ЭКРАНОВ
      // ============================================================
      if (ErrorHandler::hasActiveErrors()) {
        errorScreenBlocking = true;

        if (displayDirty) {
          displayErrorScreen();
          displayDirty = false;
        } else {
          blinkErrorIcon();
        }
      } else {
        errorScreenBlocking = false;

        if (displayDirty) {
          if (currentState == SystemState::CALIBRATING && !calibrationCompleted) {
            displayCalibrationScreen();
          } else if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
            updateTestDisplay();
          } else if (currentState == SystemState::OTA_MODE || otaInProgress) {
            displayOTAScreen();   // прогресс обновления показываем в любом режиме
          } else {
            displayMainScreen();
          }
          displayDirty = false;
        }
      }

      // ============================================================
      // ОСВОБОЖДАЕМ МЬЮТЕКС
      // ============================================================
      xSemaphoreGive(xDisplayMutex);

    } else {
      static uint32_t lastMutexWarn = 0;
      if (millis() - lastMutexWarn > 10000) {
        lastMutexWarn = millis();
        Serial.println("[DISPLAY] Mutex timeout!");
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(framePeriod);
  }
}



void watchdogTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_Watchdog;
  esp_task_wdt_add(NULL);
  for (;;) {
    TaskPool::markRun(taskIndex_Watchdog);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_WATCHDOG);
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void otaTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  extern uint8_t taskIndex_OTA;
  for (;;) {
    TaskPool::markRun(taskIndex_OTA);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_OTA);

    GitHubOtaRequest ghReq = takeGitHubOtaRequest();
    if (ghReq == GitHubOtaRequest::CHECK_AND_INSTALL) {
      Logger::log(Logger::INFO, "GH-OTA", "Проверка GitHub (фоновая задача)");
      if (checkGitHubUpdate(false)) {
        Logger::log(Logger::INFO, "GH-OTA", "Найдено обновление, установка...");
        checkGitHubUpdate(true);
      } else {
        Logger::log(Logger::INFO, "GH-OTA", "Новых обновлений нет или нет интернета");
      }
    } else if (ghReq == GitHubOtaRequest::FETCH_LIST) {
      Logger::log(Logger::INFO, "GH-OTA", "Получение списка релизов");
      fetchGitHubReleaseList();
      displayDirty = true;
    } else if (ghReq == GitHubOtaRequest::INSTALL_INDEX) {
      const int8_t idx = takeGitHubOtaIndex();
      Logger::logf(Logger::INFO, "GH-OTA", "Установка релиза #%d", static_cast<int>(idx));
      installGitHubReleaseIndex(idx);
      displayDirty = true;
    }

    if (getSystemState() == SystemState::OTA_MODE) handleOTA();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void errorRecoveryTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ⚠️ CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ⚠️ Stack low: %d bytes free\n",
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
          cleared = mpu.testConnection();
          if (cleared) {
            Serial.println("[RECOVERY] MPU6050 подключился!");
            initializeDMP();
          }
          break;

        case ErrorHandler::Error::SENSOR:
          {
#if !ENABLE_SIMULATION
            // ✅ ИСПРАВЛЕНО: readADC_SingleEnded возвращает 0 при ошибке
            int16_t raw = ads.readADC_SingleEnded(ADS1015_PRESSURE_CHANNEL);

            // ✅ НОРМАЛЬНЫЙ ДИАПАЗОН ДЛЯ ИСПРАВНОГО ДАТЧИКА ПРИ 0 ДАВЛЕНИИ
            const int SENSOR_ZERO_MIN = 80;
            const int SENSOR_ZERO_MAX = 180;

            // ✅ ПРОВЕРЯЕМ ТОЛЬКО ЕСЛИ КАЛИБРОВКА ЗАВЕРШЕНА
            if (calibrationCompleted) {
              bool sensorOk = (raw > 10 && raw < 2040 && raw != 0);
              bool sensorInRange = (raw >= SENSOR_ZERO_MIN && raw <= SENSOR_ZERO_MAX);

              if (sensorOk && sensorInRange) {
                goodReads++;
                if (goodReads >= 3) {
                  cleared = true;
                  goodReads = 0;
                  Serial.printf("[RECOVERY] Датчик давления восстановлен! raw=%d\n", raw);
                }
              } else {
                goodReads = 0;
                cleared = false;
                if (raw != 0) {
                  Serial.printf("[RECOVERY] Датчик все еще неисправен! raw=%d (норма: %d-%d)\n",
                                raw, SENSOR_ZERO_MIN, SENSOR_ZERO_MAX);
                }
              }
            } else {
              // ✅ КАЛИБРОВКА НЕ ЗАВЕРШЕНА - НЕ УДАЛЯЕМ ОШИБКУ
              cleared = false;
              Serial.println("[RECOVERY] SENSOR сохранена (калибровка не завершена)");
            }
#else
            cleared = true;
            Serial.println("[RECOVERY] SIM: Датчик давления исправен");
#endif
            break;
          }

        case ErrorHandler::Error::VALVE:
          {
            static uint32_t valveErrorTime = 0;
            if (valveErrorTime == 0) {
              valveErrorTime = millis();
            }
            if (millis() - valveErrorTime > 10000) {
              cleared = true;
              Serial.println("[RECOVERY] VALVE ошибка устранена (таймаут)");
              valveErrorTime = 0;
            }
            break;
          }

        case ErrorHandler::Error::WATCHDOG:
          {
            static uint32_t watchdogTime = 0;
            if (watchdogTime == 0) {
              watchdogTime = millis();
            }
            if (millis() - watchdogTime > 30000) {
              cleared = true;
              Serial.println("[RECOVERY] WATCHDOG ошибка устранена (система стабильна)");
              watchdogTime = 0;
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
              WiFi.mode(WIFI_AP);
              WiFi.softAPConfig(local_ip, gateway, subnet);
              WiFi.softAP(wifi_ssid, wifi_password);
              cleared = true;
              Serial.println("[RECOVERY] OTA ошибка устранена (WiFi перезапущен)");
              otaErrorTime = 0;
            }
            break;
          }

        default:
          cleared = false;
          break;
      }

      // ✅ ЕСЛИ ОШИБКА УСТРАНЕНА - ПОМЕЧАЕМ НА УДАЛЕНИЕ
      if (cleared) {
        ErrorHandler::markErrorCleared(err);
        anyCleared = true;
      } else {
        // ✅ ЕСЛИ НЕ УДАЛЯЕМ - ОТМЕНЯЕМ PENDING CLEAR (если была установлена)
        if (ErrorHandler::isPendingClear(err)) {
          ErrorHandler::cancelClear(err);
          Serial.printf("[RECOVERY] Отменено удаление ошибки %d (условия не выполнены)\n", (int)err);
        }
      }
    }

    // ✅ ЕСЛИ ВСЕ ОШИБКИ УСТРАНЕНЫ - ВОССТАНАВЛИВАЕМ СИСТЕМУ
    if (anyCleared && !ErrorHandler::hasActiveErrors()) {
      Serial.println("[RECOVERY] Все ошибки устранены, восстанавливаем систему");
      vTaskDelay(pdMS_TO_TICKS(500));

      // ✅ ВОССТАНАВЛИВАЕМ РЕЖИМ
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

bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Нет интернет-соединения");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, sha256Url)) return false;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  String expected = code == HTTP_CODE_OK ? http.getString() : String();
  http.end();
  expected.trim();
  if (expected.length() < 64) {
    Serial.println("[GH-OTA] SHA-256 asset отсутствует или некорректен");
    return false;
  }
  expected = expected.substring(0, 64);
  expected.toLowerCase();

  if (!http.begin(client, firmwareUrl)) return false;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[GH-OTA] Ошибка загрузки: HTTP %d\n", code);
    http.end();
    return false;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0 || !Update.begin(static_cast<size_t>(contentLength))) {
    Serial.println("[GH-OTA] Не удалось подготовить раздел прошивки");
    http.end();
    return false;
  }

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);
  WiFiClient *stream = http.getStreamPtr();
  uint8_t buffer[2048];
  int remaining = contentLength;
  while (remaining > 0) {
    size_t available = stream->available();
    if (available == 0) {
      delay(1);
      continue;
    }
    size_t readSize = min(static_cast<size_t>(remaining), min(available, sizeof(buffer)));
    int read = stream->readBytes(buffer, readSize);
    if (read <= 0 || Update.write(buffer, read) != static_cast<size_t>(read)) {
      Update.abort();
      mbedtls_sha256_free(&sha);
      http.end();
      return false;
    }
    mbedtls_sha256_update(&sha, buffer, read);
    remaining -= read;
    otaProgress = ((contentLength - remaining) * 100) / contentLength;
  }
  http.end();

  unsigned char digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  char actual[65];
  for (uint8_t i = 0; i < sizeof(digest); i++) {
    snprintf(actual + i * 2, 3, "%02x", digest[i]);
  }
  actual[64] = '\0';
  if (strcmp(actual, expected.c_str()) != 0) {
    Serial.printf("[GH-OTA] SHA-256 mismatch: %s\n", releaseTag);
    Update.abort();
    return false;
  }
  if (!Update.end(true)) {
    Serial.println("[GH-OTA] Ошибка завершения Update");
    return false;
  }
  return true;
}

bool checkGitHubUpdate(bool install) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Подключите устройство к Wi-Fi с интернетом");
    return false;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  if (!http.begin(client, GITHUB_API_URL)) return false;
  http.addHeader("User-Agent", "kamaz-leveler");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[GH-OTA] GitHub API HTTP %d\n", code);
    http.end();
    return false;
  }
  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, http.getString());
  http.end();
  if (error) return false;

  const char *tag = doc["tag_name"] | "";
  const char *firmwareUrl = nullptr;
  const char *sha256Url = nullptr;
  for (JsonObject asset : doc["assets"].as<JsonArray>()) {
    const char *name = asset["name"] | "";
    if (strcmp(name, GITHUB_ASSET_NAME) == 0) firmwareUrl = asset["browser_download_url"];
    if (strcmp(name, GITHUB_SHA256_ASSET_NAME) == 0) sha256Url = asset["browser_download_url"];
  }
  if (!tag[0] || !firmwareUrl || !sha256Url) {
    Serial.println("[GH-OTA] Release не содержит bin и sha256 assets");
    return false;
  }
  Serial.printf("[GH-OTA] Последний release: %s (локально %s)\n", tag, VERSION);
  if (!install) {
    bool newer = isRemoteSemVerNewer(VERSION, tag);
    Serial.printf("[GH-OTA] remote newer: %s\n", newer ? "yes" : "no");
    return newer;
  }

  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(firmwareUrl, sha256Url, tag)) {
    otaValveLock = false;
    return false;
  }
  ESP.restart();
  return true;
}

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

void requestPressureMeasurement() {
  if (xPressureWakeupQueue == nullptr) {
    Serial.println("[PRESS] Wakeup queue is null!");
    return;
  }

  static uint32_t lastRequestTime = 0;
  uint32_t now = millis();

  if (now - lastRequestTime < 1000) {
    static uint32_t throttleWarnTime = 0;
    if (now - throttleWarnTime > 30000) {
      throttleWarnTime = now;
      Serial.println("[PRESS] Request throttled (too frequent)");
    }
    return;
  }

  uint32_t dummy = 1;
  BaseType_t result = xQueueSend(xPressureWakeupQueue, &dummy, pdMS_TO_TICKS(10));

  if (result == pdTRUE) {
    lastRequestTime = now;
    Serial.println("[PRESS] Measurement requested");
  } else {
    Serial.println("[PRESS] Failed to request measurement - queue full");
  }
}


static void refreshInfoPage();   // определена ниже

/** Живые строки страницы «Обновления». */
static void refreshOtaPage() {
  static char sCur[48];
  static char sStatus[72];
  snprintf(sCur, sizeof(sCur), "Версия: %s", VERSION);
  snprintf(sStatus, sizeof(sStatus), "Состояние: %s", otaListStatus);
  itemOtaCurrent.setTitle(sCur);
  itemOtaStatus.setTitle(sStatus);
  // «Выход из режима OTA» показываем только когда режим активен
  if (currentState == SystemState::OTA_MODE) {
    itemOtaExit.show();
  } else {
    itemOtaExit.hide();
  }
}

/** Заголовки пунктов списка прошивок. */
static void refreshOtaListPage() {
  for (uint8_t i = 0; i < OTA_LIST_MAX; i++) {
    if (i < otaReleaseCount) {
      const OtaRelease &r = otaReleases[i];
      snprintf(otaItemTitle[i], sizeof(otaItemTitle[i]), "%s · %s · %luК",
               r.tag, r.date, static_cast<unsigned long>(r.size / 1024));
      otaRelItems[i]->setTitle(otaItemTitle[i]);
    } else {
      otaRelItems[i]->setTitle("— нет данных —");
    }
  }
}

/** Строки карточки выбранного релиза. */
static void refreshOtaCard() {
  static char sTag[30];
  static char sInfo[34];
  static char sStatus[72];
  static char sSha[74];

  if (otaSelectedIndex < 0 || otaSelectedIndex >= static_cast<int8_t>(otaReleaseCount)) {
    itemCardTag.setTitle("Релиз: —");
    itemCardInfo.setTitle("данных нет");
    itemCardStatus.setTitle("обновите список");
    itemCardSha.setTitle("SHA-256: не проверен");
    return;
  }

  const OtaRelease &r = otaReleases[otaSelectedIndex];
  snprintf(sTag, sizeof(sTag), "Релиз: %s", r.tag);
  snprintf(sInfo, sizeof(sInfo), "%s · %lu КБ", r.date[0] ? r.date : "дата ?",
           static_cast<unsigned long>(r.size / 1024));

  const bool newer = isRemoteSemVerNewer(VERSION, r.tag);
  const bool installed = (strstr(VERSION, r.tag + 1) != nullptr) || (strstr(VERSION, r.tag) != nullptr);
  snprintf(sStatus, sizeof(sStatus), "Статус: %s",
           installed ? "установлена" : (newer ? "НОВЕЕ установленной" : "не новее"));

  if (r.sha256[0] != '\0') {
    snprintf(sSha, sizeof(sSha), "SHA: %.16s…%.8s", r.sha256, r.sha256 + 56);
  } else {
    strlcpy(sSha, "SHA-256: не проверен", sizeof(sSha));
  }

  itemCardTag.setTitle(sTag);
  itemCardInfo.setTitle(sInfo);
  itemCardStatus.setTitle(sStatus);
  itemCardSha.setTitle(sSha);
}

/** 8.8.0: живые строки страницы «Настройки» — просмотр всех значений из config.txt. */
static void refreshSettingsView() {
  static char b1[52], b2[52], b3[52], b4[52], b5[52], b6[52], b7[52];
  snprintf(b1, sizeof(b1), "Давл %.1f-%.1f зона %.2f", ConfigManager::getPressureMin(),
           ConfigManager::getPressureMax(), ConfigManager::getPressureDeadband());
  snprintf(b2, sizeof(b2), "Наклон %.2f/%.2f ярк %d", ConfigManager::getTiltThresholdX(),
           ConfigManager::getTiltThresholdY(), ConfigManager::getContrast());
  snprintf(b3, sizeof(b3), "Клап %d/%dс макс %dс", ConfigManager::getReleaseDelay(),
           ConfigManager::getInflateDelay(), ConfigManager::getManualMaxTimeSec());
  snprintf(b4, sizeof(b4), "Авто поп%d инт%d %.2f/%.2f/%.2f", ConfigManager::getNivCount(),
           ConfigManager::getTimeInterval(), ConfigManager::getCoarseZoneRatio(),
           ConfigManager::getFineZoneRatio(), ConfigManager::getWorseningRatio());
  snprintf(b5, sizeof(b5), "Движ %.1f/%.1f %d/%d/%dс %.2f", ConfigManager::getMovementPressureFront(),
           ConfigManager::getMovementPressureRear(), ConfigManager::getMovementDurationSec(),
           ConfigManager::getMovementSettleSec(), ConfigManager::getMovementCheckSec(),
           ConfigManager::getMovementTolerance());
  snprintf(b6, sizeof(b6), "Дисп подсв %d кадр %d %.2f/%.2f", ConfigManager::getBacklightOffMin(),
           ConfigManager::getFrameMs(), ConfigManager::getRedrawAngleThr(),
           ConfigManager::getRedrawPressureThr());
  snprintf(b7, sizeof(b7), "IMU порог %d нуль %.2f/%.2f", ConfigManager::getImuMotionDet(),
           ConfigManager::getZeroAngleX(), ConfigManager::getZeroAngleY());
  itemSet1.setTitle(b1);
  itemSet2.setTitle(b2);
  itemSet3.setTitle(b3);
  itemSet4.setTitle(b4);
  itemSet5.setTitle(b5);
  itemSet6.setTitle(b6);
  itemSet7.setTitle(b7);
}

/** Живые строки динамических страниц меню — вызывается перед отрисовкой. */
static void refreshDynamicMenu() {
  GEMPage *page = gem.getCurrentMenuPage();
  if (page == &infoPage) {
    refreshInfoPage();
  } else if (page == &otaPage) {
    refreshOtaPage();
  } else if (page == &otaListPage) {
    refreshOtaListPage();
  } else if (page == &otaCardPage) {
    refreshOtaCard();
  } else if (page == &settingsViewPage) {
    refreshSettingsView();  // 8.8.0: просмотр всех значений конфига
  }
}

/** Живые строки страницы «Информация» — вызывается перед отрисовкой меню. */
static void refreshInfoPage() {
  if (gem.getCurrentMenuPage() != &infoPage) return;

  static char sVer[48];
  static char sMode[28];
  static char sMpu[28];
  static char sMaster[28];
  static char sWifi[52];
  static char sSys[52];
  static char sErr[28];

  float localMaster = 0.0f;
  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(20));
    if (guard) localMaster = masterPressure;
  }

  const char *modeStr = (currentSystemMode == SystemMode::AUTO)       ? "АВТО"
                        : (currentSystemMode == SystemMode::MOVEMENT) ? "ДВИЖ"
                                                                     : "РУЧ";

  snprintf(sVer, sizeof(sVer), "Версия: %s", VERSION);
  snprintf(sMode, sizeof(sMode), "Режим: %s", modeStr);
  snprintf(sMpu, sizeof(sMpu), "MPU: %s", mpuOk ? "OK" : "ОШИБКА");
  snprintf(sMaster, sizeof(sMaster), "МП: %.1f бар", localMaster);

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(sWifi, sizeof(sWifi), "Wi-Fi: %s %d dBm", sta_ssid, static_cast<int>(WiFi.RSSI()));
  } else if (sta_ssid[0] != '\0') {
    snprintf(sWifi, sizeof(sWifi), "Wi-Fi: нет связи (%s)", sta_ssid);
  } else {
    snprintf(sWifi, sizeof(sWifi), "Wi-Fi: AP %s", wifi_ssid);
  }

  snprintf(sSys, sizeof(sSys), "Аптайм: %lu ч  RAM: %lu КБ",
           static_cast<unsigned long>(uptimeHours),
           static_cast<unsigned long>(ESP.getFreeHeap() / 1024));
  snprintf(sErr, sizeof(sErr), "Ошибки: %d", ErrorHandler::getActiveErrorCount());

  itemInfoVersion.setTitle(sVer);
  itemInfoMode.setTitle(sMode);
  itemInfoMpu.setTitle(sMpu);
  itemInfoMaster.setTitle(sMaster);
  itemInfoWiFi.setTitle(sWifi);
  itemInfoSystem.setTitle(sSys);
  itemInfoErrors.setTitle(sErr);
}

/** Подсказка по кнопкам меню — рисуется поверх GEM в правой части шапки. */
static void menuButtonsHint() {
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::SCREEN_W - 170, 0, 170, 14, "КН3-ОК КН4-НАЗ КН5-СОХР",
         UiHAlign::Right, UiVAlign::Middle, true);
}

/** Кн4 на главной странице меню: сохранить настройки и выйти (как Кн5). */
void menuExitAction() {
  saveMenuSettings();
  menuVisible = false;
  forceDisplayReset(true);
  displayDirty = true;
  Serial.println("[MENU] Выход с сохранением (Кн4 на главной странице)");
}

/**
 * 8.8.0: обработчик изменения пункта меню со спиннером (две перегрузки вместо шаблона —
 * шаблонное определение функции первой строкой ломает автогенерацию прототипов Arduino).
 * Копирует значение из связанной переменной GEM в зеркало edit* и помечает настройки изменёнными.
 */
static void menuSpinChanged(GEMCallbackData data, int &dst) {
  void *ptr = data.pMenuItem->getLinkedVariablePointer();
  if (!ptr) return;
  dst = *(int *)ptr;
  settingsChanged = true;
  displayDirty = true;
}

static void menuSpinChanged(GEMCallbackData data, float &dst) {
  void *ptr = data.pMenuItem->getLinkedVariablePointer();
  if (!ptr) return;
  dst = *(float *)ptr;
  settingsChanged = true;
  displayDirty = true;
}

void initGEM() {
    // Настройка внешнего вида
    GEMAppearance appearance;
    appearance.menuPointerType = GEM_POINTER_ROW;
    appearance.menuItemsPerScreen = GEM_ITEMS_COUNT_AUTO;
    appearance.menuItemHeight = ROW_HEIGHT;
    appearance.menuPageScreenTopOffset = MENU_TOP_OFFSET;
    appearance.menuValuesLeftOffset = MENU_VALUES_LEFT_OFFSET;

    gem.setAppearance(appearance);
    gem.setBackgroundColor(MENU_BG_COLOR);
    gem.setForegroundColor(MENU_TEXT_COLOR);
    gem.setFontBig(&CourierCyr9pt8b);
    gem.setFontSmall(&CourierCyr7pt8b);
    gem.setDrawMenuCallback(menuButtonsHint);   // подсказка по кнопкам в шапке меню

    // --- Страница "Система" (OTA/обновления вынесены в раздел «Обновления») ---
    static GEMItem itemReset("Сброс ошибок", []() { resetSystemErrors(); });
    static GEMItem itemWiFi("Настроить WiFi", []() { startWiFiSetup(); });
    systemPage.addMenuItem(itemReset);
    systemPage.addMenuItem(itemWiFi);

    // --- Страница "Обновления" ---
    otaPage.addMenuItem(itemOtaCurrent);
    otaPage.addMenuItem(itemOtaStatus);
    otaPage.addMenuItem(itemOtaFetch);
    otaPage.addMenuItem(itemOtaList);
    otaPage.addMenuItem(itemOtaInstallLast);
    otaPage.addMenuItem(itemOtaArduino);
    otaPage.addMenuItem(itemOtaExit);

    otaListPage.addMenuItem(itemOtaRel0);
    otaListPage.addMenuItem(itemOtaRel1);
    otaListPage.addMenuItem(itemOtaRel2);
    otaListPage.addMenuItem(itemOtaRel3);
    otaListPage.addMenuItem(itemOtaRel4);
    otaListPage.addMenuItem(itemOtaListRefresh);

    otaCardPage.addMenuItem(itemCardTag);
    otaCardPage.addMenuItem(itemCardInfo);
    otaCardPage.addMenuItem(itemCardStatus);
    otaCardPage.addMenuItem(itemCardSha);
    otaCardPage.addMenuItem(itemCardInstall);
    otaCardPage.addMenuItem(itemCardCheckSha);

    // --- Страница "Тестирование" ---
    static GEMItem itemTest("Запустить тест", []() {
        if (currentSystemMode == SystemMode::MANUAL && !ErrorHandler::hasActiveErrors()) {
            startValveTest();
            menuVisible = false;
        }
    });
    testPage.addMenuItem(itemTest);

    // 8.8.0: ручной тест клапанов (6 выходов, клапан открыт, пока удерживается КН3)
    static GEMItem itemValveTestManual("Тест клапанов (ручной)", []() { openValveTestScreen(); });
    testPage.addMenuItem(itemValveTestManual);

    // --- Страница "Клапаны" ---
    static GEMItem itemReleaseDelay("Вр.СБРОС", editReleaseDelay, spinnerRelease, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editReleaseDelay = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemInflateDelay("Вр.НАКАЧ", editInflateDelay, spinnerInflate, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editInflateDelay = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    valvePage.addMenuItem(itemReleaseDelay);
    valvePage.addMenuItem(itemInflateDelay);

    // 8.8.0: максимальное время ручной операции и зона нечувствительности по давлению
    static GEMItem itemManualMaxTime("Макс.время,с", editManualMaxTime, spinnerManualMaxTime, [](GEMCallbackData d) { menuSpinChanged(d, editManualMaxTime); });
    static GEMItem itemDeadband("Зона нечувств.", editDeadband, spinnerDeadband, [](GEMCallbackData d) { menuSpinChanged(d, editDeadband); });
    valvePage.addMenuItem(itemManualMaxTime);
    valvePage.addMenuItem(itemDeadband);

    // --- Страница "Наклон" ---
    static GEMItem itemTiltX("Поперечный", editTiltX, spinnerTiltX, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTiltX = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemTiltY("Продольный", editTiltY, spinnerTiltY, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTiltY = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    tiltPage.addMenuItem(itemTiltX);
    tiltPage.addMenuItem(itemTiltY);

    // --- Страница "Давление" ---
    static GEMItem itemPressureMin("P.МИНИМУМ", editPressureMin, spinnerPressureMin, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editPressureMin = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemPressureMax("P.МАКСИМУМ", editPressureMax, spinnerPressureMax, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editPressureMax = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    pressurePage.addMenuItem(itemPressureMin);
    pressurePage.addMenuItem(itemPressureMax);

    // 8.8.0: период проверки магистрали
    static GEMItem itemMasterCheck("Проверка МП,с", editMasterCheck, spinnerMasterCheck, [](GEMCallbackData d) { menuSpinChanged(d, editMasterCheck); });
    pressurePage.addMenuItem(itemMasterCheck);

    // --- Страница "Авторежим" ---
    static GEMItem itemNivCount("Попыток в час", editNivCount, spinnerNivCount, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editNivCount = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemTimeInterval("Интервал", editTimeInterval, spinnerTimeInterval, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTimeInterval = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    autoPage.addMenuItem(itemNivCount);
    autoPage.addMenuItem(itemTimeInterval);

    // 8.8.0: зоны авторежима
    static GEMItem itemCoarseZone("Грубая зона", editCoarseZone, spinnerCoarseZone, [](GEMCallbackData d) { menuSpinChanged(d, editCoarseZone); });
    static GEMItem itemFineZone("Точная зона", editFineZone, spinnerFineZone, [](GEMCallbackData d) { menuSpinChanged(d, editFineZone); });
    static GEMItem itemWorsening("Порог «хуже»", editWorsening, spinnerWorsening, [](GEMCallbackData d) { menuSpinChanged(d, editWorsening); });
    autoPage.addMenuItem(itemCoarseZone);
    autoPage.addMenuItem(itemFineZone);
    autoPage.addMenuItem(itemWorsening);

    // --- Страница "Дисплей" ---
    static GEMItem itemContrast("Яркость", editContrast, spinnerContrast, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editContrast = *(int*)ptr;
            analogWrite(PIN_TFT_BL, map(editContrast, 1, 100, 0, 255));
            settingsChanged = true;
            displayDirty = true;
        }
    });
    displayPage.addMenuItem(itemContrast);

    // 8.8.0: гашение подсветки, интервал кадра и пороги перерисовки
    static GEMItem itemBacklightOff("Гасить,мин", editBacklightOff, spinnerBacklightOff, [](GEMCallbackData d) { menuSpinChanged(d, editBacklightOff); });
    static GEMItem itemFrameMs("Интервал,мс", editFrameMs, spinnerFrameMs, [](GEMCallbackData d) { menuSpinChanged(d, editFrameMs); });
    static GEMItem itemRedrawAngle("Порог углов", editRedrawAngle, spinnerRedrawAngle, [](GEMCallbackData d) { menuSpinChanged(d, editRedrawAngle); });
    static GEMItem itemRedrawPressure("Порог давл.", editRedrawPressure, spinnerRedrawPressure, [](GEMCallbackData d) { menuSpinChanged(d, editRedrawPressure); });
    displayPage.addMenuItem(itemBacklightOff);
    displayPage.addMenuItem(itemFrameMs);
    displayPage.addMenuItem(itemRedrawAngle);
    displayPage.addMenuItem(itemRedrawPressure);

    // --- Страница "Движение" ---
    static GEMItem itemMovementFront("ПЕРЕДНИЕ", editMovementPressureFront, spinnerMovementFront, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editMovementPressureFront = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemMovementRear("ЗАДНИЕ", editMovementPressureRear, spinnerMovementRear, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editMovementPressureRear = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    movementPage.addMenuItem(itemMovementFront);
    movementPage.addMenuItem(itemMovementRear);

    // 8.8.0: параметры детектора движения
    static GEMItem itemMoveDuration("Длительность,с", editMoveDuration, spinnerMoveDuration, [](GEMCallbackData d) { menuSpinChanged(d, editMoveDuration); });
    static GEMItem itemMoveSettle("Успокоение,с", editMoveSettle, spinnerMoveSettle, [](GEMCallbackData d) { menuSpinChanged(d, editMoveSettle); });
    static GEMItem itemMoveCheck("Проверка,с", editMoveCheck, spinnerMoveCheck, [](GEMCallbackData d) { menuSpinChanged(d, editMoveCheck); });
    static GEMItem itemMoveTolerance("Допуск,бар", editMoveTolerance, spinnerMoveTolerance, [](GEMCallbackData d) { menuSpinChanged(d, editMoveTolerance); });
    static GEMItem itemGyroThr("Порог гиро", editGyroThreshold, spinnerGyroThr, [](GEMCallbackData d) { menuSpinChanged(d, editGyroThreshold); });
    static GEMItem itemAccelThr("Порог аксель", editAccelThreshold, spinnerAccelThr, [](GEMCallbackData d) { menuSpinChanged(d, editAccelThreshold); });
    movementPage.addMenuItem(itemMoveDuration);
    movementPage.addMenuItem(itemMoveSettle);
    movementPage.addMenuItem(itemMoveCheck);
    movementPage.addMenuItem(itemMoveTolerance);
    movementPage.addMenuItem(itemGyroThr);
    movementPage.addMenuItem(itemAccelThr);

    // --- Страница "Информация" (строки обновляются в refreshInfoPage) ---
    infoPage.addMenuItem(itemInfoVersion);
    infoPage.addMenuItem(itemInfoMode);
    infoPage.addMenuItem(itemInfoMpu);
    infoPage.addMenuItem(itemInfoMaster);
    infoPage.addMenuItem(itemInfoWiFi);
    infoPage.addMenuItem(itemInfoSystem);
    infoPage.addMenuItem(itemInfoErrors);

    // --- 8.8.0: страница «IMU / MPU6050» ---
    static GEMItem itemImuZero("Обнулить углы", []() { openImuZeroConfirm(); });
    static GEMItem itemImuCalib("Калибровка офсетов", []() { openImuCalibScreen(); });
    static GEMItem itemImuDiag("Диагностика MPU", []() { openMpuDiagScreen(); });
    static GEMItem itemImuMotionDet("Порог детектора", editImuMotionDet, spinnerImuMotionDet, [](GEMCallbackData d) { menuSpinChanged(d, editImuMotionDet); });
    static GEMItem itemZeroAngleX("Нуль X, град", editZeroAngleX, spinnerZeroAngleX, [](GEMCallbackData d) { menuSpinChanged(d, editZeroAngleX); });
    static GEMItem itemZeroAngleY("Нуль Y, град", editZeroAngleY, spinnerZeroAngleY, [](GEMCallbackData d) { menuSpinChanged(d, editZeroAngleY); });
    imuPage.addMenuItem(itemImuZero);
    imuPage.addMenuItem(itemImuCalib);
    imuPage.addMenuItem(itemImuDiag);
    imuPage.addMenuItem(itemImuMotionDet);
    imuPage.addMenuItem(itemZeroAngleX);
    imuPage.addMenuItem(itemZeroAngleY);

    // --- 8.8.0: страница «Настройки» (просмотр всех значений из config.txt) ---
    settingsViewPage.addMenuItem(itemSet1);
    settingsViewPage.addMenuItem(itemSet2);
    settingsViewPage.addMenuItem(itemSet3);
    settingsViewPage.addMenuItem(itemSet4);
    settingsViewPage.addMenuItem(itemSet5);
    settingsViewPage.addMenuItem(itemSet6);
    settingsViewPage.addMenuItem(itemSet7);

    // --- Добавляем страницы в главное меню ---
    static GEMItem linkSystem("Система", &systemPage);
    static GEMItem linkOta("Обновления", &otaPage);
    static GEMItem linkTest("Тестирование", &testPage);
    static GEMItem linkValve("Клапаны", &valvePage);
    static GEMItem linkTilt("Наклон", &tiltPage);
    static GEMItem linkPressure("Давление", &pressurePage);
    static GEMItem linkAuto("Авторежим", &autoPage);
    static GEMItem linkDisplay("Дисплей", &displayPage);
    static GEMItem linkMovement("Движение", &movementPage);
    static GEMItem linkInfo("Информация", &infoPage);
    static GEMItem linkImu("IMU / MPU6050", &imuPage);
    static GEMItem linkSettings("Настройки", &settingsViewPage);

    mainPage.addMenuItem(linkSystem);
    mainPage.addMenuItem(linkOta);
    mainPage.addMenuItem(linkTest);
    mainPage.addMenuItem(linkValve);
    mainPage.addMenuItem(linkTilt);
    mainPage.addMenuItem(linkPressure);
    mainPage.addMenuItem(linkAuto);
    mainPage.addMenuItem(linkDisplay);
    mainPage.addMenuItem(linkMovement);
    mainPage.addMenuItem(linkInfo);
    mainPage.addMenuItem(linkImu);
    mainPage.addMenuItem(linkSettings);

    // --- Кнопка сохранения ---
    static GEMItem itemSave("СОХРАНИТЬ", []() {
        saveMenuSettings();
        menuVisible = false;
        forceDisplayReset(true);
        displayDirty = true;
    });
    mainPage.addMenuItem(itemSave);

    gem.setMenuPageCurrent(mainPage);
}


/* ====================  SETUP ==================== */
/* ====================  SETUP ==================== */
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Kamaz‑Leveler (FreeRTOS OTA Optimized) ===");
  Serial.printf("Версия: %s\n", VERSION);
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

  if (!ErrorHandler::initMutex()) {
    Serial.println("[ERROR] Failed to init ErrorHandler mutex!");
    ESP.restart();
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  if (!xValveMutex || !xDisplayMutex || !xConfigMutex || !xStateMutex ||
      !xCalibMutex || !xTestMutex || !xCommandMutex) {
    Serial.println("[ERROR] Failed to create mutexes");
    ESP.restart();
  }

  // ========== 4. WIRE ==========
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000L);

// ========== ✅ ADS1015 – ПРОВЕРКА НАЛИЧИЯ ==========
// Полная инициализация будет выполнена в calibrationTask
#if !ENABLE_SIMULATION
    bool adsOk = initializeAds1015();
    if (!adsOk) {
        Serial.println("[ADS1015] ❌ Датчик не найден! Калибровка будет пропущена.");
        adsInitialized = false;
    } else {
        Serial.println("[ADS1015] ✅ Датчик найден");
        Serial.printf("[ADS1015] Адрес: 0x%02X\n", adsDetectedAddress);
    }
#else
    adsInitialized = true;
    Serial.println("[ADS1015] SIM: инициализация пропущена");
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
    ConfigManager::load();   // ← Теперь загружает config.txt
    loadWiFiConfig();
  }
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
  editNivCount = constrain(ConfigManager::getNivCount(), 1, 20);
  editTimeInterval = constrain(ConfigManager::getTimeInterval(), 1, 60);
  editContrast = constrain(ConfigManager::getContrast(), 1, 100);
  editMovementPressureFront = constrain(ConfigManager::getMovementPressureFront(), 1.0f, 6.0f);
  editMovementPressureRear = constrain(ConfigManager::getMovementPressureRear(), 1.0f, 6.0f);

  // ===== 8.8.0: зеркала новых параметров =====
  editMasterCheck = constrain(ConfigManager::getMasterCheckSec(), 30, 600);
  editManualMaxTime = constrain(ConfigManager::getManualMaxTimeSec(), 1, 30);
  editDeadband = constrain(ConfigManager::getPressureDeadband(), 0.05f, 0.5f);
  editCoarseZone = constrain(ConfigManager::getCoarseZoneRatio(), 0.2f, 0.9f);
  editFineZone = constrain(ConfigManager::getFineZoneRatio(), 0.05f, 0.3f);
  editWorsening = constrain(ConfigManager::getWorseningRatio(), 1.05f, 2.0f);
  editMoveDuration = constrain(ConfigManager::getMovementDurationSec(), 10, 120);
  editMoveSettle = constrain(ConfigManager::getMovementSettleSec(), 10, 120);
  editMoveCheck = constrain(ConfigManager::getMovementCheckSec(), 30, 300);
  editMoveTolerance = constrain(ConfigManager::getMovementTolerance(), 0.1f, 1.0f);
  editGyroThreshold = constrain(ConfigManager::getGyroThreshold(), 10, 200);
  editAccelThreshold = constrain(ConfigManager::getAccelThreshold(), 200, 3000);
  editBacklightOff = constrain(ConfigManager::getBacklightOffMin(), 0, 30);
  editFrameMs = constrain(ConfigManager::getFrameMs(), 20, 200);
  editRedrawAngle = constrain(ConfigManager::getRedrawAngleThr(), 0.01f, 0.5f);
  editRedrawPressure = constrain(ConfigManager::getRedrawPressureThr(), 0.01f, 0.5f);
  editImuMotionDet = constrain(ConfigManager::getImuMotionDet(), 20, 255);
  editZeroAngleX = constrain(ConfigManager::getZeroAngleX(), -3.0f, 3.0f);
  editZeroAngleY = constrain(ConfigManager::getZeroAngleY(), -3.0f, 3.0f);
  lastUserActivityMs = millis();  // 8.8.0: старт отсчёта гашения подсветки

  // ========== 11. ИНИЦИАЛИЗАЦИЯ GEM ==========
  initGEM();
  gem.hideVersion();       // Скрыть версию на splash-экране
  gem.setSplashDelay(0);   // Отключить splash-экран
  gem.init();

  // ========== 12. MPU6050 ==========
  initializeDMP();

  // ========== 13. ПОДСВЕТКА ==========
  analogWrite(PIN_TFT_BL, map(ConfigManager::getContrast(), 1, 100, 0, 255));

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
    { "ButtonTask", buttonTask, 6144, 4, 0, 20, nullptr },
    { "DisplayTask", displayTask, 8192, 2, 1, 33, nullptr },
    { "IMUTask", imuTask, 8192, 5, 0, 20, nullptr },
    { "PressTask", pressureTask, 8192, 3, 1, 2000, nullptr },
    { "ControlTask", controlTask, 6144, 3, 0, 500, nullptr },
    { "CalibTask", calibrationTask, 4096, 2, 1, 1000, nullptr },
    { "WatchdogTask", watchdogTask, 2048, 6, 1, 1000, nullptr },
    { "OTATask", otaTask, 12288, 2, 1, 100, nullptr },
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

  uptimeHours = 0;

  emergencyButton.setHoldTimeout(EMERGENCY_HOLD_MS);
  emergencyButton.setDebTimeout(50);

  menuCombo.setHoldTimeout(MENU_COMBO_HOLD_MS);
  menuCombo.setDebTimeout(MENU_COMBO_DEBOUNCE_MS);

  // ========== 18. ФИНАЛЬНЫЕ НАСТРОЙКИ ==========
  Serial.printf("[SETUP] final state: currentState=%d\n", (int)currentState);

  displayDirty = true;
  forceDisplayReset();
  vTaskDelay(pdMS_TO_TICKS(100));

  Logger::log(Logger::INFO, "SETUP", "Инициализация завершена");
}

void blinkErrorIcon() {
  static uint32_t lastBlinkTime = 0;
  static bool blinkState = true;
  const int16_t iconX = (SCREEN_WIDTH - 45) / 2;
  const int16_t iconY = 8;
  uint32_t now = millis();

  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    blinkState = !blinkState;

    if (blinkState) {
      drawIconB(iconX, iconY, err_Big, ST77XX_WHITE);
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
      Serial.printf("  ⚠️ WARNING: Task '%s' has very low stack!\n",
                    name ? name : "Unknown");
    }
  }

  Serial.printf("Total tasks: %d\n", TaskPool::getTaskCount());
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.println("==================\n");
}

/* ====================  LOOP ==================== */
void loop() {
  static uint32_t lastHourCounter = 0;
  static uint32_t lastMemoryCheck = 0;
  static uint32_t lastStackCheck = 0;
  static uint32_t lastTaskInfoPrint = 0;

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
        Serial.printf("[STACK] ⚠️ Task '%s' has only %d bytes free!\n",
                      name ? name : "Unknown", stackFree);
      }
    }
  }

  // ✅ Добавлена диагностика задач раз в минуту
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