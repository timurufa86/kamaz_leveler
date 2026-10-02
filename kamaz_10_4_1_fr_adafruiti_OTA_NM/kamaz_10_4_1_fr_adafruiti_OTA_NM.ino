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
#define ENABLE_MPU6050 1
#define ENABLE_SIMULATION 0
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

//const uint8_t PAD_COUNT = 4;
#define PAD_COUNT 4

/** Состояние инкрементальной шкалы угла (до авто-прототипов Arduino). */
struct AngleBarState {
  bool chromeDrawn = false;
  int8_t side = 0;  // -1 лево, 0 нет, +1 право
  int16_t fillW = 0;
  uint16_t fillCol = theme::OK;
  char valStr[12] = "";
  int16_t barX = 0, barY = 0, barW = 0, barH = 0, mid = 0;
  int16_t valX = 0, valY = 0, valW = 0;
};

bool jhmReady = false;  // флаг готовности JHM1200

/* ====================  КОНСТАНТЫ ==================== */
constexpr int16_t SCREEN_WIDTH = 320;
constexpr int16_t SCREEN_HEIGHT = 240;

constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 50;

/* ====================  НАСТРОЙКИ МЕНЮ ====================  ST77XX_WHITE */
constexpr uint8_t MENU_COLS = 60;          // символов в строке
constexpr uint8_t MENU_ROWS = 9;           // строк на экране (240/24 ? 10)
constexpr uint8_t MENU_TOP_OFFSET = 14;    // отступ сверху
constexpr uint8_t MENU_LEFT_PADDING = 15;  // отступ слева (для внешних панелей)
constexpr uint8_t ROW_HEIGHT = 22;         // высота строки (важно!)
constexpr uint8_t CURSOR_WIDTH = 2;
// Колонка значений в меню GEM (названия слева, значения с этой колонки).
constexpr uint8_t MENU_VALUES_LEFT_OFFSET = 150;

/* Бегущая строка меню — типы наверху файла, иначе Arduino auto-prototype ломает сборку */
enum MarqueeId : uint8_t {
  MQ_OTA_VER = 0,
  MQ_OTA_ST,
  MQ_LIST_HDR,
  MQ_CARD_TAG,
  MQ_CARD_INFO,
  MQ_CARD_ST,
  MQ_CARD_SHA,
  MQ_INFO_VER,
  MQ_INFO_WIFI,
  MQ_INFO_SYS,
  MQ_COUNT
};

struct MarqueeSlot {
  GEMItem *item = nullptr;
  char label[20] = "";
  char value[72] = "";
  char shown[72] = "";
  uint16_t offset = 0;
  bool enabled = false;
  bool scrolling = false;
};

/* ====================  КОМБИНАЦИИ КНОПОК ====================
 *  Меню: удержание КН3+КН4. Режим АВТО/РУЧ: КН1+КН2.
 *  Авария: удержание КН1+КН4 (обрабатывается первой, подавляет меню).
 *  КН5 отключена.
 */
constexpr uint32_t MENU_COMBO_HOLD_MS = 800;       // удержание КН3+КН4 (меню)
constexpr uint32_t MODE_TOGGLE_HOLD_MS = 800;      // удержание КН1+КН2: АВТО - РУЧНОЕ
constexpr uint32_t IMU_POLL_INTERVAL_MS = 25;      // дефолт периода DMP (см. imuPollMs в меню)
constexpr uint32_t EMERGENCY_HOLD_MS = 2000;       // удержание КН1+КН4: авария
// КН5 (GPIO34) отключена: плавающий вход давал ложные события и перезагрузки.
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
// VERSION now in app_version.h



/* 8.9.0: предварительное объявление типа — Arduino автогенерирует прототипы функций
   и вставляет их ВЫШЕ первого определения функции в файле, поэтому тип для
   прототипов (MvData — структура данных экрана ДВИЖЕНИЯ) объявляем заранее. */
struct MvData;

/* Пины — see app_pins.h */
static constexpr int CONTRAST_MIN = 10;   // мин. яркость в меню
static constexpr int CONTRAST_DIM = 5;    // уровень приглушения по бездействию
static constexpr int CONTRAST_MAX = 100;


// ========== JHM1200 (KY-3V3-IIC) 0..10 бар ==========

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
static int editMasterLowTenths;      // Низк.МП: 0…40 (= 0.0…4.0 бар), 0 = выкл.
static int editNivCount;
static int editTimeInterval;
static int editContrast;
static float editMovementPressureFront;
static float editMovementPressureRear;
static float editParkingPressure;    // давление стоянки после ДВИЖЕНИЕ→РУЧ (0 = из режима движения)
static bool settingsChanged = false;

/* ===== 8.8.0: зеркала новых параметров меню ===== */
static int   editMasterCheck;        // период проверки магистрали, с
static int   editManualMaxTime;      // максимальное время ручной операции, с
static int   editPressStabilizeMs;   // выравнивание МП после клапана, мс
static int   editPressIdleMin;       // пауза между полными опросами подушек, мин
static float editDeadband;           // зона нечувствительности по давлению, бар
static float editCoarseZone;         // грубая зона авторежима (доля порога)
static float editFineZone;           // точная зона авторежима (доля порога)
static float editWorsening;          // порог «стало хуже» (множитель)
static int   editMoveDuration;       // длительность ожидания движения, с
static int   editMoveSettle;         // время успокоения после движения, с
static int   editMoveCheck;          // период проверки давления после движения, с
static float editMoveTolerance;      // допуск давления после движения, бар
static int   editBacklightOff;       // гашение подсветки, мин (0 = никогда)
static int   editFrameMs;            // интервал кадра дисплея, мс
static float editRedrawAngle;        // порог перерисовки по углу, °
static float editRedrawPressure;     // порог перерисовки по давлению, бар
static int   editImuMotionDet;       // аппаратный порог детектора движения MPU (MOT)
static int   editGyroThreshold;      // порог |gyro−EMA| (меню ×8 → порог)
static int   editGyroBumpThreshold;  // порог |bump−EMA| — неровности дороги
static int   editAccelThreshold;     // порог |linAcc − EMA|
static float editZeroAngleX;         // программный нуль углов X, °
static float editZeroAngleY;         // программный нуль углов Y, °
static float editImuKalmanMea;       // GKalman mea_e (шум измерения)
static float editImuKalmanEst;       // GKalman est_e (шум оценки)
static float editImuKalmanQ;         // GKalman q (шум процесса)
static int   editImuPollMs;          // период опроса DMP, мс
static int   editImuFifoAvg;         // сколько FIFO-пакетов усреднять (1..8)
static float editImuEmaAlpha;        // EMA после Калмана (^ = быстрее)
static float editImuEmaSpikeAlpha;   // EMA при выбросе (v = глуше всплеск)
static float editImuEmaSpikeThr;     // порог выброса для spike-EMA, °
static float editImuSlewDps;         // макс. скорость изменения угла, °/с
static int   editImuPreset;          // 0=Плавно, 1=Быстро, 2=Баланс

/* 8.8.0: обработчик изменения пункта меню со спиннером объявлен НИЖЕ по файлу
   (перед initGEM) — в .ino первое определение функции не должно предшествовать
   объявлению типов/enum'ов и глобальных переменных, иначе автогенерация прототипов
   Arduino вставляет прототипы в начало файла и компиляция падает. */


/* ====================  SPINNER ДЛЯ МЕНЮ ==================== */

// --- Spinner для целых чисел (1-10, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_10 = { .step = 1, .min = 1, .max = 10 };
GEMSpinner spinnerRelease(spinnerInt1_10);  // Имп.СБРОС, с (AUTO и ручной)
GEMSpinner spinnerInflate(spinnerInt1_10);  // Имп.НАКАЧ, с (AUTO и ручной)

// --- Spinner для целых чисел (1-20, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_20 = { .step = 1, .min = 1, .max = 20 };
GEMSpinner spinnerNivCount(spinnerInt1_20);  // Попыток в час

// --- Spinner для целых чисел (1-60, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt1_60 = { .step = 1, .min = 1, .max = 60 };
GEMSpinner spinnerTimeInterval(spinnerInt1_60);  // Интервал попыток, мин

// --- Spinner для целых чисел (1-100, шаг 1) ---
GEMSpinnerBoundariesInt spinnerInt10_100 = { .step = 1, .min = 10, .max = 100 };
GEMSpinner spinnerContrast(spinnerInt10_100);  // Яркость (мин. 10)

// --- Spinner для float (0.0-3.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat0_3 = { .step = 0.1f, .min = 0.0f, .max = 3.0f };
GEMSpinner spinnerTiltX(spinnerFloat0_3);  // Поперечный
GEMSpinner spinnerTiltY(spinnerFloat0_3);  // Продольный

// --- Spinner для float (0.1-5.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat0_1_5 = { .step = 0.1f, .min = 0.1f, .max = 5.0f };
GEMSpinner spinnerPressureMin(spinnerFloat0_1_5);  // P.МИНИМУМ

// --- Spinner int 0…40 (= 0.0…4.0 бар) — без float ?0.00 у GEM ---
GEMSpinnerBoundariesInt spinnerInt0_40 = { .step = 1, .min = 0, .max = 40 };
GEMSpinner spinnerMasterLow(spinnerInt0_40);

// --- Spinner для float (1.0-8.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat1_8 = { .step = 0.1f, .min = 1.0f, .max = 8.0f };
GEMSpinner spinnerPressureMax(spinnerFloat1_8);  // P.МАКСИМУМ

// --- Spinner для float (1.0-6.0, шаг 0.1) ---
GEMSpinnerBoundariesFloat spinnerFloat1_6 = { .step = 0.1f, .min = 1.0f, .max = 6.0f };
GEMSpinnerBoundariesFloat spinnerFloat0_6 = { .step = 0.1f, .min = 0.0f, .max = 6.0f };  // стоянка: 0 = из движения
GEMSpinner spinnerMovementFront(spinnerFloat1_6);  // ПЕРЕДНИЕ
GEMSpinner spinnerMovementRear(spinnerFloat1_6);   // ЗАДНИЕ
GEMSpinner spinnerParkingPressure(spinnerFloat0_6); // Давл.стоянки

/* ===== 8.8.0: спиннеры новых параметров ===== */
// --- целочисленные ---
GEMSpinnerBoundariesInt spinnerInt30_600 = { .step = 30, .min = 30, .max = 600 };     // Проверка МП
GEMSpinnerBoundariesInt spinnerInt1_30 = { .step = 1, .min = 1, .max = 30 };          // Макс. время операции
GEMSpinnerBoundariesInt spinnerInt100_2000 = { .step = 50, .min = 100, .max = 2000 }; // Выравнивание МП, мс
GEMSpinnerBoundariesInt spinnerInt2_30 = { .step = 1, .min = 2, .max = 30 };          // Пауза опроса, мин
GEMSpinnerBoundariesInt spinnerInt10_120 = { .step = 5, .min = 10, .max = 120 };      // Длительность/успокоение
GEMSpinnerBoundariesInt spinnerInt30_300 = { .step = 10, .min = 30, .max = 300 };     // Проверка давления
GEMSpinnerBoundariesInt spinnerInt0_2 = { .step = 1, .min = 0, .max = 2 };            // Пресет IMU
GEMSpinnerBoundariesInt spinnerInt0_30 = { .step = 1, .min = 0, .max = 30 };          // Приглушить подсветку, мин
GEMSpinnerBoundariesInt spinnerInt20_200 = { .step = 5, .min = 20, .max = 200 };      // Интервал кадра
GEMSpinnerBoundariesInt spinnerInt20_255 = { .step = 5, .min = 20, .max = 255 };      // Порог MOT
GEMSpinnerBoundariesInt spinnerInt10_200 = { .step = 5, .min = 10, .max = 200 };      // Порог gyro (меню)
GEMSpinnerBoundariesInt spinnerInt200_3000 = { .step = 50, .min = 100, .max = 3000 };  // Порог ΔlinAcc
GEMSpinnerBoundariesInt spinnerInt15_100 = { .step = 5, .min = 15, .max = 100 };      // Период опроса IMU
GEMSpinnerBoundariesInt spinnerInt1_8 = { .step = 1, .min = 1, .max = 8 };            // FIFO усреднение

// --- float ---
GEMSpinnerBoundariesFloat spinnerFloat005_05 = { .step = 0.05f, .min = 0.05f, .max = 0.5f };  // Зона нечувствительности
GEMSpinnerBoundariesFloat spinnerFloat02_09 = { .step = 0.05f, .min = 0.2f, .max = 0.9f };    // Грубая зона
GEMSpinnerBoundariesFloat spinnerFloat005_03 = { .step = 0.01f, .min = 0.05f, .max = 0.3f };  // Точная зона
GEMSpinnerBoundariesFloat spinnerFloat105_2 = { .step = 0.05f, .min = 1.05f, .max = 2.0f };   // «Стало хуже»
GEMSpinnerBoundariesFloat spinnerFloat01_1 = { .step = 0.05f, .min = 0.1f, .max = 1.0f };     // Допуск давления
GEMSpinnerBoundariesFloat spinnerFloat001_05 = { .step = 0.01f, .min = 0.01f, .max = 0.5f };  // Пороги перерисовки
GEMSpinnerBoundariesFloat spinnerFloatNeg45_45 = { .step = 0.01f, .min = -45.0f, .max = 45.0f };  // Нуль углов (наклон рамы)
GEMSpinnerBoundariesFloat spinnerFloat05_15 = { .step = 0.5f, .min = 0.5f, .max = 25.0f };  // Калман mea/est
GEMSpinnerBoundariesFloat spinnerFloatQ = { .step = 0.001f, .min = 0.001f, .max = 0.100f };  // Калман Q
GEMSpinnerBoundariesFloat spinnerFloatEma = { .step = 0.01f, .min = 0.05f, .max = 0.50f };   // EMA alpha
GEMSpinnerBoundariesFloat spinnerFloatSpikeThr = { .step = 0.1f, .min = 0.5f, .max = 5.0f }; // порог выброса °
GEMSpinnerBoundariesFloat spinnerFloatSlew = { .step = 5.0f, .min = 5.0f, .max = 120.0f };   // slew °/с

GEMSpinner spinnerMasterCheck(spinnerInt30_600);        // Проверка МП
GEMSpinner spinnerManualMaxTime(spinnerInt1_30);        // Макс. время операции
GEMSpinner spinnerPressStabilize(spinnerInt100_2000);   // Выравнивание МП, мс
GEMSpinner spinnerPressIdle(spinnerInt2_30);            // Пауза опроса, мин
GEMSpinner spinnerDeadband(spinnerFloat005_05);         // Зона нечувствительности
GEMSpinner spinnerCoarseZone(spinnerFloat02_09);        // Грубая зона
GEMSpinner spinnerFineZone(spinnerFloat005_03);         // Точная зона
GEMSpinner spinnerWorsening(spinnerFloat105_2);         // Порог «стало хуже»
GEMSpinner spinnerMoveDuration(spinnerInt10_120);       // Длительность движения
GEMSpinner spinnerMoveSettle(spinnerInt10_120);         // Время успокоения
GEMSpinner spinnerMoveCheck(spinnerInt30_300);          // Проверка давления после движения
GEMSpinner spinnerMoveTolerance(spinnerFloat01_1);      // Допуск давления
GEMSpinner spinnerBacklightOff(spinnerInt0_30);         // Приглушить подсветку, мин
GEMSpinner spinnerFrameMs(spinnerInt20_200);            // Интервал кадра
GEMSpinner spinnerRedrawAngle(spinnerFloat001_05);      // Порог перерисовки углов
GEMSpinner spinnerRedrawPressure(spinnerFloat001_05);   // Порог перерисовки давления
GEMSpinner spinnerImuMotionDet(spinnerInt20_255);       // Порог MOT
GEMSpinner spinnerGyroThreshold(spinnerInt10_200);      // Порог gyro RMS
GEMSpinner spinnerGyroBumpThreshold(spinnerInt10_200);  // Порог pitch/roll (неровности)
GEMSpinner spinnerAccelThreshold(spinnerInt200_3000);   // Порог ΔlinAcc
GEMSpinner spinnerZeroAngleX(spinnerFloatNeg45_45);     // Нуль крен
GEMSpinner spinnerZeroAngleY(spinnerFloatNeg45_45);     // Нуль тангаж
GEMSpinner spinnerImuKalmanMea(spinnerFloat05_15);      // Калман измер.
GEMSpinner spinnerImuKalmanEst(spinnerFloat05_15);      // Калман оценка
GEMSpinner spinnerImuKalmanQ(spinnerFloatQ);            // Калман Q
GEMSpinner spinnerImuPollMs(spinnerInt15_100);          // Опрос IMU
GEMSpinner spinnerImuFifoAvg(spinnerInt1_8);            // Усреднение пакетов
GEMSpinner spinnerImuEmaAlpha(spinnerFloatEma);         // EMA
GEMSpinner spinnerImuEmaSpikeAlpha(spinnerFloatEma);    // EMA при выбросе
GEMSpinner spinnerImuEmaSpikeThr(spinnerFloatSpikeThr); // порог выброса
GEMSpinner spinnerImuSlewDps(spinnerFloatSlew);         // макс. скорость угла
GEMSpinner spinnerImuPreset(spinnerInt0_2);             // Пресет: Плавно/Быстро/Баланс

/* Переменные состояния */
// Pad, SystemState, SystemMode, TestState, Mode, TestStep — see app_types.h

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

// calibrationCompleted — see app_globals.cpp
volatile bool firstPressureMeasurementDone = false;  // < ДОБАВИТЬ

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
constexpr uint32_t VALVE_OPERATION_TIMEOUT_MS = 15000;
constexpr uint32_t VALVE_MAX_COMMAND_MS = 15000;
// 9.2.0: формат 5 — EMA/slew/FIFO + тюнинг углов в меню «IMU».
// Файлы версий 1–4 читаются, новые поля = значения по умолчанию.
constexpr uint32_t CONFIG_FORMAT_VERSION = 8;

constexpr uint32_t WDT_TIMEOUT_MS = 30000;
constexpr uint32_t TASK_WDT_TIMEOUT_MS = 10000;
constexpr uint8_t TASK_COUNT = 11;  // Event..Valve + ErrorRecovery (индекс 0..10)

constexpr uint32_t MANUAL_TARGET_CHECK_INTERVAL_MS = 120000;
constexpr uint32_t MANUAL_ADJUSTMENT_COOLDOWN_MS = 3000;
constexpr float MANUAL_PRESSURE_TOLERANCE = 0.1f;

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

/** TLS для GitHub OTA. SHA-256 файла — основная проверка целостности. */
static void otaConfigureTls(WiFiClientSecure *client) {
  if (!client) return;
#if KAMAZ_OTA_CERT_BUNDLE
  extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
  extern const uint8_t rootca_crt_bundle_end[] asm("_binary_x509_crt_bundle_end");
  const size_t bundleLen = static_cast<size_t>(rootca_crt_bundle_end - rootca_crt_bundle_start);
  if (bundleLen > 0) {
    client->setCACertBundle(rootca_crt_bundle_start, bundleLen);
    return;
  }
#endif
  client->setInsecure();
}

// taskIndex_* — see app_globals.cpp

/* ====================  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==================== */
SPIClass spi(VSPI);
Adafruit_ST7789 tft(&spi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GEM_adafruit_gfx gem(tft);

void menuExitAction();   // Кн4 на главной странице меню: сохранить и закрыть

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

/* --- Страница «Информация»: строки обновляются динамически (refreshInfoPage) ---
 * Важно: LABEL-пункт создаётся конструктором с ОДНИМ аргументом (GEMItem.cpp:43
 * выставляет type = GEM_ITEM_LABEL). Вариант ("текст", nullptr, GEM_ITEM_LABEL)
 * даёт readonly-ссылку, а не метку.
 */
static GEMItem itemInfoVersion("Версия: …");
// 10.1.0: строки страницы «Просмотр» (обновляются в refreshSettingsView)
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
// Калман для углов DMP (°): mea^ / qv > плавнее (вибрация кабины).
GKalman filterX(7.0f, 4.5f, 0.005f);
GKalman filterY(7.0f, 4.5f, 0.005f);

Button button0(PIN_BUT1, INPUT_PULLUP, LOW);
Button button1(PIN_BUT2, INPUT_PULLUP, LOW);
Button button2(PIN_BUT3, INPUT_PULLUP, LOW);
Button button3(PIN_BUT4, INPUT_PULLUP, LOW);
// КН5 отключена (GPIO34 плавает) — не тикаем, функции на КН1+КН2 / КН1+КН4.
Button button4(PIN_BUT5, INPUT, LOW);

VirtButton emergencyButton;  // КН1+КН4 авария

// xValveMutex … xPressureWakeupQueue — see app_globals.cpp

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

// angleX, angleY, temperature — see app_globals.cpp
float pressure[PAD_COUNT] = { 0 };
float masterPressure = 0;
uint32_t pressureStampMs[PAD_COUNT] = { 0 };  // millis() последнего удачного замера
uint32_t masterStampMs = 0;
bool pressureValid[PAD_COUNT] = { false };
bool masterValid = false;
uint32_t valveCycleCount[PAD_COUNT] = { 0 };
uint32_t valveOpenAccumMs[PAD_COUNT] = { 0 };
static volatile bool leakSuspect = false;
static char leakSuspectPad[8] = "";
// mpuOk, menuVisible — see app_globals.cpp
//bool menuRendered = false;         // < ДОБАВИТЬ!
uint32_t lastMenuInteraction = 0;  // < ДОБАВИТЬ!

/** Инкремент при forceDisplayReset — сбрасывает static-кэши главного экрана
 *  и авиагоризонта (иначе после выхода из меню: чёрный экран + нет гироскопа). */
volatile uint16_t g_displayEpoch = 0;

/** Запрос open/close меню с кнопки/Serial — обрабатывается в DisplayTask
 *  (под xDisplayMutex). Иначе ButtonTask делает tft.fillScreen без мьютекса
 *  параллельно с отрисовкой > подвисание SPI. */
enum class MenuReq : uint8_t { None = 0, Open = 1, Close = 2 };
static volatile MenuReq g_menuReq = MenuReq::None;
static volatile bool g_testHeartbeat = false;
static volatile bool g_testDebugDump = false;

// ===== 9.2.0: поток / статистика IMU для тюнинга фильтров =====
static volatile bool g_imuFilterResetReq = false;
static volatile uint32_t g_imuStreamUntilMs = 0;   // STREAM до этого millis
static volatile uint32_t g_imuStatsUntilMs = 0;    // STATS до этого millis
static volatile bool g_imuStatsActive = false;
static float g_imuRawX = 0.0f, g_imuRawY = 0.0f;  // до фильтров (после FIFO avg)
static float g_imuAbsX = 0.0f, g_imuAbsY = 0.0f;  // после фильтров, ДО вычитания нуля
static float g_imuOutX = 0.0f, g_imuOutY = 0.0f;  // после фильтров + нуль

// ===== 10.2.0: отладка детектора движения (MOT + gyro + linAcc RMS) =====
static volatile uint32_t g_motStreamUntilMs = 0;
static volatile uint32_t g_motStreamPeriodMs = 100;  // период строк [MOT], мс
static float g_motLastGyroRms = -1.0f;
static float g_motLastGyroDelta = -1.0f;  // |gyro − EMA| для порога
static float g_motLastGyroThr = 0.0f;
static float g_motLastGyroBump = -1.0f;   // max(|gx|,|gy|) — кивок/крен на кочках
static float g_motLastGyroBumpDelta = -1.0f;  // |bump − EMA|
static float g_motLastGyroBumpThr = 0.0f;
static float g_motLastLinAccRms = -1.0f;   // |lin - EMA| (дельта для порога)
static float g_motLastLinAccRaw = -1.0f;   // сырой |aaReal| RMS (на столе ~bias)
static float g_motLastLinAccThr = 0.0f;
static uint8_t g_motLastIntStatus = 0;
static uint8_t g_motLastMotStatus = 0;  // MOT_DETECT_STATUS (оси)
static uint8_t g_motDurMs = 40;         // MOT_DUR, мс (1 LSB = 1 ms)
static bool g_motLastMotPulse = false;
static bool g_motLastGyroBusy = false;
static bool g_motLastGyroBumpBusy = false;
static bool g_motLastLinAccBusy = false;
static bool g_motLastHold = false;
static uint32_t g_motLastActivityMs = 0;
static uint32_t g_motSampleCount = 0;
static uint32_t g_motMotPulseCount = 0;
static uint32_t g_motGyroBusyCount = 0;
static uint32_t g_motGyroBumpBusyCount = 0;
static uint32_t g_motLinAccBusyCount = 0;
static uint32_t g_motHoldEdgeCount = 0;  // 0→1 переходы hold
static uint32_t g_motModeEnterCount = 0; // входы в SystemMode::MOVEMENT
static uint32_t g_motStatsSinceMs = 0;
// аккумулятор STATS (пишется в imuTask, читается при завершении)
static uint32_t g_imuStatN = 0;
static uint32_t g_imuStatSpikes = 0;
static float g_imuStatSumX = 0, g_imuStatSumY = 0;
static float g_imuStatSumXX = 0, g_imuStatSumYY = 0;
static float g_imuStatMinX = 0, g_imuStatMaxX = 0;
static float g_imuStatMinY = 0, g_imuStatMaxY = 0;
static float g_imuStatMaxStep = 0;
static float g_imuStatPrevX = 0, g_imuStatPrevY = 0;
static uint32_t g_imuStatT0 = 0;

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
// otaInProgress, otaValveLock — see app_globals.cpp
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

/* ===== 8.9.0: данные для «живого» экрана ДВИЖЕНИЯ ===== */
uint32_t movementLastAdjustFront = 0;     // время последней корректировки передних подушек
uint32_t movementLastAdjustRear = 0;      // время последней корректировки задних подушек
int8_t movementLastAdjustFrontDir = 0;    // +1 — накачка, -1 — стравливание
int8_t movementLastAdjustRearDir = 0;
uint32_t movementStartMs = 0;             // начало обнаруженного движения (для «в движении N с»)
bool mvScreenWasActive = false;           // активен ли сейчас экран ДВИЖЕНИЯ

// calibrationValid — see app_globals.cpp
float g_pressureZeroBar = 0.0f;  // программный нуль (бар) после калибровки


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
static void updateMainTiltLive();
static void updateMainInfoLine(bool isMoving);
/** СТАТ + иконка «авто стоит»; мигает при детекции движения до входа в MOVEMENT. */
static void updateMainStatBadge();
static void toggleAutoManualMode(const char *via);
static bool tryModeToggleHold(uint32_t now);
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
bool initializeJhm1200();
void resetSystemErrors();
bool checkPressureLimits();
void saveMenuSettings();
void applyRuntimeSettings();  // 8.8.0: применение настроек к железу без перезагрузки
static void requestMenuOpen(const char *via);
static void requestMenuClose(const char *via);
static void processSerialTestCommands();
static void processTestCommandLine(char *line);
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs);
void maintainMovementPressure();
bool detectMotionFromIMU(uint8_t intStatus, uint8_t motStatus, uint32_t nowMs, float gyroRms,
                         float linAccRms, float gyroBump);
void checkAndAdjustMasterPressure();
void maintainManualPressure();
void setManualTargetPressure(Pad pad);
void setAllManualTargetsFromCurrent();
void setManualTargetsFromParkingPolicy();
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
// OTA_LIST_MAX, OTA_FETCH_MAX, OtaRelease — see app_types.h
// Список: по одному релизу за запрос (полный JSON ~8–12 КБ) — надёжнее на ESP32
constexpr char GITHUB_RELEASES_URL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=1";
constexpr char GITHUB_RELEASES_URL_SMALL[] =
    "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=1&page=1";

// otaReleases, otaReleaseCount, otaSelectedIndex, otaListStatus,
// otaLatestTag, otaUiNeedFullRedraw — see app_globals.cpp

// --- Страницы раздела ---
GEMPage otaPage("Обновления", mainPage);
GEMPage otaListPage("Прошивки (GitHub)", otaPage);
GEMPage otaCardPage("Релиз", otaListPage);

// --- Прототипы обработчиков пунктов (реализация ниже) ---
static void otaCheckUpdates();
static void otaOpenFirmwareList();
static void otaInstallLast();
static void otaSelectRelease(uint8_t idx);
static void otaCardInstall();
static void otaCardCheckSha();
static void otaArduinoMode();
static void otaExitMode();
static void refreshOtaCard();
static void refreshOtaListPage();
static void beginGitHubOtaProgressUi(const char *status);
static void endGitHubOtaProgressUi(bool reopenMenu);
static void handleOtaSerialCommands();
static void mqOnMenuPageEnter();
static bool mqTick();
static void gemRestoreMenuItemFont();

// --- Пункты страницы «Обновления» ---
static GEMItem itemOtaCurrent("Версия: ...");
static GEMItem itemOtaStatus("Статус: ...");
static GEMItem itemOtaList("Список прошивок", []() { otaOpenFirmwareList(); });
static GEMItem itemOtaInstallLast("Установить последнюю", []() { otaInstallLast(); });
static GEMItem itemOtaArduino("Режим ArduinoOTA (Wi-Fi)", []() { otaArduinoMode(); });
static GEMItem itemOtaExit("Выход из режима OTA", []() { otaExitMode(); });

// --- Пункты страницы «Прошивки (GitHub)» ---
static char otaListHdrBuf[40] = "загрузка с GitHub...";
static char otaItemTitle[OTA_LIST_MAX][34];
static GEMItem itemOtaListHdr(otaListHdrBuf);  // статус загрузки / ошибка
static GEMItem itemOtaRel0("—", []() { otaSelectRelease(0); });
static GEMItem itemOtaRel1("—", []() { otaSelectRelease(1); });
static GEMItem itemOtaRel2("—", []() { otaSelectRelease(2); });
static GEMItem *const otaRelItems[OTA_LIST_MAX] = {
  &itemOtaRel0, &itemOtaRel1, &itemOtaRel2
};

// --- Пункты карточки релиза ---
static GEMItem itemCardTag("Релиз: —");
static GEMItem itemCardInfo("данных нет");
static GEMItem itemCardStatus("обновите список");
static GEMItem itemCardSha("SHA-256: не проверен");
static GEMItem itemCardInstall("УСТАНОВИТЬ", []() { otaCardInstall(); });
static GEMItem itemCardCheckSha("Проверить SHA-256", []() { otaCardCheckSha(); });

/* --- Сетевые операции (вызываются из otaTask) --- */

static void githubOtaPrintHeap(const char *tag);
static void githubOtaHeartbeat();

/** Собрать дыры кучи: alloc всех крупных блоков → free → один contiguous maxBlk. */
static void githubOtaDefragHeap() {
  void *hold[64];
  int n = 0;
  size_t sz = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  while (n < 64 && sz >= 1024) {
    hold[n] = heap_caps_malloc(sz, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!hold[n]) {
      sz /= 2;
      continue;
    }
    n++;
    sz = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  }
  while (n > 0) {
    heap_caps_free(hold[--n]);
  }
}

/** Ранний contiguous DRAM под mbedtls (16K+16K SSL + PK). Без этого > PK alloc fail. */
static void *s_tlsHeapReserve = nullptr;
static constexpr size_t kTlsHeapReserveBytes = 72 * 1024;

static void githubOtaReserveTlsHeap(const char *why) {
  if (s_tlsHeapReserve) return;
  githubOtaDefragHeap();

  size_t maxBlk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (maxBlk < 28 * 1024) {
    Serial.printf("[GH-OTA] TLS-reserve skip (%s) maxBlk=%u\n", why ? why : "?",
                  static_cast<unsigned>(maxBlk));
    return;
  }
  size_t want = (maxBlk > 1024) ? (maxBlk - 1024) : maxBlk;
  if (want > kTlsHeapReserveBytes) want = kTlsHeapReserveBytes;
  while (want >= 28 * 1024) {
    s_tlsHeapReserve = heap_caps_malloc(want, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (s_tlsHeapReserve) {
      Serial.printf("[GH-OTA] TLS-reserve +%u (%s) free=%u maxBlk=%u\n",
                    static_cast<unsigned>(want), why ? why : "?",
                    static_cast<unsigned>(ESP.getFreeHeap()),
                    static_cast<unsigned>(ESP.getMaxAllocHeap()));
      return;
    }
    want -= 1024;
  }
  Serial.printf("[GH-OTA] TLS-reserve FAIL (%s) free=%u maxBlk=%u\n", why ? why : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));
}

static void githubOtaReleaseTlsHeap(const char *why) {
  if (!s_tlsHeapReserve) return;
  heap_caps_free(s_tlsHeapReserve);
  s_tlsHeapReserve = nullptr;
  Serial.printf("[GH-OTA] TLS-reserve free (%s) free=%u maxBlk=%u\n", why ? why : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));
}

/** На время TLS приостановить «шумные» задачи (стеки остаются, но нет новых alloc). */
static volatile bool s_otaWorkersPaused = false;

static void githubOtaPauseWorkers(bool pause) {
  s_otaWorkersPaused = pause;
  const uint8_t idxs[] = {taskIndex_Display, taskIndex_IMU, taskIndex_Control,
                          taskIndex_Valve,   taskIndex_Pressure, taskIndex_ErrRec,
                          taskIndex_Button};
  for (uint8_t idx : idxs) {
    if (idx == 0xFF) continue;
    if (pause) TaskPool::suspendTask(idx);
    else TaskPool::resumeTask(idx);
  }
}

static void githubListFilter(JsonDocument &filter) {
  filter[0]["tag_name"] = true;
  filter[0]["published_at"] = true;
  filter[0]["assets"][0]["name"] = true;
  filter[0]["assets"][0]["size"] = true;
  filter[0]["assets"][0]["browser_download_url"] = true;
}

/**
 * HTTPS GET > файл LittleFS, затем разбор.
 * Не держим весь JSON в String (на ESP32 ~20–30 КБ + TLS = TOO_LESS_RAM).
 * @return HTTP-код или -1; путь файла в outPath при успехе записи.
 */
static int githubHttpsDownloadToFile(const char *url, const char *outPath,
                                     uint32_t timeoutMs = 25000,
                                     const char *acceptHdr = "application/json",
                                     const char *rangeHdr = nullptr) {
  if (!url || !outPath) return -1;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] download: WiFi не STA");
    return -1;
  }

  LittleFS.remove(outPath);
  WiFi.setSleep(false);

  static SemaphoreHandle_t s_ghMutex = nullptr;
  if (!s_ghMutex) s_ghMutex = xSemaphoreCreateMutex();
  if (s_ghMutex && xSemaphoreTake(s_ghMutex, pdMS_TO_TICKS(45000)) != pdTRUE) {
    Serial.println("[GH-OTA] download: mutex timeout");
    return -1;
  }

  githubOtaReleaseTlsHeap("pre-GET");
  // Во время OTA-сессии воркеры уже на паузе — не будим между чанками.
  if (!otaInProgress) {
    githubOtaPauseWorkers(true);
  } else if (!s_tlsHeapReserve) {
    // Уже в сессии: воркеры должны оставаться suspended.
    githubOtaPauseWorkers(true);
  }
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(80));

  auto unlock = [&]() {
    if (otaInProgress) {
      // Не резервируем и не резюмим между SHA/чанками — иначе TLS thrash + HTTP -1.
      if (s_ghMutex) xSemaphoreGive(s_ghMutex);
      return;
    }
    // Не глотаем largest block после LIST/ping: reserve оставлял maxBlk~34K
    // и следующий GET отбрасывался порогом 40K → «нет связи TLS».
    githubOtaPauseWorkers(false);
    if (s_ghMutex) xSemaphoreGive(s_ghMutex);
  };

  Serial.printf("[GH-OTA] TLS prep heap=%u maxBlk=%u internal=%u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));

  {
    IPAddress ip;
    const bool dnsOk = WiFi.hostByName("api.github.com", ip);
    Serial.printf("[GH-OTA] DNS api.github.com > %s (%s)\n",
                  dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  }

  int lastCode = -1;
  for (int attempt = 1; attempt <= 4; attempt++) {
    const uint32_t freeH = ESP.getFreeHeap();
    const uint32_t maxBlk = ESP.getMaxAllocHeap();
    Serial.printf("[GH-OTA] GET try %d/4 heap=%u maxBlk=%u url=%s\n", attempt, freeH, maxBlk,
                  url);
    // После неудачного handshake куча часто фрагментирована — всегда собираем.
    githubOtaDefragHeap();
    const uint32_t maxBlk2 = ESP.getMaxAllocHeap();
    if (maxBlk2 < maxBlk) {
      Serial.printf("[GH-OTA] defrag maxBlk %u → %u\n", maxBlk, maxBlk2);
    }
    // setInsecure + буферы 8K: достаточно ~24–28K contiguous (не 40K).
    if (maxBlk2 < 24000u) {
      Serial.println("[GH-OTA] мало непрерывной RAM для TLS — пауза/defrag");
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      githubOtaDefragHeap();
      if (ESP.getMaxAllocHeap() < 20000u) {
        lastCode = -1;
        continue;
      }
    }

    // Клиент в куче; HTTPClient в отдельном scope — уничтожается ДО delete client.
    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      Serial.println("[GH-OTA] new WiFiClientSecure failed");
      lastCode = -1;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    otaConfigureTls(client);
    // setHandshakeTimeout — секунды; setTimeout — миллисекунды.
    client->setHandshakeTimeout(30);
    client->setTimeout(timeoutMs);

    int code = -1;
    int contentLen = -1;
    size_t total = 0;
    bool wroteOk = false;

    {
      HTTPClient http;
      http.setConnectTimeout(20000);
      http.setTimeout(timeoutMs);
      if (!http.begin(*client, url)) {
        Serial.println("[GH-OTA] http.begin failed");
        lastCode = -1;
      } else {
        http.useHTTP10(true);
        http.setReuse(false);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.addHeader("User-Agent", "kamaz-leveler/9.4.0");
        http.addHeader("Accept", acceptHdr ? acceptHdr : "*/*");
        http.addHeader("Accept-Encoding", "identity");
        http.addHeader("Connection", "close");
        if (rangeHdr && rangeHdr[0]) {
          http.addHeader("Range", rangeHdr);
          Serial.printf("[GH-OTA] Range %s\n", rangeHdr);
        }

        Serial.println("[GH-OTA] GET…");
        const uint32_t tGet0 = millis();
        code = http.GET();
        contentLen = http.getSize();
        Serial.printf("[GH-OTA] GET > HTTP %d (%s) size=%d in %lums heap=%u maxBlk=%u\n", code,
                      (code <= 0) ? HTTPClient::errorToString(code).c_str() : "ok", contentLen,
                      static_cast<unsigned long>(millis() - tGet0),
                      static_cast<unsigned>(ESP.getFreeHeap()),
                      static_cast<unsigned>(ESP.getMaxAllocHeap()));
        if (code <= 0) {
          char errBuf[128];
          const int elen = client->lastError(errBuf, sizeof(errBuf));
          Serial.printf("[GH-OTA] TLS lastError(%d): %s\n", elen, errBuf);
        }
        lastCode = code;

        if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
          File f = LittleFS.open(outPath, "w");
          if (!f) {
            Serial.println("[GH-OTA] LittleFS write open failed");
            lastCode = -1;
          } else {
            WiFiClient *stream = http.getStreamPtr();
            const uint32_t t0 = millis();
            uint32_t lastDataMs = millis();
            uint8_t buf[512];
            const bool ranged = (rangeHdr && rangeHdr[0]);
            wroteOk = true;
            if (ranged && contentLen > 0) {
              client->setTimeout(8000);
              while (static_cast<int>(total) < contentLen && (millis() - t0) < timeoutMs) {
                const size_t want =
                    min(sizeof(buf), static_cast<size_t>(contentLen) - total);
                const int n = stream->readBytes(buf, want);
                if (n <= 0) {
                  if (!client->connected() && total > 0) break;
                  delay(10);
                  continue;
                }
                if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                  Serial.println("[GH-OTA] LittleFS write error");
                  wroteOk = false;
                  break;
                }
                total += static_cast<size_t>(n);
                lastDataMs = millis();
              }
            } else {
              while ((millis() - t0) < timeoutMs) {
                int avail = stream->available();
                if (avail <= 0) {
                  const bool sockAlive = client->connected();
                  const uint32_t quiet = millis() - lastDataMs;
                  if (contentLen > 0 && static_cast<int>(total) >= contentLen) break;
                  if (quiet > (sockAlive ? 5000u : 1500u) && total >= 16) break;
                  delay(5);
                  continue;
                }
                while (stream->available()) {
                  const int n = stream->readBytes(buf, sizeof(buf));
                  if (n <= 0) break;
                  if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                    Serial.println("[GH-OTA] LittleFS write error");
                    wroteOk = false;
                    goto gh_rw_done;
                  }
                  total += static_cast<size_t>(n);
                  lastDataMs = millis();
                  if (contentLen > 0 && static_cast<int>(total) >= contentLen) goto gh_rw_done;
                }
              }
            }
          gh_rw_done:
            f.flush();
            f.close();
            if (!wroteOk) {
              LittleFS.remove(outPath);
              lastCode = -1;
            }
          }
        }
        http.end();
      }
    }  // HTTPClient destroyed before client->stop/delete

    client->stop();
    delete client;
    client = nullptr;

    if (code > 0 && code != HTTP_CODE_OK && code != HTTP_CODE_PARTIAL_CONTENT) {
      unlock();
      return code;
    }
    if (wroteOk && (total >= 16 || ((rangeHdr && rangeHdr[0]) && total >= 1))) {
      const bool ranged = (rangeHdr && rangeHdr[0]);
      if (ranged && contentLen > 0 && static_cast<int>(total) < contentLen) {
        Serial.printf("[GH-OTA] incomplete range: got %u want %d — retry\n",
                      static_cast<unsigned>(total), contentLen);
        LittleFS.remove(outPath);
        lastCode = -1;
        vTaskDelay(pdMS_TO_TICKS(400 * attempt));
        continue;
      }
      Serial.printf("[GH-OTA] saved %u байт > %s heap=%u\n", static_cast<unsigned>(total),
                    outPath, static_cast<unsigned>(ESP.getFreeHeap()));
      unlock();
      return (code == HTTP_CODE_PARTIAL_CONTENT) ? HTTP_CODE_PARTIAL_CONTENT : HTTP_CODE_OK;
    }

    LittleFS.remove(outPath);
    lastCode = (code > 0) ? code : -1;
    vTaskDelay(pdMS_TO_TICKS(400 * attempt));
  }

  unlock();
  return lastCode;
}

/** Завершить OTA TLS-сессию: вернуть reserve и воркеры. */
static void githubOtaEndInstallSession(const char *why) {
  otaInProgress = false;
  githubOtaReserveTlsHeap(why ? why : "ota-end");
  githubOtaPauseWorkers(false);
}
static void githubOtaPingDiag() {
  Serial.println("[OTA-PING] begin");
  githubOtaPrintHeap("ping");
  Serial.printf("[OTA-PING] wifi=%d IP=%s RSSI=%d\n", static_cast<int>(WiFi.status()),
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  IPAddress ip;
  const bool dnsOk = WiFi.hostByName("api.github.com", ip);
  Serial.printf("[OTA-PING] DNS api.github.com > %s (%s)\n",
                dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  if (!dnsOk) {
    Serial.println("[OTA-PING] FAIL dns");
    return;
  }

  WiFiClient tcp;
  tcp.setTimeout(5000);
  const uint32_t tTcp = millis();
  const bool tcpOk = tcp.connect(ip, 443);
  Serial.printf("[OTA-PING] TCP %s:443 > %s in %lums\n", ip.toString().c_str(),
                tcpOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTcp));
  if (tcpOk) tcp.stop();
  if (!tcpOk) {
    Serial.println("[OTA-PING] FAIL tcp");
    return;
  }

  WiFi.setSleep(false);
  githubOtaReleaseTlsHeap("ping-TLS");
  githubOtaPauseWorkers(true);
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(50));
  githubOtaPrintHeap("ping-pre-tls");

  WiFiClientSecure *sc = new (std::nothrow) WiFiClientSecure();
  if (!sc) {
    githubOtaReserveTlsHeap("ping-fail");
    githubOtaPauseWorkers(false);
    Serial.println("[OTA-PING] FAIL new secure");
    return;
  }
  otaConfigureTls(sc);
  sc->setHandshakeTimeout(30);
  sc->setTimeout(15000);
  const uint32_t tTls = millis();
  const bool tlsOk = sc->connect("api.github.com", 443);
  Serial.printf("[OTA-PING] TLS api.github.com:443 > %s in %lums heap=%u\n",
                tlsOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTls),
                static_cast<unsigned>(ESP.getFreeHeap()));
  if (!tlsOk) {
    char errBuf[128];
    const int elen = sc->lastError(errBuf, sizeof(errBuf));
    Serial.printf("[OTA-PING] TLS lastError(%d): %s\n", elen, errBuf);
  }
  if (tlsOk) {
    sc->print("GET /zen HTTP/1.0\r\nHost: api.github.com\r\nUser-Agent: kamaz-ping\r\n"
              "Connection: close\r\n\r\n");
    char line[128];
    size_t n = sc->readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = '\0';
    Serial.printf("[OTA-PING] HTTP first-line: %s\n", line);
  }
  sc->stop();
  delete sc;
  githubOtaReserveTlsHeap("ping-post-tls");
  githubOtaPauseWorkers(false);

  constexpr char kUrl[] = "https://api.github.com/zen";
  constexpr char kPath[] = "/ota_ping.txt";
  const int code = githubHttpsDownloadToFile(kUrl, kPath, 20000, "*/*");
  if (code == HTTP_CODE_OK) {
    File f = LittleFS.open(kPath, "r");
    String body = f ? f.readString() : String();
    if (f) f.close();
    LittleFS.remove(kPath);
    Serial.printf("[OTA-PING] HTTPS GET zen > HTTP %d body='%s'\n", code, body.c_str());
    Serial.println("[OTA-PING] OK");
  } else {
    Serial.printf("[OTA-PING] HTTPS GET zen > HTTP %d FAIL\n", code);
  }
}

/** Разбор JSON-файла списка релизов (Filter, без полной String). */
static bool parseGitHubReleaseListFile(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f || f.size() < 16) {
    strlcpy(otaListStatus, "пустой ответ", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "пустой ответ", sizeof(otaListHdrBuf));
    if (f) f.close();
    return false;
  }
  Serial.printf("[GH-OTA] Список file %u байт heap=%u\n", static_cast<unsigned>(f.size()),
                static_cast<unsigned>(ESP.getFreeHeap()));

  JsonDocument filter;
  githubListFilter(filter);
  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, f, DeserializationOption::Filter(filter));
  f.close();
  LittleFS.remove(path);

  if (err) {
    snprintf(otaListStatus, sizeof(otaListStatus), "JSON %s", err.c_str());
    strlcpy(otaListHdrBuf, "ошибка JSON", sizeof(otaListHdrBuf));
    Serial.printf("[GH-OTA] Список: JSON %s\n", err.c_str());
    return false;
  }

  if (!doc.is<JsonArray>()) {
    strlcpy(otaListStatus, "не массив JSON", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "неверный JSON", sizeof(otaListHdrBuf));
    return false;
  }

  OtaRelease collected[OTA_FETCH_MAX];
  uint8_t collectedN = 0;
  for (JsonObject rel : doc.as<JsonArray>()) {
    if (collectedN >= OTA_FETCH_MAX) break;
    OtaRelease r{};
    strlcpy(r.tag, rel["tag_name"] | "", sizeof(r.tag));
    if (r.tag[0] == '\0') continue;

    const char *published = rel["published_at"] | "";
    if (strlen(published) >= 10) {
      memcpy(r.date, published, 10);
      r.date[10] = '\0';
    }

    for (JsonObject asset : rel["assets"].as<JsonArray>()) {
      const char *name = asset["name"] | "";
      const char *url = asset["browser_download_url"] | "";
      if (name[0] == '\0' || url[0] == '\0') continue;
      if (strcmp(name, GITHUB_ASSET_NAME) == 0) {
        strlcpy(r.binUrl, url, sizeof(r.binUrl));
        r.size = asset["size"] | 0UL;
      } else if (strcmp(name, GITHUB_SHA256_ASSET_NAME) == 0) {
        strlcpy(r.shaUrl, url, sizeof(r.shaUrl));
      }
    }
    if (r.binUrl[0] == '\0') {
      Serial.printf("[GH-OTA]  пропуск %s — нет %s\n", r.tag, GITHUB_ASSET_NAME);
      continue;
    }
    collected[collectedN++] = r;
  }

  for (uint8_t i = 0; i + 1 < collectedN; i++) {
    for (uint8_t j = i + 1; j < collectedN; j++) {
      SemVer a{}, b{};
      parseSemVer(collected[i].tag, a);
      parseSemVer(collected[j].tag, b);
      if (!a.valid || !b.valid) continue;
      if (compareSemVer(b, a) > 0) {
        OtaRelease tmp = collected[i];
        collected[i] = collected[j];
        collected[j] = tmp;
      }
    }
  }

  uint8_t n = collectedN < OTA_LIST_MAX ? collectedN : OTA_LIST_MAX;
  for (uint8_t i = 0; i < n; i++) {
    otaReleases[i] = collected[i];
    Serial.printf("[GH-OTA]  #%u %s  %s  %lu Б  sha=%s\n", i, otaReleases[i].tag,
                  otaReleases[i].date, static_cast<unsigned long>(otaReleases[i].size),
                  otaReleases[i].shaUrl[0] ? "yes" : "NO");
  }
  otaReleaseCount = n;
  if (n > 0) {
    strlcpy(otaLatestTag, otaReleases[0].tag, sizeof(otaLatestTag));
    snprintf(otaListStatus, sizeof(otaListStatus), "список: %u (актуал. %s)", n, otaLatestTag);
    snprintf(otaListHdrBuf, sizeof(otaListHdrBuf), "последние %u", n);
  } else {
    strlcpy(otaListStatus, "нет прошивок с bin", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет bin в релизе", sizeof(otaListHdrBuf));
  }
  Serial.printf("[GH-OTA] Получено релизов: %u\n", n);
  return n > 0;
}

static bool fetchGitHubReleaseListOnce(const char *url) {
  constexpr char kPath[] = "/gh_list.json";
  const int code = githubHttpsDownloadToFile(url, kPath, 45000);
  if (code < 0) {
    strlcpy(otaListStatus, "ошибка соединения", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет соединения", sizeof(otaListHdrBuf));
    LittleFS.remove(kPath);
    return false;
  }
  if (code != HTTP_CODE_OK) {
    snprintf(otaListStatus, sizeof(otaListStatus), "GitHub HTTP %d", code);
    snprintf(otaListHdrBuf, sizeof(otaListHdrBuf), "HTTP %d", code);
    Serial.printf("[GH-OTA] Список: HTTP %d\n", code);
    LittleFS.remove(kPath);
    return false;
  }
  return parseGitHubReleaseListFile(kPath);
}

/** Загрузить список релизов (топ по SemVer). true, если получен хотя бы один. */
static bool fetchGitHubReleaseList() {
  otaReleaseCount = 0;
  otaSelectedIndex = -1;
  for (uint8_t i = 0; i < OTA_LIST_MAX; i++) {
    otaReleases[i] = OtaRelease{};
  }

  if (WiFi.status() != WL_CONNECTED) {
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет Wi-Fi (STA)", sizeof(otaListHdrBuf));
    Serial.println("[GH-OTA] Список: нет Wi-Fi");
    if (menuVisible) displayDirty = true;
    return false;
  }

  // /tags даёт ~1–2 КБ вместо ~12 КБ на релиз — ESP32 не обрезает TLS-тело
  strlcpy(otaListHdrBuf, "связь с GitHub...", sizeof(otaListHdrBuf));
  if (menuVisible) displayDirty = true;
  constexpr char kTagsUrl[] =
      "https://api.github.com/repos/timurufa86/kamaz_leveler/tags?per_page=3";
  constexpr char kPath[] = "/gh_tags.json";
  Serial.printf("[GH-OTA] Список tags, heap %u\n", static_cast<unsigned>(ESP.getFreeHeap()));
  const int code = githubHttpsDownloadToFile(kTagsUrl, kPath, 22000, "application/json");
  if (code != HTTP_CODE_OK) {
    if (code <= 0) {
      strlcpy(otaListStatus, "сеть/TLS ошибка", sizeof(otaListStatus));
      strlcpy(otaListHdrBuf, "нет связи TLS", sizeof(otaListHdrBuf));
    } else {
      snprintf(otaListStatus, sizeof(otaListStatus), "GitHub HTTP %d", code);
      snprintf(otaListHdrBuf, sizeof(otaListHdrBuf), "HTTP %d", code);
    }
    LittleFS.remove(kPath);
    if (menuVisible) displayDirty = true;
    return false;
  }

  File f = LittleFS.open(kPath, "r");
  if (!f || f.size() < 8) {
    strlcpy(otaListStatus, "пустой ответ", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "пустой ответ", sizeof(otaListHdrBuf));
    if (f) f.close();
    LittleFS.remove(kPath);
    if (menuVisible) displayDirty = true;
    return false;
  }
  Serial.printf("[GH-OTA] tags file %u байт\n", static_cast<unsigned>(f.size()));
  strlcpy(otaListHdrBuf, "разбор списка...", sizeof(otaListHdrBuf));
  if (menuVisible) displayDirty = true;

  JsonDocument filter;
  filter[0]["name"] = true;
  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, f, DeserializationOption::Filter(filter));
  f.close();
  LittleFS.remove(kPath);
  if (err || !doc.is<JsonArray>()) {
    snprintf(otaListStatus, sizeof(otaListStatus), "JSON %s", err ? err.c_str() : "arr");
    strlcpy(otaListHdrBuf, "ошибка JSON", sizeof(otaListHdrBuf));
    Serial.printf("[GH-OTA] tags JSON %s\n", err ? err.c_str() : "not array");
    if (menuVisible) displayDirty = true;
    return false;
  }

  OtaRelease collected[OTA_FETCH_MAX];
  uint8_t collectedN = 0;
  for (JsonObject tagObj : doc.as<JsonArray>()) {
    if (collectedN >= OTA_FETCH_MAX) break;
    const char *name = tagObj["name"] | "";
    if (name[0] == '\0') continue;
    OtaRelease r{};
    strlcpy(r.tag, name, sizeof(r.tag));
    // Прямые URL GitHub Releases (без assets из API)
    snprintf(r.binUrl, sizeof(r.binUrl),
             "https://github.com/timurufa86/kamaz_leveler/releases/download/%s/%s", r.tag,
             GITHUB_ASSET_NAME);
    snprintf(r.shaUrl, sizeof(r.shaUrl),
             "https://github.com/timurufa86/kamaz_leveler/releases/download/%s/%s", r.tag,
             GITHUB_SHA256_ASSET_NAME);
    r.date[0] = '\0';
    r.size = 0;
    collected[collectedN++] = r;
  }

  for (uint8_t i = 0; i + 1 < collectedN; i++) {
    for (uint8_t j = i + 1; j < collectedN; j++) {
      SemVer a{}, b{};
      parseSemVer(collected[i].tag, a);
      parseSemVer(collected[j].tag, b);
      if (!a.valid || !b.valid) continue;
      if (compareSemVer(b, a) > 0) {
        OtaRelease tmp = collected[i];
        collected[i] = collected[j];
        collected[j] = tmp;
      }
    }
  }

  uint8_t n = collectedN < OTA_LIST_MAX ? collectedN : OTA_LIST_MAX;
  for (uint8_t i = 0; i < n; i++) {
    otaReleases[i] = collected[i];
    Serial.printf("[GH-OTA]  #%u %s\n", i, otaReleases[i].tag);
  }
  otaReleaseCount = n;
  if (n > 0) {
    strlcpy(otaLatestTag, otaReleases[0].tag, sizeof(otaLatestTag));
    snprintf(otaListStatus, sizeof(otaListStatus), "список: %u (актуал. %s)", n, otaLatestTag);
    snprintf(otaListHdrBuf, sizeof(otaListHdrBuf), "последние %u", n);
  } else {
    strlcpy(otaListStatus, "нет тегов на GitHub", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет тегов", sizeof(otaListHdrBuf));
  }
  Serial.printf("[GH-OTA] Получено релизов: %u\n", n);
  if (menuVisible) displayDirty = true;
  return n > 0;
}

/** Скачать sha256-ассет релиза (только проверка, без установки). */
static bool fetchReleaseSha256(uint8_t idx) {
  if (idx >= otaReleaseCount) return false;
  OtaRelease &r = otaReleases[idx];
  if (r.shaUrl[0] == '\0' || WiFi.status() != WL_CONNECTED) return false;

  static constexpr char kShaPath[] = "/gh_sha_chk.txt";
  const int code = githubHttpsDownloadToFile(r.shaUrl, kShaPath, 45000, "*/*");
  if (code != HTTP_CODE_OK) {
    LittleFS.remove(kShaPath);
    strlcpy(otaListStatus, "sha256 недоступен", sizeof(otaListStatus));
    return false;
  }
  File f = LittleFS.open(kShaPath, "r");
  char body[72] = "";
  if (f) {
    size_t n = f.readBytes(body, 64);
    body[n < 64 ? n : 64] = '\0';
    f.close();
  }
  LittleFS.remove(kShaPath);
  if (strlen(body) < 64) {
    strlcpy(otaListStatus, "sha256 недоступен", sizeof(otaListStatus));
    return false;
  }
  body[64] = '\0';
  strlcpy(r.sha256, body, sizeof(r.sha256));
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

static void beginGitHubOtaProgressUi(const char *status) {
  // Не ставим otaInProgress здесь — иначе LIST/SHA думают, что уже install-сессия
  // и не возвращают TLS-reserve / воркеров.
  otaProgress = 0;
  strlcpy(otaStatus, status ? status : "OTA…", sizeof(otaStatus));
  otaUiNeedFullRedraw = true;
  menuVisible = false;
  displayDirty = true;
}

static void endGitHubOtaProgressUi(bool reopenMenu) {
  otaInProgress = false;
  otaProgress = 0;
  otaStatus[0] = '\0';
  if (reopenMenu) {
    menuVisible = true;
    displayDirty = true;
  } else {
    forceDisplayReset(true);
    displayDirty = true;
  }
}

static void otaCheckUpdates() {
  strlcpy(otaListStatus, "проверка...", sizeof(otaListStatus));
  displayDirty = true;
  requestGitHubOtaCheckOnly();
  Serial.println("[GH-OTA] Запрос проверки обновления");
}

static void otaOpenFirmwareList() {
  // Автозагрузка при входе: статус + плейсхолдеры слотов, затем FETCH_LIST.
  // НЕ вызывать gem.drawMenu() здесь — колбэк GEM из ButtonTask; SPI без
  // display-mutex > артефакты/ребут. Рисует DisplayTask по displayDirty.
  otaReleaseCount = 0;
  otaSelectedIndex = -1;
  strlcpy(otaListStatus, "загрузка списка...", sizeof(otaListStatus));
  strlcpy(otaListHdrBuf, "загрузка с GitHub...", sizeof(otaListHdrBuf));
  mqBind(MQ_LIST_HDR, itemOtaListHdr, "Статус: ", otaListHdrBuf);
  itemOtaListHdr.show();
  for (uint8_t i = 0; i < OTA_LIST_MAX; i++) {
    snprintf(otaItemTitle[i], sizeof(otaItemTitle[i]), "...");
    otaRelItems[i]->setTitle(otaItemTitle[i]);
    otaRelItems[i]->show();
  }
  gem.setMenuPageCurrent(otaListPage);
  displayDirty = true;
  requestGitHubOtaFetchList();
  Serial.println("[GH-OTA] Открыт список прошивок, запрос GitHub");
}

static void otaInstallLast() {
  strlcpy(otaListStatus, "установка...", sizeof(otaListStatus));
  beginGitHubOtaProgressUi("Проверка GitHub...");
  requestGitHubOtaCheckAndInstall();
  Serial.println("[GH-OTA] Запрос на установку последнего релиза");
}

static void otaSelectRelease(uint8_t idx) {
  if (idx >= otaReleaseCount) {
    strlcpy(otaListStatus, "сначала откройте список", sizeof(otaListStatus));
    displayDirty = true;
    return;
  }
  otaSelectedIndex = static_cast<int8_t>(idx);
  gem.setMenuPageCurrent(otaCardPage);
  displayDirty = true;
}

static void otaCardInstall() {
  if (otaSelectedIndex < 0) return;
  beginGitHubOtaProgressUi("Загрузка релиза…");
  requestGitHubOtaInstallIndex(otaSelectedIndex);
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
  float measurements_[20] = { 0 };
  uint8_t measureCount_ = 0;
  float finalZeroBar_ = 0.0f;

  // При открытом сбросе абсолютное показание JHM1200 (до вычитания нуля)
  // должно быть около атмосферного/нуля модуля — обычно |bar| < 1.5.
  static constexpr float ZERO_ABS_MAX = 1.5f;
  static constexpr float ZERO_SPREAD_MAX = 0.15f;

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
    finalZeroBar_ = 0.0f;

    float testBar = 0.0f;
    bool ok = false;
    {
      MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(100));
      if (i2c) ok = Jhm1200::readBar(testBar);
    }
    Serial.printf("[CALIB] start(): probe=%s bar=%.3f status=0x%02X\n",
                  ok ? "ok" : "fail", testBar, Jhm1200::lastStatus());

    Serial.println("\n========================================");
    Serial.println("     АВТОМАТИЧЕСКАЯ КАЛИБРОВКА НУЛЯ");
    Serial.println("     (JHM1200 0..10 бар)");
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
          if (now - stepStartTime_ >= 3000) {
            Serial.println("[CALIB] Ожидание стабилизации завершено");
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
            float bar = 0.0f;
            bool ok = false;
            {
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
              if (i2c) ok = Jhm1200::readBar(bar);
            }
            lastMeasureTime = now;
            if (!ok) {
              Serial.println("[CALIB] JHM1200 read fail (пропуск)");
              break;
            }
            measurements_[measureCount_++] = bar;
            Serial.printf("[CALIB] Измерение %u: %.3f бар\n", measureCount_, bar);
          }
        } else {
          float sum = 0.0f;
          float minVal = measurements_[5];
          float maxVal = measurements_[5];
          for (int i = 5; i < 20; i++) {
            sum += measurements_[i];
            if (measurements_[i] < minVal) minVal = measurements_[i];
            if (measurements_[i] > maxVal) maxVal = measurements_[i];
          }
          finalZeroBar_ = sum / 15.0f;
          const float range = maxVal - minVal;
          Serial.printf("[CALIB] Среднее: %.3f бар, разброс: %.3f\n", finalZeroBar_, range);

          if (fabsf(finalZeroBar_) > ZERO_ABS_MAX) {
            Serial.printf("[CALIB] Необычный ноль |%.3f| > %.1f бар\n", finalZeroBar_, ZERO_ABS_MAX);
            if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
              ErrorHandler::handleError(ErrorHandler::Error::SENSOR,
                                        "JHM1200 - аномальный ноль");
            }
            calibrationValid = false;
          } else {
            if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
              ErrorHandler::removeError(ErrorHandler::Error::SENSOR);
            }
            calibrationValid = true;
          }
          if (range > ZERO_SPREAD_MAX) {
            Serial.printf("[CALIB] Нестабильные измерения, разброс %.3f бар\n", range);
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
            g_pressureZeroBar = finalZeroBar_;
            calibrationDone_ = true;
            Serial.printf("[CALIB] Ноль сохранён: %.3f бар\n", g_pressureZeroBar);
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

      default:
        break;
    }
  }

  bool isDone() const { return calibrationDone_; }
  float getZeroValue() const { return finalZeroBar_; }
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

    // Низкая магистраль блокирует только НАКАЧКУ; сброс должен оставаться доступен.
    if (durationMs > 0 && inflate) {
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
          Serial.printf("[VALVE] Накачка заблокирована: МП %.1f < мин %.1f\n",
                        currentMaster, minPressure);
        }

        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        return;
      }
    }
    uint32_t safeDuration;
    // UINT32_MAX = удержание до stopManualOperation / таймаута (не «закрыть»).
    if (durationMs == 0) {
      safeDuration = 0;
    } else if (durationMs == UINT32_MAX) {
      safeDuration = VALVE_MAX_COMMAND_MS;
    } else {
      safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
    }

    if (safeDuration > 0 && !ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {

      if (lastCmd.waitingForCompletion) {
        Serial.println("[VALVE] Новая команда до анализа предыдущей, предыдущая отменена");
        lastCmd.waitingForCompletion = false;
      }

      lastCmd.startTime = millis();
      lastCmd.duration = safeDuration;
      lastCmd.pad = pad;
      lastCmd.inflate = inflate;
      lastCmd.waitingForCompletion = (durationMs != UINT32_MAX);
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

    if (safeDuration > 0) {
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

/* ===== Подсветка: рабочая яркость / приглушение по бездействию ===== */
bool backlightDimmed = false;     // true = подсветка на уровне CONTRAST_DIM (не OFF)
uint32_t lastUserActivityMs = 0;

static int contrastToPwm(int level) {
  return map(constrain(level, 1, CONTRAST_MAX), 1, CONTRAST_MAX, 0, 255);
}

static void applyBacklightPwm(int level) {
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
static bool valveTestUiFullRedraw = true;
static bool imuZeroUiFullRedraw = true;
static bool imuCalibUiFullRedraw = true;
static bool mpuDiagUiFullRedraw = true;

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

static void uiRedrawLabel(int16_t x, int16_t y, int16_t w, int16_t h, char *prev, size_t prevSz,
                          const char *next, uint16_t fg, uint16_t bg, UiFont font,
                          UiHAlign ha = UiHAlign::Left);

/* ===== 8.8.0: отрисовка служебных экранов ===== */
static constexpr int16_t VT_ROW_H = 22;
static constexpr int16_t VT_ROWS_Y = 22;
static constexpr int16_t VT_STATUS_Y = VT_ROWS_Y + MANUAL_VALVE_COUNT * VT_ROW_H + 2;
static constexpr int16_t VT_HINT_H = 18;

static void drawValveTestRow(uint8_t i) {
  if (i >= MANUAL_VALVE_COUNT) return;
  const int16_t y = VT_ROWS_Y + static_cast<int16_t>(i) * VT_ROW_H;
  const int16_t x = theme::MARGIN - 2;
  const int16_t w = theme::SCREEN_W - 2 * theme::MARGIN + 4;
  const bool selected = (i == manualValveIndex);
  const bool isOpen = (manualValveOpenIndex == i);

  tft.fillRect(x, y - 1, w, VT_ROW_H, theme::BG);
  if (selected) {
    tft.fillRoundRect(x, y - 1, w, VT_ROW_H, 3, isOpen ? theme::OK : theme::PANEL);
  }
  ui.setTransparent(true);
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.boxf(theme::MARGIN + 4, y, 150, VT_ROW_H - 2, UiHAlign::Left, UiVAlign::Middle, false,
          "%s %s", selected ? ">" : " ", manualValveNames[i]);
  ui.box(theme::SCREEN_W - theme::MARGIN - 110, y, 110, VT_ROW_H - 2, isOpen ? "ОТКРЫТ" : "закрыт",
         UiHAlign::Right, UiVAlign::Middle, false);
}

static void drawValveTestStatus() {
  const uint32_t holdSec =
      (manualValveOpenIndex == 0xFF) ? 0 : (millis() - manualValveOpenSince) / 1000;
  tft.fillRect(theme::MARGIN, VT_STATUS_Y, theme::SCREEN_W - 2 * theme::MARGIN, VT_HINT_H, theme::BG);
  ui.setTransparent(true);
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.boxf(theme::MARGIN, VT_STATUS_Y, theme::SCREEN_W - 2 * theme::MARGIN, VT_HINT_H, UiHAlign::Left,
          UiVAlign::Middle, false, "МП: %.2f бар   открыт: %lu с / 15 с", masterPressure,
          (unsigned long)holdSec);
}

static void drawValveTestScreen() {
  static uint8_t drawnIndex = 0xFF;
  static uint8_t drawnOpen = 0xFE;
  static float drawnP = -999.0f;
  static uint32_t drawnHoldSec = 0xFFFFFFFFu;

  ui.setTransparent(true);
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  if (valveTestUiFullRedraw) {
    tft.fillScreen(theme::BG);
    ui.setFont(UiFont::SmallB);
    ui.box(theme::MARGIN, 2, theme::SCREEN_W - 2 * theme::MARGIN, 18, "РУЧНОЙ ТЕСТ КЛАПАНОВ",
           UiHAlign::Center, UiVAlign::Middle, false);
    ui.setFont(UiFont::Small);
    for (uint8_t i = 0; i < MANUAL_VALVE_COUNT; i++) drawValveTestRow(i);
    drawValveTestStatus();
    ui.box(theme::MARGIN, VT_STATUS_Y + VT_HINT_H + 2, theme::SCREEN_W - 2 * theme::MARGIN, VT_HINT_H,
           "КН1/КН2-ВЫБОР  КН3-УДЕРЖ=ОТКРЫТ", UiHAlign::Left, UiVAlign::Middle, true);
    ui.box(theme::MARGIN, VT_STATUS_Y + 2 * VT_HINT_H + 4, theme::SCREEN_W - 2 * theme::MARGIN, VT_HINT_H,
           "КН4-ВЫХОД (все закрыть)  АВАРИЯ КН1+КН4", UiHAlign::Left, UiVAlign::Middle, true);
    drawnIndex = manualValveIndex;
    drawnOpen = manualValveOpenIndex;
    drawnP = masterPressure;
    drawnHoldSec = (manualValveOpenIndex == 0xFF) ? 0 : (millis() - manualValveOpenSince) / 1000;
    valveTestUiFullRedraw = false;
    return;
  }

  if (drawnIndex != manualValveIndex || drawnOpen != manualValveOpenIndex) {
    const uint8_t prevIdx = drawnIndex;
    const uint8_t prevOpen = drawnOpen;
    drawnIndex = manualValveIndex;
    drawnOpen = manualValveOpenIndex;
    uint8_t redraw[4];
    uint8_t n = 0;
    auto addRow = [&](uint8_t idx) {
      if (idx >= MANUAL_VALVE_COUNT) return;
      for (uint8_t k = 0; k < n; k++) {
        if (redraw[k] == idx) return;
      }
      redraw[n++] = idx;
    };
    addRow(prevIdx);
    addRow(prevOpen);
    addRow(manualValveIndex);
    addRow(manualValveOpenIndex);
    for (uint8_t k = 0; k < n; k++) drawValveTestRow(redraw[k]);
  }

  const uint32_t holdSec =
      (manualValveOpenIndex == 0xFF) ? 0 : (millis() - manualValveOpenSince) / 1000;
  if (holdSec != drawnHoldSec || fabsf(masterPressure - drawnP) >= 0.02f) {
    drawnHoldSec = holdSec;
    drawnP = masterPressure;
    drawValveTestStatus();
  }
}

static void drawImuZeroConfirmScreen() {
  static char angBuf[48] = "";
  static char zeroBuf[48] = "";
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  if (imuZeroUiFullRedraw) {
    tft.fillScreen(theme::BG);
    ui.box(theme::MARGIN, 6, theme::SCREEN_W - 2 * theme::MARGIN, 20,
           "ОБНУЛИТЬ УГЛЫ ПО ТЕКУЩЕМУ ПОЛОЖЕНИЮ", UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 40, theme::SCREEN_W - 2 * theme::MARGIN, 16,
           "Текущее положение станет 0.00 / 0.00", UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 150, theme::SCREEN_W - 2 * theme::MARGIN, 18,
           "Машина должна стоять на месте!", UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 190, theme::SCREEN_W - 2 * theme::MARGIN, 18,
           "КН3 - ПОДТВЕРДИТЬ     КН4 - ОТМЕНА", UiHAlign::Center, UiVAlign::Middle, false);
    angBuf[0] = zeroBuf[0] = '\0';
    imuZeroUiFullRedraw = false;
  }

  char buf[48];
  snprintf(buf, sizeof(buf), "Поперечный: %.2f   Продольный: %.2f", angleX, angleY);
  uiRedrawLabel(theme::MARGIN, 70, theme::SCREEN_W - 2 * theme::MARGIN, 18, angBuf, sizeof(angBuf),
                buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
  snprintf(buf, sizeof(buf), "Текущий нуль: %.2f / %.2f", ConfigManager::getZeroAngleX(),
           ConfigManager::getZeroAngleY());
  uiRedrawLabel(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, zeroBuf, sizeof(zeroBuf),
                buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
}

static void drawImuCalibScreen() {
  static char stBuf[40] = "";
  static char gyroBuf[40] = "";
  static char accelBuf[40] = "";
  static char ageBuf[40] = "";
  static int8_t lastResult = 127;
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  if (imuCalibUiFullRedraw) {
    tft.fillScreen(theme::BG);
    ui.box(theme::MARGIN, 6, theme::SCREEN_W - 2 * theme::MARGIN, 20, "КАЛИБРОВКА ОФСЕТОВ IMU",
           UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 40, theme::SCREEN_W - 2 * theme::MARGIN, 16,
           "Машина ровно, НЕПОДВИЖНО 10-20 с", UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 62, theme::SCREEN_W - 2 * theme::MARGIN, 16,
           "DMP на время калибровки отключается", UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 200, theme::SCREEN_W - 2 * theme::MARGIN, 18,
           "КН3 - КАЛИБРОВАТЬ     КН4 - ВЫХОД", UiHAlign::Center, UiVAlign::Middle, false);
    stBuf[0] = gyroBuf[0] = accelBuf[0] = ageBuf[0] = '\0';
    lastResult = 127;
    imuCalibUiFullRedraw = false;
  }

  if (lastResult != imuCalibResult) {
    lastResult = imuCalibResult;
    const char *st = "Готово к запуску";
    if (imuCalibResult == 1) st = "ОФСЕТЫ СОХРАНЕНЫ В КОНФИГ";
    else if (imuCalibResult < 0) st = "ОШИБКА: MPU6050 недоступен";
    uiRedrawLabel(theme::MARGIN, 100, theme::SCREEN_W - 2 * theme::MARGIN, 18, stBuf, sizeof(stBuf),
                  st, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
  }

  char buf[48];
  snprintf(buf, sizeof(buf), "гиро: %d %d %d", ConfigManager::getImuGyroOffX(),
           ConfigManager::getImuGyroOffY(), ConfigManager::getImuGyroOffZ());
  uiRedrawLabel(theme::MARGIN, 126, theme::SCREEN_W - 2 * theme::MARGIN, 16, gyroBuf, sizeof(gyroBuf),
                buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
  snprintf(buf, sizeof(buf), "аксель: %d %d %d", ConfigManager::getImuAccelOffX(),
           ConfigManager::getImuAccelOffY(), ConfigManager::getImuAccelOffZ());
  uiRedrawLabel(theme::MARGIN, 144, theme::SCREEN_W - 2 * theme::MARGIN, 16, accelBuf,
                sizeof(accelBuf), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
  snprintf(buf, sizeof(buf), "Последняя калибровка: %lu с назад",
           (unsigned long)(imuCalibLastRun ? (millis() - imuCalibLastRun) / 1000 : 0));
  uiRedrawLabel(theme::MARGIN, 170, theme::SCREEN_W - 2 * theme::MARGIN, 16, ageBuf, sizeof(ageBuf),
                buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
}

static void drawMpuDiagScreen() {
  static char line[8][56] = {};
  static bool chrome = false;
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);

  if (mpuDiagUiFullRedraw || !chrome) {
    tft.fillScreen(theme::BG);
    ui.box(theme::MARGIN, 4, theme::SCREEN_W - 2 * theme::MARGIN, 18, "ДИАГНОСТИКА MPU6050",
           UiHAlign::Center, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 116, theme::SCREEN_W - 2 * theme::MARGIN, 16, "Сырые данные getMotion6:",
           UiHAlign::Left, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 214, theme::SCREEN_W - 2 * theme::MARGIN, 16, "КН4 - ВЫХОД",
           UiHAlign::Center, UiVAlign::Middle, false);
    for (uint8_t i = 0; i < 8; i++) line[i][0] = '\0';
    chrome = true;
    mpuDiagUiFullRedraw = false;
  }

  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  int deviceId = -1;
  float temp = 0.0f;
  bool dmpOn = false;
  if (mpuOk) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(50));
    if (i2c) {
      deviceId = mpu.getDeviceID();
      temp = mpu.getTemperature() / 340.0f + 36.53f;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      dmpOn = mpu.getDMPEnabled();
    }
  }

  char buf[56];
  const int16_t ys[8] = { 26, 44, 62, 80, 98, 132, 148, 164 };
  snprintf(buf, sizeof(buf), "ID: 0x%02X   MPU: %s   DMP: %s", deviceId, mpuOk ? "OK" : "ОШИБКА",
           (mpuOk && dmpOn) ? "готов" : "нет");
  uiRedrawLabel(theme::MARGIN, ys[0], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[0],
                sizeof(line[0]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Темп: %.1f C  det:%d  poll:%dms", temp, ConfigManager::getImuMotionDet(),
           ConfigManager::getImuPollMs());
  uiRedrawLabel(theme::MARGIN, ys[1], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[1],
                sizeof(line[1]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Калман: mea=%.1f est=%.1f q=%.3f", ConfigManager::getImuKalmanMea(),
           ConfigManager::getImuKalmanEst(), ConfigManager::getImuKalmanQ());
  uiRedrawLabel(theme::MARGIN, ys[2], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[2],
                sizeof(line[2]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Углы (с нулём): X=%.2f  Y=%.2f", angleX, angleY);
  uiRedrawLabel(theme::MARGIN, ys[3], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[3],
                sizeof(line[3]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Нуль конфига: X=%.2f  Y=%.2f", ConfigManager::getZeroAngleX(),
           ConfigManager::getZeroAngleY());
  uiRedrawLabel(theme::MARGIN, ys[4], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[4],
                sizeof(line[4]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "ax=%d ay=%d az=%d", ax, ay, az);
  uiRedrawLabel(theme::MARGIN, ys[5], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[5],
                sizeof(line[5]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "gx=%d gy=%d gz=%d", gx, gy, gz);
  uiRedrawLabel(theme::MARGIN, ys[6], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[6],
                sizeof(line[6]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Офсеты гиро: %d %d %d", ConfigManager::getImuGyroOffX(),
           ConfigManager::getImuGyroOffY(), ConfigManager::getImuGyroOffZ());
  uiRedrawLabel(theme::MARGIN, ys[7], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[7],
                sizeof(line[7]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  // аксель офсеты — отдельная строка 180
  static char accelOff[56] = "";
  snprintf(buf, sizeof(buf), "Офсеты аксель: %d %d %d", ConfigManager::getImuAccelOffX(),
           ConfigManager::getImuAccelOffY(), ConfigManager::getImuAccelOffZ());
  uiRedrawLabel(theme::MARGIN, 180, theme::SCREEN_W - 2 * theme::MARGIN, 16, accelOff,
                sizeof(accelOff), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);

  static char sys1[56] = "";
  static char sys2[56] = "";
  const uint32_t now = millis();
  snprintf(buf, sizeof(buf), "Клц: %lu/%lu/%lu/%lu  heap:%u",
           (unsigned long)valveCycleCount[0], (unsigned long)valveCycleCount[1],
           (unsigned long)valveCycleCount[2], (unsigned long)valveCycleCount[3],
           (unsigned)ESP.getFreeHeap());
  uiRedrawLabel(theme::MARGIN, 196, theme::SCREEN_W - 2 * theme::MARGIN, 14, sys1, sizeof(sys1), buf,
                theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Pвозраст с: %lu/%lu/%lu/%lu  МП:%lu",
           pressureValid[0] ? (unsigned long)((now - pressureStampMs[0]) / 1000) : 999UL,
           pressureValid[1] ? (unsigned long)((now - pressureStampMs[1]) / 1000) : 999UL,
           pressureValid[2] ? (unsigned long)((now - pressureStampMs[2]) / 1000) : 999UL,
           pressureValid[3] ? (unsigned long)((now - pressureStampMs[3]) / 1000) : 999UL,
           masterValid ? (unsigned long)((now - masterStampMs) / 1000) : 999UL);
  uiRedrawLabel(theme::MARGIN, 210, theme::SCREEN_W - 2 * theme::MARGIN, 14, sys2, sizeof(sys2), buf,
                theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
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

/* ============================================================================
   8.9.0: ЭКРАН ДВИЖЕНИЯ — «живой» экран режима MOVEMENT.
   Показывает: цели/факт по осям, карточки 4 подушек с метками целей, таймеры
   (в движении / стабилизация / следующая проверка / последняя корректировка),
   активные клапаны с анимацией потока и тренд давлений за 30 с.
   Данные только ЧИТАЮТСЯ — логика движения и выравнивания не изменяется.
   ============================================================================ */
constexpr uint8_t MV_TREND_N = 60;              // 60 сэмплов по 500 мс = 30 с
constexpr uint32_t MV_TREND_DT_MS = 500;        // период записи сэмпла тренда
constexpr uint32_t MV_ACTIVE_WINDOW_MS = 4000;  // окно «активности» клапана (для анимации)
constexpr int16_t MV_X = theme::MARGIN;         // 6
constexpr int16_t MV_CARD_W = 152;              // ширина карточки подушки
constexpr int16_t MV_CARD_H = 40;               // высота карточки подушки

static float mvTrendF[MV_TREND_N];  // тренд среднего по передним подушкам
static float mvTrendR[MV_TREND_N];  // тренд среднего по задним подушкам
static uint8_t mvTrendIdx = 0;
static uint8_t mvTrendCount = 0;
static uint32_t mvTrendLast = 0;

static float mvPadDisp[PAD_COUNT] = { 0, 0, 0, 0 };  // сглаженные (отображаемые) значения
static float mvFrontDisp = 0, mvRearDisp = 0, mvMasterDisp = 0, mvTempDisp = 0;

/** Экспоненциальное сглаживание — плавное «догоняние» значения. */
static inline float mvEase(float cur, float target, float k) {
  return cur + (target - cur) * k;
}

/** Снимок данных для экрана движения (состояние читается под мьютексом). */
struct MvData {
  float pad[PAD_COUNT];
  float master;
  float temp;
  float frontAvg, rearAvg;
  float targetFront, targetRear;
  float tol;
  uint32_t inMotionMs;
  uint32_t settleLeftMs, settleTotalMs;
  uint32_t nextCheckMs;
  uint32_t lastAdjFrontMs, lastAdjRearMs;  // 0 — корректировок ещё не было
  int8_t dirFront, dirRear;
  bool activeFront, activeRear;
};

static void mvReadData(MvData &d) {
  d.targetFront = ConfigManager::getMovementPressureFront();
  d.targetRear = ConfigManager::getMovementPressureRear();
  d.tol = ConfigManager::getMovementTolerance();
  d.master = 0.0f;
  d.temp = 0.0f;
  for (uint8_t i = 0; i < PAD_COUNT; i++) d.pad[i] = 0.0f;
  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      for (uint8_t i = 0; i < PAD_COUNT; i++) d.pad[i] = pressure[i];
      d.master = masterPressure;
      d.temp = temperature;
    }
  }
  d.frontAvg = (d.pad[PAD_FRONT_LEFT] + d.pad[PAD_FRONT_RIGHT]) * 0.5f;
  d.rearAvg = (d.pad[PAD_REAR_LEFT] + d.pad[PAD_REAR_RIGHT]) * 0.5f;

  const uint32_t now = millis();
  d.inMotionMs = (movementStartMs > 0) ? (now - movementStartMs) : 0;

  d.settleTotalMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
  d.settleLeftMs = 0;
  if (movementEndTime > 0) {
    const uint32_t passed = now - movementEndTime;
    d.settleLeftMs = (passed < d.settleTotalMs) ? (d.settleTotalMs - passed) : 0;
  }

  const uint32_t checkInterval = (uint32_t)ConfigManager::getMovementCheckSec() * 1000UL;
  const uint32_t nextCheckAt = movementPressureLastCheck + checkInterval;
  d.nextCheckMs = (nextCheckAt > now) ? (nextCheckAt - now) : 0;

  d.lastAdjFrontMs = (movementLastAdjustFront > 0) ? (now - movementLastAdjustFront) : 0;
  d.lastAdjRearMs = (movementLastAdjustRear > 0) ? (now - movementLastAdjustRear) : 0;
  d.dirFront = movementLastAdjustFrontDir;
  d.dirRear = movementLastAdjustRearDir;
  d.activeFront = (movementLastAdjustFront > 0) && ((now - movementLastAdjustFront) < MV_ACTIVE_WINDOW_MS);
  d.activeRear = (movementLastAdjustRear > 0) && ((now - movementLastAdjustRear) < MV_ACTIVE_WINDOW_MS);
}

/** Полоса давления с меткой цели (вертикальная риска). */
static void mvDrawBar(int16_t x, int16_t y, int16_t w, int16_t h, float value, float scale,
                      float target, uint16_t color) {
  tft.fillRect(x, y, w, h, theme::TRACK);
  const int16_t fillW = (int16_t)((float)w * constrain(value / scale, 0.0f, 1.0f));
  if (fillW > 0) tft.fillRect(x, y, fillW, h, color);
  const int16_t tx = (int16_t)(x + (float)w * constrain(target / scale, 0.0f, 1.0f));
  tft.drawFastVLine(constrain(tx, (int16_t)(x + 1), (int16_t)(x + w - 2)), (int16_t)(y - 2),
                    (int16_t)(h + 4), theme::TEXT);
}

/** «Бегущие штрихи» вдоль полосы — индикация потока воздуха (анимация). */
static void mvDrawFlow(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint32_t phaseMs) {
  const int16_t step = 8;
  const int16_t offset = (int16_t)((phaseMs / 40U) % (uint32_t)step);
  for (int16_t px = (int16_t)(x - step + offset); px < (int16_t)(x + w); px += step) {
    const int16_t x0 = (px < x) ? x : px;
    int16_t x1 = (int16_t)(px + 3);
    if (x1 > (int16_t)(x + w - 1)) x1 = (int16_t)(x + w - 1);
    if (x1 > x0) tft.drawFastHLine(x0, (int16_t)(y + h / 2), (int16_t)(x1 - x0), color);
  }
}

/** Тренд давлений за 30 с: две линии (перед/зад) + штриховые линии целей. */
static void mvDrawTrend(int16_t x, int16_t y, int16_t w, int16_t h, float targetFront,
                        float targetRear, float scale) {
  tft.fillRect(x, y, w, h, theme::PANEL);
  tft.drawRect(x, y, w, h, theme::BORDER);
  if (mvTrendCount < 2) return;

  const int16_t inner = (int16_t)(h - 4);
  const float sp = (float)(w - 3) / (float)(MV_TREND_N - 1);
  const int16_t yTf = (int16_t)(y + h - 2 - constrain(targetFront / scale, 0.0f, 1.0f) * (float)inner);
  const int16_t yTr = (int16_t)(y + h - 2 - constrain(targetRear / scale, 0.0f, 1.0f) * (float)inner);
  for (int16_t i = 2; i < (int16_t)(w - 4); i += 6) {
    tft.drawFastHLine((int16_t)(x + i), yTf, 3, theme::TEXT_DIM);
    tft.drawFastHLine((int16_t)(x + i), yTr, 3, theme::TEXT_DIM);
  }

  int16_t pxF = -1, pyF = 0, pxR = -1, pyR = 0;
  for (uint8_t k = 0; k < mvTrendCount; k++) {
    const uint8_t idx = (uint8_t)((mvTrendIdx + MV_TREND_N - mvTrendCount + k) % MV_TREND_N);
    const int16_t px = (int16_t)(x + 2 + (float)k * sp);
    const int16_t py1 = (int16_t)(y + h - 2 - constrain(mvTrendF[idx] / scale, 0.0f, 1.0f) * (float)inner);
    const int16_t py2 = (int16_t)(y + h - 2 - constrain(mvTrendR[idx] / scale, 0.0f, 1.0f) * (float)inner);
    if (pxF >= 0) {
      tft.drawLine(pxF, pyF, px, py1, theme::ACCENT);  // передние — cyan
      tft.drawLine(pxR, pyR, px, py2, theme::OK);      // задние — зелёный
    }
    pxF = px; pyF = py1; pxR = px; pyR = py2;
  }
}

/** Живой экран режима ДВИЖЕНИЯ (вызывается из displayTask каждый кадр). */
static void displayMovementScreen() {
  static bool mvFirst = true;
  static uint32_t mvLastRender = 0;

  if (!mvScreenWasActive) {
    mvScreenWasActive = true;
    mvFirst = true;
  }

  const uint32_t now = millis();
  const uint32_t frameMs = ConfigManager::getFrameMs();
  const uint32_t interval = (frameMs < 20U) ? 20U : frameMs;
  if (!mvFirst && ((now - mvLastRender) < interval)) return;
  mvLastRender = now;

  MvData d;
  mvReadData(d);

  /* --- интерполяция значений (плавность) --- */
  const float k = 0.30f;
  if (mvFirst) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) mvPadDisp[i] = d.pad[i];
    mvFrontDisp = d.frontAvg;
    mvRearDisp = d.rearAvg;
    mvMasterDisp = d.master;
    mvTempDisp = d.temp;
  } else {
    for (uint8_t i = 0; i < PAD_COUNT; i++) mvPadDisp[i] = mvEase(mvPadDisp[i], d.pad[i], k);
    mvFrontDisp = mvEase(mvFrontDisp, d.frontAvg, k);
    mvRearDisp = mvEase(mvRearDisp, d.rearAvg, k);
    mvMasterDisp = mvEase(mvMasterDisp, d.master, k);
    mvTempDisp = mvEase(mvTempDisp, d.temp, k);
  }

  /* --- тренд: один сэмпл каждые 500 мс --- */
  static bool mvTrendNeedsRedraw = true;
  if (mvFirst || ((now - mvTrendLast) >= MV_TREND_DT_MS)) {
    mvTrendLast = now;
    mvTrendF[mvTrendIdx] = d.frontAvg;
    mvTrendR[mvTrendIdx] = d.rearAvg;
    mvTrendIdx = (uint8_t)((mvTrendIdx + 1) % MV_TREND_N);
    if (mvTrendCount < MV_TREND_N) mvTrendCount++;
    mvTrendNeedsRedraw = true;
  }

  const float pScale = (ConfigManager::getPressureMax() > 0.1f) ? ConfigManager::getPressureMax() : 8.0f;
  const float pMin = ConfigManager::getPressureMin();

  ui.setTransparent(true);

  /* --- каркас рисуем один раз при входе на экран --- */
  static char mvTimer[16] = "";
  static char mvTempMp[28] = "";
  static char mvFrontVal[12] = "";
  static char mvRearVal[12] = "";
  static char mvFrontTgt[20] = "";
  static char mvRearTgt[20] = "";
  static char mvFrontDf[12] = "";
  static char mvRearDf[12] = "";
  static char mvPadVal[PAD_COUNT][12] = {};
  static char mvPadDf[PAD_COUNT][12] = {};
  static char mvSettle[36] = "";
  static char mvCheck[28] = "";
  static char mvCorr[36] = "";
  static char mvActive[36] = "";
  static bool mvPulseOn = false;
  static float mvLastFrontBar = -999.f, mvLastRearBar = -999.f;
  static float mvLastPadBar[PAD_COUNT] = { -999, -999, -999, -999 };

  if (mvFirst) {
    tft.fillScreen(theme::BG);
    tft.drawFastHLine(0, theme::STATUS_H, theme::SCREEN_W, theme::BORDER);
    tft.fillRoundRect(MV_X, 24, theme::SCREEN_W - 2 * MV_X, 68, theme::RADIUS, theme::PANEL);
    tft.drawRoundRect(MV_X, 24, theme::SCREEN_W - 2 * MV_X, 68, theme::RADIUS, theme::BORDER);
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      const int16_t cx = (i % 2 == 0) ? MV_X : (int16_t)(theme::SCREEN_W / 2 + 1);
      const int16_t cy = (i < 2) ? 94 : 136;
      tft.fillRoundRect(cx, cy, MV_CARD_W, MV_CARD_H, theme::RADIUS, theme::PANEL);
      tft.drawRoundRect(cx, cy, MV_CARD_W, MV_CARD_H, theme::RADIUS, theme::BORDER);
      ui.setFont(UiFont::SmallB);
      ui.setColors(theme::TEXT_DIM, theme::PANEL);
      ui.box(cx + 6, cy + 2, 34, 16, padNames[i], UiHAlign::Left, UiVAlign::Middle, false);
    }
    drawIconL(4, 0, iconMovingL, theme::WARN);
    ui.setFont(UiFont::SmallB);
    ui.setColors(theme::WARN, theme::BG);
    ui.box(28, 0, 92, theme::STATUS_H, "ДВИЖЕНИЕ", UiHAlign::Left, UiVAlign::Middle, false);

    ui.setFont(UiFont::SmallB);
    ui.setColors(theme::TEXT_DIM, theme::PANEL);
    ui.box(MV_X + 4, 26, 52, 16, "ПЕРЕД", UiHAlign::Left, UiVAlign::Middle, false);
    ui.box(MV_X + 4, 58, 52, 16, "ЗАД", UiHAlign::Left, UiVAlign::Middle, false);

    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(MV_X, 178, 160, 12, "ТРЕНД 30 с:  ПЕРЕД  ЗАД", UiHAlign::Left, UiVAlign::Middle, false);

    mvTimer[0] = mvTempMp[0] = mvFrontVal[0] = mvRearVal[0] = '\0';
    mvFrontTgt[0] = mvRearTgt[0] = mvFrontDf[0] = mvRearDf[0] = '\0';
    mvSettle[0] = mvCheck[0] = mvCorr[0] = mvActive[0] = '\0';
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      mvPadVal[i][0] = '\0';
      mvPadDf[i][0] = '\0';
      mvLastPadBar[i] = -999.f;
    }
    mvLastFrontBar = mvLastRearBar = -999.f;
    mvPulseOn = false;
    mvTrendNeedsRedraw = true;
    mvFirst = false;
  }

  /* --- пульс (только точка) --- */
  const bool pulseOn = (((now / 500U) % 2U) == 0U);
  if (pulseOn != mvPulseOn) {
    mvPulseOn = pulseOn;
    tft.fillCircle(124, (int16_t)(theme::STATUS_H / 2), 3, pulseOn ? theme::WARN : theme::BG);
  }

  char buf[40];
  snprintf(buf, sizeof(buf), "%lu:%02lu", (unsigned long)(d.inMotionMs / 60000UL),
           (unsigned long)((d.inMotionMs / 1000UL) % 60UL));
  uiRedrawLabel(134, 0, 66, theme::STATUS_H, mvTimer, sizeof(mvTimer), buf, theme::TEXT, theme::BG,
                UiFont::Small, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "%.0f C  МП %.1f", mvTempDisp, mvMasterDisp);
  uiRedrawLabel(200, 0, theme::SCREEN_W - 206, theme::STATUS_H, mvTempMp, sizeof(mvTempMp), buf,
                theme::ACCENT, theme::BG, UiFont::Small, UiHAlign::Right);

  const float dF = d.frontAvg - d.targetFront;
  const float dR = d.rearAvg - d.targetRear;
  const uint16_t colF =
      (fabsf(dF) <= d.tol * 0.5f) ? theme::OK : ((fabsf(dF) <= d.tol) ? theme::WARN : theme::ERR);
  const uint16_t colR =
      (fabsf(dR) <= d.tol * 0.5f) ? theme::OK : ((fabsf(dR) <= d.tol) ? theme::WARN : theme::ERR);
  const uint16_t colRealF =
      (d.frontAvg > pScale) ? theme::ERR : ((d.frontAvg < pMin) ? theme::WARN : colF);
  const uint16_t colRealR =
      (d.rearAvg > pScale) ? theme::ERR : ((d.rearAvg < pMin) ? theme::WARN : colR);

  snprintf(buf, sizeof(buf), "%.2f", mvFrontDisp);
  uiRedrawLabel(MV_X + 56, 25, 76, 19, mvFrontVal, sizeof(mvFrontVal), buf, theme::TEXT,
                theme::PANEL, UiFont::Med, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "%.2f", mvRearDisp);
  uiRedrawLabel(MV_X + 56, 57, 76, 19, mvRearVal, sizeof(mvRearVal), buf, theme::TEXT, theme::PANEL,
                UiFont::Med, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "цель %.2f", d.targetFront);
  uiRedrawLabel(MV_X + 134, 27, 64, 16, mvFrontTgt, sizeof(mvFrontTgt), buf, theme::TEXT_DIM,
                theme::PANEL, UiFont::Small, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "цель %.2f", d.targetRear);
  uiRedrawLabel(MV_X + 134, 59, 64, 16, mvRearTgt, sizeof(mvRearTgt), buf, theme::TEXT_DIM,
                theme::PANEL, UiFont::Small, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "%+.2f", dF);
  uiRedrawLabel(theme::SCREEN_W - MV_X - 76, 27, 72, 16, mvFrontDf, sizeof(mvFrontDf), buf, colF,
                theme::PANEL, UiFont::SmallB, UiHAlign::Right);
  snprintf(buf, sizeof(buf), "%+.2f", dR);
  uiRedrawLabel(theme::SCREEN_W - MV_X - 76, 59, 72, 16, mvRearDf, sizeof(mvRearDf), buf, colR,
                theme::PANEL, UiFont::SmallB, UiHAlign::Right);

  if (fabsf(mvFrontDisp - mvLastFrontBar) >= 0.02f || d.activeFront) {
    mvLastFrontBar = mvFrontDisp;
    mvDrawBar(MV_X + 4, 46, theme::SCREEN_W - 2 * MV_X - 8, 6, mvFrontDisp, pScale, d.targetFront,
              colRealF);
    if (d.activeFront)
      mvDrawFlow(MV_X + 4, 46, theme::SCREEN_W - 2 * MV_X - 8, 6,
                 (d.dirFront >= 0) ? theme::OK : theme::WARN, now);
  }
  if (fabsf(mvRearDisp - mvLastRearBar) >= 0.02f || d.activeRear) {
    mvLastRearBar = mvRearDisp;
    mvDrawBar(MV_X + 4, 78, theme::SCREEN_W - 2 * MV_X - 8, 6, mvRearDisp, pScale, d.targetRear,
              colRealR);
    if (d.activeRear)
      mvDrawFlow(MV_X + 4, 78, theme::SCREEN_W - 2 * MV_X - 8, 6,
                 (d.dirRear >= 0) ? theme::OK : theme::WARN, now);
  }

  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    const int16_t cx = (i % 2 == 0) ? MV_X : (int16_t)(theme::SCREEN_W / 2 + 1);
    const int16_t cy = (i < 2) ? 94 : 136;
    const float target = (i < 2) ? d.targetFront : d.targetRear;
    const float diff = d.pad[i] - target;
    const uint16_t col =
        (fabsf(diff) <= d.tol * 0.5f) ? theme::OK : ((fabsf(diff) <= d.tol) ? theme::WARN : theme::ERR);
    const uint16_t barCol =
        (d.pad[i] > pScale) ? theme::ERR : ((d.pad[i] < pMin) ? theme::WARN : theme::OK);
    const bool isActive = (i < 2) ? d.activeFront : d.activeRear;

    snprintf(buf, sizeof(buf), "%.2f", mvPadDisp[i]);
    uiRedrawLabel(cx + 42, cy + 1, 66, 19, mvPadVal[i], sizeof(mvPadVal[i]), buf, theme::TEXT,
                  theme::PANEL, UiFont::Med, UiHAlign::Left);
    snprintf(buf, sizeof(buf), "%+.2f", diff);
    uiRedrawLabel(cx + MV_CARD_W - 72, cy + 3, 66, 15, mvPadDf[i], sizeof(mvPadDf[i]), buf, col,
                  theme::PANEL, UiFont::Small, UiHAlign::Right);

    if (fabsf(mvPadDisp[i] - mvLastPadBar[i]) >= 0.02f || isActive) {
      mvLastPadBar[i] = mvPadDisp[i];
      mvDrawBar(cx + 6, cy + MV_CARD_H - 9, MV_CARD_W - 12, 5, mvPadDisp[i], pScale, target, barCol);
      if (isActive) {
        const int8_t dir = (i < 2) ? d.dirFront : d.dirRear;
        mvDrawFlow(cx + 6, cy + MV_CARD_H - 9, MV_CARD_W - 12, 5, (dir >= 0) ? theme::OK : theme::WARN,
                   now);
      }
    }
  }

  if (mvTrendNeedsRedraw) {
    mvTrendNeedsRedraw = false;
    mvDrawTrend(MV_X, 192, 150, 42, d.targetFront, d.targetRear, pScale);
  }

  if (d.settleLeftMs > 0) {
    snprintf(buf, sizeof(buf), "СТАБ %lu с  (возврат)", (unsigned long)(d.settleLeftMs / 1000UL));
  } else {
    snprintf(buf, sizeof(buf), "СТАБ — (не начата)");
  }
  uiRedrawLabel(164, 178, 148, 14, mvSettle, sizeof(mvSettle), buf,
                (d.settleLeftMs > 0) ? theme::WARN : theme::TEXT_DIM, theme::BG, UiFont::Tiny,
                UiHAlign::Left);
  if (d.settleLeftMs > 0) {
    const float pr = 1.0f - ((float)d.settleLeftMs / (float)d.settleTotalMs);
    tft.fillRect(164, 192, 148, 4, theme::TRACK);
    tft.fillRect(164, 192, (int16_t)(148.0f * constrain(pr, 0.0f, 1.0f)), 4, theme::WARN);
  } else {
    tft.fillRect(164, 192, 148, 4, theme::BG);
  }

  snprintf(buf, sizeof(buf), "ПРОВЕРКА %lu:%02lu", (unsigned long)(d.nextCheckMs / 60000UL),
           (unsigned long)((d.nextCheckMs / 1000UL) % 60UL));
  uiRedrawLabel(164, 198, 148, 14, mvCheck, sizeof(mvCheck), buf, theme::ACCENT, theme::BG,
                UiFont::Small, UiHAlign::Left);

  if (d.lastAdjFrontMs == 0 && d.lastAdjRearMs == 0) {
    snprintf(buf, sizeof(buf), "КОРРЕКТИРОВОК НЕ БЫЛО");
  } else {
    snprintf(buf, sizeof(buf), "КОРР П:%luс З:%luс", (unsigned long)(d.lastAdjFrontMs / 1000UL),
             (unsigned long)(d.lastAdjRearMs / 1000UL));
  }
  uiRedrawLabel(164, 212, 148, 14, mvCorr, sizeof(mvCorr), buf, theme::TEXT_DIM, theme::BG,
                UiFont::Small, UiHAlign::Left);

  if (!d.activeFront && !d.activeRear) {
    snprintf(buf, sizeof(buf), "АКТИВНО: —");
  } else if (d.activeFront && d.activeRear) {
    snprintf(buf, sizeof(buf), "АКТИВНО: ПЕРЕД+ЗАД");
  } else if (d.activeFront) {
    snprintf(buf, sizeof(buf), "АКТИВНО: %s ПЕРЕД", (d.dirFront >= 0) ? "НАКАЧ" : "СБРОС");
  } else {
    snprintf(buf, sizeof(buf), "АКТИВНО: %s ЗАД", (d.dirRear >= 0) ? "НАКАЧ" : "СБРОС");
  }
  uiRedrawLabel(164, 226, 148, 14, mvActive, sizeof(mvActive), buf,
                (d.activeFront || d.activeRear) ? theme::OK : theme::TEXT_DIM, theme::BG,
                UiFont::Small, UiHAlign::Left);
}

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

/** После ДВИЖЕНИЕ→РУЧ: цели из «Давл.стоянки»; 0.00 = перед/зад режима движения. */
void setManualTargetsFromParkingPolicy() {
  const float parking = ConfigManager::getParkingPressureBar();
  float targets[PAD_COUNT];
  if (parking > 0.00f) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      targets[i] = parking;
    }
  } else {
    const float front = ConfigManager::getMovementPressureFront();
    const float rear = ConfigManager::getMovementPressureRear();
    targets[PAD_FRONT_LEFT] = front;
    targets[PAD_FRONT_RIGHT] = front;
    targets[PAD_REAR_LEFT] = rear;
    targets[PAD_REAR_RIGHT] = rear;
  }

  MutexGuard guard(xStateMutex);
  if (guard) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      manualTargetPressure[i] = targets[i];
      manualTargetSet[i] = true;
    }
  }
  Serial.printf("[MANUAL] Цели стоянки: FL=%.2f FR=%.2f RL=%.2f RR=%.2f (parking=%.2f)\n",
                targets[PAD_FRONT_LEFT], targets[PAD_FRONT_RIGHT],
                targets[PAD_REAR_LEFT], targets[PAD_REAR_RIGHT], parking);
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
        // Импульс = значение меню в секундах (AUTO и ручной)
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

/**
 * Детектор движения для wasMoving (MotionApps 6.12):
 *  - аппаратный MOT: INT 0x40 и/или MOT_DETECT_STATUS оси (не ZRMOT);
 *  - |gyroRMS − EMA| — изменение общей вибрации (холостой ход → baseline);
 *  - |gyroBump − EMA| — изменение кивка/крена (без yaw);
 *  - |linAccRMS − EMA| — изменение линейного ускорения.
 * Езда = сумма времени busy-импульсов ≥ durationSec; settleSec — только хвост
 * hold (не считать afterglow в duration, иначе стол → ложный MOVEMENT).
 */
bool detectMotionFromIMU(uint8_t intStatus, uint8_t motStatus, uint32_t nowMs, float gyroRms,
                         float linAccRms, float gyroBump) {
#if ENABLE_MPU6050
  static uint32_t lastActivityMs = 0;
  static float linEma = 0.0f;
  static bool linEmaInit = false;
  static float gyroEma = 0.0f;
  static bool gyroEmaInit = false;
  static float bumpEma = 0.0f;
  static bool bumpEmaInit = false;
  constexpr float kMotEmaAlpha = 0.05f;

  // 0xFC = X±/Y±/Z± в MOT_DETECT_STATUS; бит 0 = ZRMOT — не считаем «движением».
  const bool motPulse = ((intStatus & 0x40) != 0) || ((motStatus & 0xFC) != 0);
  const float gyroThr = (float)ConfigManager::getGyroThreshold() * 8.0f;
  const float bumpThr = (float)ConfigManager::getGyroBumpThreshold() * 8.0f;
  const float linAccThr = (float)ConfigManager::getAccelThreshold();
  const bool gyroValid = (gyroRms >= 0.0f) && (gyroRms <= 2000.0f);
  const bool bumpValid = (gyroBump >= 0.0f) && (gyroBump <= 2000.0f);

  float gyroDelta = -1.0f;
  bool gyroBusy = false;
  if (gyroValid) {
    if (!gyroEmaInit) {
      gyroEma = gyroRms;
      gyroEmaInit = true;
      gyroDelta = 0.0f;
    } else {
      gyroDelta = fabsf(gyroRms - gyroEma);
      gyroEma += kMotEmaAlpha * (gyroRms - gyroEma);
    }
    gyroBusy = gyroDelta >= gyroThr;
  }

  float bumpDelta = -1.0f;
  bool bumpBusy = false;
  if (bumpValid) {
    if (!bumpEmaInit) {
      bumpEma = gyroBump;
      bumpEmaInit = true;
      bumpDelta = 0.0f;
    } else {
      bumpDelta = fabsf(gyroBump - bumpEma);
      bumpEma += kMotEmaAlpha * (gyroBump - bumpEma);
    }
    bumpBusy = bumpDelta >= bumpThr;
  }

  float linDelta = -1.0f;
  bool linAccBusy = false;
  if (linAccRms >= 0.0f) {
    if (!linEmaInit) {
      linEma = linAccRms;
      linEmaInit = true;
      linDelta = 0.0f;
    } else {
      linDelta = fabsf(linAccRms - linEma);
      linEma += kMotEmaAlpha * (linAccRms - linEma);
    }
    linAccBusy = linDelta >= linAccThr;
  }

  const bool pulse = motPulse || gyroBusy || bumpBusy || linAccBusy;
  if (pulse) {
    lastActivityMs = nowMs;
  }

  g_motLastIntStatus = intStatus;
  g_motLastMotStatus = motStatus;
  g_motLastMotPulse = motPulse;
  g_motLastGyroRms = gyroRms;
  g_motLastGyroDelta = gyroDelta;
  g_motLastGyroThr = gyroThr;
  g_motLastGyroBump = gyroBump;
  g_motLastGyroBumpDelta = bumpDelta;
  g_motLastGyroBumpThr = bumpThr;
  g_motLastLinAccRaw = linAccRms;
  g_motLastLinAccRms = linDelta;
  g_motLastLinAccThr = linAccThr;
  g_motLastGyroBusy = gyroBusy;
  g_motLastGyroBumpBusy = bumpBusy;
  g_motLastLinAccBusy = linAccBusy;
  g_motLastActivityMs = lastActivityMs;
  g_motSampleCount++;
  if (motPulse) g_motMotPulseCount++;
  if (gyroBusy) g_motGyroBusyCount++;
  if (bumpBusy) g_motGyroBumpBusyCount++;
  if (linAccBusy) g_motLinAccBusyCount++;

  if (lastActivityMs == 0) {
    g_motLastHold = false;
    return false;
  }
  const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
  const bool hold = (nowMs - lastActivityMs) < settleMs;
  if (hold && !g_motLastHold) g_motHoldEdgeCount++;
  g_motLastHold = hold;
  return hold;
#else
  (void)intStatus;
  (void)motStatus;
  (void)nowMs;
  (void)gyroRms;
  (void)linAccRms;
  (void)gyroBump;
  return false;
#endif
}

/** Восстановление I2C, если ведомый держит SDA (типичный зависон MPU). */
static void i2cBusRecover() {
  Wire.end();
  pinMode(PIN_OLED_SDA, INPUT_PULLUP);
  pinMode(PIN_OLED_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_OLED_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_OLED_SCL, LOW);
    delayMicroseconds(5);
  }
  pinMode(PIN_OLED_SDA, OUTPUT);
  digitalWrite(PIN_OLED_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_OLED_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_OLED_SDA, HIGH);
  delayMicroseconds(5);
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(100000L);
  Wire.setTimeOut(50);
}

static void i2cScanLog(const char* tag) {
  uint8_t found = 0;
  Serial.printf("[I2C] scan (%s):", tag);
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (!found) Serial.print(" (пусто)");
  Serial.println();
}

void initializeDMP() {
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

/** Шкалы наклона ±10°: крен (лево/право) и тангаж (зад/перед). */
static constexpr float ATT_GAUGE_MAX_DEG = 10.0f;

/** Стереть старый текст цветом фона, затем нарисовать новый (без fillRect всей области). */
static void uiRedrawLabel(int16_t x, int16_t y, int16_t w, int16_t h, char *prev, size_t prevSz,
                          const char *next, uint16_t fg, uint16_t bg, UiFont font,
                          UiHAlign ha) {
  if (!next) next = "";
  if (prev[0] != '\0' && strcmp(prev, next) == 0) return;
  ui.setTransparent(true);
  ui.setFont(font);
  if (prev[0] != '\0') {
    ui.setColors(bg, bg);
    ui.box(x, y, w, h, prev, ha, UiVAlign::Middle, true);
  }
  if (next[0] != '\0') {
    ui.setColors(fg, bg);
    ui.box(x, y, w, h, next, ha, UiVAlign::Middle, true);
  }
  strncpy(prev, next, prevSz - 1);
  prev[prevSz - 1] = '\0';
}

static uint16_t angleFillColor(float angleDeg) {
  const float a = fabsf(angleDeg);
  if (a >= ATT_GAUGE_MAX_DEG * 0.7f) return theme::WARN;
  if (a >= 0.4f) return theme::ACCENT;
  return theme::OK;
}

/** Инкрементальная заливка шкалы от центра + число erase/redraw. */
static void updateCenteredAngleBar(int16_t x, int16_t y, int16_t w, int16_t barH, float angleDeg,
                                   const char *title, const char *leftLbl, const char *rightLbl,
                                   AngleBarState &st, bool forceChrome) {
  if (forceChrome || !st.chromeDrawn) {
    ui.setTransparent(true);
    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(x, y, w / 2, 14, title, UiHAlign::Left, UiVAlign::Middle, true);

    const int16_t labY = y + 16;
    ui.box(x, labY, 36, 14, leftLbl, UiHAlign::Left, UiVAlign::Middle, true);
    ui.box(x + w - 40, labY, 40, 14, rightLbl, UiHAlign::Right, UiVAlign::Middle, true);

    st.barX = x + 38;
    st.barW = w - 80;
    st.barY = labY + 1;
    st.barH = barH;
    st.mid = st.barX + st.barW / 2;
    st.valX = x + w / 2;
    st.valY = y;
    st.valW = w / 2;
    tft.fillRect(st.barX, st.barY, st.barW, st.barH, theme::TRACK);
    tft.drawRect(st.barX, st.barY, st.barW, st.barH, theme::BORDER);
    tft.fillRect(st.mid - 1, st.barY, 3, st.barH, theme::TEXT);
    st.side = 0;
    st.fillW = 0;
    st.valStr[0] = '\0';
    st.chromeDrawn = true;
  }

  char nextVal[12];
  snprintf(nextVal, sizeof(nextVal), "%+.1f°", angleDeg);
  const uint16_t valCol =
      (fabsf(angleDeg) >= ATT_GAUGE_MAX_DEG * 0.85f) ? theme::WARN : theme::TEXT;
  uiRedrawLabel(st.valX, st.valY, st.valW, 14, st.valStr, sizeof(st.valStr), nextVal, valCol,
                theme::BG, UiFont::Tiny, UiHAlign::Right);

  const float a = constrain(angleDeg, -ATT_GAUGE_MAX_DEG, ATT_GAUGE_MAX_DEG);
  const int16_t half = st.barW / 2;
  const int16_t newFillW = (int16_t)(fabsf(a) / ATT_GAUGE_MAX_DEG * (float)(half - 2));
  const int8_t newSide = (a < -0.05f) ? -1 : ((a > 0.05f) ? 1 : 0);
  const uint16_t newCol = angleFillColor(angleDeg);

  auto clearSide = [&](int8_t side, int16_t fw) {
    if (side < 0 && fw > 0) {
      tft.fillRect(st.mid - fw, st.barY + 2, fw, st.barH - 4, theme::TRACK);
    } else if (side > 0 && fw > 0) {
      tft.fillRect(st.mid + 1, st.barY + 2, fw, st.barH - 4, theme::TRACK);
    }
  };
  auto paintSide = [&](int8_t side, int16_t fw, uint16_t col) {
    if (side < 0 && fw > 0) {
      tft.fillRect(st.mid - fw, st.barY + 2, fw, st.barH - 4, col);
    } else if (side > 0 && fw > 0) {
      tft.fillRect(st.mid + 1, st.barY + 2, fw, st.barH - 4, col);
    }
  };

  if (newSide != st.side || newCol != st.fillCol) {
    clearSide(st.side, st.fillW);
    paintSide(newSide, newFillW, newCol);
    st.side = newSide;
    st.fillW = newFillW;
    st.fillCol = newCol;
  } else if (newFillW > st.fillW) {
    // Дорисовать прирост
    const int16_t d = newFillW - st.fillW;
    if (newSide < 0) {
      tft.fillRect(st.mid - newFillW, st.barY + 2, d, st.barH - 4, newCol);
    } else if (newSide > 0) {
      tft.fillRect(st.mid + 1 + st.fillW, st.barY + 2, d, st.barH - 4, newCol);
    }
    st.fillW = newFillW;
  } else if (newFillW < st.fillW) {
    // Стереть хвост цветом дорожки
    const int16_t d = st.fillW - newFillW;
    if (newSide < 0) {
      tft.fillRect(st.mid - st.fillW, st.barY + 2, d, st.barH - 4, theme::TRACK);
    } else if (newSide > 0) {
      tft.fillRect(st.mid + 1 + newFillW, st.barY + 2, d, st.barH - 4, theme::TRACK);
    } else {
      clearSide(st.side, st.fillW);
    }
    st.fillW = newFillW;
    st.side = newSide;
  }

  // Центральная риска поверх заливки
  tft.fillRect(st.mid - 1, st.barY, 3, st.barH, theme::TEXT);
}

static AngleBarState g_rollBar;
static AngleBarState g_pitchBar;
static bool g_gaugeFooterDrawn = false;

/** Правая колонка: две шкалы, инкрементальное обновление. */
static void drawAttitudeGauges(int16_t x, int16_t y, int16_t w, int16_t h, float angleX,
                               float angleY) {
  static float lastX = 999.0f, lastY = 999.0f;
  static uint16_t lastEpoch = 0;
  const bool force = (lastEpoch != g_displayEpoch);
  if (!force && fabsf(angleX - lastX) < 0.08f && fabsf(angleY - lastY) < 0.08f) {
    return;
  }
  if (force) {
    lastEpoch = g_displayEpoch;
    tft.fillRect(x, y, w, h, theme::BG);
    g_rollBar.chromeDrawn = false;
    g_pitchBar.chromeDrawn = false;
    g_gaugeFooterDrawn = false;
  }
  lastX = angleX;
  lastY = angleY;

  updateCenteredAngleBar(x + 2, y + 8, w - 4, 20, angleX, "КРЕН", "ЛЕВ", "ПРАВ", g_rollBar,
                         force);
  updateCenteredAngleBar(x + 2, y + 72, w - 4, 20, angleY, "ТАНГАЖ", "ЗАД", "ПЕРЕД", g_pitchBar,
                         force);

  if (!g_gaugeFooterDrawn) {
    ui.setTransparent(true);
    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(x + 2, y + h - 18, w - 4, 14, "шкала ±10°  |  от центра", UiHAlign::Center,
           UiVAlign::Middle, true);
    g_gaugeFooterDrawn = true;
  }
}

/** Живое обновление шкал наклона каждый кадр. */
static void updateMainTiltLive() {
  if (menuVisible || errorScreenBlocking || wifiSetupActive || wifiScanInProgress) return;
  if (serviceScreen != ServiceScreen::NONE) return;
  if (currentSystemMode == SystemMode::MOVEMENT) return;
  if (currentState != SystemState::RUNNING) return;
  if (otaMode || otaInProgress) return;
  if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) return;

  float localAngleX = 0, localAngleY = 0;
  bool localIsMoving = false;
  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(5));
    if (!guard) return;
    localAngleX = angleX;
    localAngleY = angleY;
    localIsMoving = isMoving;
  }

  constexpr int16_t GAUGE_X = 156;
  constexpr int16_t GAUGE_Y = 42;
  constexpr int16_t GAUGE_W = 158;
  constexpr int16_t GAUGE_H = 128;

  drawAttitudeGauges(GAUGE_X, GAUGE_Y, GAUGE_W, GAUGE_H, localAngleX, localAngleY);
  // Секунды «ПОДДЕРЖАНИЕ» / АВТО-таймер — каждый кадр, не только при displayDirty
  updateMainInfoLine(localIsMoving);
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

  float masterLow = ConfigManager::getMasterLowBar();
  if (masterLow > 0.0f && currentMaster < masterLow) {
    Serial.printf("[MOVEMENT] Низкое давление в магистрали: %.1f бар < %.1f\n",
                  currentMaster, masterLow);

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
    movementLastAdjustFront = currentTime;                              // 8.9.0: экран ДВИЖЕНИЯ
    movementLastAdjustFrontDir = (diffFront > 0) ? 1 : -1;
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
    movementLastAdjustRear = currentTime;                               // 8.9.0: экран ДВИЖЕНИЯ
    movementLastAdjustRearDir = (diffRear > 0) ? 1 : -1;
    setDisplayDirty();
    requestPressureMeasurement();
  }
}

class AutoLevelingController {
private:
  // ? ВЫНЕСЕНЫ В КОНСТАНТЫ КЛАССА
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
      Serial.printf("[AUTO] ?? Давление не изменилось! Было: %.1f, Стало: %.1f\n",
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
                                      ConfigManager::getReleaseDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 1000, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentX < -thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - стравливание левых");
              if (executeValveCommand(PAD_FRONT_LEFT, false,
                                      ConfigManager::getReleaseDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_LEFT, false,
                                         ConfigManager::getReleaseDelay() * 1000, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;

          } else if (coarseStep == 1) {
            if (currentX > thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВЛЕВО - накачка левых");
              if (executeValveCommand(PAD_FRONT_LEFT, true,
                                      ConfigManager::getInflateDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_LEFT, true,
                                         ConfigManager::getInflateDelay() * 1000, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentX < -thX * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: КРЕН ВПРАВО - накачка правых");
              if (executeValveCommand(PAD_FRONT_RIGHT, true,
                                      ConfigManager::getInflateDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 1000, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
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
            Serial.printf("[AUTO] ?? Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
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
                                      ConfigManager::getReleaseDelay() * 1000, 500)
                  && executeValveCommand(PAD_FRONT_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 1000, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentY < -thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВНИЗ - стравливание задних");
              if (executeValveCommand(PAD_REAR_LEFT, false,
                                      ConfigManager::getReleaseDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, false,
                                         ConfigManager::getReleaseDelay() * 1000, 500)) {
                coarseStep = 1;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            }
            stageStartTime = now;

          } else if (coarseStep == 1) {
            if (currentY > thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВВЕРХ - накачка задних");
              if (executeValveCommand(PAD_REAR_LEFT, true,
                                      ConfigManager::getInflateDelay() * 1000, 500)
                  && executeValveCommand(PAD_REAR_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 1000, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
                currentStage = LevelingStage::COMPLETED;
              }
            } else if (currentY < -thY * ConfigManager::getCoarseZoneRatio()) {
              Serial.println("[AUTO] Грубо: НОС ВНИЗ - накачка передних");
              if (executeValveCommand(PAD_FRONT_LEFT, true,
                                      ConfigManager::getInflateDelay() * 1000, 500)
                  && executeValveCommand(PAD_FRONT_RIGHT, true,
                                         ConfigManager::getInflateDelay() * 1000, 500)) {
                coarseStep = 0;
                currentStage = LevelingStage::WAITING_STABLE;
              } else {
                Serial.println("[AUTO] ? Ошибка выполнения команды, завершение");
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
            Serial.printf("[AUTO] ?? Положение ухудшилось! Было: %.2f, Стало: %.2f\n",
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
                                    ConfigManager::getReleaseDelay() * 500, 300)) {
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
                                  ConfigManager::getInflateDelay() * 500, 300);
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
              Serial.printf("[AUTO] ?? Выравнивание не завершено! X=%.2f, Y=%.2f (пороги: %.1f, %.1f)\n",
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
        Serial.println("[AUTO] ? Выравнивание успешно завершено!");
        break;
    }
  }

public:
  bool isBusy() const { return currentStage != LevelingStage::IDLE; }
  uint8_t fineIterations() const { return fineTuningIterations; }
  const char *stageName() const {
    switch (currentStage) {
      case LevelingStage::COARSE_ROLL: return "КРЕН";
      case LevelingStage::COARSE_PITCH: return "ТАНГ";
      case LevelingStage::FINE_TUNING: return "ТОЧНО";
      case LevelingStage::WAITING_STABLE: return "СТАБ";
      case LevelingStage::COMPLETED: return "ГОТОВО";
      default: return "ОЖИД";
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
  // Ждём ACK кусками — иначе Control молчит > TASK_WDT (импульсы клапанов до 10 с).
  uint32_t waited = 0;
  BaseType_t result = pdFALSE;
  while (waited < timeoutMs) {
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CONTROL);
    uint32_t chunk = timeoutMs - waited;
    if (chunk > 500) chunk = 500;
    result = xQueueReceive(ackQueue, &success, pdMS_TO_TICKS(chunk));
    if (result == pdTRUE) break;
    waited += chunk;
  }

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

bool initializeJhm1200() {
  MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(500));
  if (!i2c && xI2CMutex != nullptr) return false;

  if (!Jhm1200::begin(Wire)) {
    jhmReady = false;
    Serial.println("[JHM1200] Не найден на адресе 0x78");
    return false;
  }
  jhmReady = true;
  return true;
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
    Serial.println("[WiFi] SSID роутера не выбран — AP не поднимаем (только вручную из меню/OTA)");
    WiFi.mode(WIFI_STA);
    wifiConnected = false;
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
    Serial.println("[WiFi] STA не подключен — без fallback AP (ручной AP: OTA/настройки)");
    wifiConnected = false;
    WiFi.mode(WIFI_STA);  // остаёмся в STA, без softAP
  }
}

/** Запрос из меню GEM (ButtonTask) — само сканирование в otaTask. */
static void requestWiFiSetup() {
  if (wifiScanInProgress || wifiSetupRequested) return;
  wifiSetupRequested = true;
  wifiScanInProgress = true;  // сразу экран «сканирование…»
  wifiSetupActive = false;
  menuVisible = false;
  wifiUiFullRedraw = true;
  displayDirty = true;
  Serial.println("[WiFi] Запрос сканирования (otaTask)");
}

/**
 * Старт скана Wi‑Fi — только из otaTask.
 * Синхронный WiFi.scanNetworks(false) блокировал ядро → TWDT idle.
 * Async + опрос в otaTask. Перед сканом отпускаем TLS-reserve (~54K),
 * иначе NetworkEvents::postEvent / calloc AP → OOM → abort() (nothrow new
 * без исключений на ESP32 Arduino 3.x).
 */
void startWiFiSetup() {
  wifiSetupRequested = false;
  wifiScanInProgress = true;
  wifiSetupActive = false;
  menuVisible = false;
  wifi_scan_count = 0;
  wifiUiFullRedraw = true;
  displayDirty = true;

  githubOtaReleaseTlsHeap("wifi-scan");
  githubOtaDefragHeap();
  Serial.printf("[WiFi] Сканирование (async) free=%u maxBlk=%u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));

  const wifi_mode_t prevMode = WiFi.getMode();
  if (prevMode == WIFI_MODE_NULL) {
    WiFi.mode(WIFI_STA);
  } else if (prevMode == WIFI_MODE_AP) {
    WiFi.mode(WIFI_AP_STA);
  }

  // Сброс прошлого результата, иначе scanComplete() может сразу вернуть старый n.
  WiFi.scanDelete();
  const int16_t rc = WiFi.scanNetworks(/*async=*/true, /*hidden=*/true);
  if (rc == WIFI_SCAN_FAILED) {
    Serial.println("[WiFi] scan start FAILED");
    wifiScanInProgress = false;
    wifiSetupActive = false;
    menuVisible = true;
    wifiUiFullRedraw = true;
    displayDirty = true;
    githubOtaReserveTlsHeap("wifi-scan-fail");
  }
}

/** Забрать результат async-скана (otaTask). true = скан ещё идёт. */
static bool pollWiFiScanComplete() {
  if (!wifiScanInProgress) return false;

  const int16_t found = WiFi.scanComplete();
  if (found == WIFI_SCAN_RUNNING) return true;

  wifi_scan_count = 0;
  if (found > 0) {
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
  } else if (found == WIFI_SCAN_FAILED) {
    Serial.println("[WiFi] scanComplete FAILED");
  }
  WiFi.scanDelete();

  wifiScanInProgress = false;
  wifi_scan_selected = 0;
  for (uint8_t i = 0; i < wifi_scan_count; i++) {
    if (sta_ssid[0] != '\0' && strcmp(wifi_scan_ssids[i], sta_ssid) == 0) {
      wifi_scan_selected = i;
      break;
    }
  }
  wifiSetupActive = wifi_scan_count > 0;
  wifiUiFullRedraw = true;
  displayDirty = true;
  Serial.printf("[WiFi] Найдено сетей: %u (stackHWM=%u free=%u)\n", wifi_scan_count,
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                static_cast<unsigned>(ESP.getFreeHeap()));
  githubOtaReserveTlsHeap("post-wifi-scan");
  if (wifi_scan_count == 0) {
    Serial.println("[WiFi] Сети не найдены — возврат в меню");
    menuVisible = true;
    displayDirty = true;
  }
  return false;
}

/** Одна строка списка Wi-Fi (без полной перерисовки экрана). */
static void drawWiFiListRow(uint8_t i, bool selected) {
  if (i >= wifi_scan_count) return;
  constexpr int16_t LIST_Y = 66;
  constexpr int16_t ROW = 20;
  const int16_t y = LIST_Y + i * ROW;
  const int rssi = wifi_scan_rssi[i];
  const bool isCurrent = (sta_ssid[0] != '\0' && strcmp(wifi_scan_ssids[i], sta_ssid) == 0);

  if (selected) {
    tft.fillRoundRect(theme::MARGIN - 2, y, theme::SCREEN_W - 2 * theme::MARGIN + 4, ROW, 3, theme::PANEL);
  } else {
    tft.fillRect(theme::MARGIN - 2, y, theme::SCREEN_W - 2 * theme::MARGIN + 4, ROW, theme::BG);
  }

  const uint8_t bars = (rssi > -55) ? 4 : (rssi > -70) ? 3 : (rssi > -85) ? 2 : 1;
  for (uint8_t b = 0; b < 4; b++) {
    const int16_t bx = static_cast<int16_t>(theme::SCREEN_W - theme::MARGIN - 66 + b * 6);
    const int16_t bh = static_cast<int16_t>(4 + b * 3);
    const uint16_t barColor = (b < bars) ? ((bars >= 3) ? theme::OK : theme::WARN) : theme::TRACK;
    tft.fillRect(bx, static_cast<int16_t>(y + ROW - 5 - bh), 4, bh, barColor);
  }

  char nameBuf[36];
  if (isCurrent) {
    snprintf(nameBuf, sizeof(nameBuf), "*%s", wifi_scan_ssids[i]);
  } else {
    strlcpy(nameBuf, wifi_scan_ssids[i], sizeof(nameBuf));
  }

  ui.setFont(selected ? UiFont::SmallB : UiFont::Small);
  ui.setColors(selected ? theme::ACCENT : (isCurrent ? theme::OK : theme::TEXT),
               selected ? theme::PANEL : theme::BG);
  ui.box(theme::MARGIN + 2, y, 186, ROW, ui.ellipsize(nameBuf, 180),
         UiHAlign::Left, UiVAlign::Middle, true);

  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, selected ? theme::PANEL : theme::BG);
  ui.boxf(theme::SCREEN_W - theme::MARGIN - 26, y, 26, ROW, UiHAlign::Right, UiVAlign::Middle, true,
          "%d", rssi);
}

/** Нижняя строка: текущее подключение (STA / AP / нет). */
static void drawWiFiStatusBar() {
  constexpr int16_t Y = 214;
  tft.fillRect(0, Y, theme::SCREEN_W, 26, theme::BG);

  char line[64];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(line, sizeof(line), "Сейчас: %s  %d dBm  %s",
             WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
    ui.setColors(theme::OK, theme::BG);
  } else if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
    snprintf(line, sizeof(line), "Сейчас: AP %s  %s",
             wifi_ssid, WiFi.softAPIP().toString().c_str());
    ui.setColors(theme::WARN, theme::BG);
  } else if (sta_ssid[0] != '\0') {
    snprintf(line, sizeof(line), "Сейчас: нет связи (сохр. %s)", sta_ssid);
    ui.setColors(theme::ERR, theme::BG);
  } else {
    strlcpy(line, "Сейчас: не подключено", sizeof(line));
    ui.setColors(theme::TEXT_DIM, theme::BG);
  }

  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.box(theme::MARGIN, Y, theme::SCREEN_W - 2 * theme::MARGIN, 22,
         ui.ellipsize(line, theme::SCREEN_W - 2 * theme::MARGIN),
         UiHAlign::Left, UiVAlign::Middle, true);
}

void displayWiFiSetupScreen() {
  static bool frameReady = false;
  static int8_t lastSelected = -1;
  static uint32_t lastStatusMs = 0;
  static bool lastScanProgress = false;
  static char lastStatusLine[64] = "";

  // Полный кадр ТОЛЬКО по явному флагу / смене фазы сканирования —
  // НЕ по глобальному displayDirty (его ставят IMU/ошибки).
  if (wifiUiFullRedraw || lastScanProgress != wifiScanInProgress) {
    frameReady = false;
    lastSelected = -1;
    lastStatusLine[0] = '\0';
    lastScanProgress = wifiScanInProgress;
    wifiUiFullRedraw = false;
  }

  if (!frameReady) {
    tft.fillScreen(theme::BG);

    tft.fillRoundRect(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24,
                      theme::RADIUS, theme::PANEL_ALT);
    ui.setTransparent(true);
    ui.setFont(UiFont::Med);
    ui.setColors(theme::ACCENT, theme::PANEL_ALT);
    ui.box(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24, "ВЫБОР WI-FI",
           UiHAlign::Center, UiVAlign::Middle, false);

    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(theme::MARGIN, 32, 280, 14, "КН1/КН2 — выбор,  КН3 — подключить",
           UiHAlign::Left, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 46, 280, 14, "КН4 — отмена   * = сохранённая сеть",
           UiHAlign::Left, UiVAlign::Middle, false);

    if (wifiScanInProgress) {
      ui.setFont(UiFont::Small);
      ui.setColors(theme::WARN, theme::BG);
      ui.box(theme::MARGIN, 90, 250, 18, "Сканирование...", UiHAlign::Left, UiVAlign::Middle, true);
      drawWiFiStatusBar();
      lastStatusLine[0] = '\0';  // принудительно обновить после скана
      frameReady = true;
      return;
    }

    for (uint8_t i = 0; i < wifi_scan_count; i++) {
      drawWiFiListRow(i, i == wifi_scan_selected);
    }
    lastSelected = static_cast<int8_t>(wifi_scan_selected);
    drawWiFiStatusBar();
    lastStatusMs = millis();
    lastStatusLine[0] = '\0';
    frameReady = true;
    return;
  }

  // Только смена выделения — две строки
  if (!wifiScanInProgress && lastSelected != static_cast<int8_t>(wifi_scan_selected)) {
    if (lastSelected >= 0 && lastSelected < static_cast<int8_t>(wifi_scan_count)) {
      drawWiFiListRow(static_cast<uint8_t>(lastSelected), false);
    }
    drawWiFiListRow(wifi_scan_selected, true);
    lastSelected = static_cast<int8_t>(wifi_scan_selected);
  }

  // Статус — не чаще 1 с и только если текст изменился
  if (millis() - lastStatusMs >= 1000) {
    char line[64];
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(line, sizeof(line), "Сейчас: %s  %d dBm  %s",
               WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
    } else if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
      snprintf(line, sizeof(line), "Сейчас: AP %s  %s",
               wifi_ssid, WiFi.softAPIP().toString().c_str());
    } else if (sta_ssid[0] != '\0') {
      snprintf(line, sizeof(line), "Сейчас: нет связи (сохр. %s)", sta_ssid);
    } else {
      strlcpy(line, "Сейчас: не подключено", sizeof(line));
    }
    if (strcmp(line, lastStatusLine) != 0) {
      strlcpy(lastStatusLine, line, sizeof(lastStatusLine));
      drawWiFiStatusBar();
    }
    lastStatusMs = millis();
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

/* ====================  Чтение давления через JHM1200 ==================== */
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

  if (!jhmReady || !Jhm1200::isReady()) {
    return 0.0f;
  }

  static uint32_t lastReadTime = 0;
  static float lastGoodBar = 0.0f;
  static bool haveLast = false;
  const uint32_t now = millis();
  // Минимальный интервал ? settle датчика; при частых вызовах отдаём кэш.
  if (now - lastReadTime < 12) {
    return haveLast ? lastGoodBar : -1.0f;
  }
  lastReadTime = now;

  float bar = 0.0f;
  bool ok = false;
  {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(150));
    if (!i2c) return haveLast ? lastGoodBar : -1.0f;
    ok = Jhm1200::readBar(bar);
  }

  static uint8_t consecutiveFails = 0;
  if (!ok) {
    consecutiveFails++;
    // Несколько фейлов подряд (I2C/MPU) — ещё не SENSOR; порог выше ложных срабатываний.
    if (consecutiveFails < 8) {
      return haveLast ? lastGoodBar : -1.0f;
    }
    consecutiveFails = 0;
    static uint32_t lastErrorTime = 0;
    if (millis() - lastErrorTime > 5000) {
      lastErrorTime = millis();
      Serial.println("[WARN] JHM1200: ошибка чтения!");
    }
    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "JHM1200 - ошибка чтения");
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::SENSOR);
    }
    return 0.0f;
  }
  consecutiveFails = 0;

  if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) && calibrationCompleted) {
    ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
  }

  float localZeroBar = 0.0f;
  {
    MutexGuard guard(xCalibMutex);
    if (guard) {
      localZeroBar = g_pressureZeroBar;
    }
  }
  bar -= localZeroBar;
  if (bar < 0.02f) bar = 0.0f;
  if (bar > 10.0f) bar = 10.0f;

  // При работе клапанов — почти сырое значение; в простое — лёгкое EMA.
  const bool fastPath = manualControlActive || autoLevelingController.isBusy();
  if (!haveLast) {
    lastGoodBar = bar;
    haveLast = true;
  } else if (fastPath) {
    lastGoodBar = 0.35f * lastGoodBar + 0.65f * bar;
  } else {
    const float delta = bar - lastGoodBar;
    if (fabsf(delta) > 0.8f) {
      bar = lastGoodBar + (delta > 0 ? 0.8f : -0.8f);
    }
    lastGoodBar = 0.55f * lastGoodBar + 0.45f * bar;
  }
  return lastGoodBar;
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

void displayErrorScreen() {
  uint32_t now = millis();

  if (!ErrorHandler::hasUiBlockingErrors()) {
    errorScreenBlocking = false;
    displayDirty = true;
    return;
  }

  int totalActiveErrors = ErrorHandler::getActiveErrorCount();
  errorScreenBlocking = true;

  ErrorHandler::Error currentErr = ErrorHandler::getCurrentActiveError();
  const int currentIndex = ErrorHandler::getCurrentDisplayIndex();

  static bool chromeDrawn = false;
  static ErrorHandler::Error lastDisplayedError = ErrorHandler::Error::NONE;
  static int lastTotalErrors = -1;
  static int lastDisplayIndex = -1;

  if (forceErrorScreenRedraw) {
    forceErrorScreenRedraw = false;
    chromeDrawn = false;
    Serial.println("[DISPLAY] Принудительная перерисовка после сброса");
  }

  const int16_t frameX = 10;
  const int16_t frameY = 10;
  const int16_t frameW = SCREEN_WIDTH - 20;
  const int16_t frameH = SCREEN_HEIGHT - 20;
  const int16_t iconX = (SCREEN_WIDTH - 45) / 2;
  const int16_t iconY = 38;
  const int16_t textPad = 16;
  const int16_t lineH = 26;
  // Зона текста + счётчика (рамку и подсказки не трогаем при смене ошибки)
  const int16_t contentY = 16;
  const int16_t contentH = frameY + frameH - 36 - contentY;

  auto resolveErrorLines = [](ErrorHandler::Error err, const char *&line1, const char *&line2) {
    line1 = nullptr;
    line2 = nullptr;
    switch (err) {
      case ErrorHandler::Error::LOW_PRESSURE:
        line1 = "НИЗКОЕ ДАВЛЕНИЕ";
        line2 = "В МАГИСТРАЛИ";
        break;
      case ErrorHandler::Error::MPU:
        line1 = "ОШИБКА MPU6050";
        line2 = "НЕТ ОТВЕТА I2C";
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
        line1 = ErrorHandler::getErrorMessage(err);
        break;
    }
  };

  auto drawErrorContent = [&](ErrorHandler::Error err, int index, int total, bool clearBg) {
    const char *line1 = nullptr;
    const char *line2 = nullptr;
    resolveErrorLines(err, line1, line2);

    if (clearBg) {
      tft.fillRect(frameX + 2, contentY, frameW - 4, contentH, COLOR_ERROR);
    }

    drawIconB(iconX, iconY, err_Big, theme::TEXT);

    ui.setTransparent(true);
    ui.setColors(theme::TEXT, COLOR_ERROR);
    ui.setFont(UiFont::Med);
    const int16_t textTop = (line2 != nullptr) ? 92 : 108;
    ui.box(frameX + textPad, textTop, frameW - 2 * textPad, lineH, line1,
           UiHAlign::Center, UiVAlign::Middle, true);
    if (line2 != nullptr) {
      ui.box(frameX + textPad, textTop + lineH + 2, frameW - 2 * textPad, lineH, line2,
             UiHAlign::Center, UiVAlign::Middle, true);
    }

    char indicatorStr[12];
    snprintf(indicatorStr, sizeof(indicatorStr), "%d/%d", index + 1, total);
    ui.setFont(UiFont::BodyWide);
    ui.setColors(theme::TEXT, COLOR_ERROR);
    ui.box(frameX + frameW - 78, 18, 66, 20, indicatorStr, UiHAlign::Right, UiVAlign::Middle,
           true);
  };

  if (!chromeDrawn) {
    Serial.printf("[DISPLAY] Полный экран ошибки. Код: %d\n", (int)currentErr);
    tft.fillScreen(COLOR_ERROR);
    drawErrorContent(currentErr, currentIndex, totalActiveErrors, false);

    ui.setTransparent(true);
    ui.setFont(UiFont::Small);
    ui.setColors(theme::TEXT, COLOR_ERROR);
    ui.box(frameX + 8, frameY + frameH - 28, 120, 18, "КН3-СБРОС", UiHAlign::Left,
           UiVAlign::Middle, true);
    ui.box(frameX + frameW - 148, frameY + frameH - 28, 140, 18, "КН3+КН4-МЕНЮ",
           UiHAlign::Right, UiVAlign::Middle, true);

    tft.drawRect(frameX, frameY, frameW, frameH, theme::TEXT);
    tft.drawRect(frameX + 1, frameY + 1, frameW - 2, frameH - 2, theme::TEXT);

    chromeDrawn = true;
    lastDisplayedError = currentErr;
    lastTotalErrors = totalActiveErrors;
    lastDisplayIndex = currentIndex;
    Serial.println("[DISPLAY] Экран ошибки нарисован");
  } else if (lastDisplayedError != currentErr || lastTotalErrors != totalActiveErrors ||
             lastDisplayIndex != currentIndex) {
    // Смена ошибки / счётчика: только контент, рамка и подсказки КН без перерисовки
    Serial.printf("[DISPLAY] Контент ошибки %d>%d (%d/%d)\n", (int)lastDisplayedError,
                  (int)currentErr, currentIndex + 1, totalActiveErrors);
    drawErrorContent(currentErr, currentIndex, totalActiveErrors, true);
    lastDisplayedError = currentErr;
    lastTotalErrors = totalActiveErrors;
    lastDisplayIndex = currentIndex;
  }

  static uint32_t lastBlinkTime = 0;
  static bool blinkState = true;

  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    blinkState = !blinkState;

    if (blinkState) {
      drawIconB(iconX, iconY, err_Big, theme::TEXT);
    } else {
      tft.fillRect(iconX, iconY, 45, 45, COLOR_ERROR);
    }
    // Рамку не трогаем — иконка ниже верхнего края, мигание не должно дёргать chrome
  }
}

void displayCalibrationScreen() {
  // ? ЕСЛИ КАЛИБРОВКА ЗАВЕРШЕНА - ВЫХОДИМ
  if (calibrationCompleted) {
    return;
  }

  if (errorScreenBlocking || ErrorHandler::hasUiBlockingErrors()) {
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

/** Монитор утечек: при простое сравнивает падение давления за 10–15 мин. */
static void updateLeakMonitor() {
  static float baseline[PAD_COUNT];
  static uint32_t baselineMs = 0;
  static bool haveBaseline = false;

  if (manualControlActive || autoLevelingController.isBusy() ||
      currentSystemMode == SystemMode::MOVEMENT || otaInProgress ||
      ErrorHandler::hasCriticalPneumaticErrors()) {
    haveBaseline = false;
    return;
  }

  const uint32_t now = millis();
  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    if (!pressureValid[i] || (now - pressureStampMs[i]) > 60000UL) {
      haveBaseline = false;
      return;
    }
  }

  if (!haveBaseline) {
    memcpy(baseline, pressure, sizeof(baseline));
    baselineMs = now;
    haveBaseline = true;
    return;
  }

  if (now - baselineMs < 600000UL) return;  // 10 мин

  float worstDrop = 0.0f;
  int worstPad = -1;
  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    const float drop = baseline[i] - pressure[i];
    if (drop > worstDrop) {
      worstDrop = drop;
      worstPad = (int)i;
    }
  }
  if (worstPad >= 0 && worstDrop >= 0.35f) {
    leakSuspect = true;
    strlcpy(leakSuspectPad, padNames[worstPad], sizeof(leakSuspectPad));
    Serial.printf("[LEAK] Подозрение: %s падение %.2f бар за %lu с\n", leakSuspectPad, worstDrop,
                  (unsigned long)((now - baselineMs) / 1000));
  } else {
    leakSuspect = false;
    leakSuspectPad[0] = '\0';
  }
  haveBaseline = false;  // новый цикл наблюдения
}

/** СТАТ + иконка «авто стоит» (iconAutoStopL). Мигает, пока isMoving,
 *  но режим ещё не MOVEMENT (набор длительности до входа в ДВИЖЕНИЕ). */
static void updateMainStatBadge() {
  constexpr int16_t ICON_X = 58;
  constexpr int16_t ICON_Y = 0;
  constexpr int16_t TEXT_X = 80;
  constexpr int16_t TEXT_W = 40;

  static char stStat[12] = "";
  static bool iconDrawn = false;
  static bool lastVisible = true;
  static uint16_t lastCol = 0xFFFF;
  static uint16_t epoch = 0;

  if (epoch != g_displayEpoch) {
    stStat[0] = '\0';
    iconDrawn = false;
    lastVisible = true;
    lastCol = 0xFFFF;
    epoch = g_displayEpoch;
  }

  const bool inMovement = (currentSystemMode == SystemMode::MOVEMENT);
  // Детекция движения до перехода в режим ДВИЖЕНИЕ
  const bool pendingMotion = isMoving && !inMovement;
  const bool blinkOn = !pendingMotion || (((millis() / 400U) & 1U) != 0);

  const char *label = inMovement ? "ДВИЖ" : "СТАТ";
  const uint16_t col = inMovement ? theme::WARN : (pendingMotion ? theme::WARN : theme::OK);

  if (blinkOn != lastVisible || col != lastCol || stStat[0] == '\0') {
    const bool colorChanged = (col != lastCol);
    lastVisible = blinkOn;
    lastCol = col;
    if (!blinkOn) {
      tft.fillRect(ICON_X, ICON_Y, 20, theme::STATUS_H, theme::BG);
      iconDrawn = false;
      uiRedrawLabel(TEXT_X, 0, TEXT_W, theme::STATUS_H, stStat, sizeof(stStat), "",
                    col, theme::BG, UiFont::Small, UiHAlign::Left);
    } else {
      if (!iconDrawn || colorChanged) {
        tft.fillRect(ICON_X, ICON_Y, 20, theme::STATUS_H, theme::BG);
        drawIconL(ICON_X, ICON_Y, iconAutoStopL, col);
        iconDrawn = true;
      }
      if (colorChanged || (stStat[0] != '\0' && strcmp(stStat, label) != 0)) {
        stStat[0] = '\0';
      }
      uiRedrawLabel(TEXT_X, 0, TEXT_W, theme::STATUS_H, stStat, sizeof(stStat), label,
                    col, theme::BG, UiFont::Small, UiHAlign::Left);
    }
  } else if (blinkOn && !iconDrawn) {
    drawIconL(ICON_X, ICON_Y, iconAutoStopL, col);
    iconDrawn = true;
  }
}

/** Инфо-строка рабочего экрана: префикс (раз) + секунды (по тику). */
static void updateMainInfoLine(bool isMoving) {
  constexpr int16_t INFO_X = theme::MARGIN;
  constexpr int16_t INFO_Y = 200;
  constexpr int16_t INFO_H = 16;
  constexpr int16_t INFO_W = theme::SCREEN_W - 2 * theme::MARGIN;

  enum class Kind : uint8_t { Empty, Full, Maintain, Return };
  static Kind prevKind = Kind::Empty;
  static char prefixPrev[36] = "";
  static char secPrev[16] = "";
  static char fullPrev[64] = "";
  static uint16_t epoch = 0;

  if (epoch != g_displayEpoch) {
    prefixPrev[0] = secPrev[0] = fullPrev[0] = '\0';
    prevKind = Kind::Empty;
    epoch = g_displayEpoch;
  }

  Kind kind = Kind::Empty;
  char fullMsg[64] = "";
  const char *prefix = "";
  char secBuf[16] = "";

  if (manualControlActive && currentSystemMode != SystemMode::MOVEMENT) {
    kind = Kind::Full;
    snprintf(fullMsg, sizeof(fullMsg), "РУЧ: %s %s", padNames[manualPadIndex],
             manualInflate ? "НАКАЧ" : "СБРОС");
  } else if (currentSystemMode == SystemMode::MOVEMENT && movementEndTime > 0) {
    const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
    const uint32_t remaining = settleMs - (millis() - movementEndTime);
    if (remaining < settleMs) {
      kind = Kind::Return;
      prefix = "ВОЗВРАТ:";
      snprintf(secBuf, sizeof(secBuf), " %lu с", (unsigned long)(remaining / 1000));
    }
  } else if (currentSystemMode == SystemMode::AUTO && !isMoving) {
    kind = Kind::Full;
    if (autoLevelingController.isBusy()) {
      snprintf(fullMsg, sizeof(fullMsg), "АВТО:%s  ПОП:%d/%d",
               autoLevelingController.stageName(), levelingAttemptsThisHour,
               ConfigManager::getNivCount());
    } else {
      const uint32_t nextCheck =
          (lastLevelingCheckTime + ConfigManager::getTimeInterval() * 60000UL - millis()) / 1000;
      if (nextCheck < 600) {
        snprintf(fullMsg, sizeof(fullMsg), "ПОП: %d/%d  СЛЕД: %luс", levelingAttemptsThisHour,
                 ConfigManager::getNivCount(), (unsigned long)nextCheck);
      } else {
        snprintf(fullMsg, sizeof(fullMsg), "ПОП: %d/%d", levelingAttemptsThisHour,
                 ConfigManager::getNivCount());
      }
    }
  } else if (leakSuspect) {
    kind = Kind::Full;
    snprintf(fullMsg, sizeof(fullMsg), "УТЕЧКА? %s", leakSuspectPad);
  } else if (currentSystemMode == SystemMode::MANUAL && !manualControlActive) {
    kind = Kind::Maintain;
    prefix = "ПОДДЕРЖАНИЕ:";
    const uint32_t secLeft =
        (MANUAL_PRESSURE_CHECK_INTERVAL_MS - (millis() % MANUAL_PRESSURE_CHECK_INTERVAL_MS)) / 1000;
    snprintf(secBuf, sizeof(secBuf), " %lu с", (unsigned long)secLeft);
  }

  if (kind != prevKind) {
    tft.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, theme::BG);
    prefixPrev[0] = secPrev[0] = fullPrev[0] = '\0';
    prevKind = kind;
  }
  if (kind == Kind::Empty) return;

  if (kind == Kind::Full) {
    uiRedrawLabel(INFO_X, INFO_Y, INFO_W, INFO_H, fullPrev, sizeof(fullPrev), fullMsg,
                  theme::ACCENT, theme::BG, UiFont::Small, UiHAlign::Left);
    return;
  }

  // Maintain / Return: статичный префикс + только меняющиеся секунды
  ui.setFont(UiFont::Small);
  const int16_t prefixW = (int16_t)(ui.width(prefix) + 4);
  uiRedrawLabel(INFO_X, INFO_Y, prefixW, INFO_H, prefixPrev, sizeof(prefixPrev), prefix,
                theme::ACCENT, theme::BG, UiFont::Small, UiHAlign::Left);
  uiRedrawLabel(INFO_X + prefixW, INFO_Y, (int16_t)(INFO_W - prefixW), INFO_H, secPrev,
                sizeof(secPrev), secBuf, theme::ACCENT, theme::BG, UiFont::Small, UiHAlign::Left);
}

void displayMainScreen() {
  if (menuVisible) return;  // меню — отдельный путь отрисовки
  static uint32_t lastRenderTime = 0;
  uint32_t now = millis();

  if (errorScreenBlocking) return;

  static bool firstRun = true;
  static uint16_t lastEpoch = 0;
  // С экрана ДВИЖЕНИЯ: принудительно полный каркас (даже если epoch не сдвинули).
  if (mvScreenWasActive) {
    mvScreenWasActive = false;
    firstRun = true;
  }
  if (lastEpoch != g_displayEpoch) {
    lastEpoch = g_displayEpoch;
    firstRun = true;  // перерисовать каркас после fillScreen (выход из меню и т.п.)
  }

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

  if (firstRun) {
    // После сброса экрана всегда полная отрисовка, даже если углы/давления
    // совпали с кэшем — иначе остаётся чёрный fillScreen.
    lastState.angleX = 999.0f;
    lastState.angleY = 999.0f;
    lastState.temp = 999.0f;
    lastState.masterPressure = 999.0f;
    lastState.isMoving = !localIsMoving;
    lastSystemMode = (SystemMode)255;
  }

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

  if (!needRedraw) {
    // Секунды инфо-строки тикают и без перерисовки давлений/углов
    updateMainInfoLine(localIsMoving);
    updateMainStatBadge();  // мигание СТАТ при pending motion
    return;
  }

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
  constexpr int16_t HINT_Y = 220;
  constexpr int16_t GAUGE_X = 156;
  constexpr int16_t GAUGE_Y = 42;
  constexpr int16_t GAUGE_W = 158;
  constexpr int16_t GAUGE_H = 128;

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
      ui.setFont(UiFont::SmallB);
      ui.setColors(theme::TEXT_DIM, theme::PANEL);
      ui.box(CARD_X + 6, y + 2, 34, 18, padNames[i], UiHAlign::Left, UiVAlign::Middle, false);
    }

    tft.fillRoundRect(CARD_X, MASTER_Y, CARD_W, MASTER_H, theme::RADIUS, theme::PANEL_ALT);
    tft.drawRoundRect(CARD_X, MASTER_Y, CARD_W, MASTER_H, theme::RADIUS, theme::BORDER);
    ui.setFont(UiFont::SmallB);
    ui.setColors(theme::TEXT_DIM, theme::PANEL_ALT);
    ui.box(CARD_X + 6, MASTER_Y + 3, 34, 18, "МП", UiHAlign::Left, UiVAlign::Middle, false);

    ui.setFont(UiFont::Small);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(CARD_X, ANGLES_Y, CARD_W, 18, "НАКЛОН >", UiHAlign::Left, UiVAlign::Middle, false);

    firstRun = false;
  }

  /* --------------------------- статус-строка -------------------------- */
  static char stTemp[12] = "";
  static char stMode[16] = "";
  static char stWifi[10] = "";
  static float stTempDrawn = 999.0f;
  static uint16_t stEpoch = 0;
  if (stEpoch != g_displayEpoch) {
    stTemp[0] = stMode[0] = stWifi[0] = '\0';
    stTempDrawn = 999.0f;
    stEpoch = g_displayEpoch;
  }
  // Температуру в статус не дёргаем чаще 0.5°C (даже если кадр перерисован из?за углов).
  if (fabsf(localTemp - stTempDrawn) >= 0.5f || stTemp[0] == '\0') {
    char tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "T:%4.1f", localTemp);
    uiRedrawLabel(theme::MARGIN, 0, 50, theme::STATUS_H, stTemp, sizeof(stTemp), tbuf,
                  theme::TEXT, theme::BG, UiFont::Small, UiHAlign::Left);
    stTempDrawn = localTemp;
  }

  updateMainStatBadge();

  const char *wifiStr = "нет";
  uint16_t wifiCol = theme::TEXT_DIM;
  if (WiFi.status() == WL_CONNECTED) {
    wifiStr = "WiFi";
    wifiCol = theme::OK;
  } else {
    const wifi_mode_t wm = WiFi.getMode();
    if (wm == WIFI_AP || wm == WIFI_AP_STA) {
      wifiStr = "AP";
      wifiCol = theme::WARN;
    }
  }
  uiRedrawLabel(122, 0, 42, theme::STATUS_H, stWifi, sizeof(stWifi), wifiStr, wifiCol, theme::BG,
                UiFont::Small, UiHAlign::Center);

  const char *modeStr = "РУЧ";
  uint16_t modeColor = theme::ACCENT;
  if (currentSystemMode == SystemMode::AUTO) {
    modeStr = "АВТО";
    modeColor = theme::OK;
  } else if (currentSystemMode == SystemMode::MOVEMENT) {
    modeStr = "ДВИЖ";
    modeColor = theme::WARN;
  }
  uiRedrawLabel(168, 0, theme::SCREEN_W - 168 - theme::MARGIN, theme::STATUS_H, stMode,
                sizeof(stMode), modeStr, modeColor, theme::BG, UiFont::Small, UiHAlign::Right);

  /* ----------------------- давление в подушках ------------------------- */
  static char padValStr[PAD_COUNT][10] = {};
  static int16_t padFillW[PAD_COUNT] = { -1, -1, -1, -1 };
  static uint16_t padFillCol[PAD_COUNT] = {};
  static uint16_t padEpoch = 0;
  if (padEpoch != g_displayEpoch) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      padValStr[i][0] = '\0';
      padFillW[i] = -1;
    }
    padEpoch = g_displayEpoch;
  }

  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    const int16_t y = CARD_Y0 + i * CARD_DY;
    const float v = localPressure[i];

    char vbuf[10];
    snprintf(vbuf, sizeof(vbuf), "%.1f", v);
    uiRedrawLabel(CARD_X + CARD_W - 106, y + 1, 100, 22, padValStr[i], sizeof(padValStr[i]),
                  vbuf, theme::TEXT, theme::PANEL, UiFont::Med, UiHAlign::Right);

    const int16_t barX = CARD_X + 6;
    const int16_t barY = y + CARD_H - 7;
    const int16_t barW = CARD_W - 12;
    const float ratio = constrain(v / pScale, 0.0f, 1.0f);
    const int16_t fillW = static_cast<int16_t>(barW * ratio);
    uint16_t barColor = theme::OK;
    if (v > pMaxAbs) {
      barColor = theme::ERR;
    } else if (v < pMin) {
      barColor = theme::WARN;
    }

    if (padFillW[i] < 0 || barColor != padFillCol[i]) {
      tft.fillRect(barX, barY, barW, 4, theme::TRACK);
      if (fillW > 0) tft.fillRect(barX, barY, fillW, 4, barColor);
      padFillW[i] = fillW;
      padFillCol[i] = barColor;
    } else if (fillW > padFillW[i]) {
      tft.fillRect(barX + padFillW[i], barY, fillW - padFillW[i], 4, barColor);
      padFillW[i] = fillW;
    } else if (fillW < padFillW[i]) {
      tft.fillRect(barX + fillW, barY, padFillW[i] - fillW, 4, theme::TRACK);
      padFillW[i] = fillW;
    }
  }

  /* ---------------------- магистральное давление ----------------------- */
  static char masterValStr[10] = "";
  static uint16_t masterEpoch = 0;
  if (masterEpoch != g_displayEpoch) {
    masterValStr[0] = '\0';
    masterEpoch = g_displayEpoch;
  }

  char mbuf[10];
  snprintf(mbuf, sizeof(mbuf), "%.1f", localMasterPressure);
  uiRedrawLabel(CARD_X + CARD_W - 106, MASTER_Y + 3, 100, 18, masterValStr, sizeof(masterValStr),
                mbuf, theme::TEXT, theme::PANEL_ALT, UiFont::Med, UiHAlign::Right);

  /* ---------------------- шкалы крена / тангажа ------------------------ */
  drawAttitudeGauges(GAUGE_X, GAUGE_Y, GAUGE_W, GAUGE_H, localAngleX, localAngleY);

  /* ------------------------- строка информации ------------------------- */
  updateMainInfoLine(localIsMoving);

  static bool hintDrawn = false;
  static uint16_t hintEpoch = 0;
  if (hintEpoch != g_displayEpoch) {
    hintDrawn = false;
    hintEpoch = g_displayEpoch;
  }

  /* ------------------------------ подсказка ---------------------------- */
  // Текст постоянный: рисуем один раз на эпоху экрана (не на каждый кадр углов).
  if (!hintDrawn) {
    constexpr char kHint[] = "МЕНЮ:КН3+КН4  РЕЖИМ:КН1+КН2  СТОП:КН1+КН4";
    ui.setTransparent(true);
    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::WARN, theme::BG);
    ui.box(8, HINT_Y, theme::SCREEN_W - 16, 18, kHint, UiHAlign::Center, UiVAlign::Middle, true);
    hintDrawn = true;
  }
}

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

  // LOW_PRESSURE: МП ниже «Низк.МП»; порог 0.00 = проверка выключена
  float masterLow = ConfigManager::getMasterLowBar();
  if (masterLow <= 0.0f) {
    if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::removeError(ErrorHandler::Error::LOW_PRESSURE);
      Serial.println("[CHECK] LOW_PRESSURE снята сразу (Низк.МП=0, проверка выкл.)");
    }
    return false;
  }

  if (localMaster < masterLow) {
    if (ErrorHandler::isPendingClear(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::cancelClear(ErrorHandler::Error::LOW_PRESSURE);
      Serial.println("[CHECK] Отменено удаление LOW_PRESSURE (давление снова упало)");
    }

    if (!ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::handleError(ErrorHandler::Error::LOW_PRESSURE,
                                "Низкое давление в магистрали");
      Serial.printf("[CHECK] LOW_PRESSURE создана! МП: %.2f бар (порог: %.2f)\n",
                    localMaster, masterLow);
    } else {
      ErrorHandler::updateErrorTime(ErrorHandler::Error::LOW_PRESSURE);
    }
    return true;
  }

  // ? ДАВЛЕНИЕ В НОРМЕ - УДАЛЯЕМ ОШИБКУ
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    // ? ПРОВЕРЯЕМ, НЕ НАХОДИТСЯ ЛИ УЖЕ ОШИБКА В pendingClear
    if (!ErrorHandler::isPendingClear(ErrorHandler::Error::LOW_PRESSURE)) {
      ErrorHandler::markErrorCleared(ErrorHandler::Error::LOW_PRESSURE);
      Serial.printf("[CHECK] LOW_PRESSURE будет удалена через 3 сек. Давление: %.2f бар\n",
                    localMaster);
    } else {
      // ? ОШИБКА УЖЕ В ОЧЕРЕДИ НА УДАЛЕНИЕ - НЕ ТРОГАЕМ ТАЙМЕР
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

  // ? Проверка давления в магистрали
  float currentMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentMaster = masterPressure;
  }


  float minPressure = ConfigManager::getPressureMin();
  // Сброс разрешён при низком МП; накачка — нет.
  if (inflate && currentMaster < minPressure) {
    Serial.printf("[MANUAL] Низкое давление в магистрали (%.1f < %.1f), накачка запрещена\n",
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

void valveTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
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
          if (activeCmd.async.pad < PAD_COUNT) {
            valveCycleCount[activeCmd.async.pad]++;
          }

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
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Control;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  IMUData imu = { 0 };
  PressureData press = { 0 };

  // ========== Переменные health-check JHM1200 ==========
  static uint32_t lastAdsCheck = 0;  // период health JHM1200
  static bool adsErrorReported = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Control);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CONTROL);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    uint32_t currentTime = millis();

    // ========== ? ДОБАВЛЕНО: СБРОС lastCmd ПРИ ОШИБКЕ SENSOR ==========
    if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
      if (lastCmd.waitingForCompletion || lastCmd.commandActive) {
        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        Serial.println("[CONTROL] lastCmd сброшен из-за ошибки SENSOR");
      }
    }


// ========== ПРОВЕРКА JHM1200 КАЖДЫЕ ~100 С ==========
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

      float testBar = 0.0f;
      bool ok = false;
      {
        MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
        if (i2c) ok = Jhm1200::readBar(testBar);
      }

      static uint8_t healthFails = 0;
      if (!ok) {
        if (healthFails < 255) healthFails++;
        // Одна неудача health-check (~раз в 100 с) не должна поднимать SENSOR.
        if (healthFails >= 3 && !ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR) &&
            !adsErrorReported) {
          adsErrorReported = true;
          ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "JHM1200 не отвечает");
          Serial.println("[JHM1200] Ошибка чтения! Проверьте подключение.");
        }
      } else {
        healthFails = 0;
        if (adsErrorReported) {
          adsErrorReported = false;
          if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
            ErrorHandler::markErrorCleared(ErrorHandler::Error::SENSOR);
            Serial.printf("[JHM1200] Датчик восстановлен (%.2f бар)\n", testBar);
          }
        }
      }
    }

#endif
    if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
      if (xQueueReceive(xIMUQueue, &imu, pdMS_TO_TICKS(10)) == pdTRUE) {
        MutexGuard guard(xStateMutex);
        if (guard) {
          // Нуль уже вычтен в imuTask
          angleX = imu.angleX;
          angleY = imu.angleY;
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
        // Нуль уже вычтен в imuTask
        angleX = imu.angleX;
        angleY = imu.angleY;
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
      } else if (!ErrorHandler::hasCriticalPneumaticErrors()) {
        static uint32_t lastMovementCheck = 0;
        if (currentTime - lastMovementCheck >= (uint32_t)ConfigManager::getMovementCheckSec() * 1000UL) {
          maintainMovementPressure();
          lastMovementCheck = currentTime;
        }
      }
    } else if (currentSystemMode == SystemMode::AUTO && currentState == SystemState::RUNNING) {
      // ? Проверка ошибок перед выравниванием
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
    if (manualControlActive && (millis() - manualStartTime > (uint32_t)ConfigManager::getManualMaxTimeSec() * 1000UL)) {
      stopManualOperation();
    }

    static uint32_t lastLeakCheck = 0;
    if (currentTime - lastLeakCheck >= 30000UL) {
      updateLeakMonitor();
      lastLeakCheck = currentTime;
    }

    bool sensorActive = ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR);
    bool mpuActive = ErrorHandler::isErrorActive(ErrorHandler::Error::MPU);


    if (!sensorActive) {
      // ДАТЧИК РАБОТАЕТ - проверяем давление
      checkPressureLimits();
    } else {
      // ДАТЧИК НЕИСПРАВЕН - LOW_PRESSURE НЕ ИМЕЕТ СМЫСЛА
      if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
        // ? Ждем 3 секунды перед удалением
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
              ErrorHandler::updateErrorTime(ErrorHandler::Error::VALVE);  // ?
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
static void requestMenuOpen(const char *via) {
  g_menuReq = MenuReq::Open;
  Serial.printf("[MENU] OPEN req via %s\n", via ? via : "?");
}

/** Запрос закрытия с сохранением — безопасен из ButtonTask / Serial / GEM.
 *  Реальная работа (cancel edit, save, fillScreen) в DisplayTask. */
static void requestMenuClose(const char *via) {
  g_menuReq = MenuReq::Close;
  Serial.printf("[MENU] CLOSE req via %s\n", via ? via : "?");
}

/** Сброс KN3/KN4 после срабатывания комбо (чтобы не было ложного click/step). */
static uint32_t g_menuNavIgnoreUntilMs = 0;

static void resetMenuComboState() {
  button2.reset();
  button3.reset();
  button0.reset();
  button1.reset();
  // Пока не отпустят пару и ещё ~0.5 с — не слать OK/BACK/^/v (иначе вход
  // сразу проваливается в подменю или вылетает по «клику» отпускания).
  g_menuNavIgnoreUntilMs = millis() + 600;
}

/** Обе кнопки пары меню физически зажаты. */
static bool menuPairPressedNow() {
  return digitalRead(PIN_BUT3) == LOW && digitalRead(PIN_BUT4) == LOW;
}

/** КН1+КН2 — смена АВТО/РУЧ. */
static bool modePairPressedNow() {
  return digitalRead(PIN_BUT1) == LOW && digitalRead(PIN_BUT2) == LOW;
}

/** КН1+КН4 — авария (вместо КН4+КН5). */
static bool emergencyPairPressedNow() {
  return digitalRead(PIN_BUT1) == LOW && digitalRead(PIN_BUT4) == LOW;
}

/** Переключение АВТО - РУЧНОЕ (не трогает MOVEMENT). */
static void toggleAutoManualMode(const char *via) {
  if (currentSystemMode == SystemMode::MOVEMENT) return;

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
  } else {
    currentSystemMode = SystemMode::AUTO;
    currentMode = Mode::AUTO;
    lastLevelingCheckTime = millis();
    lastLevelingAttemptTime = millis();
    levelingAttemptsThisHour = 0;
    lastHourResetTime = millis();
    Logger::log(Logger::INFO, "MODE", "Переключено в АВТО режим");
  }
  forceDisplayReset(true);
  setDisplayDirty();
  Serial.printf("[MODE] toggle via %s > %s\n", via ? via : "?",
                currentSystemMode == SystemMode::AUTO ? "AUTO" : "MANUAL");
}

/**
 * Удержание КН1+КН2 ? MODE_TOGGLE_HOLD_MS > смена режима.
 * Меню (КН3+КН4) и авария (КН1+КН4) подавляют переключение.
 */
static bool tryModeToggleHold(uint32_t now) {
  static uint32_t pairStartMs = 0;
  static bool pairFired = false;

  const bool modePair = modePairPressedNow();
  const bool menuPair = menuPairPressedNow();
  const bool emergPair = emergencyPairPressedNow();

  if (!modePair || menuPair || emergPair || menuVisible) {
    pairStartMs = 0;
    pairFired = false;
    return false;
  }

  if (pairStartMs == 0) {
    pairStartMs = now;
  }
  if (!pairFired && (now - pairStartMs) >= MODE_TOGGLE_HOLD_MS) {
    pairFired = true;
    toggleAutoManualMode("KN1+KN2");
    return true;
  }
  return false;
}

void buttonTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
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
    // button4 (КН5) не тикаем — отключена
    emergencyButton.tick(button0, button3);  // КН1+КН4 авария
    // НЕ вызываем VirtButton::tick для меню — свой таймер по GPIO.

    // DEBUG: лог нажатий для теста железа (уберу после проверки)
    {
      static bool prev[4] = {};
      const bool nowPress[4] = {
        button0.pressing(), button1.pressing(), button2.pressing(), button3.pressing()
      };
      const uint8_t pins[4] = { PIN_BUT1, PIN_BUT2, PIN_BUT3, PIN_BUT4 };
      for (uint8_t i = 0; i < 4; i++) {
        if (nowPress[i] != prev[i]) {
          Serial.printf("[BTN] KN%d %s gpio%d=%d\n", i + 1,
                        nowPress[i] ? "DOWN" : "UP",
                        pins[i], digitalRead(pins[i]));
          prev[i] = nowPress[i];
        }
      }
      if (menuPairPressedNow()) {
        static uint32_t lastPairMs = 0;
        if (millis() - lastPairMs >= 500) {
          lastPairMs = millis();
          Serial.println("[BTN] пара КН3+КН4 удерживается...");
        }
      }
    }

    // ============================================================
    // 1.05. Активность пользователя и пробуждение подсветки
    //       Важно: НЕ использовать button4.pressing() — GPIO34 без подтяжки
    //       часто «висит» в 0 и вечно сбрасывает таймер приглушения.
    // ============================================================
    {
      const bool activityEdge =
          button0.press() || button0.click() || button0.step() ||
          button1.press() || button1.click() || button1.step() ||
          button2.press() || button2.click() || button2.step() ||
          button3.press() || button3.click() || button3.step();

      if (backlightDimmed && activityEdge) {
        backlightDimmed = false;
        lastUserActivityMs = millis();
        applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        button0.reset();
        button1.reset();
        button2.reset();
        button3.reset();
        Serial.println("[DISP] Подсветка восстановлена (нажатие не выполняет действие)");
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }
      if (activityEdge) {
        lastUserActivityMs = millis();
      }
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
        Serial.println("[SERVICE] АВАРИЙНАЯ ОСТАНОВКА (КН1+КН4): экран закрыт, клапаны закрыты");
      } else {
        handleServiceInput();
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 1.1. МЕНЮ: удержание пары КН3+КН4 — открыть / закрыть с сохранением
    //      Свой таймер по pin/pressing (без VirtButton) — иначе повторный
    //      выход после входа не срабатывал (clear/reset физ. кнопок в tick).
    // ============================================================
    {
      static uint32_t comboStartMs = 0;
      static bool comboFired = false;
      static uint32_t lastComboLogMs = 0;
      const bool both = menuPairPressedNow();

      if (both) {
        if (comboStartMs == 0) {
          comboStartMs = now;
          lastComboLogMs = now;
          Serial.printf("[MENU] pair down (menu=%d)\n", menuVisible ? 1 : 0);
        }
        const uint32_t held = now - comboStartMs;
        if (!comboFired && held >= 400 && (now - lastComboLogMs) >= 400) {
          lastComboLogMs = now;
          Serial.printf("[MENU] pair hold %lu/%lu ms menu=%d\n",
                        (unsigned long)held, (unsigned long)MENU_COMBO_HOLD_MS,
                        menuVisible ? 1 : 0);
        }
        if (!comboFired && held >= MENU_COMBO_HOLD_MS) {
          comboFired = true;
          if (menuVisible) {
            requestMenuClose("KN3+KN4");
          } else {
            requestMenuOpen("KN3+KN4");
          }
          resetMenuComboState();
        }
      } else {
        if (comboStartMs != 0 || comboFired) {
          Serial.printf("[MENU] pair up (fired=%d held=%lu)\n",
                        comboFired ? 1 : 0,
                        (unsigned long)(comboStartMs ? (now - comboStartMs) : 0));
        }
        comboStartMs = 0;
        comboFired = false;
      }
    }

    // ============================================================
    // 2. АВАРИЙНАЯ ОСТАНОВКА (КН1 + КН4)
    //      Не срабатывает, пока удерживается меню (КН3+КН4).
    // ============================================================
    {
      const bool menuPair = menuPairPressedNow();
      const bool modePair = modePairPressedNow();
      if (!menuPair && !modePair && emergencyButton.hold()) {
        if (!emergencyProcessed) {
          emergencyStop();

          if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
            currentTestState = TestState::COMPLETED;
            closeAllValves();
            Serial.println("[TEST] Тест принудительно остановлен аварийной кнопкой!");
          }

          emergencyProcessed = true;
          lastEmergencyTime = millis();
          Serial.println("[EMERGENCY] АВАРИЙНАЯ ОСТАНОВКА! (КН1+КН4)");
          displayDirty = true;
        }
      } else if (emergencyButton.release()) {
        if (emergencyProcessed) {
          emergencyProcessed = false;
          if (!menuVisible) {
            displayDirty = true;
          }
        }
      }
    }

    if (wifiSetupActive) {
      if (button0.click() && wifi_scan_count > 0) {
        wifi_scan_selected = wifi_scan_selected == 0
                                 ? wifi_scan_count - 1
                                 : wifi_scan_selected - 1;
        // без displayDirty — частичная перерисовка по смене selected
      }
      if (button1.click() && wifi_scan_count > 0) {
        wifi_scan_selected = (wifi_scan_selected + 1) % wifi_scan_count;
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
    if (ErrorHandler::hasUiBlockingErrors() && !menuVisible) {
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

      // КН1+КН2 удержание — АВТО - РУЧ
      tryModeToggleHold(now);

      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 4. РЕЖИМ МЕНЮ
    // ============================================================
    if (menuVisible) {
      // Пока держат пару КН3+КН4 или ещё идёт «хвост» после комбо — глотаем события.
      const bool menuPairHeld = menuPairPressedNow();
      const bool navIgnore = menuPairHeld || (millis() < g_menuNavIgnoreUntilMs);
      if (navIgnore) {
        (void)button0.click();
        (void)button1.click();
        (void)button2.click();
        (void)button3.click();
        (void)button0.step();
        (void)button1.step();
        // Пока хотя бы одна кнопка пары ещё зажата — продлеваем игнор после отпускания.
        if (menuPairHeld) {
          g_menuNavIgnoreUntilMs = millis() + 450;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      auto menuKey = [](Button &btn, byte gemKey) {
        const bool isClick = btn.click();
        const bool isStep = btn.step();
        if (!isClick && !isStep) return;
        int reps = 1;
        if (isStep && gem.isEditMode()) {
          const uint16_t held = btn.holdFor();
          if (held > 2500) reps = 10;
          else if (held > 1500) reps = 6;
          else if (held > 900) reps = 4;
          else if (held > 400) reps = 2;
        }
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
        if (!guard) return;
        gemRestoreMenuItemFont();
        for (int i = 0; i < reps; i++) {
          gem.registerKeyPress(gemKey);
        }
        // ^/v вне edit: GEM уже перерисовал prev+current (drawMenuPointer) или
        // весь экран при смене «страницы» списка. Не ставим displayDirty —
        // иначе DisplayTask делает полный drawMenu() и меню рябит.
        if (gemKey == GEM_KEY_UP || gemKey == GEM_KEY_DOWN) {
          return;
        }
        if (!gem.isEditMode()) displayDirty = true;
      };
      menuKey(button0, GEM_KEY_UP);
      menuKey(button1, GEM_KEY_DOWN);
      if (button2.click()) {  // Кн3 - ВЫБОР/OK
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
        if (guard) {
          gemRestoreMenuItemFont();
          gem.registerKeyPress(GEM_KEY_OK);
          if (!gem.isEditMode()) displayDirty = true;
        }
        Serial.println("[MENU] SELECT");
      }
      if (button3.click()) {  // Кн4 - НАЗАД/CANCEL
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
        if (guard) {
          gemRestoreMenuItemFont();
          gem.registerKeyPress(GEM_KEY_CANCEL);
          if (!gem.isEditMode()) displayDirty = true;
        }
        Serial.println("[MENU] BACK");
      }

      vTaskDelay(pdMS_TO_TICKS(20));
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
      // 7.1. ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ — удержание КН1+КН2 ?800 мс
      // ============================================================
      tryModeToggleHold(now);

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
        const bool menuPairPressed = menuPairPressedNow();
        const bool modePairPressed = modePairPressedNow();
        const bool emergPairPressed = emergencyPairPressedNow();

        for (uint8_t i = 0; i < 4; i++) {
          // Пока пара меню / режима / аварии — подушки не трогаем
          if (menuPairPressed && (i == 2 || i == 3)) continue;
          if (modePairPressed && (i == 0 || i == 1)) continue;
          if (emergPairPressed && (i == 0 || i == 3)) {
            if (manualControlActive && (manualPadIndex == 0 || manualPadIndex == 3)) {
              stopManualOperation();
            }
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

static uint8_t reinitAttempts = 0;

void imuTask(void *pvParameters) {

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

void pressureTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Pressure;
  PressureData pd = { 0 };
  uint8_t curPad = 0;
  Event event;

  // ? ФЛАГ ПЕРВОГО ЗАМЕРА
  static bool firstMeasurementDone = false;
  // После полного круга (4 подушки + МП) — ждать Пауза опроса,мин
  static bool needIdlePause = false;

  for (;;) {
    TaskPool::markRun(taskIndex_Pressure);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_PRESSURE);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Выравн.МП,мс — пауза перед чтением датчика; Пауза опроса,мин — между полными кругами.
    const uint32_t stabilizeMs =
        (uint32_t)constrain(ConfigManager::getPressureStabilizeMs(), 100, 2000);
    const uint32_t idleMs =
        (uint32_t)constrain(ConfigManager::getPressureIdleMin(), 2, 30) * 60000UL;

    // ============================================================
    // ОПРЕДЕЛЕНИЕ ТАЙМАУТА ОЖИДАНИЯ
    // ============================================================
    const bool valveWork =
        manualControlActive || autoLevelingController.isBusy();
    uint32_t timeout;
    if (valveWork) {
      // Во время клапанов — чаще будим цикл.
      timeout = 200;
    } else if (needIdlePause) {
      // Простой: пауза 2…30 мин до следующего полного опроса (прерывается wakeup).
      // Soft-WATCHDOG не должен подменять это на 60 с — иначе опрос «залипает».
      timeout = idleMs;
    } else {
      // Середина круга подушек — сразу следующий замер.
      timeout = 0;
    }

    // ============================================================
    // ОЖИДАНИЕ ПРОБУЖДЕНИЯ (кусочками ?1 с — heartbeat soft-WDT)
    // ============================================================
    uint32_t dummy;
    bool woken = false;
    constexpr uint32_t kWdtSliceMs = 1000;
    if (timeout == 0) {
      if (xQueueReceive(xPressureWakeupQueue, &dummy, 0) == pdTRUE) {
        woken = true;
      }
    } else {
      uint32_t waited = 0;
      while (waited < timeout) {
        TaskMonitor::updateTaskStatus(TaskMonitor::TASK_PRESSURE);
        uint32_t chunk = timeout - waited;
        if (chunk > kWdtSliceMs) chunk = kWdtSliceMs;
        if (xQueueReceive(xPressureWakeupQueue, &dummy, pdMS_TO_TICKS(chunk)) == pdTRUE) {
          woken = true;
          break;
        }
        waited += chunk;
      }
    }
    if (woken) {
      Serial.println("[PRESS] Wakeup by request");
      needIdlePause = false;
    } else if (needIdlePause && timeout == idleMs) {
      needIdlePause = false;
      Serial.printf("[PRESS] Пауза %d мин истекла — новый опрос подушек\n",
                    ConfigManager::getPressureIdleMin());
    }

    // Во время калибровки нуля не трогаем клапаны — иначе искажается zero
    {
      SystemState st = getSystemState();
      if (st == SystemState::BOOT || st == SystemState::CALIBRATING || !calibrationCompleted) {
        vTaskDelay(pdMS_TO_TICKS(100));
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

    needIdlePause = true;

#else
    // ============================================================
    // РЕАЛЬНЫЙ РЕЖИМ
    // ============================================================

    if (ErrorHandler::isErrorActive(ErrorHandler::Error::VALVE) || otaValveLock) {
      // Soft-WATCHDOG не блокирует опрос: иначе давление залипает на 0 и ошибка не сходит.
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (valveWork) {
      // Клапаны уже коммутируют контур; stabilizeMs — выравнивание давления в магистрали.
      vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
      float p = readPressure();
      if (p >= 0.0f) {
        if (manualControlActive) {
          const uint8_t idx = static_cast<uint8_t>(manualPadIndex);
          if (idx < PAD_COUNT) {
            pd.pressure[idx] = p;
            pressureStampMs[idx] = millis();
            pressureValid[idx] = true;
          }
        } else {
          pd.pressure[curPad] = p;
          pressureStampMs[curPad] = millis();
          pressureValid[curPad] = true;
        }
      }
    } else if (xSemaphoreTake(xValveMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      setValve(Pad(curPad), HIGH);
      vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
      float p = readPressure();
      setValve(Pad(curPad), LOW);

      if (p >= 0.0f) {
        pd.pressure[curPad] = p;
        pressureStampMs[curPad] = millis();
        pressureValid[curPad] = true;
      }

      curPad = (curPad + 1) % PAD_COUNT;
      if (curPad == 0) {
        digitalWrite(PIN_INFL, HIGH);
        vTaskDelay(pdMS_TO_TICKS(stabilizeMs));
        float master = readPressure();
        digitalWrite(PIN_INFL, LOW);
        if (master >= 0.0f) {
          pd.masterPressure = master;
          masterStampMs = millis();
          masterValid = true;
        }

        if (!firstMeasurementDone && pd.masterPressure >= 0.0f) {
          firstMeasurementDone = true;
          firstPressureMeasurementDone = true;
          Serial.printf("[PRESS] Первый замер выполнен! Давление: %.2f бар\n", pd.masterPressure);
          checkPressureLimits();
        }
        // Полный круг закончен — дальше пауза 2…30 мин (если не клапаны).
        needIdlePause = true;
      }
      xSemaphoreGive(xValveMutex);
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
            // ШАГ 1: ИНИЦИАЛИЗАЦИЯ JHM1200
            // ============================================================
            Serial.println("[CALIB] ШАГ 1/3: Инициализация JHM1200...");

            bool jhmOk = initializeJhm1200();
            if (!jhmOk) {
                Serial.println("[CALIB] JHM1200 НЕ НАЙДЕН! Повтор через 2 с…");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "JHM1200 не найден");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);
                continue;
            }

            Serial.println("[CALIB] JHM1200 инициализирован (0x78)");

            // ============================================================
            // ШАГ 2: ПРОВЕРКА ДАТЧИКА ДАВЛЕНИЯ
            // ============================================================
            Serial.println("[CALIB] ШАГ 2/3: Проверка датчика давления...");

            float probeBar = 0.0f;
            bool probeOk = false;
            for (int attempt = 0; attempt < 8 && !probeOk; attempt++) {
              if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(30));
              MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(200));
              if (i2c) probeOk = Jhm1200::readBar(probeBar);
            }

            if (!probeOk) {
                Serial.println("[CALIB] ОШИБКА ЧТЕНИЯ JHM1200! Повтор через 2 с…");
                ErrorHandler::handleError(ErrorHandler::Error::SENSOR, "JHM1200 - ошибка чтения");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);  // повтор полного цикла
                continue;
            }

            Serial.printf("[CALIB] Датчик давления исправен (%.3f бар, status=0x%02X)\n",
                          probeBar, Jhm1200::lastStatus());

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
            if (!mpuOk) {
              initializeDMP();
            }
            if (!mpuOk) {
                Serial.println("[CALIB] ? MPU6050 НЕ ОТВЕЧАЕТ!");
            } else {
                Serial.println("[CALIB] ? MPU6050 инициализирован успешно");
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

            // ? СОЗДАЁМ ЛОКАЛЬНЫЙ ОБЪЕКТ КАЛИБРАТОРА
            ZeroCalibrator localCalibrator;
            bool calibratorStarted = false;
            bool sensorErrorDuringCalib = false;

            uint32_t start = millis();

            while (millis() - start < CALIB_TIME_MS) {
                TaskMonitor::updateTaskStatus(TaskMonitor::TASK_CALIB);

                // Проверяем, не появилась ли ошибка SENSOR во время калибровки
                if (ErrorHandler::isErrorActive(ErrorHandler::Error::SENSOR)) {
                    sensorErrorDuringCalib = true;
                    Serial.println("[CALIB] ? Ошибка SENSOR во время калибровки!");
                    break;
                }

                if (!calibratorStarted && (millis() - start >= 2000)) {
                    localCalibrator.start();  // < ИСПОЛЬЗУЕМ ЛОКАЛЬНЫЙ
                    calibratorStarted = true;
                }

                if (calibratorStarted && !localCalibrator.isDone()) {
                    localCalibrator.process();  // < ИСПОЛЬЗУЕМ ЛОКАЛЬНЫЙ
                }

                static uint32_t lastDirtySet = 0;
                if (millis() - lastDirtySet > 100) {
                    displayDirty = true;
                    lastDirtySet = millis();
                }

                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // ? ПОСЛЕ КАЛИБРОВКИ - СОХРАНЯЕМ РЕЗУЛЬТАТ
            if (sensorErrorDuringCalib) {
                Serial.println("[CALIB] ? Калибровка прервана из-за ошибки SENSOR!");
                setSystemState(SystemState::RUNNING);
                calibrationCompleted = false;
                displayDirty = true;
                forceDisplayReset(true);
                vTaskDelay(pdMS_TO_TICKS(2000));
                setSystemState(SystemState::BOOT);
                continue;
            }

            if (calibratorStarted && !localCalibrator.isDone()) {
                Serial.println("[CALIB] Таймаут калибровки, использую последнее значение");
                g_pressureZeroBar = localCalibrator.getZeroValue();
            } else if (localCalibrator.isDone()) {
                g_pressureZeroBar = localCalibrator.getZeroValue();
            }

            Serial.printf("[CALIB] Калибровка завершена: zero=%.3f бар\n", g_pressureZeroBar);
            Logger::log(Logger::INFO, "CALIB", "Прогрев и калибровка завершены");

            // ============================================================
            // ПЕРЕХОД В РЕЖИМ RUNNING
            // ============================================================
            setSystemState(SystemState::RUNNING);
            calibrationCompleted = true;
            calibrationValid = true;
            displayDirty = true;
            forceDisplayReset(true);

            Serial.println("[CALIB] ? Система готова к работе!");
            Serial.println("[CALIB] Ожидание первого замера давления...");
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void displayTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Display;
  TickType_t framePeriod = pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL_MS);  // 8.8.0: обновляется в цикле из настроек

  static bool lastHasError = false;
  static uint32_t errorHoldUntil = 0;
  const uint32_t ERROR_HOLD_MS = 3000;

  for (;;) {
    TaskPool::markRun(taskIndex_Display);
    // 8.8.0: интервал кадра из настроек (Меню > Дисплей > Интервал,мс)
    framePeriod = pdMS_TO_TICKS(ConfigManager::getFrameMs());

    // Приглушение подсветки по бездействию (0 = никогда).
    // Не гасим экран — только PWM уровня CONTRAST_DIM. В меню не приглушаем.
    {
      const int blMin = ConfigManager::getBacklightOffMin();
      if (blMin > 0 && !backlightDimmed && !menuVisible) {
        const uint32_t idleMs = millis() - lastUserActivityMs;
        const uint32_t needMs = (uint32_t)blMin * 60000UL;
        if (idleMs > needMs) {
          backlightDimmed = true;
          applyBacklightPwm(CONTRAST_DIM);
          Serial.printf("[DISP] Подсветка приглушена до %d (бездействие %d мин, idle=%lu с)\n",
                        CONTRAST_DIM, blMin, (unsigned long)(idleMs / 1000UL));
        }
      }
    }
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_DISPLAY);

    // ============================================================
    // ЗАХВАТ МЬЮТЕКСА ДИСПЛЕЯ
    // ============================================================
    if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(500)) == pdTRUE) {

      // ============================================================
      // 0. ЗАПРОСЫ OPEN/CLOSE МЕНЮ (из ButtonTask / Serial / GEM)
      // ============================================================
      {
        const MenuReq req = g_menuReq;
        if (req != MenuReq::None) {
          g_menuReq = MenuReq::None;
          if (req == MenuReq::Open) {
            if (!menuVisible) {
              openMenu();
            } else {
              Serial.println("[MENU] OPEN skip (already open)");
            }
          } else if (req == MenuReq::Close) {
            if (menuVisible) {
              if (gem.isEditMode()) {
                gemRestoreMenuItemFont();
                gem.registerKeyPress(GEM_KEY_CANCEL);
              }
              menuVisible = false;
              // Сначала восстановить главный экран, потом LittleFS — иначе
              // долгое сохранение + fillScreen без firstRun = чёрный экран.
              forceDisplayReset(true);
              displayDirty = true;
              if (!ErrorHandler::hasActiveErrors() &&
                  serviceScreen == ServiceScreen::NONE &&
                  !wifiSetupActive && !wifiScanInProgress) {
                if (currentSystemMode == SystemMode::MOVEMENT &&
                    currentState == SystemState::RUNNING) {
                  displayMovementScreen();
                } else {
                  displayMainScreen();
                }
                displayDirty = false;
              }
              xSemaphoreGive(xDisplayMutex);
              saveMenuSettings();
              Serial.println("[MENU] CLOSE ok");
              vTaskDelay(framePeriod);
              continue;
            } else {
              Serial.println("[MENU] CLOSE skip (already closed)");
            }
          }
        }
      }

      // ============================================================
      // 1. ОТОБРАЖЕНИЕ МЕНЮ (САМЫЙ ПРИОРИТЕТНЫЙ!)
      // ============================================================
      if (menuVisible) {
        // Только ручная перерисовка (openMenu / выход). Фоновые dirty от IMU
        // отфильтрованы в setDisplayDirty(); здесь дополнительно игнорируем
        // чужие displayDirty, чтобы меню не «дышало».
        mqOnMenuPageEnter();
        // Живой статус OTA: после TLS Display снова в работе — если буфер
        // статуса сменился, перерисовать меню (иначе висит «проверка...»).
        {
          GEMPage *page = gem.getCurrentMenuPage();
          static char s_otaStSeen[64] = "";
          static char s_otaHdrSeen[40] = "";
          if (page == &otaPage && strcmp(s_otaStSeen, otaListStatus) != 0) {
            strlcpy(s_otaStSeen, otaListStatus, sizeof(s_otaStSeen));
            displayDirty = true;
          } else if (page == &otaListPage &&
                     strcmp(s_otaHdrSeen, otaListHdrBuf) != 0) {
            strlcpy(s_otaHdrSeen, otaListHdrBuf, sizeof(s_otaHdrSeen));
            displayDirty = true;
          }
        }
        if (displayDirty) {
          // В режиме редактирования значения GEM сам рисует цифру (spinner/digit).
          // Полный drawMenu() перезатирает её старым linkedVariable — «цифра не
          // меняется, пока не нажмёшь Выбор».
          if (!gem.isEditMode()) {
            refreshDynamicMenu();
            gemRestoreMenuItemFont();
            gem.drawMenu();
          }
          displayDirty = false;
        } else if (!gem.isEditMode()) {
          mqTick();  // может выставить displayDirty для следующего кадра
        }
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

      if (wifiSetupActive || wifiScanInProgress) {
        displayWiFiSetupScreen();  // сам решает full vs partial
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

  // 1.5. СЛУЖЕБНЫЕ ЭКРАНЫ — полный кадр при входе, дальше только изменившиеся поля.
  if (serviceScreen != ServiceScreen::NONE) {
        drawServiceScreen();
        xSemaphoreGive(xDisplayMutex);
        vTaskDelay(framePeriod);
        continue;
      }

      // ============================================================
      // 2. ОБРАБОТКА ОШИБОК
      // ============================================================
      bool hasErrorNow = ErrorHandler::hasUiBlockingErrors();
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
          displayDirty = true;  // без fillScreen — иначе мерцание ~каждые 3 с
        }
      }

      // ============================================================
      // 3. ОТОБРАЖЕНИЕ ЭКРАНОВ
      // ============================================================
      if (ErrorHandler::hasUiBlockingErrors()) {
        errorScreenBlocking = true;
        // Всегда вызываем: внутри — смена ошибки каждые 2 с, счётчик N/M и мигание иконки.
        // Раньше после первого кадра шёл только blinkErrorIcon > чередование «замирало».
        displayErrorScreen();
        displayDirty = false;
      } else {
        errorScreenBlocking = false;

        // ============================================================
        // 3.1. 8.9.0: ЖИВОЙ ЭКРАН ДВИЖЕНИЯ (только в режиме MOVEMENT).
        //      Рисуется постоянно: внутри — интерполяция и dirty-обновления.
        // ============================================================
        const bool movementUi =
            (currentSystemMode == SystemMode::MOVEMENT && currentState == SystemState::RUNNING &&
             (currentTestState == TestState::IDLE || currentTestState == TestState::COMPLETED) &&
             !otaMode && !otaInProgress);
        // Выход ДВИЖЕНИЕ→АВТО/РУЧ: без полного кадра главный экран
        // наслаивается на графику экрана движения (dirty только частичный).
        static bool s_wasMovementUi = false;
        if (s_wasMovementUi && !movementUi) {
          mvScreenWasActive = false;
          forceDisplayReset(true);
          displayDirty = true;
          Serial.println("[DISPLAY] Full redraw after MOVEMENT → main");
        }
        s_wasMovementUi = movementUi;

        if (movementUi) {
          displayMovementScreen();
          displayDirty = false;
          xSemaphoreGive(xDisplayMutex);
          vTaskDelay(framePeriod);
          continue;
        }

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
        } else {
          // Гироскоп/авиагоризонт — каждый кадр, не ждём dirty от давлений
          updateMainTiltLive();
          // Мигание СТАТ+иконки при детекции движения до входа в MOVEMENT
          if (currentState == SystemState::RUNNING &&
              currentSystemMode != SystemMode::MOVEMENT &&
              serviceScreen == ServiceScreen::NONE) {
            updateMainStatBadge();
          }
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

void otaTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }
  Serial.println("[OTA-CMD] ready: OTA HEAP|VER|PING|LIST|INSTALL n|SELFTEST|ABORT");
  extern uint8_t taskIndex_OTA;
  static bool bootGithubListDone = false;
  for (;;) {
    TaskPool::markRun(taskIndex_OTA);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_OTA);

    handleOtaSerialCommands();
    // Пока в RX есть байты — крутимся чаще (иначе TX-спам других задач глотает команды).
    if (Serial.available() > 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // Wi-Fi scan: нельзя из ButtonTask/GEM; sync-scan валил TWDT idle → reboot.
    if (wifiSetupRequested) {
      startWiFiSetup();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (wifiScanInProgress) {
      if (pollWiFiScanComplete()) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Автозагрузка списка при старте отключена: TLS на фоне дробит heap
    // и даёт HTTP -1 при ручном OTA. Список — только по запросу из меню.
    (void)bootGithubListDone;
    bootGithubListDone = true;

    GitHubOtaRequest ghReq = takeGitHubOtaRequest();
    if (ghReq == GitHubOtaRequest::CHECK_ONLY) {
      Logger::log(Logger::INFO, "GH-OTA", "Проверка обновлений (без установки)");
      checkGitHubUpdate(false);
      // Как FETCH_LIST: сразу перерисовать статус на странице «Обновления»,
      // иначе displayDirty «проглатывается» и строка висит на «проверка...».
      {
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(300));
        if (guard && menuVisible && gem.getCurrentMenuPage() == &otaPage) {
          refreshOtaPage();
          gemRestoreMenuItemFont();
          gem.drawMenu();
          displayDirty = false;
        } else {
          displayDirty = true;
        }
      }
    } else if (ghReq == GitHubOtaRequest::CHECK_AND_INSTALL) {
      Logger::log(Logger::INFO, "GH-OTA", "Проверка GitHub (фоновая задача)");
      strlcpy(otaStatus, "Проверка…", sizeof(otaStatus));
      displayDirty = true;
      // Всегда тянем список; ставим latest, если он новее ИЛИ тот же (переустановка).
      // Если локально новее GitHub — не даунгрейдим, пишем статус в меню.
      if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
        if (otaListStatus[0] == '\0') {
          strlcpy(otaListStatus, "нет ответа GitHub", sizeof(otaListStatus));
        }
        endGitHubOtaProgressUi(true);
      } else {
        SemVer local{}, remote{};
        parseSemVer(VERSION, local);
        parseSemVer(otaReleases[0].tag, remote);
        const int cmp = (local.valid && remote.valid) ? compareSemVer(remote, local) : 1;
        strlcpy(otaLatestTag, otaReleases[0].tag, sizeof(otaLatestTag));
        if (cmp < 0) {
          snprintf(otaListStatus, sizeof(otaListStatus), "локально новее (%s)", otaReleases[0].tag);
          Serial.printf("[GH-OTA] Skip install: local %s > remote %s\n", VERSION, otaReleases[0].tag);
          endGitHubOtaProgressUi(true);
        } else if (cmp == 0) {
          snprintf(otaListStatus, sizeof(otaListStatus), "уже актуально (%s)", otaReleases[0].tag);
          Serial.printf("[GH-OTA] Skip reinstall same %s (список — для принудительной)\n",
                        otaReleases[0].tag);
          endGitHubOtaProgressUi(true);
        } else {
          snprintf(otaListStatus, sizeof(otaListStatus), "установка %s", otaReleases[0].tag);
          Logger::log(Logger::INFO, "GH-OTA", "Установка последнего релиза…");
          strlcpy(otaStatus, "Загрузка…", sizeof(otaStatus));
          displayDirty = true;
          if (!checkGitHubUpdate(true)) {
            if (otaListStatus[0] == '\0') {
              strlcpy(otaListStatus, "ошибка установки", sizeof(otaListStatus));
            }
            endGitHubOtaProgressUi(true);
          }
          // успех > ESP.restart() внутри checkGitHubUpdate(true)
        }
      }
    } else if (ghReq == GitHubOtaRequest::FETCH_LIST) {
      Serial.println("[GH-OTA] FETCH_LIST begin");
      Logger::log(Logger::INFO, "GH-OTA", "Получение списка релизов");
      // Во время калибровки heap падает — TLS к GitHub не проходит.
      for (int w = 0; w < 40; w++) {
        if (getSystemState() == SystemState::RUNNING && ESP.getMaxAllocHeap() >= 30000u) break;
        vTaskDelay(pdMS_TO_TICKS(250));
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      Serial.printf("[GH-OTA] pre-list heap=%u maxBlk=%u internal=%u\n",
                    static_cast<unsigned>(ESP.getFreeHeap()),
                    static_cast<unsigned>(ESP.getMaxAllocHeap()),
                    static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
      fetchGitHubReleaseList();
      Serial.printf("[GH-OTA] FETCH_LIST done status=%s\n", otaListStatus);
      // Сразу перерисовать список под мьютексом — иначе displayDirty может
      // «проглотиться» и теги появятся только после нажатия кнопки.
      {
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(300));
        if (guard && menuVisible && gem.getCurrentMenuPage() == &otaListPage) {
          refreshOtaListPage();
          gemRestoreMenuItemFont();
          gem.drawMenu();
          displayDirty = false;
        } else {
          displayDirty = true;
        }
      }
    } else if (ghReq == GitHubOtaRequest::INSTALL_INDEX) {
      const int8_t idx = takeGitHubOtaIndex();
      Logger::logf(Logger::INFO, "GH-OTA", "Установка релиза #%d", static_cast<int>(idx));
      if (!installGitHubReleaseIndex(idx)) {
        strlcpy(otaListStatus, "ошибка установки", sizeof(otaListStatus));
        endGitHubOtaProgressUi(true);
      }
      displayDirty = true;
    }

    if (getSystemState() == SystemState::OTA_MODE) handleOTA();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

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
static esp_ota_handle_t s_ghOtaHandle = 0;
static bool s_ghOtaActive = false;
static const esp_partition_t *s_ghOtaPart = nullptr;

static void githubOtaClearUpdate(const char *why) {
  if (s_ghOtaActive) {
    Serial.printf("[GH-OTA] abort esp_ota (%s) handle=%u\n", why ? why : "?",
                  static_cast<unsigned>(s_ghOtaHandle));
    esp_ota_abort(s_ghOtaHandle);
    s_ghOtaActive = false;
    s_ghOtaHandle = 0;
    s_ghOtaPart = nullptr;
  }
  if (Update.isRunning()) {
    Serial.printf("[GH-OTA] abort Update (%s)\n", why ? why : "?");
    Update.abort();
  }
  Update.clearError();
}

static void githubOtaHeartbeat() {
  extern uint8_t taskIndex_OTA;
  TaskPool::markRun(taskIndex_OTA);
  TaskMonitor::updateTaskStatus(TaskMonitor::TASK_OTA);
}

static void githubOtaPrintHeap(const char *tag) {
  Serial.printf("[OTA-HEAP] %s free=%u maxBlk=%u int=%u\n", tag ? tag : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
}

/** Serial harness: OTA HEAP|VER|PING|LIST|INSTALL n|SELFTEST|ABORT */
static void handleOtaSerialCommands() {
  static char line[96];
  static size_t len = 0;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      len = 0;
      if (line[0] == '\0') continue;

      Serial.printf("[OTA-CMD] rx: %s\n", line);

      // OTA читает Serial первым — TEST MENU* обрабатываем здесь
      if (strcmp(line, "TEST MENU OPEN") == 0 || strcmp(line, "test menu open") == 0) {
        requestMenuOpen("TEST");
        Serial.println("[TEST] OK MENU_OPEN_REQ");
        continue;
      }
      if (strcmp(line, "TEST MENU CLOSE") == 0 || strcmp(line, "test menu close") == 0) {
        requestMenuClose("TEST");
        Serial.println("[TEST] OK MENU_CLOSE_REQ");
        continue;
      }
      if (strcmp(line, "TEST MENU DOWN") == 0 || strcmp(line, "test menu down") == 0) {
        if (menuVisible) {
          MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(200));
          if (guard) {
            gemRestoreMenuItemFont();
            gem.registerKeyPress(GEM_KEY_DOWN);
          }
          Serial.println("[TEST] OK MENU_DOWN");
        } else {
          Serial.println("[TEST] FAIL MENU_DOWN_CLOSED");
        }
        continue;
      }
      if (strcmp(line, "TEST MENU UP") == 0 || strcmp(line, "test menu up") == 0) {
        if (menuVisible) {
          MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(200));
          if (guard) {
            gemRestoreMenuItemFont();
            gem.registerKeyPress(GEM_KEY_UP);
          }
          Serial.println("[TEST] OK MENU_UP");
        } else {
          Serial.println("[TEST] FAIL MENU_UP_CLOSED");
        }
        continue;
      }
      if (strcmp(line, "TEST MENU OK") == 0 || strcmp(line, "test menu ok") == 0) {
        if (menuVisible) {
          MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(200));
          if (guard) {
            gemRestoreMenuItemFont();
            gem.registerKeyPress(GEM_KEY_OK);
            if (!gem.isEditMode()) displayDirty = true;
          }
          Serial.println("[TEST] OK MENU_OK");
        } else {
          Serial.println("[TEST] FAIL MENU_OK_CLOSED");
        }
        continue;
      }

      // Остальные TEST* — в общий парсер (иначе OTA глотает Serial и loop() их не видит)
      if (strncmp(line, "TEST", 4) == 0 || strncmp(line, "test", 4) == 0) {
        processTestCommandLine(line);
        continue;
      }

      char upper[96];
      size_t i = 0;
      for (; line[i] && i + 1 < sizeof(upper); ++i) {
        const char ch = line[i];
        upper[i] = (ch >= 'a' && ch <= 'z') ? static_cast<char>(ch - 'a' + 'A') : ch;
      }
      upper[i] = '\0';

      if (strcmp(upper, "OTA HEAP") == 0) {
        githubOtaPrintHeap("cmd");
      } else if (strcmp(upper, "OTA VER") == 0) {
        Serial.printf("[OTA-CMD] VERSION=%s\n", VERSION);
      } else if (strcmp(upper, "OTA PING") == 0) {
        Serial.println("[OTA-CMD] PING…");
        for (int w = 0; w < 40; w++) {
          if (getSystemState() == SystemState::RUNNING) break;
          vTaskDelay(pdMS_TO_TICKS(250));
          githubOtaHeartbeat();
        }
        githubOtaPingDiag();
      } else if (strcmp(upper, "OTA ABORT") == 0) {
        githubOtaClearUpdate("serial-abort");
        githubOtaEndInstallSession("dl-exit");
        otaValveLock = false;
        Serial.println("[OTA-CMD] aborted");
      } else if (strcmp(upper, "OTA LIST") == 0) {
        Serial.println("[OTA-CMD] LIST…");
        githubOtaPrintHeap("pre-list");
        // Как в FETCH_LIST: не бить TLS во время калибровки.
        for (int w = 0; w < 40; w++) {
          // maxBlk низкий пока держим TLS-reserve — ждём только RUNNING.
          if (getSystemState() == SystemState::RUNNING) break;
          vTaskDelay(pdMS_TO_TICKS(250));
          githubOtaHeartbeat();
        }
        beginGitHubOtaProgressUi("Список…");
        if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
          Serial.printf("[OTA-CMD] LIST fail status=%s\n", otaListStatus);
          endGitHubOtaProgressUi(true);
        } else {
          for (uint8_t k = 0; k < otaReleaseCount; ++k) {
            Serial.printf("[OTA-CMD] #%u %s bin=%s\n", static_cast<unsigned>(k),
                          otaReleases[k].tag, otaReleases[k].binUrl);
          }
          Serial.printf("[OTA-CMD] LIST ok count=%u\n", static_cast<unsigned>(otaReleaseCount));
          endGitHubOtaProgressUi(true);
        }
      } else if (strncmp(upper, "OTA INSTALL ", 12) == 0) {
        const int idx = atoi(line + 12);
        Serial.printf("[OTA-CMD] INSTALL %d\n", idx);
        beginGitHubOtaProgressUi("Установка…");
        if (otaReleaseCount == 0 && !fetchGitHubReleaseList()) {
          Serial.println("[OTA-CMD] INSTALL: no list");
          endGitHubOtaProgressUi(true);
        } else if (!installGitHubReleaseIndex(static_cast<int8_t>(idx))) {
          Serial.printf("[OTA-CMD] INSTALL fail status=%s\n", otaListStatus);
          endGitHubOtaProgressUi(true);
        }
        // success > ESP.restart inside install
      } else if (strcmp(upper, "OTA SELFTEST") == 0) {
        Serial.println("[OTA-CMD] SELFTEST…");
        githubOtaPrintHeap("pre-selftest");
        beginGitHubOtaProgressUi("Selftest…");
        if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
          Serial.printf("[OTA-CMD] SELFTEST list fail: %s\n", otaListStatus);
          endGitHubOtaProgressUi(true);
        } else {
          int8_t pick = 0;
          // Предпочитаем latest (0) — переустановка той же/новой версии.
          Serial.printf("[OTA-CMD] SELFTEST pick #%d %s (local %s)\n", static_cast<int>(pick),
                        otaReleases[pick].tag, VERSION);
          if (!installGitHubReleaseIndex(pick)) {
            Serial.printf("[OTA-CMD] SELFTEST FAIL status=%s\n", otaListStatus);
            endGitHubOtaProgressUi(true);
          }
        }
      } else if (strncmp(upper, "OTA ", 4) == 0) {
        Serial.println("[OTA-CMD] usage: OTA HEAP|VER|PING|LIST|INSTALL n|SELFTEST|ABORT");
      }
      continue;
    }
    if (len + 1 < sizeof(line)) {
      line[len++] = c;
    } else {
      len = 0;
    }
  }
}

bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Нет интернет-соединения");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }

  otaInProgress = true;
  otaProgress = 0;
  strlcpy(otaStatus, "SHA-256…", sizeof(otaStatus));
  displayDirty = true;
  // Одна TLS-сессия на весь install: reserve снят, воркеры на паузе до конца.
  githubOtaReleaseTlsHeap("install-begin");
  githubOtaPauseWorkers(true);
  githubOtaClearUpdate("before-start");
  githubOtaHeartbeat();
  githubOtaPrintHeap("start");

  // Confirm running image so next OTA is allowed (rollback pending > begin fail).
  {
    const esp_err_t mv = esp_ota_mark_app_valid_cancel_rollback();
    if (mv != ESP_OK && mv != ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
      Serial.printf("[GH-OTA] mark_valid: %s\n", esp_err_to_name(mv));
    }
  }

  static constexpr char kShaPath[] = "/gh_sha.txt";
  const int shaCode = githubHttpsDownloadToFile(sha256Url, kShaPath, 45000, "*/*");
  githubOtaHeartbeat();
  char expected[65] = "";
  if (shaCode == HTTP_CODE_OK) {
    File sf = LittleFS.open(kShaPath, "r");
    if (sf) {
      size_t n = sf.readBytes(expected, 64);
      expected[n < 64 ? n : 64] = '\0';
      sf.close();
    }
  }
  LittleFS.remove(kShaPath);
  for (char *p = expected; *p; ++p) {
    const char c = *p;
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
      *p = '\0';
      break;
    }
    if (c >= 'A' && c <= 'F') *p = static_cast<char>(c - 'A' + 'a');
  }
  if (strlen(expected) < 64) {
    Serial.printf("[GH-OTA] SHA-256 asset отсутствует или некорректен (HTTP %d, url=%s)\n",
                  shaCode, sha256Url);
    strlcpy(otaListStatus, "нет sha256", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  expected[64] = '\0';
  Serial.printf("[GH-OTA] expected sha256: %.16s… tag=%s\n", expected, releaseTag ? releaseTag : "?");
  githubOtaPrintHeap("after-sha");

  strlcpy(otaStatus, "OTA begin…", sizeof(otaStatus));
  displayDirty = true;
  WiFi.setSleep(false);
  vTaskDelay(pdMS_TO_TICKS(30));

  // КРИТИЧНО: esp_ota_begin ДО TLS на bin. Arduino Update.begin под живым TLS
  // не выделял 4K буфер (err=No Error) при free~44K / maxBlk~36K.
  s_ghOtaPart = esp_ota_get_next_update_partition(nullptr);
  if (!s_ghOtaPart) {
    Serial.println("[GH-OTA] нет OTA partition");
    strlcpy(otaListStatus, "нет OTA part", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  Serial.printf("[GH-OTA] part subtype=%u size=%u offset=0x%X\n",
                static_cast<unsigned>(s_ghOtaPart->subtype),
                static_cast<unsigned>(s_ghOtaPart->size),
                static_cast<unsigned>(s_ghOtaPart->address));
  githubOtaPrintHeap("pre-begin");

  esp_err_t beginErr = esp_ota_begin(s_ghOtaPart, OTA_WITH_SEQUENTIAL_WRITES, &s_ghOtaHandle);
  if (beginErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_begin(SEQ) fail %s — retry SIZE_UNKNOWN\n",
                  esp_err_to_name(beginErr));
    beginErr = esp_ota_begin(s_ghOtaPart, OTA_SIZE_UNKNOWN, &s_ghOtaHandle);
  }
  if (beginErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_begin fail %s heap=%u maxBlk=%u\n",
                  esp_err_to_name(beginErr),
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    strlcpy(otaListStatus, "OTA begin fail", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  s_ghOtaActive = true;
  Serial.printf("[GH-OTA] esp_ota_begin OK handle=%u\n", static_cast<unsigned>(s_ghOtaHandle));
  githubOtaPrintHeap("post-begin");

  strlcpy(otaStatus, "Загрузка…", sizeof(otaStatus));
  displayDirty = true;

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);

  uint8_t stackBuf[1024];
  uint8_t *rdBuf = stackBuf;
  size_t rdCap = sizeof(stackBuf);

  size_t written = 0;
  size_t totalSize = 0;
  bool sizeKnown = false;
  int lastPct = -1;
  const uint32_t tStart = millis();
  static constexpr uint32_t kHardTimeoutMs = 900000UL;  // 15 мин — дальше FS-хвост должен добить
  static constexpr uint32_t kStallMs = 12000;
  static constexpr uint32_t kStreamTimeoutMs = 120000;
  size_t streamFails = 0;
  totalSize = 1455600;
  sizeKnown = true;

  Serial.printf("[GH-OTA] stream+resume total~%u heap=%u maxBlk=%u\n",
                static_cast<unsigned>(totalSize),
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));

  while (written < totalSize) {
    if (millis() - tStart > kHardTimeoutMs) {
      Serial.printf("[GH-OTA] hard timeout after %u байт\n", static_cast<unsigned>(written));
      githubOtaClearUpdate("timeout");
      mbedtls_sha256_free(&sha);
      strlcpy(otaListStatus, "таймаут загрузки", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }

    // Хвост ?256КБ — через проверенный FS Range (stream на конце часто зависает).
    if ((totalSize - written) > 0 && (totalSize - written) <= (256u * 1024u)) {
      const size_t from = written;
      const size_t to = totalSize - 1;
      char rangeHdr[48];
      snprintf(rangeHdr, sizeof(rangeHdr), "bytes=%u-%u", static_cast<unsigned>(from),
               static_cast<unsigned>(to));
      Serial.printf("[GH-OTA] tail FS %s\n", rangeHdr);
      bool tailOk = false;
      for (int attempt = 1; attempt <= 5; attempt++) {
        const int code =
            githubHttpsDownloadToFile(firmwareUrl, "/ota_tail.bin", 120000, "*/*", rangeHdr);
        if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
          File tf = LittleFS.open("/ota_tail.bin", "r");
          if (tf && tf.size() > 0) {
            while (tf.available() && written < totalSize) {
              const int n = tf.read(rdBuf, rdCap);
              if (n <= 0) break;
              if (esp_ota_write(s_ghOtaHandle, rdBuf, static_cast<size_t>(n)) != ESP_OK) {
                tf.close();
                LittleFS.remove("/ota_tail.bin");
                githubOtaClearUpdate("tail-write");
                mbedtls_sha256_free(&sha);
                strlcpy(otaListStatus, "сбой записи", sizeof(otaListStatus));
                githubOtaEndInstallSession("dl-exit");
                return false;
              }
              mbedtls_sha256_update(&sha, rdBuf, static_cast<size_t>(n));
              written += static_cast<size_t>(n);
            }
            tf.close();
            LittleFS.remove("/ota_tail.bin");
            tailOk = (written >= totalSize);
            if (tailOk) break;
          }
        }
        LittleFS.remove("/ota_tail.bin");
        Serial.printf("[GH-OTA] tail attempt %d code fail, retry\n", attempt);
        vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
        if (attempt >= 2) {
          WiFi.reconnect();
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
      }
      if (!tailOk) {
        githubOtaClearUpdate("tail-fail");
        mbedtls_sha256_free(&sha);
        strlcpy(otaListStatus, "обрыв хвоста", sizeof(otaListStatus));
        githubOtaEndInstallSession("dl-exit");
        return false;
      }
      break;
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[GH-OTA] WiFi lost — reconnect");
      WiFi.reconnect();
      for (int w = 0; w < 50 && WiFi.status() != WL_CONNECTED; w++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        githubOtaHeartbeat();
      }
    }

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      streamFails++;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    otaConfigureTls(client);
    client->setHandshakeTimeout(30);
    client->setTimeout(kStreamTimeoutMs);

    int httpCode = -1;
    int contentLen = -1;
    {
      HTTPClient http;
      http.setConnectTimeout(25000);
      http.setTimeout(kStreamTimeoutMs);
      if (!http.begin(*client, firmwareUrl)) {
        Serial.println("[GH-OTA] stream begin fail");
        delete client;
        streamFails++;
        vTaskDelay(pdMS_TO_TICKS(800));
        continue;
      }
      http.useHTTP10(true);
      http.setReuse(false);
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      http.addHeader("User-Agent", "kamaz-leveler/9.4.0");
      http.addHeader("Accept", "*/*");
      http.addHeader("Accept-Encoding", "identity");
      http.addHeader("Connection", "close");
      const char *hk[] = {"Content-Range", "Content-Length"};
      http.collectHeaders(hk, 2);
      if (written > 0) {
        char rangeHdr[48];
        snprintf(rangeHdr, sizeof(rangeHdr), "bytes=%u-", static_cast<unsigned>(written));
        http.addHeader("Range", rangeHdr);
        Serial.printf("[GH-OTA] resume %s\n", rangeHdr);
      }

      githubOtaHeartbeat();
      const uint32_t t0 = millis();
      httpCode = http.GET();
      contentLen = http.getSize();
      Serial.printf("[GH-OTA] stream GET > %d size=%d in %lums written=%u\n", httpCode, contentLen,
                    static_cast<unsigned long>(millis() - t0), static_cast<unsigned>(written));

      if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_PARTIAL_CONTENT) {
        http.end();
      } else if (written > 0 && httpCode != HTTP_CODE_PARTIAL_CONTENT) {
        // Resume без 206 — сервер отдал файл с начала; не дописываем поверх.
        Serial.println("[GH-OTA] resume: ожидали 206 Partial, получили 200 — abort chunk");
        http.end();
        httpCode = -1;
      } else {
        String cr = http.header("Content-Range");
        if (written > 0) {
          // Content-Range: bytes START-END/TOTAL
          long rangeStart = -1;
          if (cr.startsWith("bytes ")) {
            rangeStart = cr.substring(6).toInt();
          }
          if (rangeStart < 0 || static_cast<size_t>(rangeStart) != written) {
            Serial.printf("[GH-OTA] resume: Content-Range start=%ld != written=%u — abort\n",
                          rangeStart, static_cast<unsigned>(written));
            http.end();
            httpCode = -1;
          }
        }
        if (httpCode < 0) {
          // already aborted
        } else if (cr.length() > 0) {
          const int slash = cr.lastIndexOf('/');
          if (slash >= 0) {
            const size_t ts = static_cast<size_t>(cr.substring(slash + 1).toInt());
            if (ts > 100000u) {
              totalSize = ts;
              sizeKnown = true;
            }
          }
        } else if (httpCode == HTTP_CODE_OK && contentLen > 100000) {
          totalSize = static_cast<size_t>(contentLen);
          sizeKnown = true;
        }

        if (httpCode < 0) {
          // skip body
        } else {
        WiFiClient *stream = http.getStreamPtr();
        size_t sessionGot = 0;
        const size_t expect =
            (contentLen > 0) ? static_cast<size_t>(contentLen) : (totalSize - written);
        client->setTimeout(8000);
        const uint32_t tRead0 = millis();
        uint32_t lastDataMs = millis();
        while (sessionGot < expect && written < totalSize &&
               (millis() - tRead0) < kStreamTimeoutMs) {
          githubOtaHeartbeat();
          if ((millis() - lastDataMs) > 25000u) {
            Serial.println("[GH-OTA] stream stall 25s — resume");
            break;
          }
          size_t want = rdCap;
          if (expect - sessionGot < want) want = expect - sessionGot;
          if (totalSize - written < want) want = totalSize - written;
          const int n = stream->readBytes(rdBuf, want);
          if (n <= 0) {
            if (!client->connected()) break;
            delay(20);
            continue;
          }
          lastDataMs = millis();
          const esp_err_t werr = esp_ota_write(s_ghOtaHandle, rdBuf, static_cast<size_t>(n));
          if (werr != ESP_OK) {
            Serial.printf("[GH-OTA] esp_ota_write fail at %u: %s\n",
                          static_cast<unsigned>(written), esp_err_to_name(werr));
            http.end();
            client->stop();
            delete client;
            githubOtaClearUpdate("write-fail");
            mbedtls_sha256_free(&sha);
            strlcpy(otaListStatus, "сбой записи", sizeof(otaListStatus));
            githubOtaEndInstallSession("dl-exit");
            return false;
          }
          mbedtls_sha256_update(&sha, rdBuf, static_cast<size_t>(n));
          written += static_cast<size_t>(n);
          sessionGot += static_cast<size_t>(n);

          otaProgress = static_cast<int>((written * 100u) / totalSize);
          if (otaProgress > 99) otaProgress = 99;
          if (otaProgress != lastPct) {
            lastPct = otaProgress;
            displayDirty = true;
            if ((otaProgress % 5) == 0 || otaProgress >= 99) {
              Serial.printf("[GH-OTA] download %d%% (%u/%u) heap=%u\n", otaProgress,
                            static_cast<unsigned>(written), static_cast<unsigned>(totalSize),
                            static_cast<unsigned>(ESP.getFreeHeap()));
            }
          }
        }
        Serial.printf("[GH-OTA] session +%u total=%u expect=%u connected=%d\n",
                      static_cast<unsigned>(sessionGot), static_cast<unsigned>(written),
                      static_cast<unsigned>(expect), client->connected() ? 1 : 0);
        http.end();
        if (sessionGot > 0) streamFails = 0;
        else streamFails++;
        }  // else: read body
      }    // else: OK/206 path
    }
    client->stop();
    delete client;

    if (written >= totalSize) break;

    if (streamFails >= 10) {
      githubOtaClearUpdate("stream-fail");
      mbedtls_sha256_free(&sha);
      strlcpy(otaListStatus, "обрыв потока", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }
    if (httpCode <= 0) {
      Serial.println("[GH-OTA] stream fail > WiFi.reconnect()");
      WiFi.disconnect(false, false);
      vTaskDelay(pdMS_TO_TICKS(1000));
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(5000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(600));
    }
  }

  if (written < totalSize) {
    Serial.printf("[GH-OTA] incomplete %u/%u\n", static_cast<unsigned>(written),
                  static_cast<unsigned>(totalSize));
    githubOtaClearUpdate("incomplete");
    mbedtls_sha256_free(&sha);
    strlcpy(otaListStatus, "неполная загрузка", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  Serial.printf("[GH-OTA] скачано %u байт (chunked-FS)\n", static_cast<unsigned>(written));
  strlcpy(otaStatus, "Проверка SHA…", sizeof(otaStatus));
  otaProgress = 99;
  displayDirty = true;

  unsigned char digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  char actual[65];
  for (uint8_t i = 0; i < sizeof(digest); i++) {
    snprintf(actual + i * 2, 3, "%02x", digest[i]);
  }
  actual[64] = '\0';
  if (strcmp(actual, expected) != 0) {
    Serial.printf("[GH-OTA] SHA-256 mismatch: %s\n", releaseTag);
    Serial.printf("[GH-OTA] expected=%s\n", expected);
    Serial.printf("[GH-OTA] actual  =%s\n", actual);
    githubOtaClearUpdate("sha-mismatch");
    strlcpy(otaListStatus, "SHA не совпал", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  strlcpy(otaStatus, "Фиксация…", sizeof(otaStatus));
  displayDirty = true;
  const esp_err_t endErr = esp_ota_end(s_ghOtaHandle);
  s_ghOtaActive = false;
  s_ghOtaHandle = 0;
  if (endErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_end fail: %s\n", esp_err_to_name(endErr));
    s_ghOtaPart = nullptr;
    strlcpy(otaListStatus, "ошибка ota_end", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  const esp_err_t bootErr = esp_ota_set_boot_partition(s_ghOtaPart);
  if (bootErr != ESP_OK) {
    Serial.printf("[GH-OTA] set_boot fail: %s\n", esp_err_to_name(bootErr));
    s_ghOtaPart = nullptr;
    strlcpy(otaListStatus, "ошибка boot part", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  Serial.printf("[GH-OTA] OK set_boot %s > reboot\n", releaseTag ? releaseTag : "?");
  s_ghOtaPart = nullptr;
  otaProgress = 100;
  strlcpy(otaStatus, "Готово, перезагрузка", sizeof(otaStatus));
  displayDirty = true;
  githubOtaEndInstallSession("dl-ok");
  return true;
}

bool checkGitHubUpdate(bool install) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Подключите устройство к Wi-Fi с интернетом");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }

  // Сначала список тегов (лёгкий JSON); latest release API слишком тяжёлый для ESP32
  if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
    if (otaListStatus[0] == '\0') {
      strlcpy(otaListStatus, "нет ответа GitHub", sizeof(otaListStatus));
    }
    return false;
  }

  static char tagBuf[16];
  static char binBuf[176];
  static char shaBuf[176];
  strlcpy(tagBuf, otaReleases[0].tag, sizeof(tagBuf));
  strlcpy(binBuf, otaReleases[0].binUrl, sizeof(binBuf));
  strlcpy(shaBuf, otaReleases[0].shaUrl, sizeof(shaBuf));
  strlcpy(otaLatestTag, tagBuf, sizeof(otaLatestTag));
  Serial.printf("[GH-OTA] Последний release: %s (локально %s)\n", tagBuf, VERSION);

  if (!install) {
    bool newer = isRemoteSemVerNewer(VERSION, tagBuf);
    if (newer) {
      snprintf(otaListStatus, sizeof(otaListStatus), "есть обновление %s", tagBuf);
    } else {
      SemVer local{}, remote{};
      if (parseSemVer(VERSION, local) && parseSemVer(tagBuf, remote) &&
          compareSemVer(local, remote) > 0) {
        snprintf(otaListStatus, sizeof(otaListStatus), "новее GitHub (%s)", tagBuf);
      } else {
        snprintf(otaListStatus, sizeof(otaListStatus), "актуально (%s)", tagBuf);
      }
    }
    Serial.printf("[GH-OTA] remote newer: %s\n", newer ? "yes" : "no");
    return newer;
  }

  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(binBuf, shaBuf, tagBuf)) {
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

  if (now - lastRequestTime < 80) {
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

/* --- Бегущая строка через GEM setTitle (без оверлея) --- */
static MarqueeSlot g_mq[MQ_COUNT];
static GEMPage *s_prevMenuPage = nullptr;

// Ширина строки меню CourierCyr9 ? 28 глифов (не байт UTF-8!)
static constexpr size_t MQ_MAX_GLYPHS = 28;

static uint8_t mqTextStep(const char *s, size_t i) {
  const uint8_t c = static_cast<uint8_t>(s[i]);
  if (c == 0) return 0;
  if (c < 0x80) return 1;
  if ((c & 0xE0) == 0xC0 && (static_cast<uint8_t>(s[i + 1]) & 0xC0) == 0x80) return 2;
  if ((c & 0xF0) == 0xE0 && (static_cast<uint8_t>(s[i + 1]) & 0xC0) == 0x80 &&
      (static_cast<uint8_t>(s[i + 2]) & 0xC0) == 0x80)
    return 3;
  return 1;
}

static size_t mqGlyphCount(const char *s) {
  size_t n = 0, i = 0;
  while (s[i]) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    i += st;
    n++;
  }
  return n;
}

static size_t mqAlignOffset(const char *s, size_t off) {
  size_t i = 0;
  while (s[i] && i < off) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    if (i + st > off) return i;
    i += st;
  }
  return i;
}

/** Скопировать до maxGlyphs символов UTF-8 из src в dst; вернуть число байт. */
static size_t mqCopyGlyphs(char *dst, size_t dstCap, const char *src, size_t maxGlyphs) {
  size_t w = 0, g = 0, i = 0;
  while (src[i] && g < maxGlyphs && w + 1 < dstCap) {
    const uint8_t st = mqTextStep(src, i);
    if (st == 0 || w + st >= dstCap) break;
    memcpy(dst + w, src + i, st);
    w += st;
    i += st;
    g++;
  }
  dst[w] = '\0';
  return w;
}

static void mqRender(MarqueeSlot &m) {
  if (!m.item) return;
  const size_t labelGlyphs = mqGlyphCount(m.label);
  if (labelGlyphs >= MQ_MAX_GLYPHS) {
    mqCopyGlyphs(m.shown, sizeof(m.shown), m.label, MQ_MAX_GLYPHS);
    m.item->setTitle(m.shown);
    m.scrolling = false;
    return;
  }

  const size_t availGlyphs = MQ_MAX_GLYPHS - labelGlyphs;
  const size_t labelBytes = mqCopyGlyphs(m.shown, sizeof(m.shown), m.label, labelGlyphs);
  const size_t vlenGlyphs = mqGlyphCount(m.value);

  if (vlenGlyphs <= availGlyphs) {
    mqCopyGlyphs(m.shown + labelBytes, sizeof(m.shown) - labelBytes, m.value, availGlyphs);
    m.scrolling = false;
  } else if (availGlyphs == 0) {
    m.shown[labelBytes] = '\0';
    m.scrolling = false;
  } else {
    m.scrolling = true;
    char loopBuf[160];
    snprintf(loopBuf, sizeof(loopBuf), "%s   ", m.value);
    const size_t loopLen = strlen(loopBuf);
    size_t pos = (loopLen > 0) ? mqAlignOffset(loopBuf, m.offset % loopLen) : 0;
    size_t wpos = labelBytes;
    size_t copied = 0;
    while (copied < availGlyphs && wpos + 1 < sizeof(m.shown) && loopLen > 0) {
      if (!loopBuf[pos]) pos = 0;
      const uint8_t st = mqTextStep(loopBuf, pos);
      if (st == 0 || wpos + st >= sizeof(m.shown)) break;
      memcpy(m.shown + wpos, loopBuf + pos, st);
      wpos += st;
      pos += st;
      copied++;
    }
    m.shown[wpos] = '\0';
  }
  m.item->setTitle(m.shown);
}

static void mqBind(MarqueeId id, GEMItem &item, const char *label, const char *value) {
  MarqueeSlot &m = g_mq[id];
  const bool changed = !m.enabled || m.item != &item || strcmp(m.label, label ? label : "") != 0 ||
                       strcmp(m.value, value ? value : "") != 0;
  m.item = &item;
  m.enabled = true;
  strlcpy(m.label, label ? label : "", sizeof(m.label));
  strlcpy(m.value, value ? value : "", sizeof(m.value));
  if (changed) m.offset = 0;
  mqRender(m);
}

static bool mqItemOnCurrentPage(GEMItem *item) {
  GEMPage *page = gem.getCurrentMenuPage();
  if (!page || !item) return false;
  GEMItem *it = page->getMenuItem(0);
  while (it != nullptr) {
    if (it == item) return true;
    it = it->getMenuItemNext();
  }
  return false;
}

/** Восстановить GFX-шрифт пунктов GEM (после ui/U8g2 иначе кириллица «ломается»). */
static void gemRestoreMenuItemFont() {
  tft.setFont(&CourierCyr9pt8b);  // как gem.setFontBig — пункты меню
  tft.setTextSize(1);
  tft.setTextWrap(false);
}

/** Y верха строки пункта на текущем экране GEM; -1 если не виден. */
static int16_t mqItemRowTop(GEMItem *item, bool *selectedOut) {
  if (selectedOut) *selectedOut = false;
  if (!item || !menuVisible) return -1;
  GEMPage *page = gem.getCurrentMenuPage();
  if (!page) return -1;

  const byte perScreen = static_cast<byte>((SCREEN_HEIGHT - MENU_TOP_OFFSET) / ROW_HEIGHT);
  const byte focusIdx = page->getCurrentMenuItemIndex();
  const byte screenNum = focusIdx / perScreen;
  GEMItem *it = page->getMenuItem(screenNum * perScreen);
  byte i = 0;
  int16_t y = MENU_TOP_OFFSET;
  while (it != nullptr && i < perScreen) {
    if (it == item) {
      if (selectedOut) *selectedOut = (it == page->getCurrentMenuItem());
      return y;
    }
    it = it->getMenuItemNext();
    y += ROW_HEIGHT;
    i++;
  }
  return -1;
}

/** Перерисовать только строку бегущей ленты — без fillScreen всего меню. */
static void mqSoftRedrawRow(MarqueeSlot &m) {
  bool selected = false;
  const int16_t rowTop = mqItemRowTop(m.item, &selected);
  if (rowTop < 0) return;

  gemRestoreMenuItemFont();
  // Как GEM_POINTER_ROW: выбранная строка = инверсия (белый фон, чёрный текст)
  const uint16_t bg = selected ? MENU_TEXT_COLOR : MENU_BG_COLOR;
  const uint16_t textFg = selected ? MENU_BG_COLOR : MENU_TEXT_COLOR;
  tft.fillRect(0, rowTop, SCREEN_WIDTH - 2, ROW_HEIGHT, bg);

  // CourierCyr9: baseline ? 3/4 высоты строки
  const int16_t textY = rowTop + ((ROW_HEIGHT > 18) ? (ROW_HEIGHT * 3) / 4 : ROW_HEIGHT - 4);
  tft.setTextColor(textFg);
  tft.setCursor(5, textY);
  const char *s = m.shown;
  size_t glyphs = 0;
  for (size_t i = 0; s[i] != '\0' && glyphs < MQ_MAX_GLYPHS; ) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    for (uint8_t b = 0; b < st; b++) tft.write(static_cast<uint8_t>(s[i + b]));
    i += st;
    glyphs++;
  }
}

/** Сдвиг бегущих строк — только своя строка, экран целиком не мигает. */
static bool mqTick() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < 280) return false;
  last = now;

  bool any = false;
  for (uint8_t id = 0; id < MQ_COUNT; id++) {
    MarqueeSlot &m = g_mq[id];
    if (!m.enabled || !m.item || !m.scrolling) continue;
    if (!mqItemOnCurrentPage(m.item)) continue;
    char loopBuf[160];
    snprintf(loopBuf, sizeof(loopBuf), "%s   ", m.value);
    const size_t loopLen = strlen(loopBuf);
    if (loopLen == 0) continue;
    size_t pos = mqAlignOffset(loopBuf, m.offset % loopLen);
    const uint8_t st = mqTextStep(loopBuf, pos);
    m.offset = static_cast<uint16_t>(pos + (st ? st : 1));
    mqRender(m);
    mqSoftRedrawRow(m);
    any = true;
  }
  return any;
}

/** При входе на страницу «Обновления» — сразу проверка. */
static void mqOnMenuPageEnter() {
  GEMPage *page = gem.getCurrentMenuPage();
  if (page == &otaPage && s_prevMenuPage != &otaPage) {
    otaCheckUpdates();
  }
  s_prevMenuPage = page;
}

/** Живые строки страницы «Обновления». */
static void refreshOtaPage() {
  // Короткие подписи > больше места под бегущее значение
  mqBind(MQ_OTA_VER, itemOtaCurrent, "Верс: ", VERSION);
  mqBind(MQ_OTA_ST, itemOtaStatus, "Статус: ", otaListStatus);
  // «Выход из режима OTA» показываем только когда режим активен
  if (currentState == SystemState::OTA_MODE) {
    itemOtaExit.show();
  } else {
    itemOtaExit.hide();
  }
}

/** Заголовки пунктов списка прошивок (только тег, без даты). */
static void refreshOtaListPage() {
  mqBind(MQ_LIST_HDR, itemOtaListHdr, "Статус: ", otaListHdrBuf);
  itemOtaListHdr.show();
  for (uint8_t i = 0; i < OTA_LIST_MAX; i++) {
    if (i < otaReleaseCount) {
      const OtaRelease &r = otaReleases[i];
      if (r.shaUrl[0] == '\0') {
        snprintf(otaItemTitle[i], sizeof(otaItemTitle[i]), "%s !sha", r.tag);
      } else {
        snprintf(otaItemTitle[i], sizeof(otaItemTitle[i]), "%s", r.tag);
      }
      otaRelItems[i]->setTitle(otaItemTitle[i]);
      otaRelItems[i]->show();
    } else {
      snprintf(otaItemTitle[i], sizeof(otaItemTitle[i]), "—");
      otaRelItems[i]->setTitle(otaItemTitle[i]);
      otaRelItems[i]->hide();
    }
  }
}

/** Строки карточки выбранного релиза. */
static void refreshOtaCard() {
  if (otaSelectedIndex < 0 || otaSelectedIndex >= static_cast<int8_t>(otaReleaseCount)) {
    mqBind(MQ_CARD_TAG, itemCardTag, "Релиз: ", "—");
    mqBind(MQ_CARD_INFO, itemCardInfo, "Инфо: ", "данных нет");
    mqBind(MQ_CARD_ST, itemCardStatus, "Статус: ", "откройте список");
    mqBind(MQ_CARD_SHA, itemCardSha, "SHA-256: ", "не проверен");
    return;
  }

  const OtaRelease &r = otaReleases[otaSelectedIndex];
  static char sInfo[40];
  static char sStatusVal[48];
  static char sShaVal[72];
  snprintf(sInfo, sizeof(sInfo),
           (r.size > 0) ? "%lu КБ" : "размер н/д",
           static_cast<unsigned long>(r.size / 1024));

  const bool newer = isRemoteSemVerNewer(VERSION, r.tag);
  const bool installed = (strstr(VERSION, r.tag + 1) != nullptr) || (strstr(VERSION, r.tag) != nullptr);
  strlcpy(sStatusVal, installed ? "установлена" : (newer ? "НОВЕЕ установленной" : "не новее"),
          sizeof(sStatusVal));

  if (r.sha256[0] != '\0') {
    snprintf(sShaVal, sizeof(sShaVal), "%.16s…%.8s", r.sha256, r.sha256 + 56);
  } else {
    strlcpy(sShaVal, "не проверен", sizeof(sShaVal));
  }

  mqBind(MQ_CARD_TAG, itemCardTag, "Релиз: ", r.tag);
  mqBind(MQ_CARD_INFO, itemCardInfo, "Инфо: ", sInfo);
  mqBind(MQ_CARD_ST, itemCardStatus, "Статус: ", sStatusVal);
  mqBind(MQ_CARD_SHA, itemCardSha, "SHA-256: ", sShaVal);
}

/** Живые строки страницы «Просмотр» — сводка по доменам меню. */
static void refreshSettingsView() {
  static char b1[52], b2[52], b3[52], b4[52], b5[52], b6[52], b7[52];
  // Давление: лимиты, deadband, выравнивание МП, пауза опроса
  snprintf(b1, sizeof(b1), "Давл %.1f-%.1f МПн%.1f з%.2f", ConfigManager::getPressureMin(),
           ConfigManager::getPressureMax(), ConfigManager::getMasterLowBar(),
           ConfigManager::getPressureDeadband());
  // Клапаны: только импульсы и макс.время
  snprintf(b2, sizeof(b2), "Клап сбр%d/нак%dс макс %dс", ConfigManager::getReleaseDelay(),
           ConfigManager::getInflateDelay(), ConfigManager::getManualMaxTimeSec());
  // Авто: наклоны + зоны + попытки
  snprintf(b3, sizeof(b3), "Авто %.2f/%.2f° %.2f/%.2f/%.2f", ConfigManager::getTiltThresholdX(),
           ConfigManager::getTiltThresholdY(), ConfigManager::getCoarseZoneRatio(),
           ConfigManager::getFineZoneRatio(), ConfigManager::getWorseningRatio());
  snprintf(b4, sizeof(b4), "Авто поп%d инт%dm МП%dс", ConfigManager::getNivCount(),
           ConfigManager::getTimeInterval(), ConfigManager::getMasterCheckSec());
  // Движение: цели давления + порог MOT
  snprintf(b5, sizeof(b5), "Движ %.1f/%.1f бар MOT%d", ConfigManager::getMovementPressureFront(),
           ConfigManager::getMovementPressureRear(), ConfigManager::getImuMotionDet());
  // Дисплей
  snprintf(b6, sizeof(b6), "Дисп ярк%d %dмин %.2f°/%.2fб", ConfigManager::getContrast(),
           ConfigManager::getBacklightOffMin(), ConfigManager::getRedrawAngleThr(),
           ConfigManager::getRedrawPressureThr());
  // IMU фильтр углов
  snprintf(b7, sizeof(b7), "IMU K%.1f/%.1f/%.3f %dмс", ConfigManager::getImuKalmanMea(),
           ConfigManager::getImuKalmanEst(), ConfigManager::getImuKalmanQ(),
           ConfigManager::getImuPollMs());
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

  static char sMode[28];
  static char sMpu[28];
  static char sMaster[28];
  static char sWifiVal[48];
  static char sSysVal[48];
  static char sErr[28];

  float localMaster = 0.0f;
  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(20));
    if (guard) localMaster = masterPressure;
  }

  const char *modeStr = (currentSystemMode == SystemMode::AUTO)       ? "АВТО"
                        : (currentSystemMode == SystemMode::MOVEMENT) ? "ДВИЖ"
                                                                     : "РУЧ";

  snprintf(sMode, sizeof(sMode), "Режим: %s", modeStr);
  snprintf(sMpu, sizeof(sMpu), "MPU: %s", mpuOk ? "OK" : "ОШИБКА");
  snprintf(sMaster, sizeof(sMaster), "МП: %.1f бар", localMaster);

  if (WiFi.status() == WL_CONNECTED) {
    snprintf(sWifiVal, sizeof(sWifiVal), "%s %d dBm", sta_ssid, static_cast<int>(WiFi.RSSI()));
  } else if (sta_ssid[0] != '\0') {
    snprintf(sWifiVal, sizeof(sWifiVal), "нет связи (%s)", sta_ssid);
  } else {
    snprintf(sWifiVal, sizeof(sWifiVal), "AP %s", wifi_ssid);
  }

  snprintf(sSysVal, sizeof(sSysVal), "%lu ч  RAM %lu КБ",
           static_cast<unsigned long>(uptimeHours),
           static_cast<unsigned long>(ESP.getFreeHeap() / 1024));
  snprintf(sErr, sizeof(sErr), "Ошибки: %d", ErrorHandler::getActiveErrorCount());

  mqBind(MQ_INFO_VER, itemInfoVersion, "Версия: ", VERSION);
  itemInfoMode.setTitle(sMode);
  itemInfoMpu.setTitle(sMpu);
  itemInfoMaster.setTitle(sMaster);
  mqBind(MQ_INFO_WIFI, itemInfoWiFi, "Wi-Fi: ", sWifiVal);
  mqBind(MQ_INFO_SYS, itemInfoSystem, "Аптайм: ", sSysVal);
  itemInfoErrors.setTitle(sErr);
}

/** Подсказка по кнопкам меню — рисуется поверх GEM в правой части шапки. */
static void menuButtonsHint() {
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::SCREEN_W - 200, 0, 200, 14, "КН3+КН4 выход",
         UiHAlign::Right, UiVAlign::Middle, true);
  // U8g2 не должен оставлять GFX без шрифта GEM — иначе при смене пункта кириллица «кракозябрами»
  gemRestoreMenuItemFont();
}

/** Кн4 на главной странице меню: сохранить настройки и выйти (как Кн5). */
void menuExitAction() {
  requestMenuClose("menuExitAction");
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

/** Подпись спиннера Низк.МП: индекс 0…40 > «0.00»…«4.00» (не float-спиннер GEM). */
static const char *masterLowSpinnerLabel(GEMSpinner * /*spinner*/, int index, GEMItem * /*item*/) {
  static char buf[8];
  float v = constrain(index, 0, 40) * 0.1f;
  dtostrf(v, 4, 2, buf);
  return buf;
}

/** Подпись спиннера пресета MPU: 0=Плавно, 1=Быстро, 2=Баланс. */
static const char *imuPresetSpinnerLabel(GEMSpinner * /*spinner*/, int index, GEMItem * /*item*/) {
  static const char *const names[] = {"Плавно", "Быстро", "Баланс"};
  if (index < 0 || index > 2) return "—";
  return names[index];
}

/** Загрузить параметры пресета IMU в зеркала edit* (нужен СОХРАНИТЬ).
 *  Значения кратны шагам спиннеров (slew step=5 и т.д.) и совпадают с TEST IMU PRESET.
 *  10.1.28: Баланс/Плавно чуть ослаблены (быстрее реакция, меньше «залипание»). */
static void loadImuPresetToEdit(int idx) {
  switch (constrain(idx, 0, 2)) {
    case 0:  // Плавно (= SMOOTH) — ослаблено
      editImuKalmanMea = 8.0f; editImuKalmanEst = 5.0f; editImuKalmanQ = 0.004f;
      editImuPollMs = 25; editImuFifoAvg = 4;
      editImuEmaAlpha = 0.18f; editImuEmaSpikeAlpha = 0.07f; editImuEmaSpikeThr = 1.4f;
      editImuSlewDps = 25.0f; editRedrawAngle = 0.07f;
      break;
    case 1:  // Быстро (= FAST) — без изменений
      editImuKalmanMea = 3.5f; editImuKalmanEst = 3.0f; editImuKalmanQ = 0.015f;
      editImuPollMs = 15; editImuFifoAvg = 1;
      editImuEmaAlpha = 0.50f; editImuEmaSpikeAlpha = 0.18f; editImuEmaSpikeThr = 2.5f;
      editImuSlewDps = 90.0f; editRedrawAngle = 0.04f;
      break;
    default: // Баланс (= BAL) — ослаблено
      editImuKalmanMea = 5.5f; editImuKalmanEst = 3.5f; editImuKalmanQ = 0.008f;
      editImuPollMs = 20; editImuFifoAvg = 2;
      editImuEmaAlpha = 0.28f; editImuEmaSpikeAlpha = 0.10f; editImuEmaSpikeThr = 1.8f;
      editImuSlewDps = 70.0f; editRedrawAngle = 0.05f;
      break;
  }
  editImuPreset = constrain(idx, 0, 2);
}

void initGEM() {
    // Настройка внешнего вида (sprites=nullptr > дефолтные иконки GEM)
    GEMAppearance appearance{};
    appearance.menuPointerType = GEM_POINTER_ROW;
    appearance.menuItemsPerScreen = GEM_ITEMS_COUNT_AUTO;
    appearance.menuItemHeight = ROW_HEIGHT;
    appearance.menuPageScreenTopOffset = MENU_TOP_OFFSET;
    appearance.menuValuesLeftOffset = MENU_VALUES_LEFT_OFFSET;
    appearance.sprites = nullptr;

    gem.setAppearance(appearance);
    gem.setBackgroundColor(MENU_BG_COLOR);
    gem.setForegroundColor(MENU_TEXT_COLOR);
    gem.setFontBig(&CourierCyr9pt8b);
    gem.setFontSmall(&CourierCyr7pt8b);
    gem.setDrawMenuCallback(menuButtonsHint);   // подсказка по кнопкам в шапке меню

    // --- Страница "Система" (OTA/обновления вынесены в раздел «Обновления») ---
    static GEMItem itemReset("Сброс ошибок", []() { resetSystemErrors(); });
    static GEMItem itemWiFi("Настроить WiFi", []() { requestWiFiSetup(); });
    systemPage.addMenuItem(itemReset);
    systemPage.addMenuItem(itemWiFi);

    // --- Страница "Обновления" ---
    otaPage.addMenuItem(itemOtaCurrent);
    otaPage.addMenuItem(itemOtaStatus);
    otaPage.addMenuItem(itemOtaList);
    otaPage.addMenuItem(itemOtaInstallLast);
    otaPage.addMenuItem(itemOtaArduino);
    otaPage.addMenuItem(itemOtaExit);

    otaListPage.addMenuItem(itemOtaListHdr);
    otaListPage.addMenuItem(itemOtaRel0);
    otaListPage.addMenuItem(itemOtaRel1);
    otaListPage.addMenuItem(itemOtaRel2);

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

    // --- Страница "Клапаны" (только импульсы и лимит ручной операции) ---
    static GEMItem itemReleaseDelay("Сброс,с", editReleaseDelay, spinnerRelease, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editReleaseDelay = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemInflateDelay("Накачка,с", editInflateDelay, spinnerInflate, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editInflateDelay = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemManualMaxTime("Макс.время,с", editManualMaxTime, spinnerManualMaxTime, [](GEMCallbackData d) { menuSpinChanged(d, editManualMaxTime); });
    valvePage.addMenuItem(itemReleaseDelay);
    valvePage.addMenuItem(itemInflateDelay);
    valvePage.addMenuItem(itemManualMaxTime);

    // --- Страница "Давление" (лимиты, deadband, опрос МП) ---
    static GEMItem itemPressureMin("P мин,бар", editPressureMin, spinnerPressureMin, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editPressureMin = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemPressureMax("P макс,бар", editPressureMax, spinnerPressureMax, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editPressureMax = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemMasterLow("Низк.МП,бар", editMasterLowTenths, spinnerMasterLow, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (!ptr) return;
        editMasterLowTenths = constrain(*(int*)ptr, 0, 40);
        *(int*)ptr = editMasterLowTenths;
        const float bar = editMasterLowTenths * 0.1f;
        ConfigManager::setMasterLowBar(bar);  // сразу в рантайм — не ждать СОХРАНИТЬ
        settingsChanged = true;
        displayDirty = true;
        if (editMasterLowTenths <= 0 &&
            ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
          ErrorHandler::removeError(ErrorHandler::Error::LOW_PRESSURE);
          Serial.println("[MENU] LOW_PRESSURE снята (Низк.МП=0.00)");
        }
    });
    static GEMItem itemDeadband("Зона нечув,бар", editDeadband, spinnerDeadband, [](GEMCallbackData d) { menuSpinChanged(d, editDeadband); });
    static GEMItem itemMasterCheck("Проверка МП,с", editMasterCheck, spinnerMasterCheck, [](GEMCallbackData d) { menuSpinChanged(d, editMasterCheck); });
    static GEMItem itemPressStabilize("Выравн.МП,мс", editPressStabilizeMs, spinnerPressStabilize, [](GEMCallbackData d) { menuSpinChanged(d, editPressStabilizeMs); });
    static GEMItem itemPressIdle("Пауза опроса,мин", editPressIdleMin, spinnerPressIdle, [](GEMCallbackData d) { menuSpinChanged(d, editPressIdleMin); });
    pressurePage.addMenuItem(itemPressureMin);
    pressurePage.addMenuItem(itemPressureMax);
    pressurePage.addMenuItem(itemMasterLow);
    pressurePage.addMenuItem(itemDeadband);
    pressurePage.addMenuItem(itemMasterCheck);
    pressurePage.addMenuItem(itemPressStabilize);
    pressurePage.addMenuItem(itemPressIdle);

    // --- Страница "Авторежим" (пороги наклона + зоны + лимиты попыток) ---
    static GEMItem itemTiltX("Поперечный,гра", editTiltX, spinnerTiltX, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTiltX = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemTiltY("Продольный,гра", editTiltY, spinnerTiltY, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTiltY = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemNivCount("Попыток в час", editNivCount, spinnerNivCount, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editNivCount = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemTimeInterval("Интервал,мин", editTimeInterval, spinnerTimeInterval, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editTimeInterval = *(int*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemCoarseZone("Грубая,пор.", editCoarseZone, spinnerCoarseZone, [](GEMCallbackData d) { menuSpinChanged(d, editCoarseZone); });
    static GEMItem itemFineZone("Точная,пор.", editFineZone, spinnerFineZone, [](GEMCallbackData d) { menuSpinChanged(d, editFineZone); });
    static GEMItem itemWorsening("Хуже", editWorsening, spinnerWorsening, [](GEMCallbackData d) { menuSpinChanged(d, editWorsening); });
    itemCoarseZone.setMultiplySep(true);
    itemFineZone.setMultiplySep(true);
    itemWorsening.setMultiplySep(true);
    autoPage.addMenuItem(itemTiltX);
    autoPage.addMenuItem(itemTiltY);
    autoPage.addMenuItem(itemCoarseZone);
    autoPage.addMenuItem(itemFineZone);
    autoPage.addMenuItem(itemWorsening);
    autoPage.addMenuItem(itemNivCount);
    autoPage.addMenuItem(itemTimeInterval);

    // --- Страница "Дисплей" ---
    static GEMItem itemContrast("Яркость,%", editContrast, spinnerContrast, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editContrast = constrain(*(int*)ptr, CONTRAST_MIN, CONTRAST_MAX);
            backlightDimmed = false;
            applyBacklightPwm(editContrast);
            settingsChanged = true;
        }
    });
    itemContrast.setPreviewCallback([](GEMPreviewCallbackData d) {
      if (d.type == GEM_VAL_INTEGER) {
        backlightDimmed = false;
        applyBacklightPwm(constrain(d.previewValInt, CONTRAST_MIN, CONTRAST_MAX));
      }
    });
    static GEMItem itemBacklightOff("Приглуш.,мин", editBacklightOff, spinnerBacklightOff, [](GEMCallbackData d) {
      menuSpinChanged(d, editBacklightOff);
      ConfigManager::setBacklightOffMin(editBacklightOff);
      backlightDimmed = false;
      lastUserActivityMs = millis();
      applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
      Serial.printf("[DISP] Приглушение через %d мин (0=выкл)\n", editBacklightOff);
    });
    static GEMItem itemFrameMs("Интервал,мс", editFrameMs, spinnerFrameMs, [](GEMCallbackData d) { menuSpinChanged(d, editFrameMs); });
    static GEMItem itemRedrawAngle("Порог углов,гра", editRedrawAngle, spinnerRedrawAngle, [](GEMCallbackData d) { menuSpinChanged(d, editRedrawAngle); });
    static GEMItem itemRedrawPressure("Порог давл,бар", editRedrawPressure, spinnerRedrawPressure, [](GEMCallbackData d) { menuSpinChanged(d, editRedrawPressure); });
    displayPage.addMenuItem(itemContrast);
    displayPage.addMenuItem(itemBacklightOff);
    displayPage.addMenuItem(itemFrameMs);
    displayPage.addMenuItem(itemRedrawAngle);
    displayPage.addMenuItem(itemRedrawPressure);

    // --- Страница «Движение» (режим MOVEMENT + аппаратный MOT) ---
    static GEMItem itemMovementFront("Давл.перед,бар", editMovementPressureFront, spinnerMovementFront, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editMovementPressureFront = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemMovementRear("Давл.зад,бар", editMovementPressureRear, spinnerMovementRear, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editMovementPressureRear = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemParkingPressure("Давл.стоянки,бар", editParkingPressure, spinnerParkingPressure, [](GEMCallbackData data) {
        void* ptr = data.pMenuItem->getLinkedVariablePointer();
        if (ptr) {
            editParkingPressure = *(float*)ptr;
            settingsChanged = true;
            displayDirty = true;
        }
    });
    static GEMItem itemMoveDuration("До входа,с", editMoveDuration, spinnerMoveDuration, [](GEMCallbackData d) { menuSpinChanged(d, editMoveDuration); });
    static GEMItem itemMoveSettle("Успокоение,с", editMoveSettle, spinnerMoveSettle, [](GEMCallbackData d) { menuSpinChanged(d, editMoveSettle); });
    static GEMItem itemMoveCheck("Проверка давл.,с", editMoveCheck, spinnerMoveCheck, [](GEMCallbackData d) { menuSpinChanged(d, editMoveCheck); });
    static GEMItem itemMoveTolerance("Допуск давл.,бар", editMoveTolerance, spinnerMoveTolerance, [](GEMCallbackData d) { menuSpinChanged(d, editMoveTolerance); });
    static GEMItem itemImuMotionDet("Порог MOT,ед", editImuMotionDet, spinnerImuMotionDet, [](GEMCallbackData d) { menuSpinChanged(d, editImuMotionDet); });
    static GEMItem itemGyroThr("Порог Δgyro,ед", editGyroThreshold, spinnerGyroThreshold, [](GEMCallbackData d) { menuSpinChanged(d, editGyroThreshold); });
    static GEMItem itemGyroBump("Порог неровн.,ед", editGyroBumpThreshold, spinnerGyroBumpThreshold, [](GEMCallbackData d) { menuSpinChanged(d, editGyroBumpThreshold); });  // Δ bump = |bump−EMA|
    static GEMItem itemAccelThr("Порог Δlin,ед", editAccelThreshold, spinnerAccelThreshold, [](GEMCallbackData d) { menuSpinChanged(d, editAccelThreshold); });
    movementPage.addMenuItem(itemMovementFront);
    movementPage.addMenuItem(itemMovementRear);
    movementPage.addMenuItem(itemParkingPressure);
    movementPage.addMenuItem(itemMoveDuration);
    movementPage.addMenuItem(itemMoveSettle);
    movementPage.addMenuItem(itemMoveCheck);
    movementPage.addMenuItem(itemMoveTolerance);
    movementPage.addMenuItem(itemImuMotionDet);
    movementPage.addMenuItem(itemGyroThr);
    movementPage.addMenuItem(itemGyroBump);
    movementPage.addMenuItem(itemAccelThr);

    // --- Страница "Информация" ---
    infoPage.addMenuItem(itemInfoVersion);
    infoPage.addMenuItem(itemInfoMode);
    infoPage.addMenuItem(itemInfoMpu);
    infoPage.addMenuItem(itemInfoMaster);
    infoPage.addMenuItem(itemInfoWiFi);
    infoPage.addMenuItem(itemInfoSystem);
    infoPage.addMenuItem(itemInfoErrors);

    // --- Страница «IMU» (углы / фильтры; MOT — в «Движение») ---
    static GEMItem itemImuZero("Обнулить углы", []() { openImuZeroConfirm(); });
    static GEMItem itemImuCalib("Калибровка офсетов", []() { openImuCalibScreen(); });
    static GEMItem itemImuDiag("Диагностика MPU", []() { openMpuDiagScreen(); });
    static GEMItem itemImuPreset("Пресет фильтра", editImuPreset, spinnerImuPreset, [](GEMCallbackData data) {
        void *ptr = data.pMenuItem->getLinkedVariablePointer();
        if (!ptr) return;
        editImuPreset = constrain(*(int *)ptr, 0, 2);
        *(int *)ptr = editImuPreset;
        loadImuPresetToEdit(editImuPreset);
        settingsChanged = true;
        // Не ставим displayDirty: GEM после callback сам drawMenu() из ButtonTask;
        // второй полный кадр с DisplayTask давил SPI/стек при длинной странице IMU.
        Serial.printf("[IMU] Пресет «%s» загружен в меню (СОХРАНИТЬ)\n",
                      editImuPreset == 0 ? "Плавно" : (editImuPreset == 1 ? "Быстро" : "Баланс"));
    });
    static GEMItem itemZeroAngleX("Нуль крен,гра", editZeroAngleX, spinnerZeroAngleX, [](GEMCallbackData d) { menuSpinChanged(d, editZeroAngleX); });
    static GEMItem itemZeroAngleY("Нуль тангаж,гра", editZeroAngleY, spinnerZeroAngleY, [](GEMCallbackData d) { menuSpinChanged(d, editZeroAngleY); });
    static GEMItem itemImuKalmanMea("Калман измер.", editImuKalmanMea, spinnerImuKalmanMea, [](GEMCallbackData d) { menuSpinChanged(d, editImuKalmanMea); });
    static GEMItem itemImuKalmanEst("Калман оценка", editImuKalmanEst, spinnerImuKalmanEst, [](GEMCallbackData d) { menuSpinChanged(d, editImuKalmanEst); });
    static GEMItem itemImuKalmanQ("Калман Q", editImuKalmanQ, spinnerImuKalmanQ, [](GEMCallbackData d) { menuSpinChanged(d, editImuKalmanQ); });
    static GEMItem itemImuPollMs("Опрос,мс", editImuPollMs, spinnerImuPollMs, [](GEMCallbackData d) { menuSpinChanged(d, editImuPollMs); });
    static GEMItem itemImuFifoAvg("Усредн.пакетов", editImuFifoAvg, spinnerImuFifoAvg, [](GEMCallbackData d) { menuSpinChanged(d, editImuFifoAvg); });
    static GEMItem itemImuEmaAlpha("Сглажив.EMA", editImuEmaAlpha, spinnerImuEmaAlpha, [](GEMCallbackData d) { menuSpinChanged(d, editImuEmaAlpha); });
    static GEMItem itemImuEmaSpike("EMA при выбросе", editImuEmaSpikeAlpha, spinnerImuEmaSpikeAlpha, [](GEMCallbackData d) { menuSpinChanged(d, editImuEmaSpikeAlpha); });
    static GEMItem itemImuSpikeThr("Порог выброса,гра", editImuEmaSpikeThr, spinnerImuEmaSpikeThr, [](GEMCallbackData d) { menuSpinChanged(d, editImuEmaSpikeThr); });
    static GEMItem itemImuSlew("Макс.скорость,гра/с", editImuSlewDps, spinnerImuSlewDps, [](GEMCallbackData d) { menuSpinChanged(d, editImuSlewDps); });
    imuPage.addMenuItem(itemImuZero);
    imuPage.addMenuItem(itemImuCalib);
    imuPage.addMenuItem(itemImuDiag);
    imuPage.addMenuItem(itemImuPreset);
    imuPage.addMenuItem(itemZeroAngleX);
    imuPage.addMenuItem(itemZeroAngleY);
    imuPage.addMenuItem(itemImuKalmanMea);
    imuPage.addMenuItem(itemImuKalmanEst);
    imuPage.addMenuItem(itemImuKalmanQ);
    imuPage.addMenuItem(itemImuPollMs);
    imuPage.addMenuItem(itemImuFifoAvg);
    imuPage.addMenuItem(itemImuEmaAlpha);
    imuPage.addMenuItem(itemImuEmaSpike);
    imuPage.addMenuItem(itemImuSpikeThr);
    imuPage.addMenuItem(itemImuSlew);

    // --- Страница «Просмотр» (read-only сводка) ---
    settingsViewPage.addMenuItem(itemSet1);
    settingsViewPage.addMenuItem(itemSet2);
    settingsViewPage.addMenuItem(itemSet3);
    settingsViewPage.addMenuItem(itemSet4);
    settingsViewPage.addMenuItem(itemSet5);
    settingsViewPage.addMenuItem(itemSet6);
    settingsViewPage.addMenuItem(itemSet7);

    // --- Главное меню ---
    static GEMItem linkSystem("Система", &systemPage);
    static GEMItem linkOta("Обновления", &otaPage);
    static GEMItem linkTest("Тестирование", &testPage);
    static GEMItem linkPressure("Давление", &pressurePage);
    static GEMItem linkValve("Клапаны", &valvePage);
    static GEMItem linkAuto("Авторежим", &autoPage);
    static GEMItem linkMovement("Движение", &movementPage);
    static GEMItem linkDisplay("Дисплей", &displayPage);
    static GEMItem linkImu("IMU", &imuPage);
    static GEMItem linkSettings("Просмотр", &settingsViewPage);
    static GEMItem linkInfo("Информация", &infoPage);

    mainPage.addMenuItem(linkSystem);
    mainPage.addMenuItem(linkOta);
    mainPage.addMenuItem(linkTest);
    mainPage.addMenuItem(linkPressure);
    mainPage.addMenuItem(linkValve);
    mainPage.addMenuItem(linkAuto);
    mainPage.addMenuItem(linkMovement);
    mainPage.addMenuItem(linkDisplay);
    mainPage.addMenuItem(linkImu);
    mainPage.addMenuItem(linkSettings);
    mainPage.addMenuItem(linkInfo);

    // --- Кнопка сохранения ---
    static GEMItem itemSave("СОХРАНИТЬ", []() {
        requestMenuClose("SAVE");
    });
    mainPage.addMenuItem(itemSave);

    // Float: всегда два знака после запятой (x.xx)
    itemDeadband.setPrecision(2);
    itemTiltX.setPrecision(2);
    itemTiltY.setPrecision(2);
    itemPressureMin.setPrecision(2);
    itemPressureMax.setPrecision(2);
    spinnerMasterLow.setProduceOptionNameByIndexCallback(masterLowSpinnerLabel);
    spinnerImuPreset.setProduceOptionNameByIndexCallback(imuPresetSpinnerLabel);
    itemCoarseZone.setPrecision(2);
    itemFineZone.setPrecision(2);
    itemWorsening.setPrecision(2);
    itemRedrawAngle.setPrecision(2);
    itemRedrawPressure.setPrecision(2);
    itemMovementFront.setPrecision(2);
    itemMovementRear.setPrecision(2);
    itemMoveTolerance.setPrecision(2);
    itemZeroAngleX.setPrecision(2);
    itemZeroAngleY.setPrecision(2);
    itemImuKalmanMea.setPrecision(1);
    itemImuKalmanEst.setPrecision(1);
    itemImuKalmanQ.setPrecision(3);
    itemImuEmaAlpha.setPrecision(2);
    itemImuEmaSpike.setPrecision(2);
    itemImuSpikeThr.setPrecision(1);
    itemImuSlew.setPrecision(1);

    gem.setMenuPageCurrent(mainPage);
}


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

static void processTestCommandLine(char *line) {
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
