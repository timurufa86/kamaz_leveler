#include "ui_menu_build.h"
#include "ui_ota_menu.h"
#include "ui_marquee.h"
#include "ui_theme.h"
#include "ui_text.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_version.h"
#include "mutex_guard.h"
#include "wifi_setup.h"
#include "service_ui.h"
#include "valve_ctrl.h"
#include "task_display.h"
#include "app_pins.h"
#include "error_handler.h"
#include "logger.h"
#include "FontsRus/CourierCyr7.h"
#include "FontsRus/CourierCyr9.h"
#include <Adafruit_ST7789.h>
#include <WiFi.h>

extern uint32_t uptimeHours;
extern char wifi_ssid[32];
extern char sta_ssid[33];
uint32_t lastUserActivityMs = 0;
bool backlightDimmed = false;

void resetSystemErrors();
void requestWiFiSetup();
void saveMenuSettings();
extern void cfg_setMasterLowBar(float v);
extern void cfg_setBacklightOffMin(int v);

enum class EH_Error : uint8_t { NONE = 0, LOW_PRESSURE, MPU, SENSOR, VALVE, WATCHDOG, OTA, COUNT };

extern bool cfg_errorHasUiBlocking();
extern bool cfg_errorHasActive();
extern bool cfg_errorHasCriticalPneumatic();
extern bool cfg_errorIsActive(uint8_t err);
extern void cfg_errorRemove(uint8_t err);
extern void cfg_errorMarkCleared(uint8_t err);
extern int cfg_errorActiveCount();
extern uint8_t cfg_errorCurrent();
extern int cfg_errorDisplayIndex();
extern const char *cfg_errorMessage(uint8_t err);
extern void cfg_taskMonitorUpdate(uint8_t idx);
extern int cfg_getBacklightOffMin();
extern float cfg_getCoarseZoneRatio();
extern int cfg_getContrast();
extern float cfg_getFineZoneRatio();
extern float cfg_getImuKalmanEst();
extern float cfg_getImuKalmanMea();
extern float cfg_getImuKalmanQ();
extern int cfg_getImuMotionDet();
extern int cfg_getImuPollMs();
extern int cfg_getInflateDelay();
extern int cfg_getManualMaxTimeSec();
extern int cfg_getMasterCheckSec();
extern float cfg_getMasterLowBar();
extern float cfg_getMovementPressureFront();
extern float cfg_getMovementPressureRear();
extern int cfg_getNivCount();
extern float cfg_getPressureDeadband();
extern float cfg_getPressureMax();
extern float cfg_getPressureMin();
extern float cfg_getRedrawAngleThr();
extern float cfg_getRedrawPressureThr();
extern int cfg_getReleaseDelay();
extern float cfg_getTiltThresholdX();
extern float cfg_getTiltThresholdY();
extern int cfg_getTimeInterval();
extern float cfg_getWorseningRatio();

/* ====================  НАСТРОЙКИ МЕНЮ ====================  ST77XX_WHITE */
constexpr uint8_t MENU_COLS = 60;          // символов в строке
constexpr uint8_t MENU_ROWS = 9;           // строк на экране (240/24 ? 10)
constexpr uint8_t MENU_TOP_OFFSET = 14;    // отступ сверху
constexpr uint8_t MENU_LEFT_PADDING = 15;  // отступ слева (для внешних панелей)
constexpr uint8_t ROW_HEIGHT = 22;         // высота строки (важно!)
constexpr uint8_t CURSOR_WIDTH = 2;
// Колонка значений в меню GEM (названия слева, значения с этой колонки).
constexpr uint8_t MENU_VALUES_LEFT_OFFSET = 150;
constexpr uint16_t MENU_BG_COLOR = ST77XX_BLACK;
constexpr uint16_t MENU_TEXT_COLOR = ST77XX_WHITE;
constexpr uint16_t MENU_SEL_COLOR = ST77XX_BLUE;

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

static void refreshInfoPage();   // определена ниже
static void refreshSettingsView(); // определена ниже

/** Живые строки динамических страниц меню — вызывается перед отрисовкой. */
void refreshDynamicMenu() {
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

/* Marquee + refreshOta/DynamicMenu -> ui_marquee, ui_ota_menu */

static void refreshSettingsView() {
  static char b1[52], b2[52], b3[52], b4[52], b5[52], b6[52], b7[52];
  snprintf(b1, sizeof(b1), "Давл %.1f-%.1f МПн%.1f з%.2f", cfg_getPressureMin(),
           cfg_getPressureMax(), cfg_getMasterLowBar(),
           cfg_getPressureDeadband());
  snprintf(b2, sizeof(b2), "Клап сбр%d/нак%dс макс %dс", cfg_getReleaseDelay(),
           cfg_getInflateDelay(), cfg_getManualMaxTimeSec());
  snprintf(b3, sizeof(b3), "Авто %.2f/%.2f° %.2f/%.2f/%.2f", cfg_getTiltThresholdX(),
           cfg_getTiltThresholdY(), cfg_getCoarseZoneRatio(),
           cfg_getFineZoneRatio(), cfg_getWorseningRatio());
  snprintf(b4, sizeof(b4), "Авто поп%d инт%dm МП%dс", cfg_getNivCount(),
           cfg_getTimeInterval(), cfg_getMasterCheckSec());
  snprintf(b5, sizeof(b5), "Движ %.1f/%.1f бар MOT%d", cfg_getMovementPressureFront(),
           cfg_getMovementPressureRear(), cfg_getImuMotionDet());
  snprintf(b6, sizeof(b6), "Дисп ярк%d %dмин %.2f°/%.2fб", cfg_getContrast(),
           cfg_getBacklightOffMin(), cfg_getRedrawAngleThr(),
           cfg_getRedrawPressureThr());
  snprintf(b7, sizeof(b7), "IMU K%.1f/%.1f/%.3f %dмс", cfg_getImuKalmanMea(),
           cfg_getImuKalmanEst(), cfg_getImuKalmanQ(),
           cfg_getImuPollMs());
  itemSet1.setTitle(b1);
  itemSet2.setTitle(b2);
  itemSet3.setTitle(b3);
  itemSet4.setTitle(b4);
  itemSet5.setTitle(b5);
  itemSet6.setTitle(b6);
  itemSet7.setTitle(b7);
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
  snprintf(sErr, sizeof(sErr), "Ошибки: %d", cfg_errorActiveCount());

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
        if (currentSystemMode == SystemMode::MANUAL && !cfg_errorHasActive()) {
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
        cfg_setMasterLowBar(bar);  // сразу в рантайм — не ждать СОХРАНИТЬ
        settingsChanged = true;
        displayDirty = true;
        if (editMasterLowTenths <= 0 &&
            cfg_errorIsActive((uint8_t)EH_Error::LOW_PRESSURE)) {
          cfg_errorRemove((uint8_t)EH_Error::LOW_PRESSURE);
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
      cfg_setBacklightOffMin(editBacklightOff);
      backlightDimmed = false;
      lastUserActivityMs = millis();
      applyBacklightPwm(constrain(cfg_getContrast(), CONTRAST_MIN, CONTRAST_MAX));
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

static int contrastToPwm(int level) {
  return map(constrain(level, 1, CONTRAST_MAX), 1, CONTRAST_MAX, 0, 255);
}

void applyBacklightPwm(int level) {
  analogWrite(PIN_TFT_BL, contrastToPwm(level));
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
