#include "ui_screens.h"
#include "ui_theme.h"
#include "ui_text.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "app_version.h"
#include "mutex_guard.h"
#include "icon.h"
#include "imu_dmp.h"
#include "imu_motion.h"
#include "logger.h"
#include "auto_level.h"
#include "event_bus.h"
#include "test_harness.h"
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <cstring>

extern Adafruit_ST7789 tft;
bool errorScreenBlocking = false;
bool forceErrorScreenRedraw = false;
volatile uint16_t g_displayEpoch = 0;
bool mvScreenWasActive = false;
IMUData lastDisplayedIMU = { 0 };
PressureData lastDisplayedPressure = { 0 };
SystemMode lastDisplayedMode = SystemMode::MANUAL;
bool lastDisplayedMoving = false;
extern uint8_t manualValveIndex;
extern uint8_t manualValveOpenIndex;
extern uint32_t manualValveOpenSince;
extern int8_t imuCalibResult;
extern uint32_t imuCalibLastRun;
constexpr uint32_t CALIB_TIME_MS = 15000;
extern uint32_t uptimeHours;
extern char wifi_ssid[32];
extern char sta_ssid[33];
extern int otaProgress;
extern char otaStatus[32];

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

struct AngleBarState {
  bool chromeDrawn = false;
  int8_t side = 0;
  int16_t fillW = 0;
  uint16_t fillCol = theme::OK;
  char valStr[12] = "";
  int16_t barX = 0, barY = 0, barW = 0, barH = 0, mid = 0;
  int16_t valX = 0, valY = 0, valW = 0;
};

extern int cfg_getFrameMs();
extern int cfg_getImuAccelOffX();
extern int cfg_getImuAccelOffY();
extern int cfg_getImuAccelOffZ();
extern int cfg_getImuGyroOffX();
extern int cfg_getImuGyroOffY();
extern int cfg_getImuGyroOffZ();
extern float cfg_getImuKalmanEst();
extern float cfg_getImuKalmanMea();
extern float cfg_getImuKalmanQ();
extern int cfg_getImuMotionDet();
extern int cfg_getImuPollMs();
extern int cfg_getMovementCheckSec();
extern float cfg_getMovementPressureFront();
extern float cfg_getMovementPressureRear();
extern int cfg_getMovementSettleSec();
extern float cfg_getMovementTolerance();
extern int cfg_getNivCount();
extern float cfg_getPressureMax();
extern float cfg_getPressureMin();
extern float cfg_getRedrawAngleThr();
extern float cfg_getRedrawPressureThr();
extern int cfg_getTimeInterval();
extern float cfg_getZeroAngleX();
extern float cfg_getZeroAngleY();

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
  snprintf(buf, sizeof(buf), "Текущий нуль: %.2f / %.2f", cfg_getZeroAngleX(),
           cfg_getZeroAngleY());
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
  snprintf(buf, sizeof(buf), "гиро: %d %d %d", cfg_getImuGyroOffX(),
           cfg_getImuGyroOffY(), cfg_getImuGyroOffZ());
  uiRedrawLabel(theme::MARGIN, 126, theme::SCREEN_W - 2 * theme::MARGIN, 16, gyroBuf, sizeof(gyroBuf),
                buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Center);
  snprintf(buf, sizeof(buf), "аксель: %d %d %d", cfg_getImuAccelOffX(),
           cfg_getImuAccelOffY(), cfg_getImuAccelOffZ());
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
  snprintf(buf, sizeof(buf), "Темп: %.1f C  det:%d  poll:%dms", temp, cfg_getImuMotionDet(),
           cfg_getImuPollMs());
  uiRedrawLabel(theme::MARGIN, ys[1], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[1],
                sizeof(line[1]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Калман: mea=%.1f est=%.1f q=%.3f", cfg_getImuKalmanMea(),
           cfg_getImuKalmanEst(), cfg_getImuKalmanQ());
  uiRedrawLabel(theme::MARGIN, ys[2], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[2],
                sizeof(line[2]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Углы (с нулём): X=%.2f  Y=%.2f", angleX, angleY);
  uiRedrawLabel(theme::MARGIN, ys[3], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[3],
                sizeof(line[3]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Нуль конфига: X=%.2f  Y=%.2f", cfg_getZeroAngleX(),
           cfg_getZeroAngleY());
  uiRedrawLabel(theme::MARGIN, ys[4], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[4],
                sizeof(line[4]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "ax=%d ay=%d az=%d", ax, ay, az);
  uiRedrawLabel(theme::MARGIN, ys[5], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[5],
                sizeof(line[5]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "gx=%d gy=%d gz=%d", gx, gy, gz);
  uiRedrawLabel(theme::MARGIN, ys[6], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[6],
                sizeof(line[6]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  snprintf(buf, sizeof(buf), "Офсеты гиро: %d %d %d", cfg_getImuGyroOffX(),
           cfg_getImuGyroOffY(), cfg_getImuGyroOffZ());
  uiRedrawLabel(theme::MARGIN, ys[7], theme::SCREEN_W - 2 * theme::MARGIN, 16, line[7],
                sizeof(line[7]), buf, theme::TEXT_DIM, theme::BG, UiFont::Tiny, UiHAlign::Left);
  // аксель офсеты — отдельная строка 180
  static char accelOff[56] = "";
  snprintf(buf, sizeof(buf), "Офсеты аксель: %d %d %d", cfg_getImuAccelOffX(),
           cfg_getImuAccelOffY(), cfg_getImuAccelOffZ());
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
void drawServiceScreen() {
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
  d.targetFront = cfg_getMovementPressureFront();
  d.targetRear = cfg_getMovementPressureRear();
  d.tol = cfg_getMovementTolerance();
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

  d.settleTotalMs = (uint32_t)cfg_getMovementSettleSec() * 1000UL;
  d.settleLeftMs = 0;
  if (movementEndTime > 0) {
    const uint32_t passed = now - movementEndTime;
    d.settleLeftMs = (passed < d.settleTotalMs) ? (d.settleTotalMs - passed) : 0;
  }

  const uint32_t checkInterval = (uint32_t)cfg_getMovementCheckSec() * 1000UL;
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
void displayMovementScreen() {
  static bool mvFirst = true;
  static uint32_t mvLastRender = 0;

  if (!mvScreenWasActive) {
    mvScreenWasActive = true;
    mvFirst = true;
  }

  const uint32_t now = millis();
  const uint32_t frameMs = cfg_getFrameMs();
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

  const float pScale = (cfg_getPressureMax() > 0.1f) ? cfg_getPressureMax() : 8.0f;
  const float pMin = cfg_getPressureMin();

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
/** Шкалы наклона ±10°: крен (лево/право) и тангаж (зад/перед). */
static constexpr float ATT_GAUGE_MAX_DEG = 10.0f;

/** Стереть старый текст цветом фона, затем нарисовать новый (без fillRect всей области). */
void uiRedrawLabel(int16_t x, int16_t y, int16_t w, int16_t h, char *prev, size_t prevSz,
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
void updateMainTiltLive() {
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
/* ====================  UI-ХЕЛПЕРЫ  ==================== */

/** Строка «подпись: значение» для служебных экранов. */
void uiInfoRow(int16_t y, const char *label, const char *value) {
  ui.setTransparent(true);
  ui.setFont(UiFont::Small);
  ui.setColors(theme::TEXT_DIM, theme::BG);
  ui.box(theme::MARGIN + 2, y, 80, 18, label, UiHAlign::Left, UiVAlign::Middle, false);
  ui.setColors(theme::TEXT, theme::BG);
  ui.box(theme::MARGIN + 84, y, theme::SCREEN_W - 2 * theme::MARGIN - 84, 18, value,
         UiHAlign::Left, UiVAlign::Middle, false);
}

/** Шапка экрана: скруглённая панель с заголовком по центру. */
void uiHeader(const char *title, uint16_t titleColor) {
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
void uiHintBar(const char *text, uint16_t color) {
  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.setColors(color, theme::BG);
  ui.box(0, theme::SCREEN_H - 20, theme::SCREEN_W, 20, text, UiHAlign::Center, UiVAlign::Middle, true);
}
void displayErrorScreen() {
  uint32_t now = millis();

  if (!cfg_errorHasUiBlocking()) {
    errorScreenBlocking = false;
    displayDirty = true;
    return;
  }

  int totalActiveErrors = cfg_errorActiveCount();
  errorScreenBlocking = true;

  EH_Error currentErr = ((EH_Error)cfg_errorCurrent());
  const int currentIndex = cfg_errorDisplayIndex();

  static bool chromeDrawn = false;
  static EH_Error lastDisplayedError = EH_Error::NONE;
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

  auto resolveErrorLines = [](EH_Error err, const char *&line1, const char *&line2) {
    line1 = nullptr;
    line2 = nullptr;
    switch (err) {
      case EH_Error::LOW_PRESSURE:
        line1 = "НИЗКОЕ ДАВЛЕНИЕ";
        line2 = "В МАГИСТРАЛИ";
        break;
      case EH_Error::MPU:
        line1 = "ОШИБКА MPU6050";
        line2 = "НЕТ ОТВЕТА I2C";
        break;
      case EH_Error::SENSOR:
        line1 = "НЕИСПРАВЕН ДАТЧИК";
        line2 = "ДАВЛЕНИЯ";
        break;
      case EH_Error::VALVE:
        line1 = "НЕИСПРАВЕН КЛАПАН";
        line2 = "ПРОВЕРЬТЕ ЦЕПИ";
        break;
      case EH_Error::WATCHDOG:
        line1 = "СБОЙ WATCHDOG";
        line2 = "ПЕРЕЗАГРУЗИТЕ";
        break;
      case EH_Error::OTA:
        line1 = "ОШИБКА OTA";
        line2 = "ПРОВЕРЬТЕ WiFi";
        break;
      default:
        line1 = cfg_errorMessage((uint8_t)err);
        break;
    }
  };

  auto drawErrorContent = [&](EH_Error err, int index, int total, bool clearBg) {
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

  if (errorScreenBlocking || cfg_errorHasUiBlocking()) {
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
/** СТАТ + иконка «авто стоит» (iconAutoStopL). Мигает, пока isMoving,
 *  но режим ещё не MOVEMENT (набор длительности до входа в ДВИЖЕНИЕ). */
void updateMainStatBadge() {
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
void updateMainInfoLine(bool isMoving) {
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
    const uint32_t settleMs = (uint32_t)cfg_getMovementSettleSec() * 1000UL;
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
               cfg_getNivCount());
    } else {
      const uint32_t nextCheck =
          (lastLevelingCheckTime + cfg_getTimeInterval() * 60000UL - millis()) / 1000;
      if (nextCheck < 600) {
        snprintf(fullMsg, sizeof(fullMsg), "ПОП: %d/%d  СЛЕД: %luс", levelingAttemptsThisHour,
                 cfg_getNivCount(), (unsigned long)nextCheck);
      } else {
        snprintf(fullMsg, sizeof(fullMsg), "ПОП: %d/%d", levelingAttemptsThisHour,
                 cfg_getNivCount());
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

  if (abs(localAngleX - lastState.angleX) > cfg_getRedrawAngleThr() || abs(localAngleY - lastState.angleY) > cfg_getRedrawAngleThr() ||
      abs(localTemp - lastState.temp) > 0.5f || abs(localMasterPressure - lastState.masterPressure) > cfg_getRedrawPressureThr() ||
      localIsMoving != lastState.isMoving || currentSystemMode != lastSystemMode) {
    needRedraw = true;
  }

  if (!needRedraw) {
    for (int i = 0; i < PAD_COUNT; i++) {
      if (abs(localPressure[i] - lastState.pressure[i]) > cfg_getRedrawPressureThr()) {
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

  const float pMin = cfg_getPressureMin();
  const float pMaxAbs = cfg_getPressureMax();
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

  simulationOnDisplayReset();

  forceErrorScreenRedraw = true;

  Serial.println("[DISPLAY] Force reset completed");
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

