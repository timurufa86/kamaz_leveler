/****************************************************************************************
 *  Kamaz-Leveler — wiring sketch.
 *  Business logic lives in modules (see MODULES.md). This file keeps includes,
 *  GEM / TFT / SPI / button objects, menu edit mirrors, and setup()/loop().
 *  VERSION: app_version.h ("V 10.14.22 FreeRTOS OTA").
 *****************************************************************************************/

#include <Arduino.h>
#include <new>
#if __has_include(<esp_bt.h>)
#include <esp_bt.h>
#define KAMAZ_HAS_ESP_BT 1
#endif
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
#include "service_ui.h"
#include "arduino_ota.h"
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
#include "encbutton_cfg.h"

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

/* Menu edit mirrors — GEM spinners in ui_menu_build bind to these. */
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
int   editMasterCheck;        // период проверки магистрали, мин (в конфиге masterCheckSec = мин×60)
int   editManualMaxTime;      // максимальное время ручной операции, с
int   editPressStabilizeMs;   // t до замера P после клапана, мс
int   editPressIdleMin;       // пауза между полными опросами подушек, мин
float editDeadband;           // зона нечувствительности по давлению, бар
float editCoarseZone;         // грубая зона авторежима (доля порога)
float editFineZone;           // точная зона авторежима (доля порога)
float editWorsening;          // порог «стало хуже» (множитель)
float editStableThr;          // waitForStability: допуск угла, °
int   editStableHoldMs;       // waitForStability: удержание, мс
int   editStableTimeoutSec;   // waitForStability: таймаут, с
bool  editMovementEnabled;    // разрешить режим MOVEMENT
int   editMoveDuration;       // длительность ожидания движения, с
int   editMoveSettle;         // время успокоения после движения, с
int   editMoveCheck;          // период проверки давления после движения, с
float editMoveTolerance;      // допуск давления после движения, бар
int   editBacklightOff;       // гашение подсветки, мин (0 = никогда)
int   editFrameMs;            // интервал кадра дисплея, мс
float editRedrawAngle;        // порог перерисовки по углу, °
float editRedrawPressure;     // порог перерисовки по давлению, бар
int   editImuMotionDet;       // аппаратный порог детектора движения MPU (MOT)
int   editImuDlpfMode;        // DLPF_CFG 0..6 (256/188/98/42/20/10/5 Hz)
int   editImuAccelFs;         // AFS_SEL 0..3 → ±2/±4/±8/±16G
int   editGyroThreshold;      // абс. gRms (меню 1..25, ×8 → порог)
int   editGyroBumpThreshold;  // абс. bump (меню 1..25, ×8 → порог)
int   editAccelThreshold;     // порог |linAcc − EMA| (зеркало из ступени)
int   editAccelThrStep;       // ступень Δlin 0..4 → 200/500/1000/1500/2000
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

extern const uint16_t COLOR_BG = ST77XX_BLACK;
extern const uint16_t COLOR_TEXT = ST77XX_WHITE;
extern const uint16_t COLOR_HIGHLIGHT = ST77XX_BLUE;
extern const uint16_t COLOR_ERROR = ST77XX_RED;
extern const uint16_t COLOR_SUCCESS = ST77XX_GREEN;
extern const uint16_t COLOR_WARNING = ST77XX_YELLOW;
extern const uint16_t COLOR_OTA = ST77XX_CYAN;
extern const uint16_t COLOR_WHITE = ST77XX_WHITE;

SPIClass spi(VSPI);
Adafruit_ST7789 tft(&spi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GEM_adafruit_gfx gem(tft);

GEMPage mainPage("Главное меню", menuExitAction);
GEMPage systemPage("Система", mainPage);
GEMPage valveBlockPage("Клапанный блок", mainPage);
GEMPage testPage("Тесты", valveBlockPage);
GEMPage valvePage("Клапаны", valveBlockPage);
GEMPage pressurePage("Давление", valveBlockPage);
GEMPage autoPage("Автовыравнивание", mainPage);
GEMPage autoAdvPage("Расширенные", autoPage);
GEMPage displayPage("Дисплей", systemPage);
GEMPage dispAdvPage("Расширенные", displayPage);
GEMPage movementPage("Движение", mainPage);
GEMPage movDetectPage("Распознавание", movementPage);
GEMPage movHoldPage("Поддержание P", movementPage);
GEMPage infoPage("Информация", systemPage);
GEMPage imuPage("Датчик", mainPage);
GEMPage imuAdvPage("Расширенные", imuPage);
GEMPage settingsViewPage("Просмотр", mainPage);

Button button0(PIN_BUT1, INPUT_PULLUP, LOW);
Button button1(PIN_BUT2, INPUT_PULLUP, LOW);
Button button2(PIN_BUT3, INPUT_PULLUP, LOW);
Button button3(PIN_BUT4, INPUT_PULLUP, LOW);

MultiButton modeCombo;       // КН1+КН2 — АВТО/РУЧ (EncButton MultiButton)
MultiButton menuCombo;       // КН3+КН4 — меню
MultiButton emergencyButton; // КН1+КН4 — авария

/* ====================  SETUP ==================== */
/* ====================  SETUP ==================== */
void setup() {
  // Smaller UART buffers — every KB helps mbedtls BIGNUM on non-PSRAM ESP32.
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  Serial.println("\n\n=== Kamaz?Leveler (FreeRTOS OTA Optimized) ===");
  Serial.printf("Версия: %s\n", VERSION);
#if defined(KAMAZ_HAS_ESP_BT) && defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
  {
    const esp_err_t bte = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    SERIAL_DBG("[SETUP] BT mem_release: %s\n", esp_err_to_name(bte));
  }
#endif
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
    SERIAL_DBG("[RESET] reason=%s (%d)\n", rrs, static_cast<int>(rr));
  }
  SERIAL_DBGLN("[SETUP] Система в режиме BOOT, калибровка будет запущена автоматически.");

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
  SERIAL_DBGLN("[INIT] Queues created");

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
        SERIAL_DBGLN("[JHM1200] Датчик не найден! Калибровка будет пропущена.");
        jhmReady = false;
    } else {
        SERIAL_DBGLN("[JHM1200] Датчик найден на 0x78");
    }
#else
    jhmReady = true;
    SERIAL_DBGLN("[JHM1200] SIM: инициализация пропущена");
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
  editMasterCheck = constrain((ConfigManager::getMasterCheckSec() + 30) / 60, 1, 10);
  editManualMaxTime = constrain(ConfigManager::getManualMaxTimeSec(), 1, 30);
  editPressStabilizeMs = constrain(ConfigManager::getPressureStabilizeMs(), 100, 2000);
  editPressIdleMin = constrain(ConfigManager::getPressureIdleMin(), 2, 30);
  editDeadband = constrain(ConfigManager::getPressureDeadband(), 0.05f, 0.5f);
  editCoarseZone = constrain(ConfigManager::getCoarseZoneRatio(), 0.2f, 0.9f);
  editFineZone = constrain(ConfigManager::getFineZoneRatio(), 0.05f, 0.3f);
  editWorsening = constrain(ConfigManager::getWorseningRatio(), 1.05f, 2.0f);
  editStableThr = constrain(ConfigManager::getStableThresholdDeg(), 0.05f, 0.50f);
  editStableHoldMs = constrain(ConfigManager::getStableHoldMs(), 500, 3000);
  editStableTimeoutSec = constrain(ConfigManager::getStableTimeoutSec(), 2, 15);
  editMovementEnabled = ConfigManager::getMovementEnabled();
  editMoveDuration = constrain(ConfigManager::getMovementDurationSec(), 20, 300);
  editMoveSettle = constrain(ConfigManager::getMovementSettleSec(), 20, 300);
  editMoveCheck = constrain(ConfigManager::getMovementCheckSec(), 20, 600);
  editMoveTolerance = constrain(ConfigManager::getMovementTolerance(), 0.1f, 0.5f);
  editBacklightOff = constrain(ConfigManager::getBacklightOffMin(), 0, 30);
  editFrameMs = constrain(ConfigManager::getFrameMs(), 20, 200);
  editRedrawAngle = constrain(ConfigManager::getRedrawAngleThr(), 0.01f, 0.5f);
  editRedrawPressure = constrain(ConfigManager::getRedrawPressureThr(), 0.01f, 0.5f);
  editImuMotionDet = constrain(ConfigManager::getImuMotionDet(), 20, 255);
  editImuDlpfMode = constrain(ConfigManager::getImuDlpfMode(), 0, 6);
  editImuAccelFs = constrain(ConfigManager::getImuAccelFs(), 0, 3);
  editGyroThreshold = constrain(ConfigManager::getGyroThreshold(), 1, 25);
  editGyroBumpThreshold = constrain(ConfigManager::getGyroBumpThreshold(), 1, 25);
  editAccelThreshold = constrain(ConfigManager::getAccelThreshold(), 100, 3000);
  {
    static const int kLinThr[] = { 200, 500, 1000, 1500, 2000 };
    int best = 1;  // 500 — дефолт
    int bestDiff = abs(editAccelThreshold - kLinThr[0]);
    for (int i = 0; i < 5; i++) {
      const int d = abs(editAccelThreshold - kLinThr[i]);
      if (d < bestDiff) { bestDiff = d; best = i; }
    }
    editAccelThrStep = best;
    editAccelThreshold = kLinThr[best];
  }
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

  // EncButton timeouts BEFORE ButtonTask (README setXxxTimeout)
  auto cfgBtn = [](Button &b, uint16_t holdMs) {
    b.setBtnLevel(LOW);
    b.setDebTimeout(EB_DEB_MS);
    b.setClickTimeout(EB_CLICK_MS);
    b.setHoldTimeout(holdMs);
    b.setStepTimeout(EB_STEP_MS);
  };
  cfgBtn(button0, PAD_HOLD_MS);
  cfgBtn(button1, PAD_HOLD_MS);
  cfgBtn(button2, PAD_HOLD_MS);
  cfgBtn(button3, PAD_HOLD_MS);

  auto cfgCombo = [](MultiButton &c, uint16_t holdMs) {
    // Не вызывать setBtnLevel(LOW): VirtButton::tick(b0,b1) передаёт
    // tick(true/false) как логическое «нажата», а INV инвертирует это в
    // «отпущена» → ложное pressing всех пар при GPIO=HIGH и меню само открывается.
    c.setDebTimeout(EB_DEB_MS);
    c.setClickTimeout(EB_CLICK_MS);
    c.setHoldTimeout(holdMs);
    c.setStepTimeout(EB_STEP_MS);
  };
  cfgCombo(modeCombo, MODE_TOGGLE_HOLD_MS);
  cfgCombo(menuCombo, MENU_COMBO_HOLD_MS);
  cfgCombo(emergencyButton, EMERGENCY_HOLD_MS);

  button0.reset();
  button1.reset();
  button2.reset();
  button3.reset();
  modeCombo.reset();
  menuCombo.reset();
  emergencyButton.reset();

  // ========== 17. СОЗДАНИЕ ЗАДАЧ ==========
  TaskConfig taskConfigs[] = {
    { "EventTask", eventHandlerTask, 3072, 2, 1, 20, nullptr },
    // Стеки в DRAM: каждый лишний КБ режет maxBlk → TLS BIGNUM OOM (~40KB нужно).
    { "ButtonTask", buttonTask, 6144, 4, 0, 20, nullptr },
    { "DisplayTask", displayTask, 6144, 2, 1, 33, nullptr },
    { "IMUTask", imuTask, 5120, 5, 0, 20, nullptr },
    { "PressTask", pressureTask, 4096, 3, 1, 2000, nullptr },
    { "ControlTask", controlTask, 4096, 3, 0, 500, nullptr },
    { "CalibTask", calibrationTask, 3072, 2, 1, 1000, nullptr },
    { "WatchdogTask", watchdogTask, 4096, 6, 1, 1000, nullptr },
    // TLS crypto + Json на стеке OTA; 10K → canary, 16K съедал maxBlk.
    { "OTATask", otaTask, 14336, 2, 1, 100, nullptr },
    { "ErrRecTask", errorRecoveryTask, 3072, 2, 1, 500, nullptr },
    { "ValveTask", valveTask, 3072, 4, 0, 10, nullptr }
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

  SERIAL_DBGLN("[SETUP] All tasks created successfully");
  // Do not grab a large TLS reserve here — it caps maxBlk (~34KB) and TLS handshake
  // then fails with "SSL - Memory allocation failed". Reserve only around OTA sessions.

  uptimeHours = 0;

  // ========== 18. ФИНАЛЬНЫЕ НАСТРОЙКИ ==========
  SERIAL_DBG("[SETUP] final state: currentState=%d\n", (int)currentState);

  displayDirty = true;
  forceDisplayReset();
  vTaskDelay(pdMS_TO_TICKS(100));

  Logger::log(Logger::INFO, "SETUP", "Инициализация завершена");
}

/* ====================  LOOP ==================== */
void loop() {
  testHarnessPoll();
  systemMaintenanceTick();
  vTaskDelay(pdMS_TO_TICKS(100));
}
