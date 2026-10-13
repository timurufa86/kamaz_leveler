#include "task_button.h"
#include "btn_io.h"
#include "service_ui.h"
#include "arduino_ota.h"
#include "ui_screens.h"
#include "ui_menu_build.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "task_pool.h"
#include "valve_ctrl.h"
#include "pressure_read.h"
#include "wifi_setup.h"
#include "logger.h"
#include "event_bus.h"
#include "ui_marquee.h"
#include "encbutton_cfg.h"
#include <GEM_adafruit_gfx.h>
#include <Adafruit_ST7789.h>

extern Adafruit_ST7789 tft;
extern Button button0;
extern Button button1;
extern Button button2;
extern Button button3;
extern MultiButton modeCombo;
extern MultiButton menuCombo;
extern MultiButton emergencyButton;
extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;
uint32_t lastMenuInteraction = 0;
extern bool errorScreenBlocking;
extern char wifi_scan_ssids[][33];
extern uint8_t wifi_scan_count;
extern uint8_t wifi_scan_selected;
extern char sta_ssid[33];

void resetSystemErrors();
void saveWiFiConfig();

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
extern int cfg_getContrast();
extern float cfg_getTiltThresholdX();
extern float cfg_getTiltThresholdY();

uint32_t g_menuNavIgnoreUntilMs = 0;

/** КН3+КН4 — меню: оба GPIO реально LOW (не залипший EncButton.pressing). */
bool menuPairPressedNow() {
  return btnDigitalRead(PIN_BUT3) == LOW && btnDigitalRead(PIN_BUT4) == LOW;
}

/** КН1+КН2 — смена АВТО/РУЧ. */
bool modePairPressedNow() {
  return btnDigitalRead(PIN_BUT1) == LOW && btnDigitalRead(PIN_BUT2) == LOW;
}

/** КН1+КН4 — авария. */
bool emergencyPairPressedNow() {
  return btnDigitalRead(PIN_BUT1) == LOW && btnDigitalRead(PIN_BUT4) == LOW;
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
  SERIAL_DBG("[MODE] toggle via %s > %s\n", via ? via : "?",
                currentSystemMode == SystemMode::AUTO ? "AUTO" : "MANUAL");
}

void buttonTask(void *pvParameters) {
  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    SERIAL_DBG("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    SERIAL_DBG("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_Button;
  bool lastState[4] = { false };
  bool prevDown[4] = { false };  // GPIO edge (лог / тест)
  bool failLatch[4] = { false };  // краткий блок после fail
  uint32_t failLatchUntil[4] = { 0 };
  bool skipLogged[4] = { false };
  bool emergencyProcessed = false;

  uint32_t lastButtonCheck = 0;
  const uint32_t BUTTON_CHECK_INTERVAL_MS = 50;

  for (;;) {
    TaskPool::markRun(taskIndex_Button);
    cfg_taskMonitorUpdate(1);

    uint32_t now = millis();

    if (now - lastButtonCheck < BUTTON_CHECK_INTERVAL_MS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    lastButtonCheck = now;

    // EncButton: уровни из btnDigitalRead, пары VirtButton; syncComboGpio сбрасывает залипшие комбо.
    static_cast<VirtButton &>(button0).tick(btnDigitalRead(PIN_BUT1));
    static_cast<VirtButton &>(button1).tick(btnDigitalRead(PIN_BUT2));
    static_cast<VirtButton &>(button2).tick(btnDigitalRead(PIN_BUT3));
    static_cast<VirtButton &>(button3).tick(btnDigitalRead(PIN_BUT4));

    auto syncBtnGpio = [](Button &btn, uint8_t pin) {
      if (btnDigitalRead(pin) != LOW && btn.pressing()) {
        btn.reset();
      }
    };
    syncBtnGpio(button0, PIN_BUT1);
    syncBtnGpio(button1, PIN_BUT2);
    syncBtnGpio(button2, PIN_BUT3);
    syncBtnGpio(button3, PIN_BUT4);

    static_cast<VirtButton &>(modeCombo).tick(button0, button1);
    static_cast<VirtButton &>(menuCombo).tick(button2, button3);
    static_cast<VirtButton &>(emergencyButton).tick(button0, button3);

    // Комбо не может быть «зажато», если пара GPIO не оба LOW
    auto syncComboGpio = [](MultiButton &c, uint8_t p0, uint8_t p1) {
      if (btnDigitalRead(p0) != LOW || btnDigitalRead(p1) != LOW) {
        if (c.pressing() || c.holding()) {
          c.reset();
        }
      }
    };
    syncComboGpio(modeCombo, PIN_BUT1, PIN_BUT2);
    syncComboGpio(menuCombo, PIN_BUT3, PIN_BUT4);
    syncComboGpio(emergencyButton, PIN_BUT1, PIN_BUT4);

    // До первой «чистой» отпускания всех кнопок — не принимаем hold (ложное меню)
    static bool buttonsArmed = false;
    static uint32_t buttonsIdleSinceMs = 0;
    if (!buttonsArmed) {
      const bool allUp =
          btnDigitalRead(PIN_BUT1) != LOW && btnDigitalRead(PIN_BUT2) != LOW &&
          btnDigitalRead(PIN_BUT3) != LOW && btnDigitalRead(PIN_BUT4) != LOW;
      if (allUp) {
        if (buttonsIdleSinceMs == 0) buttonsIdleSinceMs = now;
        if ((now - buttonsIdleSinceMs) >= BUTTONS_ARM_IDLE_MS) {
          buttonsArmed = true;
          button0.reset();
          button1.reset();
          button2.reset();
          button3.reset();
          modeCombo.reset();
          menuCombo.reset();
          emergencyButton.reset();
          SERIAL_DBGLN("[BTN] armed (idle settle)");
        }
      } else {
        buttonsIdleSinceMs = 0;
      }
    }

    // DEBUG: лог нажатий для теста железа
    {
      static bool prev[4] = {};
      const bool nowPress[4] = {
        button0.pressing(), button1.pressing(), button2.pressing(), button3.pressing()
      };
      const uint8_t pins[4] = { PIN_BUT1, PIN_BUT2, PIN_BUT3, PIN_BUT4 };
      for (uint8_t i = 0; i < 4; i++) {
        if (nowPress[i] != prev[i]) {
          SERIAL_DBG("[BTN] KN%d %s gpio%d=%d\n", i + 1,
                        nowPress[i] ? "DOWN" : "UP",
                        pins[i], btnDigitalRead(pins[i]));
          prev[i] = nowPress[i];
        }
      }
      if (menuPairPressedNow()) {
        static uint32_t lastPairMs = 0;
        if (millis() - lastPairMs >= 500) {
          lastPairMs = millis();
          SERIAL_DBGLN("[BTN] пара КН3+КН4 удерживается...");
        }
      }
    }

    // Активность / пробуждение подсветки (только КН1–КН4).
    {
      const bool activityEdge =
          button0.press() || button0.click() || button0.step() ||
          button1.press() || button1.click() || button1.step() ||
          button2.press() || button2.click() || button2.step() ||
          button3.press() || button3.click() || button3.step();

      if (backlightDimmed && activityEdge) {
        backlightDimmed = false;
        lastUserActivityMs = millis();
        applyBacklightPwm(constrain(cfg_getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        button0.reset();
        button1.reset();
        button2.reset();
        button3.reset();
        SERIAL_DBGLN("[DISP] Подсветка восстановлена (нажатие не выполняет действие)");
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
      if (emergencyPairPressedNow()) {
        manualValveCloseAll();
        serviceScreen = ServiceScreen::NONE;
        forceDisplayReset(true);
        displayDirty = true;
        SERIAL_DBGLN("[SERVICE] АВАРИЙНАЯ ОСТАНОВКА (КН1+КН4): экран закрыт, клапаны закрыты");
      } else {
        handleServiceInput();
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 1.1. МЕНЮ: hold КН3+КН4 только если оба GPIO реально LOW
    // ============================================================
    if (buttonsArmed && menuPairPressedNow() && menuCombo.hold()) {
      const bool wasOpen = menuVisible;
      if (wasOpen) {
        requestMenuClose("KN3+KN4");
      } else {
        requestMenuOpen("KN3+KN4");
      }
      // clear (не reset) — сбросить хвост click; navIgnore гасит отпускание
      button2.clear();
      button3.clear();
      g_menuNavIgnoreUntilMs = millis() + 450;
      SERIAL_DBG("[MENU] combo hold > %s\n", wasOpen ? "close" : "open");
    }

    // ============================================================
    // 1.2. РЕЖИМ: hold КН1+КН2 — АВТО/РУЧ
    // ============================================================
    if (buttonsArmed && !menuVisible && modePairPressedNow() && modeCombo.hold()) {
      if (manualControlActive) {
        stopManualOperation();
      }
      toggleAutoManualMode("KN1+KN2");
      button0.clear();
      button1.clear();
    }

    // ============================================================
    // 2. АВАРИЙНАЯ ОСТАНОВКА (КН1 + КН4)
    // ============================================================
    {
      if (buttonsArmed && emergencyPairPressedNow() && emergencyButton.hold()) {
        if (!emergencyProcessed) {
          emergencyStop();

          if (currentTestState != TestState::IDLE && currentTestState != TestState::COMPLETED) {
            currentTestState = TestState::COMPLETED;
            closeAllValves();
            Serial.println("[TEST] Тест принудительно остановлен аварийной кнопкой!");
          }

          emergencyProcessed = true;
          SERIAL_DBGLN("[EMERGENCY] АВАРИЙНАЯ ОСТАНОВКА! (КН1+КН4)");
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
        SERIAL_DBG("[WiFi] Выбрана сеть \"%s\"\n", sta_ssid);
        connectConfiguredWiFi();
        menuVisible = true;
        displayDirty = true;
      }
      if (button3.click()) {
        wifiSetupActive = false;
        menuVisible = true;
        displayDirty = true;
        SERIAL_DBGLN("[WiFi] Настройка отменена");
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ============================================================
    // 3. РЕЖИМ ОШИБОК
    // ============================================================
    if (cfg_errorHasUiBlocking() && !menuVisible) {
      // КНОПКА 3 - СБРОС ОШИБОК
      if (button2.click()) {
        resetSystemErrors();
        forceDisplayReset(true);
        menuVisible = false;
        errorScreenBlocking = false;
        displayDirty = true;
        SERIAL_DBGLN("[BUTTON] Ошибки сброшены (кнопка 3)");
      }

      // Меню открывается удержанием пары КН3+КН4 (см. п. 1.1)
      // Режим АВТО/РУЧ — modeCombo.hold() выше

      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ============================================================
    // 4. РЕЖИМ МЕНЮ
    // ============================================================
    if (menuVisible) {
      // Пара меню только по реальному GPIO (не залипший EncButton.pressing)
      const bool menuPairing = menuPairPressedNow();
      const bool navIgnore =
          menuPairing || (millis() < g_menuNavIgnoreUntilMs);
      if (navIgnore) {
        (void)button0.click();
        (void)button1.click();
        (void)button2.click();
        (void)button3.click();
        (void)button0.step();
        (void)button1.step();
        if (menuPairing) {
          g_menuNavIgnoreUntilMs = millis() + 450;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }
      // Пока пара — глотаем только SELECT/BACK, ^/v оставляем
      if (menuPairing) {
        (void)button2.click();
        (void)button3.click();
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
      if (!menuPairing && button2.click()) {  // Кн3 - ВЫБОР/OK
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
        if (guard) {
          gemRestoreMenuItemFont();
          gem.registerKeyPress(GEM_KEY_OK);
          if (!gem.isEditMode()) displayDirty = true;
        }
        SERIAL_DBGLN("[MENU] SELECT");
      }
      if (!menuPairing && button3.click()) {  // Кн4 - НАЗАД/CANCEL
        MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(50));
        if (guard) {
          gemRestoreMenuItemFont();
          gem.registerKeyPress(GEM_KEY_CANCEL);
          if (!gem.isEditMode()) displayDirty = true;
        }
        SERIAL_DBGLN("[MENU] BACK");
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
    // 7. ОСНОВНОЙ РЕЖИМ (RUNNING) + DRY-автотест кнопок
    // ============================================================
    if (currentState == SystemState::RUNNING || btnDryRun()) {

      // ============================================================
      // 7.1. ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ — modeCombo.hold() выше (п. 1.2)
      // ============================================================

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

        static Button *const padBtns[4] = { &button0, &button1, &button2, &button3 };
        // Инъекция: принудительно сбросить/подготовить фронты
        {
          const uint8_t rel = btnConsumeForceReleaseMask();
          const uint8_t edg = btnConsumeForceEdgeMask();
          for (uint8_t i = 0; i < 4; i++) {
            const uint8_t bit = (uint8_t)(1u << i);
            if (rel & bit) {
              if (lastState[i] && manualControlActive && manualPadIndex == i) {
                stopManualOperation();
              }
              lastState[i] = false;
              failLatch[i] = false;
              failLatchUntil[i] = 0;
              skipLogged[i] = false;
              prevDown[i] = false;
              padBtns[i]->reset();
            }
            if (edg & bit) {
              prevDown[i] = false;
            }
          }
        }

        // Пары только по реальному GPIO — не залипший EncButton.pressing
        const bool modePair = modePairPressedNow();
        const bool menuPair = menuPairPressedNow();
        const bool emergPair = emergencyPairPressedNow();

        if (menuPair && manualControlActive &&
            (manualPadIndex == PAD_REAR_LEFT || manualPadIndex == PAD_REAR_RIGHT)) {
          stopManualOperation();
        }
        if (emergPair && manualControlActive &&
            (manualPadIndex == PAD_FRONT_LEFT || manualPadIndex == PAD_REAR_RIGHT)) {
          stopManualOperation();
        }
        if (modePair && manualControlActive &&
            (manualPadIndex == PAD_FRONT_LEFT || manualPadIndex == PAD_FRONT_RIGHT)) {
          stopManualOperation();
        }

        for (uint8_t i = 0; i < 4; i++) {
          const bool down = padBtns[i]->pressing();
          const bool edgeDown = down && !prevDown[i];

          if (!down) {
            // release / !pressing — стоп клапана
            if (lastState[i] && manualControlActive && manualPadIndex == i) {
              stopManualOperation();
            }
            lastState[i] = false;
            failLatch[i] = false;
            failLatchUntil[i] = 0;
            skipLogged[i] = false;
            prevDown[i] = false;
            continue;
          }

          prevDown[i] = true;

          if (edgeDown) {
            btnTestNotePadEdge(i);
            if (i == 1) {
              SERIAL_DBG(
                  "[MANUAL] KN2 edge mode=%d kn1=%d kn2=%d manual=%d cmd=%d dry=%d\n",
                  (int)currentSystemMode, button0.pressing() ? 1 : 0, button1.pressing() ? 1 : 0,
                  manualControlActive ? 1 : 0, lastCmd.commandActive ? 1 : 0,
                  btnDryRun() ? 1 : 0);
            }
          }

          if (modePair && (i == 0 || i == 1)) {
            if (!skipLogged[i]) {
              SERIAL_DBG("[MANUAL] KN%d skip modePair\n", i + 1);
              skipLogged[i] = true;
            }
            continue;
          }
          if (menuPair && (i == 2 || i == 3)) {
            if (!skipLogged[i]) {
              SERIAL_DBG("[MANUAL] KN%d skip menuPair\n", i + 1);
              skipLogged[i] = true;
            }
            continue;
          }
          if (emergPair && (i == 0 || i == 3)) {
            if (!skipLogged[i]) {
              SERIAL_DBG("[MANUAL] KN%d skip emergPair\n", i + 1);
              skipLogged[i] = true;
            }
            continue;
          }

          if (failLatch[i] && now >= failLatchUntil[i]) {
            failLatch[i] = false;
          }

          // Старт по EncButton hold(); клапан пока pressing()
          const bool mayStart = buttonsArmed && !lastState[i] && padBtns[i]->hold();

          if (!mayStart || failLatch[i] || lastCmd.commandActive) {
            continue;
          }

          bool inflateDefault = false;
          switch (i) {
            case PAD_FRONT_LEFT:
              inflateDefault = (localX < -cfg_getTiltThresholdX()) || (localY > cfg_getTiltThresholdY());
              break;
            case PAD_FRONT_RIGHT:
              inflateDefault = (localX > cfg_getTiltThresholdX()) || (localY > cfg_getTiltThresholdY());
              break;
            case PAD_REAR_LEFT:
              inflateDefault = (localX < -cfg_getTiltThresholdX()) || (localY < -cfg_getTiltThresholdY());
              break;
            case PAD_REAR_RIGHT:
              inflateDefault = (localX > cfg_getTiltThresholdX()) || (localY < -cfg_getTiltThresholdY());
              break;
          }

          if (btnDryRun()) {
            lastState[i] = true;
            btnTestNotePadStart(i, true);
            SERIAL_DBG("[MANUAL] DRY start pad=%d kn=%d infl=%d\n", (int)i, i + 1,
                          inflateDefault ? 1 : 0);
            continue;
          }

          if (manualControlActive) {
            if (manualPadIndex == i) {
              lastState[i] = true;
              continue;
            }
            stopManualOperation();
          }

          ManualStartResult res = startManualOperation(Pad(i), inflateDefault);
          if (res == ManualStartResult::MinP && !inflateDefault) {
            res = startManualOperation(Pad(i), true);
            if (res == ManualStartResult::Ok) inflateDefault = true;
          } else if (res == ManualStartResult::MaxP && inflateDefault) {
            res = startManualOperation(Pad(i), false);
            if (res == ManualStartResult::Ok) inflateDefault = false;
          }
          const bool ok = (res == ManualStartResult::Ok);
          btnTestNotePadStart(i, ok);
          SERIAL_DBG("[MANUAL] KN%d start %s pad=%d infl=%d reason=%s\n", i + 1,
                        ok ? "OK" : "FAIL", (int)i, inflateDefault ? 1 : 0,
                        manualStartResultStr(res));
          if (ok) {
            lastState[i] = true;
            g_manualHint[0] = '\0';
          } else {
            failLatch[i] = true;
            failLatchUntil[i] = now + 400;
            char hint[48];
            snprintf(hint, sizeof(hint), "РУЧ: %s НЕВОЗМОЖНО", padNames[i]);
            setManualHint(hint, 1200);
            setDisplayDirty();
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
