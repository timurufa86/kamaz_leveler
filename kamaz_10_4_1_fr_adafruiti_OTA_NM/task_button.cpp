#include "task_button.h"
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
#include <EncButton.h>
#include <GEM_adafruit_gfx.h>
#include <Adafruit_ST7789.h>

extern Adafruit_ST7789 tft;
extern Button button0;
extern Button button1;
extern Button button2;
extern Button button3;
extern Button button4;
extern VirtButton emergencyButton;
extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;
extern uint32_t lastMenuInteraction;
extern bool errorScreenBlocking;
extern char wifi_scan_ssids[][33];
extern uint8_t wifi_scan_count;
extern uint8_t wifi_scan_selected;
extern char sta_ssid[33];

void forceDisplayReset(bool force = false);
void setDisplayDirty();
void requestMenuOpen(const char *via);
void requestMenuClose(const char *via);
void resetSystemErrors();
void saveWiFiConfig();
void startOTAMode();
void stopOTAMode();
bool setSystemState(SystemState newState);

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
bool menuPairPressedNow() {
  return digitalRead(PIN_BUT3) == LOW && digitalRead(PIN_BUT4) == LOW;
}

/** КН1+КН2 — смена АВТО/РУЧ. */
bool modePairPressedNow() {
  return digitalRead(PIN_BUT1) == LOW && digitalRead(PIN_BUT2) == LOW;
}

/** КН1+КН4 — авария (вместо КН4+КН5). */
bool emergencyPairPressedNow() {
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
    cfg_taskMonitorUpdate(1);

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
        applyBacklightPwm(constrain(cfg_getContrast(), CONTRAST_MIN, CONTRAST_MAX));
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
    if (cfg_errorHasUiBlocking() && !menuVisible) {
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
