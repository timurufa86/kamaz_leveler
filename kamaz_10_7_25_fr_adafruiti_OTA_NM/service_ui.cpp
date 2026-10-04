#include "service_ui.h"
#include "ui_screens.h"
#include "ui_menu_build.h"
#include "ui_theme.h"
#include "ui_text.h"
#include "valve_ctrl.h"
#include "config_manager.h"
#include "error_handler.h"
#include "imu_dmp.h"
#include "task_imu.h"
#include "app_globals.h"
#include "app_pins.h"
#include "app_types.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "logger.h"
#include <EncButton.h>
#include <Adafruit_ST7789.h>
#include <cstring>
#include <cmath>

extern Adafruit_ST7789 tft;
extern Button button0;
extern Button button1;
extern Button button2;
extern Button button3;

constexpr uint32_t TEST_VALVE_OPEN_TIME_MS = 1500;
constexpr uint32_t TEST_STABILIZE_TIME_MS = 500;
constexpr uint32_t TEST_PRESSURE_EQUALIZE_TIME_MS = 3000;
constexpr uint32_t TEST_TIMEOUT_MS = 90000;
constexpr float TEST_MIN_PRESS_FOR_TEST = 0.8f;
constexpr float TEST_ZERO_THRESHOLD = 0.1f;
constexpr float TEST_PRESSURE_CHANGE_THRESHOLD = 0.2f;

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
static void renderTestResults();

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

