#include "task_display.h"
#include "service_ui.h"
#include "arduino_ota.h"
#include "ui_screens.h"
#include "ui_menu_build.h"
#include "ui_ota_menu.h"
#include "ui_marquee.h"
#include "wifi_setup.h"
#include "app_globals.h"
#include "app_types.h"
#include "mutex_guard.h"
#include "task_pool.h"
#include "logger.h"
#include <Adafruit_ST7789.h>
#include <GEM_adafruit_gfx.h>

extern Adafruit_ST7789 tft;
extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;
extern bool errorScreenBlocking;
extern bool mvScreenWasActive;

void saveMenuSettings();

volatile MenuReq g_menuReq = MenuReq::None;

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
extern int cfg_getFrameMs();

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
    framePeriod = pdMS_TO_TICKS(cfg_getFrameMs());

    // Приглушение подсветки по бездействию (0 = никогда).
    // Не гасим экран — только PWM уровня CONTRAST_DIM. В меню не приглушаем.
    {
      const int blMin = cfg_getBacklightOffMin();
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
    cfg_taskMonitorUpdate(2);

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
              if (!cfg_errorHasActive() &&
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
      bool hasErrorNow = cfg_errorHasUiBlocking();
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
      if (cfg_errorHasUiBlocking()) {
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
