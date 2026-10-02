#include "task_ota.h"
#include "ota_net.h"
#include "ota_list.h"
#include "ota_install.h"
#include "ui_ota_menu.h"
#include "ui_marquee.h"
#include "wifi_setup.h"
#include "app_globals.h"
#include "app_version.h"
#include "app_types.h"
#include "task_pool.h"
#include "logger.h"
#include "mutex_guard.h"
#include "semver_utils.h"
#include "github_ota_request.h"
#include "arduino_ota.h"
#include "ui_menu_build.h"
#include "test_harness.h"

#include <GEM_adafruit_gfx.h>

extern GEM_adafruit_gfx gem;
extern SemaphoreHandle_t xDisplayMutex;


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

  /* --- inline serial harness --- */
  static char serialLine[96];
  static size_t serialLen = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_OTA);

    /* --- handleOtaSerialCommands inlined --- */
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') continue;
      if (c == '\n') {
        serialLine[serialLen] = '\0';
        serialLen = 0;
        if (serialLine[0] == '\0') continue;

        Serial.printf("[OTA-CMD] rx: %s\n", serialLine);

        if (strcmp(serialLine, "TEST MENU OPEN") == 0 || strcmp(serialLine, "test menu open") == 0) {
          requestMenuOpen("TEST");
          Serial.println("[TEST] OK MENU_OPEN_REQ");
          continue;
        }
        if (strcmp(serialLine, "TEST MENU CLOSE") == 0 || strcmp(serialLine, "test menu close") == 0) {
          requestMenuClose("TEST");
          Serial.println("[TEST] OK MENU_CLOSE_REQ");
          continue;
        }
        if (strcmp(serialLine, "TEST MENU DOWN") == 0 || strcmp(serialLine, "test menu down") == 0) {
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
        if (strcmp(serialLine, "TEST MENU UP") == 0 || strcmp(serialLine, "test menu up") == 0) {
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
        if (strcmp(serialLine, "TEST MENU OK") == 0 || strcmp(serialLine, "test menu ok") == 0) {
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

        if (strncmp(serialLine, "TEST", 4) == 0 || strncmp(serialLine, "test", 4) == 0) {
          processTestCommandLine(serialLine);
          continue;
        }

        char upper[96];
        size_t i = 0;
        for (; serialLine[i] && i + 1 < sizeof(upper); ++i) {
          const char ch = serialLine[i];
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
          for (int w = 0; w < 40; w++) {
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
          const int idx = atoi(serialLine + 12);
          Serial.printf("[OTA-CMD] INSTALL %d\n", idx);
          beginGitHubOtaProgressUi("Установка…");
          if (otaReleaseCount == 0 && !fetchGitHubReleaseList()) {
            Serial.println("[OTA-CMD] INSTALL: no list");
            endGitHubOtaProgressUi(true);
          } else if (!installGitHubReleaseIndex(static_cast<int8_t>(idx))) {
            Serial.printf("[OTA-CMD] INSTALL fail status=%s\n", otaListStatus);
            endGitHubOtaProgressUi(true);
          }
        } else if (strcmp(upper, "OTA SELFTEST") == 0) {
          Serial.println("[OTA-CMD] SELFTEST…");
          githubOtaPrintHeap("pre-selftest");
          beginGitHubOtaProgressUi("Selftest…");
          if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
            Serial.printf("[OTA-CMD] SELFTEST list fail: %s\n", otaListStatus);
            endGitHubOtaProgressUi(true);
          } else {
            int8_t pick = 0;
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
      if (serialLen + 1 < sizeof(serialLine)) {
        serialLine[serialLen++] = c;
      } else {
        serialLen = 0;
      }
    }
    /* --- end serial harness --- */

    if (Serial.available() > 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

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

    (void)bootGithubListDone;
    bootGithubListDone = true;

    GitHubOtaRequest ghReq = takeGitHubOtaRequest();
    if (ghReq == GitHubOtaRequest::CHECK_ONLY) {
      Logger::log(Logger::INFO, "GH-OTA", "Проверка обновлений (без установки)");
      checkGitHubUpdate(false);
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
      extern char otaStatus[32];
      strlcpy(otaStatus, "Проверка…", sizeof(otaStatus));
      displayDirty = true;
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
          extern char otaStatus[32];
          strlcpy(otaStatus, "Загрузка…", sizeof(otaStatus));
          displayDirty = true;
          if (!checkGitHubUpdate(true)) {
            if (otaListStatus[0] == '\0') {
              strlcpy(otaListStatus, "ошибка установки", sizeof(otaListStatus));
            }
            endGitHubOtaProgressUi(true);
          }
        }
      }
    } else if (ghReq == GitHubOtaRequest::FETCH_LIST) {
      Serial.println("[GH-OTA] FETCH_LIST begin");
      Logger::log(Logger::INFO, "GH-OTA", "Получение списка релизов");
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
