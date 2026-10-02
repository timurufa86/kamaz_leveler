#include "ui_ota_menu.h"
#include "ui_marquee.h"
#include "ota_list.h"
#include "ota_install.h"
#include "ota_net.h"
#include "app_globals.h"
#include "app_version.h"
#include "app_types.h"
#include "semver_utils.h"
#include "github_ota_request.h"

extern GEM_adafruit_gfx gem;
extern GEMPage mainPage;

void startOTAMode();             // .ino
void stopOTAMode();              // .ino
void forceDisplayReset(bool);    // .ino

GEMPage otaPage("Обновления", mainPage);
GEMPage otaListPage("Прошивки (GitHub)", otaPage);
GEMPage otaCardPage("Релиз", otaListPage);

GEMItem itemOtaCurrent("Версия: ...");
GEMItem itemOtaStatus("Статус: ...");
GEMItem itemOtaList("Список прошивок", []() { otaOpenFirmwareList(); });
GEMItem itemOtaInstallLast("Установить последнюю", []() { otaInstallLast(); });
GEMItem itemOtaArduino("Режим ArduinoOTA (Wi-Fi)", []() { otaArduinoMode(); });
GEMItem itemOtaExit("Выход из режима OTA", []() { otaExitMode(); });

char otaListHdrBuf[40] = "загрузка с GitHub...";
char otaItemTitle[OTA_LIST_MAX][34];
GEMItem itemOtaListHdr(otaListHdrBuf);
GEMItem itemOtaRel0("—", []() { otaSelectRelease(0); });
GEMItem itemOtaRel1("—", []() { otaSelectRelease(1); });
GEMItem itemOtaRel2("—", []() { otaSelectRelease(2); });
GEMItem *const otaRelItems[OTA_LIST_MAX] = {
  &itemOtaRel0, &itemOtaRel1, &itemOtaRel2
};

GEMItem itemCardTag("Релиз: —");
GEMItem itemCardInfo("данных нет");
GEMItem itemCardStatus("обновите список");
GEMItem itemCardSha("SHA-256: не проверен");
GEMItem itemCardInstall("УСТАНОВИТЬ", []() { otaCardInstall(); });
GEMItem itemCardCheckSha("Проверить SHA-256", []() { otaCardCheckSha(); });

extern int otaProgress;
extern char otaStatus[32];

void beginGitHubOtaProgressUi(const char *status) {
  otaProgress = 0;
  strlcpy(otaStatus, status ? status : "OTA…", sizeof(otaStatus));
  otaUiNeedFullRedraw = true;
  menuVisible = false;
  displayDirty = true;
}

void endGitHubOtaProgressUi(bool reopenMenu) {
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

void otaCheckUpdates() {
  strlcpy(otaListStatus, "проверка...", sizeof(otaListStatus));
  displayDirty = true;
  requestGitHubOtaCheckOnly();
  Serial.println("[GH-OTA] Запрос проверки обновления");
}

void otaOpenFirmwareList() {
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

void otaInstallLast() {
  strlcpy(otaListStatus, "установка...", sizeof(otaListStatus));
  beginGitHubOtaProgressUi("Проверка GitHub...");
  requestGitHubOtaCheckAndInstall();
  Serial.println("[GH-OTA] Запрос на установку последнего релиза");
}

void otaSelectRelease(uint8_t idx) {
  if (idx >= otaReleaseCount) {
    strlcpy(otaListStatus, "сначала откройте список", sizeof(otaListStatus));
    displayDirty = true;
    return;
  }
  otaSelectedIndex = static_cast<int8_t>(idx);
  gem.setMenuPageCurrent(otaCardPage);
  displayDirty = true;
}

void otaCardInstall() {
  if (otaSelectedIndex < 0) return;
  beginGitHubOtaProgressUi("Загрузка релиза…");
  requestGitHubOtaInstallIndex(otaSelectedIndex);
  Serial.printf("[GH-OTA] Запрос на установку релиза #%d\n", (int)otaSelectedIndex);
}

void otaCardCheckSha() {
  if (otaSelectedIndex < 0 || otaSelectedIndex >= static_cast<int8_t>(otaReleaseCount)) return;
  fetchReleaseSha256(static_cast<uint8_t>(otaSelectedIndex));
  refreshOtaCard();
  displayDirty = true;
}

void otaArduinoMode() {
  menuVisible = false;
  displayDirty = true;
  startOTAMode();
}

void otaExitMode() {
  stopOTAMode();
  displayDirty = true;
}

void refreshOtaPage() {
  mqBind(MQ_OTA_VER, itemOtaCurrent, "Верс: ", VERSION);
  mqBind(MQ_OTA_ST, itemOtaStatus, "Статус: ", otaListStatus);
  extern SystemState currentState;
  if (currentState == SystemState::OTA_MODE) {
    itemOtaExit.show();
  } else {
    itemOtaExit.hide();
  }
}

void refreshOtaListPage() {
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

void refreshOtaCard() {
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
