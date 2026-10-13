#pragma once
#include <Arduino.h>
#include <GEM_adafruit_gfx.h>

void otaCheckUpdates();
void otaOpenFirmwareList();
void otaInstallLast();
void otaSelectRelease(uint8_t idx);
void otaCardInstall();
void otaCardCheckSha();
void otaArduinoMode();
void otaExitMode();
void refreshOtaCard();
void refreshOtaListPage();
void refreshOtaPage();
void beginGitHubOtaProgressUi(const char *status);
void endGitHubOtaProgressUi(bool reopenMenu);

extern GEMPage otaPage;
extern GEMPage otaListPage;
extern GEMPage otaCardPage;

extern GEMItem itemOtaCurrent;
extern GEMItem itemOtaStatus;
extern GEMItem itemOtaList;
extern GEMItem itemOtaInstallLast;
extern GEMItem itemOtaArduino;
extern GEMItem itemOtaExit;

extern char otaListHdrBuf[];
extern char otaItemTitle[][34];
extern GEMItem itemOtaListHdr;
extern GEMItem itemOtaRel0;
extern GEMItem itemOtaRel1;
extern GEMItem itemOtaRel2;
extern GEMItem *const otaRelItems[];

extern GEMItem itemCardTag;
extern GEMItem itemCardInfo;
extern GEMItem itemCardStatus;
extern GEMItem itemCardSha;
extern GEMItem itemCardInstall;
extern GEMItem itemCardCheckSha;
