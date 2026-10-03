#pragma once
/**
 *  ui_menu_build.h — GEM pages, spinners, dynamic refresh, menu item setup.
 *  OTA pages stay in ui_ota_menu. gem / tft / GEMPage objects stay in the .ino.
 */

#include <Arduino.h>
#include <GEM_adafruit_gfx.h>

constexpr int CONTRAST_MIN = 10;
constexpr int CONTRAST_DIM = 5;
constexpr int CONTRAST_MAX = 100;

void initGEM();
void refreshDynamicMenu();
void menuExitAction();
void applyBacklightPwm(int level);
void openMenu();
void requestMenuOpen(const char *via);
void requestMenuClose(const char *via);

extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;

extern bool settingsChanged;

extern int   editReleaseDelay;
extern int   editInflateDelay;
extern float editTiltX;
extern float editTiltY;
extern float editPressureMin;
extern float editPressureMax;
extern int   editMasterLowTenths;
extern int   editNivCount;
extern int   editTimeInterval;
extern int   editContrast;
extern float editMovementPressureFront;
extern float editMovementPressureRear;
extern float editParkingPressure;
extern int   editMasterCheck;
extern int   editManualMaxTime;
extern int   editPressStabilizeMs;
extern int   editPressIdleMin;
extern float editDeadband;
extern float editCoarseZone;
extern float editFineZone;
extern float editWorsening;
extern bool  editMovementEnabled;
extern int   editMoveDuration;
extern int   editMoveSettle;
extern int   editMoveCheck;
extern float editMoveTolerance;
extern int   editBacklightOff;
extern int   editFrameMs;
extern float editRedrawAngle;
extern float editRedrawPressure;
extern int   editImuMotionDet;
extern int   editImuDlpfMode;       // DLPF_CFG 0..6 → 256…5 Hz
extern int   editImuAccelFs;        // AFS_SEL 0..3 → ±2/±4/±8/±16G
extern int   editGyroThreshold;
extern int   editGyroBumpThreshold;
extern int   editAccelThreshold;
extern int   editAccelThrStep;      // 0..4 → 200/500/1000/1500/2000
extern float editZeroAngleX;
extern float editZeroAngleY;
extern float editImuKalmanMea;
extern float editImuKalmanEst;
extern float editImuKalmanQ;
extern int   editImuPollMs;
extern int   editImuFifoAvg;
extern float editImuEmaAlpha;
extern float editImuEmaSpikeAlpha;
extern float editImuEmaSpikeThr;
extern float editImuSlewDps;
extern int   editImuPreset;

extern GEM_adafruit_gfx gem;
extern GEMPage mainPage;
extern GEMPage systemPage;
extern GEMPage valveBlockPage;
extern GEMPage testPage;
extern GEMPage valvePage;
extern GEMPage pressurePage;
extern GEMPage autoPage;
extern GEMPage displayPage;
extern GEMPage movementPage;
extern GEMPage infoPage;
extern GEMPage imuPage;
extern GEMPage settingsViewPage;
