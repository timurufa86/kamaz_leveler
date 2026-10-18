#pragma once
/**
 *  service_ui.h — valve-test sequence and service screens
 *  (manual valves, IMU zero/calib, MPU diag).
 */
#include <Arduino.h>

void manualValveCloseAll();
void openValveTestScreen();
void openImuZeroConfirm();
void openImuCalibScreen();
void openMpuDiagScreen();
void handleServiceInput();
void startValveTest();
/** true, пока идёт тест клапанов (доступно другим модулям, напр. HTTP-API). */
bool valveTestActive();
/** Полный выход из теста клапанов: закрыть клапаны, снять manifoldBusy. */
void valveTestAbort();
void runValveTestLogic();
void updateTestDisplay();
void printTestResults();
