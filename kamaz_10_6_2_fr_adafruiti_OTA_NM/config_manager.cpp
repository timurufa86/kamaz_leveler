#include "config_manager.h"
#include "logger.h"
#include "wifi_setup.h"
#include "imu_dmp.h"
#include "imu_motion.h"
#include "task_imu.h"
#include "pressure_read.h"
#include "app_types.h"

extern char wifi_ssid[32];
extern char wifi_password[64];
extern char ota_password[64];
extern char sta_ssid[33];
extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;

ConfigManager::Config ConfigManager::currentConfig;

int   cfg_getImuGyroOffX()          { return ConfigManager::getImuGyroOffX(); }
int   cfg_getImuGyroOffY()          { return ConfigManager::getImuGyroOffY(); }
int   cfg_getImuGyroOffZ()          { return ConfigManager::getImuGyroOffZ(); }
int   cfg_getImuAccelOffX()         { return ConfigManager::getImuAccelOffX(); }
int   cfg_getImuAccelOffY()         { return ConfigManager::getImuAccelOffY(); }
int   cfg_getImuAccelOffZ()         { return ConfigManager::getImuAccelOffZ(); }
int   cfg_getImuMotionDet()         { return ConfigManager::getImuMotionDet(); }
int   cfg_getImuDlpfMode()          { return ConfigManager::getImuDlpfMode(); }
int   cfg_getImuAccelFs()           { return ConfigManager::getImuAccelFs(); }
int   cfg_getGyroThreshold()        { return ConfigManager::getGyroThreshold(); }
int   cfg_getGyroBumpThreshold()    { return ConfigManager::getGyroBumpThreshold(); }
int   cfg_getAccelThreshold()       { return ConfigManager::getAccelThreshold(); }
bool  cfg_getMovementEnabled()      { return ConfigManager::getMovementEnabled(); }
int   cfg_getMovementSettleSec()    { return ConfigManager::getMovementSettleSec(); }
int   cfg_getMovementDurationSec()  { return ConfigManager::getMovementDurationSec(); }
int   cfg_getImuPollMs()            { return ConfigManager::getImuPollMs(); }
int   cfg_getImuFifoAvg()           { return ConfigManager::getImuFifoAvg(); }
float cfg_getImuKalmanMea()         { return ConfigManager::getImuKalmanMea(); }
float cfg_getImuKalmanEst()         { return ConfigManager::getImuKalmanEst(); }
float cfg_getImuKalmanQ()           { return ConfigManager::getImuKalmanQ(); }
float cfg_getImuEmaAlpha()          { return ConfigManager::getImuEmaAlpha(); }
float cfg_getImuEmaSpikeAlpha()     { return ConfigManager::getImuEmaSpikeAlpha(); }
float cfg_getImuEmaSpikeThr()       { return ConfigManager::getImuEmaSpikeThr(); }
float cfg_getImuSlewDps()           { return ConfigManager::getImuSlewDps(); }
float cfg_getZeroAngleX()           { return ConfigManager::getZeroAngleX(); }
float cfg_getZeroAngleY()           { return ConfigManager::getZeroAngleY(); }
float cfg_getMovementPressureFront(){ return ConfigManager::getMovementPressureFront(); }
float cfg_getMovementPressureRear() { return ConfigManager::getMovementPressureRear(); }
float cfg_getMovementTolerance()    { return ConfigManager::getMovementTolerance(); }

void cfg_setMasterLowBar(float v) { ConfigManager::setMasterLowBar(v); }
void cfg_setBacklightOffMin(int v) { ConfigManager::setBacklightOffMin(v); }
// cfg_taskMonitorUpdate — placed after TaskMonitor class definition below

/* ── Bridge functions for ConfigManager (pressure/valve/auto-level modules) ── */
float cfg_getPressureMin()          { return ConfigManager::getPressureMin(); }
float cfg_getPressureMax()          { return ConfigManager::getPressureMax(); }
float cfg_getPressureDeadband()     { return ConfigManager::getPressureDeadband(); }
float cfg_getMasterLowBar()         { return ConfigManager::getMasterLowBar(); }
float cfg_getParkingPressureBar()   { return ConfigManager::getParkingPressureBar(); }
int   cfg_getInflateDelay()         { return ConfigManager::getInflateDelay(); }
int   cfg_getReleaseDelay()         { return ConfigManager::getReleaseDelay(); }
int   cfg_getMasterCheckSec()       { return ConfigManager::getMasterCheckSec(); }
int   cfg_getMovementCheckSec()     { return ConfigManager::getMovementCheckSec(); }
int   cfg_getManualMaxTimeSec()     { return ConfigManager::getManualMaxTimeSec(); }
int   cfg_getPressureStabilizeMs()  { return ConfigManager::getPressureStabilizeMs(); }
int   cfg_getPressureIdleMin()      { return ConfigManager::getPressureIdleMin(); }
float cfg_getRedrawPressureThr()    { return ConfigManager::getRedrawPressureThr(); }
float cfg_getTiltThresholdX()       { return ConfigManager::getTiltThresholdX(); }
float cfg_getTiltThresholdY()       { return ConfigManager::getTiltThresholdY(); }
float cfg_getCoarseZoneRatio()      { return ConfigManager::getCoarseZoneRatio(); }
float cfg_getWorseningRatio()       { return ConfigManager::getWorseningRatio(); }
float cfg_getFineZoneRatio()        { return ConfigManager::getFineZoneRatio(); }
int   cfg_getNivCount()             { return ConfigManager::getNivCount(); }
int cfg_getBacklightOffMin() { return ConfigManager::getBacklightOffMin(); }
int cfg_getContrast() { return ConfigManager::getContrast(); }
int cfg_getFrameMs() { return ConfigManager::getFrameMs(); }
float cfg_getRedrawAngleThr() { return ConfigManager::getRedrawAngleThr(); }
int cfg_getTimeInterval() { return ConfigManager::getTimeInterval(); }

/** Применение настроек к железу сразу после сохранения (без перезагрузки). */
void applyRuntimeSettings() {
  backlightDimmed = false;
  applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));

  if (ConfigManager::getBacklightOffMin() == 0) {
    backlightDimmed = false;
  }
  lastUserActivityMs = millis();

  if (mpuOk) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(50));
    if (i2c) {
      // Accel FS + DLPF (dmpInitialize мог переписать регистры).
      static const uint8_t kAccelFs[] = {
          MPU6050_ACCEL_FS_2, MPU6050_ACCEL_FS_4, MPU6050_ACCEL_FS_8, MPU6050_ACCEL_FS_16};
      static const int kAccelG[] = { 2, 4, 8, 16 };
      const uint8_t fsIdx = (uint8_t)constrain(ConfigManager::getImuAccelFs(), 0, 3);
      mpu.setFullScaleAccelRange(kAccelFs[fsIdx]);
      const uint8_t dlpf = (uint8_t)constrain(ConfigManager::getImuDlpfMode(), 0, 6);
      mpu.setDLPFMode(dlpf);
      // Аппаратный MOT после DMP: DHPF + THR + DUR + counter decrement + INT.
      mpu.setDHPFMode(MPU6050_DHPF_1P25);
      mpu.setMotionDetectionThreshold((uint8_t)ConfigManager::getImuMotionDet());
      mpu.setMotionDetectionDuration(g_motDurMs);
      mpu.setAccelerometerPowerOnDelay(3);
      mpu.setMotionDetectionCounterDecrement(1);
      mpu.setIntEnabled(0x52);  // MOT | FIFO_OFLOW | DMP_INT
      (void)mpu.getIntStatus();
      static const int kDlpfHz[] = { 256, 188, 98, 42, 20, 10, 5 };
      Serial.printf("[IMU] accelFS=±%dG DLPF=%u(~%dHz) MOT_THR=%d MOT_DUR=%u INT_EN=0x%02X\n",
                    kAccelG[fsIdx], (unsigned)dlpf, kDlpfHz[dlpf],
                    ConfigManager::getImuMotionDet(), (unsigned)g_motDurMs,
                    (unsigned)mpu.getIntEnabled());
    }
  }

  filterX.setParameters(ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                        ConfigManager::getImuKalmanQ());
  filterY.setParameters(ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                        ConfigManager::getImuKalmanQ());
  g_imuFilterResetReq = true;
  Serial.printf("[IMU] Калман mea=%.2f est=%.2f q=%.3f | poll=%dms fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f\n",
                ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                ConfigManager::getImuKalmanQ(), ConfigManager::getImuPollMs(),
                ConfigManager::getImuFifoAvg(), ConfigManager::getImuEmaAlpha(),
                ConfigManager::getImuEmaSpikeAlpha(), ConfigManager::getImuEmaSpikeThr(),
                ConfigManager::getImuSlewDps());

  Serial.printf("[MENU] Применено: кадр=%d мс, приглуш.=%d мин, зона=%.2f бар, допуск=%.2f бар moveEn=%d\n",
                ConfigManager::getFrameMs(), ConfigManager::getBacklightOffMin(),
                ConfigManager::getPressureDeadband(), ConfigManager::getMovementTolerance(),
                ConfigManager::getMovementEnabled() ? 1 : 0);

  // Тумблер «Режим Движение»=Выкл: сразу выйти из MOVEMENT.
  if (!ConfigManager::getMovementEnabled() &&
      (movementModeActive || currentSystemMode == SystemMode::MOVEMENT)) {
    movementModeActive = false;
    movementEndTime = 0;
    {
      MutexGuard guard(xStateMutex, pdMS_TO_TICKS(50));
      if (guard) {
        currentSystemMode = previousMode;
      }
    }
    if (previousMode == SystemMode::AUTO) {
      currentMode = Mode::AUTO;
    } else {
      currentMode = Mode::MANUAL;
      setManualTargetsFromParkingPolicy();
    }
    Serial.println("[MOVEMENT] Режим выключен в меню — выход из MOVEMENT");
  }

  // Не вызываем forceDisplayReset здесь: fillScreen без xDisplayMutex с ButtonTask
  // (и вложенно из close) давал подвисание SPI. Сброс экрана — у вызывающего
  // под мьютексом дисплея или через requestMenuClose().
  displayDirty = true;
}

// 11.6. Сохранение настроек меню
void saveMenuSettings() {
  if (!settingsChanged) return;

  if (editPressureMin >= editPressureMax) {
    Serial.println("[MENU] Отклонено: pressureMin >= pressureMax");
    Logger::log(Logger::WARNING, "MENU", "min >= max — не сохранено");
    return;
  }
  editMovementPressureFront = constrain(editMovementPressureFront, editPressureMin, editPressureMax);
  editMovementPressureRear = constrain(editMovementPressureRear, editPressureMin, editPressureMax);
  if (editFineZone >= editCoarseZone) {
    editFineZone = editCoarseZone * 0.4f;
  }

  ConfigManager::setReleaseDelay(editReleaseDelay);
  ConfigManager::setInflateDelay(editInflateDelay);
  ConfigManager::setTiltThresholdX(editTiltX);
  ConfigManager::setTiltThresholdY(editTiltY);
  ConfigManager::setPressureMin(editPressureMin);
  ConfigManager::setPressureMax(editPressureMax);
  ConfigManager::setMasterLowBar(editMasterLowTenths * 0.1f);
  ConfigManager::setNivCount(editNivCount);
  ConfigManager::setTimeInterval(editTimeInterval);
  ConfigManager::setContrast(editContrast);
  ConfigManager::setMovementPressureFront(editMovementPressureFront);
  ConfigManager::setMovementPressureRear(editMovementPressureRear);
  ConfigManager::setParkingPressureBar(editParkingPressure);

  // ===== 8.8.0: новые параметры =====
  ConfigManager::setMasterCheckSec(constrain(editMasterCheck, 1, 10) * 60);
  ConfigManager::setManualMaxTimeSec(editManualMaxTime);
  ConfigManager::setPressureDeadband(editDeadband);
  ConfigManager::setPressureStabilizeMs(editPressStabilizeMs);
  ConfigManager::setPressureIdleMin(editPressIdleMin);
  ConfigManager::setCoarseZoneRatio(editCoarseZone);
  ConfigManager::setFineZoneRatio(editFineZone);
  ConfigManager::setWorseningRatio(editWorsening);
  ConfigManager::setMovementEnabled(editMovementEnabled);
  ConfigManager::setMovementDurationSec(editMoveDuration);
  ConfigManager::setMovementSettleSec(editMoveSettle);
  ConfigManager::setMovementCheckSec(editMoveCheck);
  ConfigManager::setMovementTolerance(editMoveTolerance);
  ConfigManager::setGyroThreshold(constrain(editGyroThreshold, 1, 25));
  ConfigManager::setGyroBumpThreshold(constrain(editGyroBumpThreshold, 1, 25));
  // Ступени Δlin: 0..4 → 200/500/1000/1500/2000
  static const int kLinThr[] = { 200, 500, 1000, 1500, 2000 };
  editAccelThrStep = constrain(editAccelThrStep, 0, 4);
  editAccelThreshold = kLinThr[editAccelThrStep];
  ConfigManager::setAccelThreshold(editAccelThreshold);
  ConfigManager::setBacklightOffMin(editBacklightOff);
  ConfigManager::setFrameMs(editFrameMs);
  ConfigManager::setRedrawAngleThr(editRedrawAngle);
  ConfigManager::setRedrawPressureThr(editRedrawPressure);
  ConfigManager::setImuMotionDet(editImuMotionDet);
  ConfigManager::setImuDlpfMode(editImuDlpfMode);
  ConfigManager::setImuAccelFs(constrain(editImuAccelFs, 0, 3));
  ConfigManager::setZeroAngleX(editZeroAngleX);
  ConfigManager::setZeroAngleY(editZeroAngleY);
  ConfigManager::setImuKalmanMea(editImuKalmanMea);
  ConfigManager::setImuKalmanEst(editImuKalmanEst);
  ConfigManager::setImuKalmanQ(editImuKalmanQ);
  ConfigManager::setImuPollMs(editImuPollMs);
  ConfigManager::setImuFifoAvg(editImuFifoAvg);
  ConfigManager::setImuEmaAlpha(editImuEmaAlpha);
  ConfigManager::setImuEmaSpikeAlpha(editImuEmaSpikeAlpha);
  ConfigManager::setImuEmaSpikeThr(editImuEmaSpikeThr);
  ConfigManager::setImuSlewDps(editImuSlewDps);

  const bool saved = saveConfig();
  if (!saved) {
    Serial.println("[MENU] Сохранение в LittleFS не удалось — settingsChanged оставлен");
    Logger::log(Logger::ERROR, "MENU", "Конфиг не записан (LittleFS FD?)");
    // Параметры уже в RAM — применяем, чтобы пресет MPU не «терялся» до следующего Save
    applyRuntimeSettings();
    return;
  }
  settingsChanged = false;
  applyRuntimeSettings();  // 8.8.0: новые значения действуют сразу, без перезагрузки
  Logger::log(Logger::INFO, "MENU", "Настройки сохранены");
}

bool initFileSystem() {
  Serial.println("[FS] Инициализация LittleFS...");
  // maxOpenFiles=16: при утечке FD (корень каталога и т.п.) save() иначе падает
  // с «Unable to allocate FD» — после смены пресета MPU + СОХРАНИТЬ это давало ребут/обрыв MPU.
  bool mounted = LittleFS.begin(false, "/littlefs", 16, "spiffs");
  if (!mounted) {
    Serial.println("[FS] Раздел повреждён, выполняю однократное форматирование");
    LittleFS.end();
    mounted = LittleFS.begin(true, "/littlefs", 16, "spiffs");
    if (!mounted) {
      Serial.println("[FS] Не удалось восстановить LittleFS");
      return false;
    }
    Serial.println("[FS] LittleFS отформатирована и подключена");
  }
  Serial.println("[FS] LittleFS смонтирован успешно");

  File root = LittleFS.open("/");
  int fileCount = 0;
  if (root) {
    File file = root.openNextFile();
    while (file) {
      fileCount++;
      Serial.printf("[FS] Файл: %s (%d байт)\n", file.name(), file.size());
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }

  if (fileCount == 0 || !LittleFS.exists("/config.txt")) {
    Serial.println("[FS] Конфигурация отсутствует, создаю значения по умолчанию");
    if (!ConfigManager::save()) {
      Serial.println("[FS] Не удалось сохранить конфигурацию по умолчанию");
    }
  }

  return true;
}

bool loadConfig() {
  return ConfigManager::load();
}
bool saveConfig() {
  return ConfigManager::save();
}

// initializeJhm1200 — see pressure_read.cpp

bool loadWiFiConfig() {
  Logger::log(Logger::INFO, "WiFi", "Загрузка");
  File f = LittleFS.open("/wifi_config.txt", "r");
  if (!f) {
    Logger::log(Logger::INFO, "WiFi", "Нет файла – сохраняем дефолты");
    if (!saveWiFiConfig()) {
      Logger::log(Logger::ERROR, "WiFi", "Не удалось сохранить дефолты");
    }
    return false;
  }
  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка JSON");
    return false;
  }

  strlcpy(wifi_ssid, doc["ssid"] | "Kamaz-OTA-AP", sizeof(wifi_ssid));
  strlcpy(wifi_password, doc["password"] | wifi_password, sizeof(wifi_password));
  strlcpy(ota_password, doc["otaPassword"] | ota_password, sizeof(ota_password));
  strlcpy(sta_ssid, doc["staSsid"] | "", sizeof(sta_ssid));
  initializeDefaultCredentials();
  Logger::log(Logger::INFO, "WiFi", "Конфиг загружен");
  return true;
}

bool saveWiFiConfig() {
  Logger::log(Logger::INFO, "WiFi", "Сохранение");
  StaticJsonDocument<384> doc;
  doc["ssid"] = wifi_ssid;
  doc["password"] = wifi_password;
  doc["otaPassword"] = ota_password;
  doc["staSsid"] = sta_ssid;

  File f = LittleFS.open("/wifi_config.txt", "w");
  if (!f) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка записи");
    return false;
  }

  size_t bytesWritten = serializeJson(doc, f);
  f.close();

  if (bytesWritten == 0) {
    Logger::log(Logger::ERROR, "WiFi", "Ошибка сериализации JSON");
    LittleFS.remove("/wifi_config.txt");
    return false;
  }

  Logger::log(Logger::INFO, "WiFi", "Сохранено");
  return true;
}

