#include "pressure_read.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "jhm1200.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "logger.h"
#include <Wire.h>
#include <cmath>

/* ── ConfigManager / ErrorHandler bridge functions (defined in .ino) ── */
extern float cfg_getPressureMin();
extern float cfg_getPressureMax();
extern float cfg_getPressureDeadband();
extern float cfg_getMasterLowBar();
extern float cfg_getMovementPressureFront();
extern float cfg_getMovementPressureRear();
extern float cfg_getMovementTolerance();
extern float cfg_getParkingPressureBar();
extern int   cfg_getInflateDelay();
extern int   cfg_getReleaseDelay();
extern int   cfg_getMasterCheckSec();
extern int   cfg_getMovementCheckSec();

extern bool  cfg_errorIsActive(uint8_t err);
extern void  cfg_errorHandle(uint8_t err, const char *msg);
extern void  cfg_errorMarkCleared(uint8_t err);
extern void  cfg_errorUpdateTime(uint8_t err);
extern void  cfg_errorRemove(uint8_t err);
extern bool  cfg_errorIsPendingClear(uint8_t err);
extern void  cfg_errorCancelClear(uint8_t err);
extern bool  cfg_errorHasCriticalPneumatic();

/* ── .ino helpers ── */
void setDisplayDirty();
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs);
void closeAllValves();
void emergencyStop();

/* ── ErrorHandler error codes (must match .ino) ── */
static constexpr uint8_t ERR_SENSOR       = 5; // ErrorHandler::Error::SENSOR
static constexpr uint8_t ERR_LOW_PRESSURE  = 1; // ErrorHandler::Error::LOW_PRESSURE

/* ====================  readPressure  ==================== */
float readPressure() {
#if ENABLE_SIMULATION
  static float simPressure = 4.5f;
  simPressure += ((float)random(-10, 10) / 100.0f);
  simPressure = constrain(simPressure, 3.0f, 6.0f);

  if (cfg_errorIsActive(ERR_SENSOR)) {
    cfg_errorMarkCleared(ERR_SENSOR);
  }
  return simPressure;
#endif

  if (!jhmReady || !Jhm1200::isReady()) {
    return 0.0f;
  }

  static uint32_t lastReadTime = 0;
  static float lastGoodBar = 0.0f;
  static bool haveLast = false;
  const uint32_t now = millis();
  if (now - lastReadTime < 12) {
    return haveLast ? lastGoodBar : -1.0f;
  }
  lastReadTime = now;

  float bar = 0.0f;
  bool ok = false;
  {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(150));
    if (!i2c) return haveLast ? lastGoodBar : -1.0f;
    ok = Jhm1200::readBar(bar);
  }

  static uint8_t consecutiveFails = 0;
  if (!ok) {
    consecutiveFails++;
    if (consecutiveFails < 8) {
      return haveLast ? lastGoodBar : -1.0f;
    }
    consecutiveFails = 0;
    static uint32_t lastErrorTime = 0;
    if (millis() - lastErrorTime > 5000) {
      lastErrorTime = millis();
      Serial.println("[WARN] JHM1200: ошибка чтения!");
    }
    if (!cfg_errorIsActive(ERR_SENSOR)) {
      cfg_errorHandle(ERR_SENSOR, "JHM1200 - ошибка чтения");
    } else {
      cfg_errorUpdateTime(ERR_SENSOR);
    }
    return 0.0f;
  }
  consecutiveFails = 0;

  if (cfg_errorIsActive(ERR_SENSOR) && calibrationCompleted) {
    cfg_errorMarkCleared(ERR_SENSOR);
  }

  float localZeroBar = 0.0f;
  {
    MutexGuard guard(xCalibMutex);
    if (guard) {
      localZeroBar = g_pressureZeroBar;
    }
  }
  bar -= localZeroBar;
  if (bar < 0.02f) bar = 0.0f;
  if (bar > 10.0f) bar = 10.0f;

  /* Forward declarations for autoLevelingController — bridge */
  extern bool autoLevelIsBusy();
  const bool fastPath = manualControlActive || autoLevelIsBusy();
  if (!haveLast) {
    lastGoodBar = bar;
    haveLast = true;
  } else if (fastPath) {
    lastGoodBar = 0.35f * lastGoodBar + 0.65f * bar;
  } else {
    const float delta = bar - lastGoodBar;
    if (fabsf(delta) > 0.8f) {
      bar = lastGoodBar + (delta > 0 ? 0.8f : -0.8f);
    }
    lastGoodBar = 0.55f * lastGoodBar + 0.45f * bar;
  }
  return lastGoodBar;
}

/* ====================  initializeJhm1200  ==================== */
bool initializeJhm1200() {
  MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(500));
  if (!i2c && xI2CMutex != nullptr) return false;

  if (!Jhm1200::begin(Wire)) {
    jhmReady = false;
    Serial.println("[JHM1200] Не найден на адресе 0x78");
    return false;
  }
  jhmReady = true;
  return true;
}

/* ====================  checkPressureLimits  ==================== */
bool checkPressureLimits() {
#if ENABLE_SIMULATION
  return false;
#endif

  if (!calibrationCompleted) {
    return false;
  }

  if (!firstPressureMeasurementDone) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[CHECK] Ожидание первого замера давления...");
    }
    return false;
  }

  if (cfg_errorIsActive(ERR_SENSOR)) {
    return false;
  }

  float localMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return false;
    localMaster = masterPressure;
  }

  float masterLow = cfg_getMasterLowBar();
  if (masterLow <= 0.0f) {
    if (cfg_errorIsActive(ERR_LOW_PRESSURE)) {
      cfg_errorRemove(ERR_LOW_PRESSURE);
      Serial.println("[CHECK] LOW_PRESSURE снята сразу (Низк.МП=0, проверка выкл.)");
    }
    return false;
  }

  if (localMaster < masterLow) {
    if (cfg_errorIsPendingClear(ERR_LOW_PRESSURE)) {
      cfg_errorCancelClear(ERR_LOW_PRESSURE);
      Serial.println("[CHECK] Отменено удаление LOW_PRESSURE (давление снова упало)");
    }

    if (!cfg_errorIsActive(ERR_LOW_PRESSURE)) {
      cfg_errorHandle(ERR_LOW_PRESSURE, "Низкое давление в магистрали");
      Serial.printf("[CHECK] LOW_PRESSURE создана! МП: %.2f бар (порог: %.2f)\n",
                    localMaster, masterLow);
    } else {
      cfg_errorUpdateTime(ERR_LOW_PRESSURE);
    }
    return true;
  }

  if (cfg_errorIsActive(ERR_LOW_PRESSURE)) {
    if (!cfg_errorIsPendingClear(ERR_LOW_PRESSURE)) {
      cfg_errorMarkCleared(ERR_LOW_PRESSURE);
      Serial.printf("[CHECK] LOW_PRESSURE будет удалена через 3 сек. Давление: %.2f бар\n",
                    localMaster);
    } else {
      static uint32_t lastPendingLog = 0;
      if (millis() - lastPendingLog > 5000) {
        lastPendingLog = millis();
        Serial.printf("[CHECK] LOW_PRESSURE уже в очереди на удаление. Давление: %.2f бар\n",
                      localMaster);
      }
    }
  }

  return false;
}

/* ====================  checkAndAdjustMasterPressure  ==================== */
void checkAndAdjustMasterPressure() {
  if (!calibrationCompleted || !firstPressureMeasurementDone) return;
  if (cfg_errorIsActive(ERR_SENSOR)) return;
  if (manualControlActive || otaValveLock || otaMode || otaInProgress) return;

  uint32_t now = millis();
  if (now - lastMasterPressureCheckTime < (uint32_t)cfg_getMasterCheckSec() * 1000UL) {
    return;
  }
  lastMasterPressureCheckTime = now;

  requestPressureMeasurement();

  float localMaster = 0.0f;
  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(200));
    if (!guard) {
      Serial.println("[MASTER] state mutex timeout — skip adjust");
      return;
    }
    localMaster = masterPressure;
  }

  float minP = cfg_getPressureMin();
  float maxP = cfg_getPressureMax();
  Serial.printf("[MASTER] periodic check: %.2f bar (min=%.1f max=%.1f)\n",
                localMaster, minP, maxP);

  checkPressureLimits();

  if (localMaster > maxP + cfg_getPressureDeadband()) {
    if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(200), 3, "MASTER/relieve")) {
      Serial.println("[MASTER] cannot relieve — valve mutex busy");
      return;
    }
    closeAllValves();
    digitalWrite(PIN_DEFL, HIGH);
    xSemaphoreGive(xValveMutex);

    vTaskDelay(pdMS_TO_TICKS(400));

    if (takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(200), 3, "MASTER/relieve-off")) {
      digitalWrite(PIN_DEFL, LOW);
      xSemaphoreGive(xValveMutex);
    } else {
      digitalWrite(PIN_DEFL, LOW);
    }
    requestPressureMeasurement();
    Serial.println("[MASTER] relieved excess pressure on supply line");
  }
}

/* ====================  requestPressureMeasurement  ==================== */
void requestPressureMeasurement() {
  if (xPressureWakeupQueue == nullptr) {
    Serial.println("[PRESS] Wakeup queue is null!");
    return;
  }

  static uint32_t lastRequestTime = 0;
  uint32_t now = millis();

  if (now - lastRequestTime < 80) {
    return;
  }

  uint32_t dummy = 1;
  BaseType_t result = xQueueSend(xPressureWakeupQueue, &dummy, pdMS_TO_TICKS(10));

  if (result == pdTRUE) {
    lastRequestTime = now;
    Serial.println("[PRESS] Measurement requested");
  } else {
    Serial.println("[PRESS] Failed to request measurement - queue full");
  }
}

/* ====================  updateLeakMonitor  ==================== */
void updateLeakMonitor() {
  static float baseline[PAD_COUNT];
  static uint32_t baselineMs = 0;
  static bool haveBaseline = false;

  extern bool autoLevelIsBusy();
  if (manualControlActive || autoLevelIsBusy() ||
      currentSystemMode == SystemMode::MOVEMENT || otaInProgress ||
      cfg_errorHasCriticalPneumatic()) {
    haveBaseline = false;
    return;
  }

  const uint32_t now = millis();
  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    if (!pressureValid[i] || (now - pressureStampMs[i]) > 60000UL) {
      haveBaseline = false;
      return;
    }
  }

  if (!haveBaseline) {
    memcpy(baseline, pressure, sizeof(baseline));
    baselineMs = now;
    haveBaseline = true;
    return;
  }

  if (now - baselineMs < 600000UL) return;

  float worstDrop = 0.0f;
  int worstPad = -1;
  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    const float drop = baseline[i] - pressure[i];
    if (drop > worstDrop) {
      worstDrop = drop;
      worstPad = (int)i;
    }
  }
  if (worstPad >= 0 && worstDrop >= 0.35f) {
    leakSuspect = true;
    strlcpy(leakSuspectPad, padNames[worstPad], sizeof(leakSuspectPad));
    Serial.printf("[LEAK] Подозрение: %s падение %.2f бар за %lu с\n", leakSuspectPad, worstDrop,
                  (unsigned long)((now - baselineMs) / 1000));
  } else {
    leakSuspect = false;
    leakSuspectPad[0] = '\0';
  }
  haveBaseline = false;
}

/* ====================  setManualTargetPressure  ==================== */
void setManualTargetPressure(Pad pad) {
  MutexGuard guard(xStateMutex);
  if (guard) {
    manualTargetPressure[pad] = pressure[pad];
    manualTargetSet[pad] = true;
  }
  Serial.printf("[MANUAL] Установлено целевое давление для %s: %.1f бар\n",
                padNames[pad], manualTargetPressure[pad]);
  setDisplayDirty();
}

/* ====================  setAllManualTargetsFromCurrent  ==================== */
void setAllManualTargetsFromCurrent() {
  MutexGuard guard(xStateMutex);
  if (guard) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      manualTargetPressure[i] = pressure[i];
      manualTargetSet[i] = true;
    }
  }
  Serial.println("[MANUAL] Установлены целевые давления для всех подушек из текущих значений");
  setDisplayDirty();
}

/* ====================  setManualTargetsFromParkingPolicy  ==================== */
void setManualTargetsFromParkingPolicy() {
  const float parking = cfg_getParkingPressureBar();
  float targets[PAD_COUNT];
  if (parking > 0.00f) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      targets[i] = parking;
    }
  } else {
    const float front = cfg_getMovementPressureFront();
    const float rear = cfg_getMovementPressureRear();
    targets[PAD_FRONT_LEFT] = front;
    targets[PAD_FRONT_RIGHT] = front;
    targets[PAD_REAR_LEFT] = rear;
    targets[PAD_REAR_RIGHT] = rear;
  }

  MutexGuard guard(xStateMutex);
  if (guard) {
    for (uint8_t i = 0; i < PAD_COUNT; i++) {
      manualTargetPressure[i] = targets[i];
      manualTargetSet[i] = true;
    }
  }
  Serial.printf("[MANUAL] Цели стоянки: FL=%.2f FR=%.2f RL=%.2f RR=%.2f (parking=%.2f)\n",
                targets[PAD_FRONT_LEFT], targets[PAD_FRONT_RIGHT],
                targets[PAD_REAR_LEFT], targets[PAD_REAR_RIGHT], parking);
  setDisplayDirty();
}

/* ====================  maintainManualPressure  ==================== */
void maintainManualPressure() {
  if (cfg_errorIsActive(ERR_SENSOR)) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[MANUAL] Датчик давления неисправен, поддержание давления приостановлено");
    }
    return;
  }

  if (cfg_errorIsActive(ERR_LOW_PRESSURE)) {
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 30000) {
      lastLog = millis();
      Serial.println("[MANUAL] Поддержание давления приостановлено: LOW_PRESSURE");
    }
    return;
  }

  static uint32_t lastAdjustmentTime[PAD_COUNT] = { 0 };
  static const uint32_t ADJUSTMENT_COOLDOWN_MS = 3000;
  static uint32_t lastPressureRequest = 0;

  uint32_t currentTime = millis();

  if (currentTime - lastPressureRequest >= MANUAL_TARGET_CHECK_INTERVAL_MS) {
    requestPressureMeasurement();
    lastPressureRequest = currentTime;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  for (uint8_t i = 0; i < PAD_COUNT; i++) {
    float currentPressure, targetPressure;
    bool targetIsSet;
    float minPressure = cfg_getPressureMin();
    float maxPressure = cfg_getPressureMax();

    {
      MutexGuard guard(xStateMutex);
      if (!guard) continue;

      targetIsSet = manualTargetSet[i];
      if (!targetIsSet) {
        manualTargetPressure[i] = pressure[i];
        manualTargetSet[i] = true;
        continue;
      }
      currentPressure = pressure[i];
      targetPressure = manualTargetPressure[i];
    }

    if (currentPressure < minPressure - 0.1f) {
      if (currentTime - lastAdjustmentTime[i] >= ADJUSTMENT_COOLDOWN_MS) {
        int duration = cfg_getInflateDelay() * 1000;
        sendValveCommand(Pad(i), true, duration);
        lastAdjustmentTime[i] = currentTime;
        setDisplayDirty();
        requestPressureMeasurement();
      }
      continue;
    }

    if (currentPressure > maxPressure + 0.1f) {
      if (currentTime - lastAdjustmentTime[i] >= ADJUSTMENT_COOLDOWN_MS) {
        int duration = cfg_getReleaseDelay() * 1000;
        sendValveCommand(Pad(i), false, duration);
        lastAdjustmentTime[i] = currentTime;
        setDisplayDirty();
        requestPressureMeasurement();
      }
      continue;
    }

    if (currentPressure < 0) continue;

    float pressureDiff = targetPressure - currentPressure;

    if (abs(pressureDiff) > MANUAL_PRESSURE_TOLERANCE && (currentTime - lastAdjustmentTime[i] >= MANUAL_ADJUSTMENT_COOLDOWN_MS)) {

      if (pressureDiff > 0) {
        if (currentPressure >= maxPressure - cfg_getPressureDeadband()) continue;
        int duration = cfg_getInflateDelay() * 1000;
        sendValveCommand(Pad(i), true, duration);
      } else {
        if (currentPressure <= minPressure + cfg_getPressureDeadband()) continue;
        int duration = cfg_getReleaseDelay() * 1000;
        sendValveCommand(Pad(i), false, duration);
      }
      lastAdjustmentTime[i] = currentTime;
      setDisplayDirty();
      requestPressureMeasurement();
    }
  }
}

/* ====================  maintainMovementPressure  ==================== */
void maintainMovementPressure() {
  if (cfg_errorIsActive(ERR_LOW_PRESSURE)) {
    static uint32_t lastWaitLog = 0;
    if (millis() - lastWaitLog > 30000) {
      lastWaitLog = millis();
      Serial.println("[MOVEMENT] Ожидание нормализации давления в магистрали...");
    }
    return;
  }

  static uint32_t lastAdjustmentTimeFront = 0;
  static uint32_t lastAdjustmentTimeRear = 0;
  static const uint32_t ADJUSTMENT_COOLDOWN_MS = 5000;
  static uint32_t lastPressureCheck = 0;

  uint32_t currentTime = millis();

  if (currentTime - lastPressureCheck >= 120000) {
    requestPressureMeasurement();
    lastPressureCheck = currentTime;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  float targetPressureFront = cfg_getMovementPressureFront();
  float targetPressureRear = cfg_getMovementPressureRear();
  float minPressure = cfg_getPressureMin();

  float avgPressureFront, avgPressureRear;
  float currentPressure[PAD_COUNT];
  float currentMaster;

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    memcpy(currentPressure, pressure, sizeof(pressure));
    currentMaster = masterPressure;
    avgPressureFront = (currentPressure[PAD_FRONT_LEFT] + currentPressure[PAD_FRONT_RIGHT]) / 2.0f;
    avgPressureRear = (currentPressure[PAD_REAR_LEFT] + currentPressure[PAD_REAR_RIGHT]) / 2.0f;
  }

  float masterLow = cfg_getMasterLowBar();
  if (masterLow > 0.0f && currentMaster < masterLow) {
    Serial.printf("[MOVEMENT] Низкое давление в магистрали: %.1f бар < %.1f\n",
                  currentMaster, masterLow);

    extern bool errorScreenBlocking;
    if (!cfg_errorIsActive(ERR_LOW_PRESSURE)) {
      cfg_errorHandle(ERR_LOW_PRESSURE, "Низкое давление в магистрали");
      errorScreenBlocking = true;
      emergencyStop();
    }
    return;
  }

  float diffFront = targetPressureFront - avgPressureFront;
  float maxPressure = cfg_getPressureMax();

  bool canAdjustFront = true;
  if (diffFront > 0) {
    if (currentPressure[PAD_FRONT_LEFT] >= maxPressure - cfg_getPressureDeadband() || currentPressure[PAD_FRONT_RIGHT] >= maxPressure - cfg_getPressureDeadband()) {
      canAdjustFront = false;
      Serial.println("[MOVEMENT] Передние подушки достигли MAX давления!");
    }
  } else if (diffFront < 0) {
    if (currentPressure[PAD_FRONT_LEFT] <= minPressure + cfg_getPressureDeadband() || currentPressure[PAD_FRONT_RIGHT] <= minPressure + cfg_getPressureDeadband()) {
      canAdjustFront = false;
      Serial.println("[MOVEMENT] Передние подушки достигли MIN давления!");
    }
  }

  if (canAdjustFront && abs(diffFront) > cfg_getMovementTolerance() && (currentTime - lastAdjustmentTimeFront >= ADJUSTMENT_COOLDOWN_MS)) {

    if (diffFront > 0) {
      Serial.printf("[MOVEMENT] Накачка передних: %.1f -> %.1f\n", avgPressureFront, targetPressureFront);
      sendValveCommand(PAD_FRONT_LEFT, true, 800);
      sendValveCommand(PAD_FRONT_RIGHT, true, 800);
    } else {
      Serial.printf("[MOVEMENT] Стравливание передних: %.1f -> %.1f\n", avgPressureFront, targetPressureFront);
      sendValveCommand(PAD_FRONT_LEFT, false, 600);
      sendValveCommand(PAD_FRONT_RIGHT, false, 600);
    }
    lastAdjustmentTimeFront = currentTime;
    movementLastAdjustFront = currentTime;
    movementLastAdjustFrontDir = (diffFront > 0) ? 1 : -1;
    setDisplayDirty();
    requestPressureMeasurement();
  }

  float diffRear = targetPressureRear - avgPressureRear;

  bool canAdjustRear = true;
  if (diffRear > 0) {
    if (currentPressure[PAD_REAR_LEFT] >= maxPressure - cfg_getPressureDeadband() || currentPressure[PAD_REAR_RIGHT] >= maxPressure - cfg_getPressureDeadband()) {
      canAdjustRear = false;
      Serial.println("[MOVEMENT] Задние подушки достигли MAX давления!");
    }
  } else if (diffRear < 0) {
    if (currentPressure[PAD_REAR_LEFT] <= minPressure + cfg_getPressureDeadband() || currentPressure[PAD_REAR_RIGHT] <= minPressure + cfg_getPressureDeadband()) {
      canAdjustRear = false;
      Serial.println("[MOVEMENT] Задние подушки достигли MIN давления!");
    }
  }

  if (canAdjustRear && abs(diffRear) > cfg_getMovementTolerance() && (currentTime - lastAdjustmentTimeRear >= ADJUSTMENT_COOLDOWN_MS)) {

    if (diffRear > 0) {
      Serial.printf("[MOVEMENT] Накачка задних: %.1f -> %.1f\n", avgPressureRear, targetPressureRear);
      sendValveCommand(PAD_REAR_LEFT, true, 2000);
      sendValveCommand(PAD_REAR_RIGHT, true, 2000);
    } else {
      Serial.printf("[MOVEMENT] Стравливание задних: %.1f -> %.1f\n", avgPressureRear, targetPressureRear);
      sendValveCommand(PAD_REAR_LEFT, false, 1500);
      sendValveCommand(PAD_REAR_RIGHT, false, 1500);
    }
    lastAdjustmentTimeRear = currentTime;
    movementLastAdjustRear = currentTime;
    movementLastAdjustRearDir = (diffRear > 0) ? 1 : -1;
    setDisplayDirty();
    requestPressureMeasurement();
  }
}
