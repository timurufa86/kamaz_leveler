#include "valve_ctrl.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_pins.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "logger.h"
#include "pressure_read.h"
#include "task_pool.h"

/* ── ConfigManager bridge functions (defined in .ino) ── */
extern float cfg_getPressureMin();
extern float cfg_getPressureMax();
extern float cfg_getPressureDeadband();
extern float cfg_getMasterLowBar();
extern int   cfg_getPressureStabilizeMs();
extern int   cfg_getManualMaxTimeSec();

/* ── ErrorHandler bridge ── */
extern bool cfg_errorIsActive(uint8_t err);
extern void cfg_errorHandle(uint8_t err, const char *msg);
static constexpr uint8_t ERR_SENSOR = ERR_SENSOR_CODE; // single source: app_types.h
static constexpr uint8_t ERR_VALVE  = ERR_VALVE_CODE;  // single source: app_types.h

/* ── Emergency latch (10.15.0) ──────────────────────────────
 * emergencyStop() only closed the pins and raised valveStopRequested, which
 * valveTask cleared ~10 ms later — AUTO resumed in under a second. The latch
 * blocks every manifold operation until the operator resets it explicitly. */
static volatile bool s_emergencyLatched = false;

/** Direct pin write, independent of any mutex — used in emergencies. */
static void forceAllValvePinsLow() {
  for (uint8_t i = 0; i < PAD_COUNT; i++) digitalWrite(bubPins[i], LOW);
  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
}

bool emergencyIsLatched() { return s_emergencyLatched; }

void resetEmergencyLatch() {
  s_emergencyLatched = false;
  valveStopRequested = true; // valve task closes everything once more
  Logger::log(Logger::WARNING, "EMERGENCY", "Аварийная защёлка снята (сброс)");
  SERIAL_DBGLN("[EMERGENCY] аварийная защёлка снята");
}

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
/* TaskMonitor::TaskIndex: IMU=3 PRESSURE=4 CONTROL=5 CALIB=6 VALVE=9 */
static constexpr uint8_t TASK_IMU_IDX = 3;
static constexpr uint8_t TASK_PRESSURE_IDX = 4;
static constexpr uint8_t TASK_CONTROL_IDX = 5;
static constexpr uint8_t TASK_CALIB_IDX = 6;
static constexpr uint8_t TASK_VALVE_IDX = 9;

/** Long valve holds block the caller task (often IMU) — keep soft WDT happy. */
static void feedValveBlockingHeartbeats() {
  cfg_taskMonitorUpdate(TASK_CONTROL_IDX);
  cfg_taskMonitorUpdate(TASK_IMU_IDX);
  cfg_taskMonitorUpdate(TASK_PRESSURE_IDX);
  cfg_taskMonitorUpdate(TASK_CALIB_IDX);
  cfg_taskMonitorUpdate(TASK_VALVE_IDX);
}

/* ── .ino helpers ── */
void setDisplayDirty();

/* ====================  setValve  ==================== */
void setValve(Pad pad, bool state) {
  if (pad >= PAD_COUNT) return;
  digitalWrite(bubPins[pad], state);
}

/* ====================  closeAllValves  ==================== */
void closeAllValves() {
  for (uint8_t i = 0; i < PAD_COUNT; i++) digitalWrite(bubPins[i], LOW);
  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
}

/* ====================  emergencyStop  ==================== */
void emergencyStop() {
  // Latch first and force the pins low without waiting for any mutex:
  // nothing may reopen a valve until resetEmergencyLatch().
  s_emergencyLatched = true;
  valveStopRequested = true;
  forceAllValvePinsLow();

  static SemaphoreHandle_t mutex = nullptr;
  if (mutex == nullptr) mutex = xSemaphoreCreateMutex();

  if (!takeMutexWithRetry(mutex, pdMS_TO_TICKS(100), 3, "emergencyStop/outer")) {
    SERIAL_DBGLN("[EMERGENCY] outer mutex failed — forcing valve pins LOW");
    for (uint8_t i = 0; i < PAD_COUNT; i++) digitalWrite(bubPins[i], LOW);
    digitalWrite(PIN_INFL, LOW);
    digitalWrite(PIN_DEFL, LOW);
    valveStopRequested = true;
    return;
  }

  if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(150), 3, "emergencyStop/valve")) {
    SERIAL_DBGLN("[EMERGENCY] valve mutex failed — forcing valve pins LOW");
    closeAllValves();
    valveStopRequested = true;
    xSemaphoreGive(mutex);
    return;
  }

  Logger::log(Logger::WARNING, "EMERGENCY", "Остановка всех клапанов!");
  closeAllValves();
  manualControlActive = false;
  manifoldBusy = false;
  {
    /* 10.15.6: manualTargetSet принадлежит xStateMutex во всех остальных местах;
       сброс под xValveMutex был гонкой write/write с maintainManualPressure(). */
    MutexGuard st(xStateMutex, pdMS_TO_TICKS(50));
    if (st) {
      for (int i = 0; i < PAD_COUNT; i++) manualTargetSet[i] = false;
    }
  }
  valveStopRequested = true;
  valveEmergencyStopCount++;
  xSemaphoreGive(xValveMutex);
  xSemaphoreGive(mutex);
}

/* ====================  sendValveCommand  ==================== */
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs) {
  if (pad >= PAD_COUNT) {
    SERIAL_DBG("[ERROR] Invalid pad: %d\n", (int)pad);
    return;
  }
  if (otaMode || otaInProgress || otaValveLock) {
    SERIAL_DBGLN("[VALVE] Command rejected while OTA mode is active");
    return;
  }
  /* 10.15.6: аварийная защёлка блокирует любые команды клапанов. */
  if (emergencyIsLatched()) {
    SERIAL_DBGLN("[VALVE] Command rejected (emergency latch)");
    return;
  }

  // Timed pulse → canon C (exclusive). Hold/close kept for legacy queue path.
  if (durationMs > 0 && durationMs != UINT32_MAX) {
    if (pulsePad(pad, inflate, durationMs, nullptr, nullptr)) {
      Event event;
      event.type = EventType::VALVE_COMMAND;
      event.timestamp = millis();
      event.data.valve.pad = static_cast<uint8_t>(pad);
      event.data.valve.inflate = inflate;
      event.data.valve.duration = durationMs;
      EventBus::publish(event);
    }
    return;
  }

  for (int attempt = 0; attempt < 3; attempt++) {
    MutexGuard commandGuard(xCommandMutex, pdMS_TO_TICKS(100));
    if (!commandGuard) {
      if (attempt == 2) {
        SERIAL_DBGLN("[VALVE] Command state mutex unavailable after retries");
      }
      continue;
    }

    if (xValveQueue == nullptr) {
      SERIAL_DBGLN("[ERROR] Valve queue not created!");
      return;
    }

    uint32_t safeDuration = (durationMs == 0) ? 0 : VALVE_MAX_COMMAND_MS;

    ValveCommandMsg cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.async.pad = pad;
    cmd.async.inflate = inflate;
    cmd.async.duration = (safeDuration == 0) ? 0 : pdMS_TO_TICKS(safeDuration);

    BaseType_t result = xQueueSend(xValveQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
      valveQueueDropCount++;
      SERIAL_DBGLN("[ERROR] Failed to send to valve queue!");
      return;
    }
    uint32_t depth = uxQueueMessagesWaiting(xValveQueue);
    if (depth > maxValveQueueDepth) maxValveQueueDepth = depth;
    return;
  }
}

/* ====================  Manifold exclusive helpers  ==================== */

static uint32_t stabilizeMsClamped() {
  return (uint32_t)constrain(cfg_getPressureStabilizeMs(), 100, 2000);
}

static void storePadPressure(Pad pad, float p) {
  MutexGuard guard(xStateMutex);
  if (!guard) return;
  pressure[pad] = p;
  pressureStampMs[pad] = millis();
  pressureValid[pad] = true;
}

static void storeMasterPressure(float p) {
  MutexGuard guard(xStateMutex);
  if (!guard) return;
  masterPressure = p;
  masterStampMs = millis();
  masterValid = true;
}

bool manifoldIsBusy() {
  return manifoldBusy || manualControlActive;
}

static bool acquireManifold(const char *tag) {
  if (s_emergencyLatched) {
    SERIAL_DBG("[MANIFOLD] %s rejected (emergency latch)\n", tag);
    return false;
  }
  if (otaValveLock || otaMode || otaInProgress) {
    SERIAL_DBG("[MANIFOLD] %s rejected (OTA)\n", tag);
    return false;
  }
  if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(500), 3, tag)) {
    SERIAL_DBG("[MANIFOLD] %s mutex fail\n", tag);
    return false;
  }
  if (manifoldBusy || manualControlActive) {
    xSemaphoreGive(xValveMutex);
    SERIAL_DBG("[MANIFOLD] %s rejected (busy)\n", tag);
    return false;
  }
  manifoldBusy = true;
  closeAllValves();
  return true;
}

static void releaseManifold() {
  closeAllValves();
  manifoldBusy = false;
  xSemaphoreGive(xValveMutex);
}

bool measurePad(Pad pad, float *outOpt) {
  if (pad >= PAD_COUNT) return false;
  if (!acquireManifold("measurePad")) return false;

  setValve(pad, HIGH);
  vTaskDelay(pdMS_TO_TICKS(stabilizeMsClamped()));
  float p = readPressure();
  setValve(pad, LOW);

  releaseManifold();

  if (p < 0.0f) {
    SERIAL_DBG("[MANIFOLD] measurePad %s fail\n", padNames[pad]);
    return false;
  }
  storePadPressure(pad, p);
  if (outOpt) *outOpt = p;
  return true;
}

bool measureMaster(float *outOpt) {
  if (!acquireManifold("measureMaster")) return false;

  digitalWrite(PIN_INFL, HIGH);
  vTaskDelay(pdMS_TO_TICKS(stabilizeMsClamped()));
  float p = readPressure();
  digitalWrite(PIN_INFL, LOW);

  releaseManifold();

  if (p < 0.0f) {
    SERIAL_DBGLN("[MANIFOLD] measureMaster fail");
    return false;
  }
  storeMasterPressure(p);
  lastMasterPressureCheckTime = millis();
  if (outOpt) *outOpt = p;
  return true;
}

bool pulsePad(Pad pad, bool inflate, uint32_t durationMs,
              float *pBeforeOpt, float *pAfterOpt) {
  if (pad >= PAD_COUNT) return false;
  if (durationMs == 0) return false;

  uint32_t safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
  const uint32_t tDoP = stabilizeMsClamped();
  const float minP = cfg_getPressureMin();
  const float maxP = cfg_getPressureMax();
  const float dead = cfg_getPressureDeadband();
  const float masterLow = cfg_getMasterLowBar();

  if (!acquireManifold("pulsePad")) return false;

  // C: open pad → t до P → P_before
  setValve(pad, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pBefore = readPressure();
  if (pBefore < 0.0f) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] pulsePad %s: P_before fail\n", padNames[pad]);
    return false;
  }
  storePadPressure(pad, pBefore);
  if (pBeforeOpt) *pBeforeOpt = pBefore;

  if (inflate && pBefore >= maxP - dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] %s уже на максимуме (%.1f бар)\n", padNames[pad], pBefore);
    return false;
  }
  if (!inflate && pBefore <= minP + dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] %s уже на минимуме (%.1f бар)\n", padNames[pad], pBefore);
    return false;
  }

  if (inflate && masterLow > 0.0f) {
    float currentMaster;
    {
      MutexGuard guard(xStateMutex);
      currentMaster = guard ? masterPressure : 0.0f;
    }
    if (currentMaster < masterLow) {
      releaseManifold();
      SERIAL_DBG("[MANIFOLD] Накачка запрещена: Маг %.1f < %.1f\n",
                    currentMaster, masterLow);
      return false;
    }
  }

  lastCmd.startTime = millis();
  lastCmd.duration = safeDuration;
  lastCmd.pad = pad;
  lastCmd.inflate = inflate;
  lastCmd.pressureBefore = pBefore;
  lastCmd.waitingForCompletion = true;
  lastCmd.commandActive = true;

  digitalWrite(inflate ? PIN_DEFL : PIN_INFL, LOW);
  digitalWrite(inflate ? PIN_INFL : PIN_DEFL, HIGH);
  valveCycleCount[pad]++;

  // Hold action; keep exclusivity (manifoldBusy stays set).
  // 10.15.0: давление перечитывается каждые 100 мс внутри импульса — предел
  // больше не проверяется один раз перед открытием клапана.
  uint32_t held = 0;
  bool overpressure = false;
  while (held < safeDuration) {
    feedValveBlockingHeartbeats();
    if (valveStopRequested || otaValveLock) break;
    const float pNow = readPressure();
    if (pNow >= 0.0f) {
      if (inflate && pNow >= maxP) { overpressure = true; break; }
      if (!inflate && pNow <= minP) break;
    }
    uint32_t chunk = safeDuration - held;
    if (chunk > 100) chunk = 100;
    vTaskDelay(pdMS_TO_TICKS(chunk));
    held += chunk;
  }

  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
  lastCmd.commandActive = false;

  if (overpressure) {
    setValve(pad, LOW);
    releaseManifold();
    lastCmd.waitingForCompletion = false;
    cfg_errorHandle(ERR_VALVE, "Перекачка: P подушки выше максимума");
    SERIAL_DBG("[MANIFOLD] pulsePad %s: СТОП по перекачке\n", padNames[pad]);
    return false;
  }

  // Close line only → t до P → P_after (pad still open)
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pAfter = readPressure();
  setValve(pad, LOW);

  releaseManifold();

  if (inflate && pAfter >= 0.0f && pAfter >= maxP) {
    lastCmd.waitingForCompletion = false;
    cfg_errorHandle(ERR_VALVE, "Перекачка: P после импульса выше максимума");
    SERIAL_DBG("[MANIFOLD] pulsePad %s: P_after=%.2f > max=%.2f\n",
                  padNames[pad], pAfter, maxP);
    return false;
  }

  if (pAfter < 0.0f) {
    SERIAL_DBG("[MANIFOLD] pulsePad %s: P_after fail\n", padNames[pad]);
    lastCmd.waitingForCompletion = false;
    return false;
  }
  storePadPressure(pad, pAfter);
  if (pAfterOpt) *pAfterOpt = pAfter;

  SERIAL_DBG("[MANIFOLD] pulse %s %s %lu ms: %.2f → %.2f\n",
                padNames[pad], inflate ? "INFL" : "DEFL",
                (unsigned long)safeDuration, pBefore, pAfter);
  return true;
}

bool pulseAxlePair(bool front, bool inflate, uint32_t durationMs) {
  if (durationMs == 0) return false;

  const Pad padL = front ? PAD_FRONT_LEFT : PAD_REAR_LEFT;
  const Pad padR = front ? PAD_FRONT_RIGHT : PAD_REAR_RIGHT;
  uint32_t safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
  const uint32_t tDoP = stabilizeMsClamped();
  const float minP = cfg_getPressureMin();
  const float maxP = cfg_getPressureMax();
  const float dead = cfg_getPressureDeadband();
  const float masterLow = cfg_getMasterLowBar();

  if (!acquireManifold("pulseAxle")) return false;

  // Isolated P_before for each pad (same exclusive session).
  setValve(padL, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pL = readPressure();
  setValve(padL, LOW);
  if (pL < 0.0f) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] pulseAxle %s: P_before L fail\n", front ? "FRONT" : "REAR");
    return false;
  }
  storePadPressure(padL, pL);

  setValve(padR, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pR = readPressure();
  setValve(padR, LOW);
  if (pR < 0.0f) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] pulseAxle %s: P_before R fail\n", front ? "FRONT" : "REAR");
    return false;
  }
  storePadPressure(padR, pR);

  const float avgBefore = (pL + pR) * 0.5f;
  if (inflate && avgBefore >= maxP - dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] ось %s уже на максимуме (avg %.1f)\n",
                  front ? "FRONT" : "REAR", avgBefore);
    return false;
  }
  if (!inflate && avgBefore <= minP + dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] ось %s уже на минимуме (avg %.1f)\n",
                  front ? "FRONT" : "REAR", avgBefore);
    return false;
  }
  if (inflate && masterLow > 0.0f) {
    float currentMaster;
    {
      MutexGuard guard(xStateMutex);
      currentMaster = guard ? masterPressure : 0.0f;
    }
    if (currentMaster < masterLow) {
      releaseManifold();
      SERIAL_DBG("[MANIFOLD] Накачка оси запрещена: Маг %.1f < %.1f\n",
                    currentMaster, masterLow);
      return false;
    }
  }

  lastCmd.startTime = millis();
  lastCmd.duration = safeDuration;
  lastCmd.pad = padL;
  lastCmd.inflate = inflate;
  lastCmd.pressureBefore = avgBefore;
  lastCmd.waitingForCompletion = true;
  lastCmd.commandActive = true;

  // Both selects + action together.
  setValve(padL, HIGH);
  setValve(padR, HIGH);
  digitalWrite(inflate ? PIN_DEFL : PIN_INFL, LOW);
  digitalWrite(inflate ? PIN_INFL : PIN_DEFL, HIGH);
  valveCycleCount[padL]++;
  valveCycleCount[padR]++;

  // 10.15.0: контроль давления внутри импульса (как в pulsePad).
  uint32_t held = 0;
  bool overpressure = false;
  while (held < safeDuration) {
    feedValveBlockingHeartbeats();
    if (valveStopRequested || otaValveLock) break;
    const float pNow = readPressure();
    if (pNow >= 0.0f) {
      if (inflate && pNow >= maxP) { overpressure = true; break; }
      if (!inflate && pNow <= minP) break;
    }
    uint32_t chunk = safeDuration - held;
    if (chunk > 100) chunk = 100;
    vTaskDelay(pdMS_TO_TICKS(chunk));
    held += chunk;
  }

  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
  lastCmd.commandActive = false;

  if (overpressure) {
    setValve(padL, LOW);
    setValve(padR, LOW);
    releaseManifold();
    lastCmd.waitingForCompletion = false;
    cfg_errorHandle(ERR_VALVE, "Перекачка оси: P выше максимума");
    SERIAL_DBG("[MANIFOLD] pulseAxle %s: СТОП по перекачке\n", front ? "FRONT" : "REAR");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(tDoP));
  setValve(padL, LOW);
  setValve(padR, LOW);

  // Fresh A samples after pulse.
  setValve(padL, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pL2 = readPressure();
  setValve(padL, LOW);
  setValve(padR, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pR2 = readPressure();
  setValve(padR, LOW);

  releaseManifold();

  if (pL2 >= 0.0f) storePadPressure(padL, pL2);
  if (pR2 >= 0.0f) storePadPressure(padR, pR2);

  SERIAL_DBG("[MANIFOLD] axle %s %s %lu ms: avg %.2f → %.2f\n",
                front ? "FRONT" : "REAR", inflate ? "INFL" : "DEFL",
                (unsigned long)safeDuration, avgBefore,
                (pL2 >= 0 && pR2 >= 0) ? (pL2 + pR2) * 0.5f : -1.0f);
  return (pL2 >= 0.0f && pR2 >= 0.0f);
}

bool pulseSidePair(bool left, bool inflate, uint32_t durationMs) {
  if (durationMs == 0) return false;

  const Pad padF = left ? PAD_FRONT_LEFT : PAD_FRONT_RIGHT;
  const Pad padR = left ? PAD_REAR_LEFT : PAD_REAR_RIGHT;
  uint32_t safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
  const uint32_t tDoP = stabilizeMsClamped();
  const float minP = cfg_getPressureMin();
  const float maxP = cfg_getPressureMax();
  const float dead = cfg_getPressureDeadband();
  const float masterLow = cfg_getMasterLowBar();

  if (!acquireManifold("pulseSide")) return false;

  setValve(padF, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pF = readPressure();
  setValve(padF, LOW);
  if (pF < 0.0f) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] pulseSide %s: P_before F fail\n", left ? "LEFT" : "RIGHT");
    return false;
  }
  storePadPressure(padF, pF);

  setValve(padR, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pRr = readPressure();
  setValve(padR, LOW);
  if (pRr < 0.0f) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] pulseSide %s: P_before R fail\n", left ? "LEFT" : "RIGHT");
    return false;
  }
  storePadPressure(padR, pRr);

  const float avgBefore = (pF + pRr) * 0.5f;
  if (inflate && avgBefore >= maxP - dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] сторона %s уже на максимуме (avg %.1f)\n",
                  left ? "LEFT" : "RIGHT", avgBefore);
    return false;
  }
  if (!inflate && avgBefore <= minP + dead) {
    releaseManifold();
    SERIAL_DBG("[MANIFOLD] сторона %s уже на минимуме (avg %.1f)\n",
                  left ? "LEFT" : "RIGHT", avgBefore);
    return false;
  }
  if (inflate && masterLow > 0.0f) {
    float currentMaster;
    {
      MutexGuard guard(xStateMutex);
      currentMaster = guard ? masterPressure : 0.0f;
    }
    if (currentMaster < masterLow) {
      releaseManifold();
      SERIAL_DBG("[MANIFOLD] Накачка стороны запрещена: Маг %.1f < %.1f\n",
                    currentMaster, masterLow);
      return false;
    }
  }

  lastCmd.startTime = millis();
  lastCmd.duration = safeDuration;
  lastCmd.pad = padF;
  lastCmd.inflate = inflate;
  lastCmd.pressureBefore = avgBefore;
  lastCmd.waitingForCompletion = true;
  lastCmd.commandActive = true;

  setValve(padF, HIGH);
  setValve(padR, HIGH);
  digitalWrite(inflate ? PIN_DEFL : PIN_INFL, LOW);
  digitalWrite(inflate ? PIN_INFL : PIN_DEFL, HIGH);
  valveCycleCount[padF]++;
  valveCycleCount[padR]++;

  // 10.15.0: контроль давления внутри импульса (как в pulsePad).
  uint32_t held = 0;
  bool overpressure = false;
  while (held < safeDuration) {
    feedValveBlockingHeartbeats();
    if (valveStopRequested || otaValveLock) break;
    const float pNow = readPressure();
    if (pNow >= 0.0f) {
      if (inflate && pNow >= maxP) { overpressure = true; break; }
      if (!inflate && pNow <= minP) break;
    }
    uint32_t chunk = safeDuration - held;
    if (chunk > 100) chunk = 100;
    vTaskDelay(pdMS_TO_TICKS(chunk));
    held += chunk;
  }

  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
  lastCmd.commandActive = false;

  if (overpressure) {
    setValve(padF, LOW);
    setValve(padR, LOW);
    releaseManifold();
    lastCmd.waitingForCompletion = false;
    cfg_errorHandle(ERR_VALVE, "Перекачка стороны: P выше максимума");
    SERIAL_DBG("[MANIFOLD] pulseSide %s: СТОП по перекачке\n", left ? "LEFT" : "RIGHT");
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(tDoP));
  setValve(padF, LOW);
  setValve(padR, LOW);

  setValve(padF, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pF2 = readPressure();
  setValve(padF, LOW);
  setValve(padR, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pR2 = readPressure();
  setValve(padR, LOW);

  releaseManifold();

  if (pF2 >= 0.0f) storePadPressure(padF, pF2);
  if (pR2 >= 0.0f) storePadPressure(padR, pR2);

  SERIAL_DBG("[MANIFOLD] side %s %s %lu ms: avg %.2f → %.2f\n",
                left ? "LEFT" : "RIGHT", inflate ? "INFL" : "DEFL",
                (unsigned long)safeDuration, avgBefore,
                (pF2 >= 0 && pR2 >= 0) ? (pF2 + pR2) * 0.5f : -1.0f);
  return (pF2 >= 0.0f && pR2 >= 0.0f);
}

/* ====================  sendValveCommandSync  ==================== */
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t /*waitAfterMs*/) {
  return pulsePad(pad, inflate, durationMs, nullptr, nullptr);
}

/* ====================  startManualOperation  ==================== */
const char *manualStartResultStr(ManualStartResult r) {
  switch (r) {
    case ManualStartResult::Ok: return "ok";
    case ManualStartResult::BadPad: return "bad_pad";
    case ManualStartResult::AlreadyActive: return "busy";
    case ManualStartResult::Movement: return "movement";
    case ManualStartResult::Mutex: return "mutex";
    case ManualStartResult::ReadFail: return "read_fail";
    case ManualStartResult::MaxP: return "max_p";
    case ManualStartResult::MinP: return "min_p";
    case ManualStartResult::MasterLow: return "master_low";
    case ManualStartResult::Emergency: return "emergency";
  }
  return "?";
}

ManualStartResult startManualOperation(Pad padIdx, bool inflate) {
  if (padIdx >= PAD_COUNT) return ManualStartResult::BadPad;
  if (manualControlActive) return ManualStartResult::AlreadyActive;
  if (s_emergencyLatched) return ManualStartResult::Emergency;
  if (currentSystemMode == SystemMode::MOVEMENT) return ManualStartResult::Movement;

  const float minP = cfg_getPressureMin();
  const float maxP = cfg_getPressureMax();
  const float dead = cfg_getPressureDeadband();
  const uint32_t tDoP = stabilizeMsClamped();

  // Накачка только при готовой магистрали (min P Маг.>0, нет LOW_PRESSURE, МП≥порога)
  if (inflate && !masterSupplyReadyForWork()) {
    SERIAL_DBGLN("[MANUAL] МП не готова — накачка запрещена");
    return ManualStartResult::MasterLow;
  }

  if (!acquireManifold("manualStart")) return ManualStartResult::Mutex;

  setValve(padIdx, HIGH);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float currentPressure = readPressure();
  if (currentPressure < 0.0f) {
    releaseManifold();
    SERIAL_DBGLN("[MANUAL] Не удалось прочитать давление подушки");
    return ManualStartResult::ReadFail;
  }
  storePadPressure(padIdx, currentPressure);

  if (inflate && currentPressure >= maxP - dead) {
    releaseManifold();
    SERIAL_DBG("[MANUAL] %s уже на максимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return ManualStartResult::MaxP;
  }
  if (!inflate && currentPressure <= minP + dead) {
    releaseManifold();
    SERIAL_DBG("[MANUAL] %s уже на минимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return ManualStartResult::MinP;
  }

  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[padIdx] = currentPressure;
      manualTargetSet[padIdx] = true;
    }
  }

  digitalWrite(inflate ? PIN_DEFL : PIN_INFL, LOW);
  digitalWrite(inflate ? PIN_INFL : PIN_DEFL, HIGH);
  valveCycleCount[padIdx]++;

  // Hold: keep manifoldBusy, release mutex so emergencyStop can take it.
  manualControlActive = true;
  manualPadIndex = padIdx;
  manualInflate = inflate;
  manualStartTime = millis();
  xSemaphoreGive(xValveMutex);

  Logger::logf(Logger::INFO, "MANUAL", "%s %s (давление %.1f)",
               inflate ? "НАКАЧКА" : "СТРАВЛИВАНИЕ",
               padNames[padIdx], currentPressure);

  Event event;
  event.type = EventType::MANUAL_OPERATION_START;
  event.timestamp = millis();
  event.data.manualOp.pad = static_cast<uint8_t>(padIdx);
  event.data.manualOp.inflate = inflate;
  EventBus::publish(event);

  setDisplayDirty();
  return ManualStartResult::Ok;
}

/* ====================  stopManualOperation  ==================== */
void stopManualOperation() {
  if (!manualControlActive) return;

  const Pad padIdx = manualPadIndex;
  const bool inflate = manualInflate;
  const uint32_t tDoP = stabilizeMsClamped();

  if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(500), 3, "manualStop")) {
    closeAllValves();
    manualControlActive = false;
    manifoldBusy = false;
    return;
  }

  // Close line only → t до P → measure pad → close select
  digitalWrite(PIN_INFL, LOW);
  digitalWrite(PIN_DEFL, LOW);
  vTaskDelay(pdMS_TO_TICKS(tDoP));
  float pAfter = readPressure();
  setValve(padIdx, LOW);

  if (pAfter >= 0.0f) {
    storePadPressure(padIdx, pAfter);
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[padIdx] = pAfter;
      manualTargetSet[padIdx] = true;
    }
  }

  manualControlActive = false;
  manifoldBusy = false;
  xSemaphoreGive(xValveMutex);

  Event event;
  event.type = EventType::MANUAL_OPERATION_END;
  event.timestamp = millis();
  event.data.manualOp.pad = static_cast<uint8_t>(padIdx);
  event.data.manualOp.inflate = inflate;
  EventBus::publish(event);

  Logger::log(Logger::INFO, "MANUAL", "Операция завершена");
  setDisplayDirty();
}

/* ====================  otaQuiesceForNetwork  ==================== */
/* 10.15.0: перед любой длительной сетевой/TLS-сессией (OTA-проверка, список,
   загрузка) клапаны закрываются, а новые операции запрещаются. Раньше задачи
   клапанов замораживались vTaskSuspend с открытыми выходами на всё время TLS. */
void otaQuiesceForNetwork() {
  otaValveLock = true;
  valveStopRequested = true;
  if (manualControlActive) {
    stopManualOperation(); // корректно завершить ручное удержание перед паузой
  }
  const uint32_t started = millis();
  while (manifoldIsBusy() && (millis() - started) < 1500UL) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if (manifoldIsBusy()) {
    SERIAL_DBGLN("[OTA] манифолд не освободился за 1500 мс — принудительное закрытие");
  }
  forceAllValvePinsLow();
  SERIAL_DBG("[OTA] клапаны закрыты перед сетевой сессией (%lu мс)\n",
                (unsigned long)(millis() - started));
}

/* ====================  enforceManualPressureSafety  ==================== */
/* Ручное удержание открывает линию до явного останова; раньше давление в нём
   не контролировалось вообще (только таймаут до 30 с). */
void enforceManualPressureSafety() {
  if (!manualControlActive) return;
  const Pad pad = manualPadIndex;
  if (pad >= PAD_COUNT) return;

  const float p = readPressure();
  if (p < 0.0f) return; // датчик недостоверен — не трогаем операцию

  const float maxP = cfg_getPressureMax();
  const float minP = cfg_getPressureMin();
  if (manualInflate && p >= maxP) {
    SERIAL_DBG("[MANUAL] СТОП: %.2f >= max %.2f\n", p, maxP);
    stopManualOperation();
    cfg_errorHandle(ERR_VALVE, "Перекачка в ручном режиме");
  } else if (!manualInflate && p <= minP) {
    SERIAL_DBG("[MANUAL] СТОП: %.2f <= min %.2f\n", p, minP);
    stopManualOperation();
  }
}
