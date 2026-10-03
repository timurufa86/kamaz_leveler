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
extern int   cfg_getManualMaxTimeSec();

/* ── ErrorHandler bridge ── */
extern bool cfg_errorIsActive(uint8_t err);
static constexpr uint8_t ERR_SENSOR = 5;

/* ── TaskMonitor bridge ── */
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_CONTROL_IDX = 5; // TaskMonitor::TASK_CONTROL

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
  static SemaphoreHandle_t mutex = nullptr;
  if (mutex == nullptr) mutex = xSemaphoreCreateMutex();

  if (!takeMutexWithRetry(mutex, pdMS_TO_TICKS(100), 3, "emergencyStop/outer")) {
    Serial.println("[EMERGENCY] outer mutex failed — forcing valve pins LOW");
    for (uint8_t i = 0; i < PAD_COUNT; i++) digitalWrite(bubPins[i], LOW);
    digitalWrite(PIN_INFL, LOW);
    digitalWrite(PIN_DEFL, LOW);
    valveStopRequested = true;
    return;
  }

  if (!takeMutexWithRetry(xValveMutex, pdMS_TO_TICKS(150), 3, "emergencyStop/valve")) {
    Serial.println("[EMERGENCY] valve mutex failed — forcing valve pins LOW");
    closeAllValves();
    valveStopRequested = true;
    xSemaphoreGive(mutex);
    return;
  }

  Logger::log(Logger::WARNING, "EMERGENCY", "Остановка всех клапанов!");
  closeAllValves();
  manualControlActive = false;
  for (int i = 0; i < PAD_COUNT; i++) {
    manualTargetSet[i] = false;
  }
  valveStopRequested = true;
  valveEmergencyStopCount++;
  xSemaphoreGive(xValveMutex);
  xSemaphoreGive(mutex);
}

/* ====================  sendValveCommand  ==================== */
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs) {
  for (int attempt = 0; attempt < 3; attempt++) {
    MutexGuard commandGuard(xCommandMutex, pdMS_TO_TICKS(100));
    if (!commandGuard) {
      if (attempt == 2) {
        Serial.println("[VALVE] Command state mutex unavailable after retries");
      }
      continue;
    }

    if (xValveQueue == nullptr) {
      Serial.println("[ERROR] Valve queue not created!");
      return;
    }

    if (otaMode || otaInProgress || otaValveLock) {
      Serial.println("[VALVE] Command rejected while OTA mode is active");
      lastCmd.waitingForCompletion = false;
      lastCmd.commandActive = false;
      return;
    }

    if (pad >= PAD_COUNT) {
      Serial.printf("[ERROR] Invalid pad: %d\n", (int)pad);
      return;
    }

    if (durationMs > 0 && inflate) {
      float currentMaster;
      {
        MutexGuard guard(xStateMutex);
        if (!guard) return;
        currentMaster = masterPressure;
      }

      float minPressure = cfg_getPressureMin();
      if (currentMaster < minPressure - cfg_getPressureDeadband()) {
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 5000) {
          lastLog = millis();
          Serial.printf("[VALVE] Накачка заблокирована: МП %.1f < мин %.1f\n",
                        currentMaster, minPressure);
        }

        lastCmd.waitingForCompletion = false;
        lastCmd.commandActive = false;
        return;
      }
    }
    uint32_t safeDuration;
    if (durationMs == 0) {
      safeDuration = 0;
    } else if (durationMs == UINT32_MAX) {
      safeDuration = VALVE_MAX_COMMAND_MS;
    } else {
      safeDuration = (durationMs > VALVE_MAX_COMMAND_MS) ? VALVE_MAX_COMMAND_MS : durationMs;
    }

    if (safeDuration > 0 && !cfg_errorIsActive(ERR_SENSOR)) {

      if (lastCmd.waitingForCompletion) {
        Serial.println("[VALVE] Новая команда до анализа предыдущей, предыдущая отменена");
        lastCmd.waitingForCompletion = false;
      }

      lastCmd.startTime = millis();
      lastCmd.duration = safeDuration;
      lastCmd.pad = pad;
      lastCmd.inflate = inflate;
      lastCmd.waitingForCompletion = (durationMs != UINT32_MAX);
      lastCmd.commandActive = true;

      MutexGuard guard(xStateMutex);
      if (guard) {
        lastCmd.pressureBefore = pressure[pad];
      }
    }

    ValveCommandMsg cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.async.pad = pad;
    cmd.async.inflate = inflate;
    cmd.async.duration = (safeDuration == 0) ? 0 : pdMS_TO_TICKS(safeDuration);

    BaseType_t result = xQueueSend(xValveQueue, &cmd, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
      valveQueueDropCount++;
      Serial.println("[ERROR] Failed to send to valve queue!");
      return;
    }
    uint32_t depth = uxQueueMessagesWaiting(xValveQueue);
    if (depth > maxValveQueueDepth) maxValveQueueDepth = depth;

    if (safeDuration > 0) {
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
}

/* ====================  sendValveCommandSync  ==================== */
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs) {
  if (pad >= PAD_COUNT) {
    Serial.printf("[SYNC] Invalid pad: %d\n", pad);
    return false;
  }

  uint32_t safeDuration = (durationMs > 30000) ? 30000 : durationMs;

  QueueHandle_t ackQueue = xQueueCreate(1, sizeof(bool));
  if (ackQueue == nullptr) {
    Serial.println("[SYNC] Failed to create ack queue!");
    return false;
  }

  ValveCommandMsg cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.sync.pad = pad;
  cmd.sync.inflate = inflate;
  cmd.sync.durationMs = safeDuration;
  cmd.sync.ackQueue = ackQueue;

  if (xQueueSend(xValveQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[SYNC] Failed to send to valve queue!");
    vQueueDelete(ackQueue);
    return false;
  }

  bool success = false;
  uint32_t timeoutMs = safeDuration + waitAfterMs + 2000;
  uint32_t waited = 0;
  BaseType_t result = pdFALSE;
  while (waited < timeoutMs) {
    cfg_taskMonitorUpdate(TASK_CONTROL_IDX);
    uint32_t chunk = timeoutMs - waited;
    if (chunk > 500) chunk = 500;
    result = xQueueReceive(ackQueue, &success, pdMS_TO_TICKS(chunk));
    if (result == pdTRUE) break;
    waited += chunk;
  }

  vQueueDelete(ackQueue);

  if (result != pdTRUE) {
    Serial.println("[SYNC] Command timeout!");
    return false;
  }

  if (!success) {
    Serial.println("[SYNC] Command failed!");
    return false;
  }

  if (waitAfterMs > 0) {
    vTaskDelay(pdMS_TO_TICKS(waitAfterMs));
  }

  return true;
}

/* ====================  startManualOperation  ==================== */
void startManualOperation(Pad padIdx, bool inflate) {
  if (padIdx >= PAD_COUNT) return;
  if (manualControlActive) return;
  if (currentSystemMode == SystemMode::MOVEMENT) return;

  float currentMaster;
  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentMaster = masterPressure;
  }

  float minPressure = cfg_getPressureMin();
  if (inflate && currentMaster < minPressure) {
    Serial.printf("[MANUAL] Низкое давление в магистрали (%.1f < %.1f), накачка запрещена\n",
                  currentMaster, minPressure);
    return;
  }

  float currentPressure;
  float maxPressure = cfg_getPressureMax();

  {
    MutexGuard guard(xStateMutex);
    if (!guard) return;
    currentPressure = pressure[padIdx];
  }

  if (inflate && currentPressure >= maxPressure - cfg_getPressureDeadband()) {
    Serial.printf("[MANUAL] %s уже на максимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return;
  }
  if (!inflate && currentPressure <= minPressure + cfg_getPressureDeadband()) {
    Serial.printf("[MANUAL] %s уже на минимуме (%.1f бар)\n", padNames[padIdx], currentPressure);
    return;
  }

  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[padIdx] = currentPressure;
      manualTargetSet[padIdx] = true;
    }
  }

  manualControlActive = true;
  manualPadIndex = padIdx;
  manualInflate = inflate;
  manualStartTime = millis();

  sendValveCommand(padIdx, inflate, UINT32_MAX);

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
}

/* ====================  stopManualOperation  ==================== */
void stopManualOperation() {
  if (!manualControlActive) return;

  {
    MutexGuard guard(xStateMutex);
    if (guard) {
      manualTargetPressure[manualPadIndex] = pressure[manualPadIndex];
      manualTargetSet[manualPadIndex] = true;
    }
  }

  sendValveCommand(manualPadIndex, manualInflate, 0);
  manualControlActive = false;

  Event event;
  event.type = EventType::MANUAL_OPERATION_END;
  event.timestamp = millis();
  event.data.manualOp.pad = static_cast<uint8_t>(manualPadIndex);
  event.data.manualOp.inflate = manualInflate;
  EventBus::publish(event);

  Logger::log(Logger::INFO, "MANUAL", "Операция завершена");
  setDisplayDirty();
}
