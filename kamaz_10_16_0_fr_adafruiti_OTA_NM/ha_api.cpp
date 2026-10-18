#include "ha_api.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "app_globals.h"
#include "app_types.h"
#include "app_version.h"
#include "auto_level.h"
#include "arduino_ota.h"
#include "config_manager.h"
#include "error_handler.h"
#include "logger.h"
#include "mode_ctrl.h"
#include "mutex_guard.h"
#include "service_ui.h"
#include "task_monitor.h"
#include "task_pool.h"
#include "valve_ctrl.h"

/* Общие глобалы, объявленные в .ino / wifi_setup.cpp. */
extern uint32_t uptimeHours;
extern char sta_ssid[33];

/* ── Состояние модуля ── */
static WebServer haServer(80);
static volatile bool haServerRunning = false;
static volatile bool haSuspended = false;
static uint32_t haServed = 0;

static bool haConfigDirty = false;
static uint32_t haConfigDirtyMs = 0;
static uint32_t haManualStopAt = 0;
static uint32_t haRestartAt = 0;

struct PendingPulse {
  bool valid = false;
  Pad pad = PAD_FRONT_LEFT;
  bool inflate = true;
  uint32_t ms = 0;
};
static PendingPulse s_pulse;

/* ── Вспомогательные ── */
static const char *modeName(SystemMode m) {
  switch (m) {
    case SystemMode::AUTO: return "AUTO";
    case SystemMode::MANUAL: return "MANUAL";
    case SystemMode::MOVEMENT: return "MOVEMENT";
  }
  return "?";
}

static const char *stateName(SystemState s) {
  switch (s) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::CALIBRATING: return "CALIBRATING";
    case SystemState::RUNNING: return "RUNNING";
    case SystemState::ERROR: return "ERROR";
    case SystemState::OTA_MODE: return "OTA_MODE";
  }
  return "?";
}

static const char *errName(ErrorHandler::Error e) {
  switch (e) {
    case ErrorHandler::Error::LOW_PRESSURE: return "LOW_PRESSURE";
    case ErrorHandler::Error::MPU: return "MPU";
    case ErrorHandler::Error::SENSOR: return "SENSOR";
    case ErrorHandler::Error::VALVE: return "VALVE";
    case ErrorHandler::Error::WATCHDOG: return "WATCHDOG";
    case ErrorHandler::Error::OTA: return "OTA";
    default: return "NONE";
  }
}

static const char *padKey(Pad p) {
  switch (p) {
    case PAD_FRONT_LEFT: return "fl";
    case PAD_FRONT_RIGHT: return "fr";
    case PAD_REAR_LEFT: return "rl";
    case PAD_REAR_RIGHT: return "rr";
    default: return "?";
  }
}

static bool parsePad(const String &s, Pad &out) {
  if (s == "fl" || s == "FL") { out = PAD_FRONT_LEFT; return true; }
  if (s == "fr" || s == "FR") { out = PAD_FRONT_RIGHT; return true; }
  if (s == "rl" || s == "RL") { out = PAD_REAR_LEFT; return true; }
  if (s == "rr" || s == "RR") { out = PAD_REAR_RIGHT; return true; }
  return false;
}

static bool authorized() {
  const char *token = cfg_getHaToken();
  if (token == nullptr || token[0] == '\0') return true;
  String k = haServer.header("X-Api-Key");
  if (k.length() == 0) k = haServer.arg("key");
  return k == token;
}

static void sendJson(int code, const String &body) {
  haServer.sendHeader("Connection", "close");
  haServer.sendHeader("Cache-Control", "no-store");
  haServer.send(code, "application/json; charset=utf-8", body);
}

static void sendError(int code, const char *msg, const char *detail = nullptr) {
  DynamicJsonDocument doc(384);
  doc["error"] = msg;
  if (detail) doc["detail"] = detail;
  String out;
  serializeJson(doc, out);
  sendJson(code, out);
}

/** Причина, по которой пневматика сейчас недоступна (nullptr = можно). */
static const char *pneumaticBlock() {
  if (otaInProgress || otaValveLock) return "ota_in_progress";
  if (emergencyIsLatched()) return "emergency_latched";
  if (valveTestActive()) return "valve_test_active";
  if (!calibrationCompleted) return "not_calibrated";
  if (cfg_errorHasCriticalPneumatic()) return "critical_error";
  if (cfg_errorIsActive(ERR_SENSOR_CODE)) return "sensor_error";
  return nullptr;
}

/* ── Телеметрия ── */
static void buildStateJson(String &out) {
  DynamicJsonDocument doc(6144);
  doc["version"] = VERSION;
  doc["uptime_s"] = millis() / 1000UL;

  JsonObject sys = doc.createNestedObject("system");
  sys["heap_free"] = ESP.getFreeHeap();
  sys["heap_min"] = ESP.getMinFreeHeap();
  sys["max_alloc"] = ESP.getMaxAllocHeap();
  sys["uptime_h"] = uptimeHours;

  {
    MutexGuard guard(xStateMutex, pdMS_TO_TICKS(50));
    if (guard) {
      doc["mode"] = modeName(currentSystemMode);
      doc["state"] = stateName(currentState);
      doc["angles"]["x"] = angleX;
      doc["angles"]["y"] = angleY;
      doc["temperature_c"] = temperature;
      JsonObject pr = doc.createNestedObject("pressure");
      JsonObject pv = doc.createNestedObject("pressure_valid");
      for (int i = 0; i < PAD_COUNT; i++) {
        pr[padKey(Pad(i))] = pressure[i];
        pv[padKey(Pad(i))] = pressureValid[i];
      }
      pr["master"] = masterPressure;
      pv["master"] = masterValid;
      doc["calibration"]["completed"] = calibrationCompleted;
      doc["calibration"]["valid"] = calibrationValid;
      doc["moving"] = isMoving;
      doc["movement_mode_active"] = movementModeActive;
      doc["manual"]["active"] = manualControlActive;
      doc["manual"]["pad"] = manualControlActive ? padKey(manualPadIndex) : "";
      doc["manual"]["inflate"] = manualInflate;
      doc["manual"]["elapsed_ms"] =
          manualControlActive ? (millis() - manualStartTime) : 0;
      doc["manual"]["max_ms"] = (uint32_t)cfg_getManualMaxTimeSec() * 1000UL;
      JsonArray tg = doc["manual"].createNestedArray("targets");
      JsonArray ts = doc["manual"].createNestedArray("target_set");
      for (int i = 0; i < PAD_COUNT; i++) {
        tg.add(manualTargetPressure[i]);
        ts.add(manualTargetSet[i]);
      }
    } else {
      doc["mode"] = modeName(currentSystemMode);
      doc["state"] = stateName(currentState);
    }
  }

  doc["test_state"] = valveTestActive() ? "ACTIVE" : (currentTestState == TestState::COMPLETED ? "COMPLETED" : "IDLE");
  doc["mpu_ok"] = mpuOk;
  doc["jhm_ready"] = jhmReady;
  doc["leveling"]["stage"] = autoLevelingController.stageName();
  doc["leveling"]["attempts_hour"] = levelingAttemptsThisHour;
  doc["leveling"]["niv_count"] = cfg_getNivCount();

  {
    JsonObject err = doc.createNestedObject("errors");
    const int n = ErrorHandler::getActiveErrorCount();
    JsonArray arr = err.createNestedArray("active");
    for (int i = 0; i < n; i++) {
      arr.add(errName(ErrorHandler::getActiveErrorAt(i)));
    }
    err["count"] = n;
  }

  {
    JsonObject w = doc.createNestedObject("wifi");
    w["connected"] = (WiFi.status() == WL_CONNECTED);
    w["ssid"] = sta_ssid;
    w["ip"] = WiFi.localIP().toString();
    w["rssi"] = WiFi.RSSI();
  }

  {
    JsonObject o = doc.createNestedObject("ota");
    o["in_progress"] = otaInProgress;
    o["progress"] = otaProgress;
    o["status"] = otaStatus;
  }

  doc["ha"]["enabled"] = haEnabled;
  doc["ha"]["server"] = haServerRunning;
  doc["ha"]["requests"] = haServed;

  serializeJson(doc, out);
}

static void handleState() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  String out;
  buildStateJson(out);
  sendJson(200, out);
}

/* ── Настройки ── */
static void buildConfigJson(String &out) {
  DynamicJsonDocument doc(6144);
  {
    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(200));
    if (!guard) { out = "{\"error\":\"config_busy\"}"; return; }
    ConfigManager::toJson(doc.to<JsonObject>(), false);
  }
  serializeJson(doc, out);
}

static void handleConfigGet() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  String out;
  buildConfigJson(out);
  sendJson(200, out);
}

static void handleConfigPost() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  DynamicJsonDocument doc(2048);
  DeserializationError je = deserializeJson(doc, haServer.arg("plain"));
  if (je || !doc.is<JsonObjectConst>()) { sendError(400, "bad_json"); return; }
  if (!doc.as<JsonObjectConst>().size()) { sendError(400, "empty_object"); return; }

  String applied, rejected, err;
  ConfigManager::applyJsonObject(doc.as<JsonObjectConst>(), true, &applied,
                                 &rejected, &err);
  if (err.length()) { sendError(409, err.c_str()); return; }

  haConfigDirty = true;
  haConfigDirtyMs = millis();

  DynamicJsonDocument res(768);
  res["ok"] = true;
  res["applied"] = applied.length() ? applied.c_str() : "";
  res["rejected"] = rejected.length() ? rejected.c_str() : "";
  res["saved"] = false;
  String out;
  serializeJson(res, out);
  sendJson(200, out);
}

/* ── Режим ── */
static void handleMode() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, haServer.arg("plain"))) { sendError(400, "bad_json"); return; }
  const String m = doc["mode"] | "";
  SystemMode target;
  if (m == "auto" || m == "AUTO") target = SystemMode::AUTO;
  else if (m == "manual" || m == "MANUAL") target = SystemMode::MANUAL;
  else { sendError(400, "mode_must_be_auto_or_manual"); return; }
  if (target == SystemMode::MOVEMENT) { sendError(409, "movement_not_supported"); return; }

  if (!switchSystemMode(target, "ha")) {
    sendError(409, "mode_switch_rejected");
    return;
  }
  DynamicJsonDocument res(256);
  res["ok"] = true;
  res["mode"] = modeName(target);
  String out;
  serializeJson(res, out);
  sendJson(200, out);
}

/* ── Ручное управление ── */
static void handleManual() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, haServer.arg("plain"))) { sendError(400, "bad_json"); return; }
  const String action = doc["action"] | "";

  if (action == "stop") {
    stopManualOperation();
    haManualStopAt = 0;
    DynamicJsonDocument res(128);
    res["ok"] = true;
    res["action"] = "stop";
    String out;
    serializeJson(res, out);
    sendJson(200, out);
    return;
  }

  const char *blocked = pneumaticBlock();
  if (blocked) { sendError(423, blocked); return; }

  Pad pad;
  if (!parsePad(doc["pad"] | "", pad)) { sendError(400, "bad_pad"); return; }
  const bool inflate = doc["inflate"] | false;

  if (action == "start") {
    const ManualStartResult r = startManualOperation(pad, inflate);
    if (r != ManualStartResult::Ok) {
      sendError(409, manualStartResultStr(r));
      return;
    }
    haManualStopAt = millis() + (uint32_t)cfg_getManualMaxTimeSec() * 1000UL;
    DynamicJsonDocument res(256);
    res["ok"] = true;
    res["action"] = "start";
    res["pad"] = padKey(pad);
    res["inflate"] = inflate;
    res["auto_stop_ms"] = (uint32_t)cfg_getManualMaxTimeSec() * 1000UL;
    String out;
    serializeJson(res, out);
    sendJson(200, out);
    return;
  }

  if (action == "pulse") {
    if (s_pulse.valid) { sendError(409, "pulse_busy"); return; }
    float secs = doc["seconds"] | 2.0f;
    const float maxS = (float)cfg_getManualMaxTimeSec();
    secs = constrain(secs, 0.2f, maxS);
    s_pulse.valid = true;
    s_pulse.pad = pad;
    s_pulse.inflate = inflate;
    s_pulse.ms = (uint32_t)(secs * 1000.0f);
    DynamicJsonDocument res(256);
    res["ok"] = true;
    res["action"] = "pulse";
    res["pad"] = padKey(pad);
    res["inflate"] = inflate;
    res["ms"] = s_pulse.ms;
    res["queued"] = true;
    String out;
    serializeJson(res, out);
    sendJson(202, out);
    return;
  }

  sendError(400, "action_must_be_start_stop_or_pulse");
}

/* ── Действия ── */
static void handleAction() {
  if (!authorized()) { sendError(401, "unauthorized"); return; }
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, haServer.arg("plain"))) { sendError(400, "bad_json"); return; }
  const String action = doc["action"] | "";

  if (action == "reset_errors") {
    resetSystemErrors();
  } else if (action == "stop_all") {
    stopManualOperation();
    haManualStopAt = 0;
    closeAllValves();
  } else if (action == "emergency_stop") {
    emergencyStop();
  } else if (action == "save") {
    const bool ok = ConfigManager::save();
    haConfigDirty = false;
    DynamicJsonDocument res(128);
    res["ok"] = ok;
    res["action"] = "save";
    String out;
    serializeJson(res, out);
    sendJson(ok ? 200 : 500, out);
    return;
  } else if (action == "restart") {
    haRestartAt = millis() + 300;
  } else {
    sendError(400, "unknown_action");
    return;
  }

  DynamicJsonDocument res(192);
  res["ok"] = true;
  res["action"] = action;
  String out;
  serializeJson(res, out);
  sendJson(200, out);
}

static void handleRoot() {
  haServed++;
  String body = "<html><head><meta charset='utf-8'><title>Kamaz Leveler</title></head><body>";
  body += "<h3>Kamaz Leveler</h3><p>Version: ";
  body += VERSION;
  body += "<br>Mode: ";
  body += modeName(currentSystemMode);
  body += "<br>Free heap: ";
  body += String(ESP.getFreeHeap());
  body += "</p><ul>";
  body += "<li><a href='/api/state'>/api/state</a></li>";
  body += "<li><a href='/api/config'>/api/config</a></li>";
  body += "<li><a href='/api/health'>/api/health</a></li>";
  body += "</ul></body></html>";
  haServer.sendHeader("Connection", "close");
  haServer.send(200, "text/html; charset=utf-8", body);
}

static void handleHealth() {
  haServed++;
  DynamicJsonDocument doc(192);
  doc["ok"] = true;
  doc["version"] = VERSION;
  doc["ha_enabled"] = haEnabled;
  String out;
  serializeJson(doc, out);
  sendJson(200, out);
}

static void handleNotFound() {
  haServed++;
  sendError(404, "not_found");
}

/* ── Управление сервером ── */
static void startServer() {
  if (haServerRunning) return;
  haServer.on("/", HTTP_GET, handleRoot);
  haServer.on("/api/health", HTTP_GET, handleHealth);
  haServer.on("/api/state", HTTP_GET, handleState);
  haServer.on("/api/config", HTTP_GET, handleConfigGet);
  haServer.on("/api/config", HTTP_POST, handleConfigPost);
  haServer.on("/api/mode", HTTP_POST, handleMode);
  haServer.on("/api/manual", HTTP_POST, handleManual);
  haServer.on("/api/action", HTTP_POST, handleAction);
  haServer.onNotFound(handleNotFound);
  haServer.begin();
  haServerRunning = true;
  haServed = 0;
  Logger::log(Logger::INFO, "HA", "HTTP API: 80, /api/state");
  SERIAL_DBG("[HA] server started, ip=%s\n", WiFi.localIP().toString().c_str());
}

static void stopServer() {
  if (!haServerRunning) return;
  haServer.stop();
  haServerRunning = false;
  Logger::log(Logger::INFO, "HA", "HTTP API остановлен");
  SERIAL_DBGLN("[HA] server stopped");
}

/* ── Публичный API ── */
bool haIsEnabled() { return haEnabled; }

const char *haStatusText() {
  static char buf[64];
  if (!haEnabled) return "Выкл";
  if (WiFi.status() != WL_CONNECTED) return "нет Wi-Fi";
  snprintf(buf, sizeof(buf), "http://%s/api/state",
           WiFi.localIP().toString().c_str());
  return buf;
}

void haSetEnabled(bool enabled) {
  haEnabled = enabled;
  cfg_setHaEnabled(enabled);
  const bool ok = ConfigManager::save();
  SERIAL_DBG("[HA] %s (save=%d)\n", enabled ? "ON" : "OFF", ok ? 1 : 0);
  if (!enabled) stopServer();
}

void haOtaBegin() {
  haSuspended = true;
  stopServer();
}

void haOtaEnd() {
  haSuspended = false;
}

uint32_t haRequestsServed() { return haServed; }

void haTick() {
  const uint32_t now = millis();

  // Авто-стоп ручного удержания по лимиту из конфига.
  if (haManualStopAt != 0) {
    if (!manualControlActive) {
      haManualStopAt = 0;
    } else if ((int32_t)(now - haManualStopAt) >= 0) {
      SERIAL_DBGLN("[HA] manual timeout > stop");
      stopManualOperation();
      haManualStopAt = 0;
    }
  }

  // Отложенная запись конфига (защита флеша от частых POST /api/config).
  if (haConfigDirty && (now - haConfigDirtyMs) >= 10000UL) {
    haConfigDirty = false;
    ConfigManager::save();
    SERIAL_DBGLN("[HA] config saved (deferred)");
  }

  // Отложенная перезагрузка (успеть отдать HTTP-ответ).
  if (haRestartAt != 0 && (int32_t)(now - haRestartAt) >= 0) {
    haRestartAt = 0;
    ESP.restart();
  }
}

void haTask(void *pvParameters) {
  (void)pvParameters;
  static uint32_t lastStatusLog = 0;
  for (;;) {
    TaskPool::markRun(taskIndex_HA);
    TaskMonitor::updateTaskStatus(TaskMonitor::TASK_HA);

    haTick();

    if (s_pulse.valid) {
      PendingPulse p = s_pulse;
      s_pulse.valid = false;
      if (pneumaticBlock() == nullptr) {
        const bool ok = pulsePad(p.pad, p.inflate, p.ms, nullptr, nullptr);
        SERIAL_DBG("[HA] pulse %s %s %ums > %s\n", padKey(p.pad),
                   p.inflate ? "inflate" : "deflate", (unsigned)p.ms, ok ? "ok" : "fail");
      } else {
        SERIAL_DBGLN("[HA] pulse rejected (state)");
      }
    }

    const bool wifiUp = (WiFi.status() == WL_CONNECTED);
    if (haEnabled && !haSuspended && wifiUp && currentSystemMode != SystemMode::MOVEMENT) {
      startServer();
      haServer.handleClient();
    } else {
      stopServer();
      if (!wifiUp && millis() - lastStatusLog > 30000UL) {
        lastStatusLog = millis();
        SERIAL_DBGLN("[HA] waiting for Wi-Fi STA");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ── Serial-консоль ── */
bool haHandleSerialCommand(const char *line) {
  if (line == nullptr) return false;
  if (strncmp(line, "HA", 2) != 0 && strncmp(line, "ha", 2) != 0) return false;
  if (line[2] != ' ' && line[2] != '\0') return false;

  const char *rest = line + 2;
  while (*rest == ' ') rest++;

  if (strcasecmp(rest, "STATUS") == 0 || *rest == '\0') {
    Serial.printf("[HA] enabled=%d server=%d token=%s ip=%s\n", haEnabled ? 1 : 0,
                  haServerRunning ? 1 : 0, cfg_getHaToken()[0] ? "set" : "none",
                  WiFi.localIP().toString().c_str());
    return true;
  }
  if (strcasecmp(rest, "SELFTEST") == 0) {
    /* Полный HTTP-круг к собственному серверу: проверка сокета, парсинга, авторизации. */
    WiFiClient client;
    const IPAddress ip = WiFi.localIP();
    if (!client.connect(ip, 80)) {
      Serial.println("[HA] selftest > connect fail");
      return true;
    }
    client.printf("GET /api/state HTTP/1.1\r\nHost: %s\r\n",
                  ip.toString().c_str());
    if (cfg_getHaToken()[0]) {
      client.printf("X-Api-Key: %s\r\n", cfg_getHaToken());
    }
    client.print("Connection: close\r\n\r\n");
    const uint32_t t0 = millis();
    String resp;
    while (millis() - t0 < 5000UL && (client.connected() || client.available())) {
      while (client.available()) resp += (char)client.read();
      delay(1);
    }
    client.stop();
    const int nl = resp.indexOf("\r\n");
    Serial.printf("[HA] selftest > %s (%u bytes)\n",
                  nl > 0 ? resp.substring(0, nl).c_str() : resp.c_str(),
                  (unsigned)resp.length());
    return true;
  }
  if (strcasecmp(rest, "STATE") == 0) {
    String out;
    buildStateJson(out);
    Serial.printf("[HA] %s\n", out.c_str());
    return true;
  }
  if (strcasecmp(rest, "CONFIG") == 0) {
    String out;
    buildConfigJson(out);
    Serial.printf("[HA] %s\n", out.c_str());
    return true;
  }
  if (strcasecmp(rest, "ON") == 0) {
    haSetEnabled(true);
    Serial.println("[HA] ON");
    return true;
  }
  if (strcasecmp(rest, "OFF") == 0) {
    haSetEnabled(false);
    Serial.println("[HA] OFF");
    return true;
  }
  if (strncasecmp(rest, "TOKEN", 5) == 0 && (rest[5] == ' ' || rest[5] == '\0')) {
    const char *val = rest + 5;
    while (*val == ' ') val++;
    cfg_setHaToken(val);
    ConfigManager::save();
    Serial.printf("[HA] token %s\n", strlen(val) ? "set" : "cleared");
    return true;
  }
  Serial.println("[HA] usage: HA ON|OFF|STATUS|TOKEN <str>|STATE");
  return true;
}
