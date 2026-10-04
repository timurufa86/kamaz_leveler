#include "arduino_ota.h"
#include "ui_screens.h"
#include "ui_theme.h"
#include "ui_text.h"
#include "app_version.h"
#include "app_globals.h"
#include "valve_ctrl.h"
#include "config_manager.h"
#include "error_handler.h"
#include "event_bus.h"
#include "logger.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Adafruit_ST7789.h>
#include <cstring>

extern Adafruit_ST7789 tft;

constexpr uint16_t OTA_PORT = 8266;
constexpr char OTA_HOSTNAME[] = "Kamaz-leveling-system";

int otaProgress = 0;
char otaStatus[32] = "";

extern char wifi_ssid[32];
extern char wifi_password[64];
extern char ota_password[64];
extern IPAddress local_ip;
extern IPAddress gateway;
extern IPAddress subnet;

void displayOTAScreen() {
  if (errorScreenBlocking || ErrorHandler::hasUiBlockingErrors()) {
    return;
  }

  static bool initialized = false;
  static bool lastInstallMode = false;
  static int lastProgress = -1;
  static uint8_t lastDots = 0;
  static uint32_t lastDotUpdate = 0;
  static char lastOtaStatus[32] = "";
  static uint32_t otaWaitStart = 0;

  if (otaUiNeedFullRedraw) {
    otaUiNeedFullRedraw = false;
    initialized = false;
    lastProgress = -1;
    lastOtaStatus[0] = '\0';
  }

  // Экран обслуживает два случая: режим ArduinoOTA (AP) и установку релиза
  // с GitHub. При смене режима перерисовываем шапку целиком.
  if (lastInstallMode != otaInProgress) {
    lastInstallMode = otaInProgress;
    initialized = false;
    lastProgress = -1;
    lastOtaStatus[0] = '\0';
  }

  if (!otaInProgress && initialized) {
    if (otaWaitStart == 0) otaWaitStart = millis();
    if (millis() - otaWaitStart > 120000) {
      stopOTAMode();
      return;
    }
  } else {
    otaWaitStart = 0;
  }

  constexpr int16_t BAR_X = theme::MARGIN + 4;
  constexpr int16_t BAR_Y = 108;
  constexpr int16_t BAR_W = theme::SCREEN_W - 2 * (theme::MARGIN + 4);
  constexpr int16_t BAR_H = 18;

  if (!initialized) {
    tft.fillScreen(theme::BG);
    uiHeader(otaInProgress ? "ОБНОВЛЕНИЕ" : "РЕЖИМ OTA", theme::ACCENT);

    if (otaInProgress) {
      uiInfoRow(44, "Установлено:", VERSION);
      uiInfoRow(62, "Источник:", "GitHub / ArduinoOTA");
    } else {
      uiInfoRow(44, "Сеть:", wifi_ssid);
      uiInfoRow(62, "Пароль:", "********");
      String apIp = WiFi.softAPIP().toString();
      uiInfoRow(80, "IP:", apIp.c_str());
    }

    drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, 0, theme::OK);
    uiHintBar(otaInProgress ? "Не выключайте питание" : "УДЕРЖ. КН3+КН4 — МЕНЮ", theme::TEXT_DIM);

    initialized = true;
  }

  if (otaInProgress) {
    if (otaProgress != lastProgress) {
      drawProgressBar(BAR_X, BAR_Y, BAR_W, BAR_H, static_cast<uint8_t>(otaProgress), theme::OK);

      ui.setTransparent(true);
      ui.setFont(UiFont::Large);
      ui.setColors(theme::TEXT, theme::BG);
      ui.boxf(0, BAR_Y + BAR_H + 4, theme::SCREEN_W, 34, UiHAlign::Center, UiVAlign::Middle, true,
              "%d%%", static_cast<int>(otaProgress));

      lastProgress = otaProgress;
    }

    if (strcmp(otaStatus, lastOtaStatus) != 0) {
      ui.setTransparent(true);
      ui.setFont(UiFont::Small);
      ui.setColors(theme::TEXT_DIM, theme::BG);
      ui.box(0, 168, theme::SCREEN_W, 18, otaStatus, UiHAlign::Center, UiVAlign::Middle, true);

      strncpy(lastOtaStatus, otaStatus, sizeof(lastOtaStatus) - 1);
      lastOtaStatus[sizeof(lastOtaStatus) - 1] = '\0';
    }
  } else {
    uint32_t now = millis();
    if (now - lastDotUpdate > 500) {
      lastDotUpdate = now;

      lastDots = (lastDots + 1) % 4;
      char dots[4] = "";
      for (uint8_t i = 0; i < lastDots; i++) dots[i] = '.';
      dots[lastDots] = '\0';

      char waitBuf[24];
      snprintf(waitBuf, sizeof(waitBuf), "ОЖИДАНИЕ%s", dots);

      ui.setTransparent(true);
      ui.setFont(UiFont::Small);
      ui.setColors(theme::WARN, theme::BG);
      ui.box(0, 168, theme::SCREEN_W, 18, waitBuf, UiHAlign::Center, UiVAlign::Middle, true);
    }
  }
}

void initOTA() {
  Logger::log(Logger::INFO, "OTA", "Инициализация…");
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    if (otaInProgress) {
      Serial.println("[OTA] Обновление уже идет!");
      return;
    }

    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (maxSketchSpace < 1024 * 1024) {
      Serial.printf("[OTA] Недостаточно места! Свободно: %u байт\n", maxSketchSpace);
      ErrorHandler::handleError(ErrorHandler::Error::OTA, "Недостаточно места для OTA");
      otaInProgress = false;
      otaValveLock = false;
      return;
    }

    saveConfig();
    saveWiFiConfig();
    // Keep LittleFS mounted: other tasks may still access it while OTA runs.

    Logger::log(Logger::INFO, "OTA", "Старт обновления");
    otaInProgress = true;
    otaValveLock = true;
    otaProgress = 0;
    strcpy(otaStatus, "Начало");
    emergencyStop();

    Event event;
    event.type = EventType::OTA_START;
    event.timestamp = millis();
    EventBus::publish(event);
  });

  ArduinoOTA.onEnd([]() {
    Logger::log(Logger::INFO, "OTA", "Обновление завершено");
    otaInProgress = false;
    otaValveLock = true;
    otaProgress = 100;
    strcpy(otaStatus, "Готово");

    Event event;
    event.type = EventType::OTA_END;
    event.timestamp = millis();
    EventBus::publish(event);

    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    otaProgress = (prog * 100) / total;
    snprintf(otaStatus, sizeof(otaStatus), "Загрузка: %d%%", otaProgress);

    Event event;
    event.type = EventType::OTA_PROGRESS;
    event.timestamp = millis();
    event.data.ota.progress = otaProgress;
    strncpy(event.data.ota.status, otaStatus, sizeof(event.data.ota.status));
    EventBus::publish(event, 0);
  });

  ArduinoOTA.onError([](ota_error_t err) {
    otaInProgress = false;
    otaValveLock = false;

    const char *errMsg;
    switch (err) {
      case OTA_AUTH_ERROR: errMsg = "Ошибка аутентификации"; break;
      case OTA_BEGIN_ERROR: errMsg = "Ошибка начала обновления"; break;
      case OTA_CONNECT_ERROR: errMsg = "Ошибка соединения"; break;
      case OTA_RECEIVE_ERROR: errMsg = "Ошибка приема данных"; break;
      case OTA_END_ERROR: errMsg = "Ошибка завершения"; break;
      default: errMsg = "Неизвестная ошибка";
    }

    char fullMsg[64];
    snprintf(fullMsg, sizeof(fullMsg), "OTA: %s (код: %d)", errMsg, err);
    ErrorHandler::handleError(ErrorHandler::Error::OTA, fullMsg);

    Logger::logf(Logger::ERROR, "OTA", "Ошибка: %s (код: %d)", errMsg, err);

    Event event;
    event.type = EventType::OTA_ERROR;
    event.timestamp = millis();
    EventBus::publish(event);
  });

  ArduinoOTA.begin();
  Logger::log(Logger::INFO, "OTA", "Готово");
}

void startOTAMode() {
  if (otaMode) return;
  Logger::log(Logger::INFO, "OTA", "Запуск режима OTA");
  menuVisible = false;
  displayDirty = true;
  otaValveLock = true;
  emergencyStop();
  otaMode = true;
  setSystemState(SystemState::OTA_MODE);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(wifi_ssid, wifi_password);
  initOTA();
}

void stopOTAMode() {
  if (!otaMode) return;
  otaMode = false;
  otaInProgress = false;
  otaValveLock = false;
  setSystemState(SystemState::RUNNING);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  forceDisplayReset(true);
}

void handleOTA() {
  if (otaMode) ArduinoOTA.handle();
}

