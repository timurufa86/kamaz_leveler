#include "wifi_setup.h"
#include "ota_net.h"
#include "app_globals.h"
#include "ui_theme.h"
#include "ui_text.h"
#include "ui_fonts.h"
#include "logger.h"

#include <WiFi.h>
#include <Adafruit_ST7789.h>

extern Adafruit_ST7789 tft;
char wifi_ssid[32] = "Kamaz-OTA-AP";
char wifi_password[64] = "";
char sta_ssid[33] = "";
static constexpr char WIFI_STA_PASSWORD[] = "asd12345";
static constexpr uint8_t WIFI_SCAN_MAX_NETWORKS = 8;
char wifi_scan_ssids[WIFI_SCAN_MAX_NETWORKS][33] = {};
int8_t wifi_scan_rssi[WIFI_SCAN_MAX_NETWORKS] = {};
uint8_t wifi_scan_count = 0;
uint8_t wifi_scan_selected = 0;
volatile bool wifiUiFullRedraw = false;
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
char ota_password[64] = "";

void requestWiFiSetup() {
  if (wifiScanInProgress || wifiSetupRequested) return;
  wifiSetupRequested = true;
  wifiScanInProgress = true;
  wifiSetupActive = false;
  menuVisible = false;
  wifiUiFullRedraw = true;
  displayDirty = true;
  Serial.println("[WiFi] Запрос сканирования (otaTask)");
}

void startWiFiSetup() {
  wifiSetupRequested = false;
  wifiScanInProgress = true;
  wifiSetupActive = false;
  menuVisible = false;
  wifi_scan_count = 0;
  wifiUiFullRedraw = true;
  displayDirty = true;

  githubOtaReleaseTlsHeap("wifi-scan");
  githubOtaDefragHeap();
  Serial.printf("[WiFi] Сканирование (async) free=%u maxBlk=%u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));

  const wifi_mode_t prevMode = WiFi.getMode();
  if (prevMode == WIFI_MODE_NULL) {
    WiFi.mode(WIFI_STA);
  } else if (prevMode == WIFI_MODE_AP) {
    WiFi.mode(WIFI_AP_STA);
  }

  WiFi.scanDelete();
  const int16_t rc = WiFi.scanNetworks(/*async=*/true, /*hidden=*/true);
  if (rc == WIFI_SCAN_FAILED) {
    Serial.println("[WiFi] scan start FAILED");
    wifiScanInProgress = false;
    wifiSetupActive = false;
    menuVisible = true;
    wifiUiFullRedraw = true;
    displayDirty = true;
    githubOtaReserveTlsHeap("wifi-scan-fail");
  }
}

bool pollWiFiScanComplete() {
  if (!wifiScanInProgress) return false;

  const int16_t found = WiFi.scanComplete();
  if (found == WIFI_SCAN_RUNNING) return true;

  wifi_scan_count = 0;
  if (found > 0) {
    for (int i = 0; i < found && wifi_scan_count < WIFI_SCAN_MAX_NETWORKS; i++) {
      String name = WiFi.SSID(i);
      if (name.length() == 0) continue;
      bool duplicate = false;
      for (uint8_t j = 0; j < wifi_scan_count; j++) {
        if (name.equals(wifi_scan_ssids[j])) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) continue;
      strlcpy(wifi_scan_ssids[wifi_scan_count], name.c_str(),
              sizeof(wifi_scan_ssids[wifi_scan_count]));
      wifi_scan_rssi[wifi_scan_count] = static_cast<int8_t>(WiFi.RSSI(i));
      wifi_scan_count++;
    }
  } else if (found == WIFI_SCAN_FAILED) {
    Serial.println("[WiFi] scanComplete FAILED");
  }
  WiFi.scanDelete();

  wifiScanInProgress = false;
  wifi_scan_selected = 0;
  for (uint8_t i = 0; i < wifi_scan_count; i++) {
    if (sta_ssid[0] != '\0' && strcmp(wifi_scan_ssids[i], sta_ssid) == 0) {
      wifi_scan_selected = i;
      break;
    }
  }
  wifiSetupActive = wifi_scan_count > 0;
  wifiUiFullRedraw = true;
  displayDirty = true;
  Serial.printf("[WiFi] Найдено сетей: %u (stackHWM=%u free=%u)\n", wifi_scan_count,
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                static_cast<unsigned>(ESP.getFreeHeap()));
  githubOtaReserveTlsHeap("post-wifi-scan");
  if (wifi_scan_count == 0) {
    Serial.println("[WiFi] Сети не найдены — возврат в меню");
    menuVisible = true;
    displayDirty = true;
  }
  return false;
}

static void drawWiFiListRow(uint8_t i, bool selected) {
  if (i >= wifi_scan_count) return;
  constexpr int16_t LIST_Y = 66;
  constexpr int16_t ROW = 20;
  const int16_t y = LIST_Y + i * ROW;
  const int rssi = wifi_scan_rssi[i];
  const bool isCurrent = (sta_ssid[0] != '\0' && strcmp(wifi_scan_ssids[i], sta_ssid) == 0);

  if (selected) {
    tft.fillRoundRect(theme::MARGIN - 2, y, theme::SCREEN_W - 2 * theme::MARGIN + 4, ROW, 3, theme::PANEL);
  } else {
    tft.fillRect(theme::MARGIN - 2, y, theme::SCREEN_W - 2 * theme::MARGIN + 4, ROW, theme::BG);
  }

  const uint8_t bars = (rssi > -55) ? 4 : (rssi > -70) ? 3 : (rssi > -85) ? 2 : 1;
  for (uint8_t b = 0; b < 4; b++) {
    const int16_t bx = static_cast<int16_t>(theme::SCREEN_W - theme::MARGIN - 66 + b * 6);
    const int16_t bh = static_cast<int16_t>(4 + b * 3);
    const uint16_t barColor = (b < bars) ? ((bars >= 3) ? theme::OK : theme::WARN) : theme::TRACK;
    tft.fillRect(bx, static_cast<int16_t>(y + ROW - 5 - bh), 4, bh, barColor);
  }

  char nameBuf[36];
  if (isCurrent) {
    snprintf(nameBuf, sizeof(nameBuf), "*%s", wifi_scan_ssids[i]);
  } else {
    strlcpy(nameBuf, wifi_scan_ssids[i], sizeof(nameBuf));
  }

  ui.setFont(selected ? UiFont::SmallB : UiFont::Small);
  ui.setColors(selected ? theme::ACCENT : (isCurrent ? theme::OK : theme::TEXT),
               selected ? theme::PANEL : theme::BG);
  ui.box(theme::MARGIN + 2, y, 186, ROW, ui.ellipsize(nameBuf, 180),
         UiHAlign::Left, UiVAlign::Middle, true);

  ui.setFont(UiFont::Tiny);
  ui.setColors(theme::TEXT_DIM, selected ? theme::PANEL : theme::BG);
  ui.boxf(theme::SCREEN_W - theme::MARGIN - 26, y, 26, ROW, UiHAlign::Right, UiVAlign::Middle, true,
          "%d", rssi);
}

static void drawWiFiStatusBar() {
  constexpr int16_t Y = 214;
  tft.fillRect(0, Y, theme::SCREEN_W, 26, theme::BG);

  char line[64];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(line, sizeof(line), "Сейчас: %s  %d dBm  %s",
             WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
    ui.setColors(theme::OK, theme::BG);
  } else if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
    snprintf(line, sizeof(line), "Сейчас: AP %s  %s",
             wifi_ssid, WiFi.softAPIP().toString().c_str());
    ui.setColors(theme::WARN, theme::BG);
  } else if (sta_ssid[0] != '\0') {
    snprintf(line, sizeof(line), "Сейчас: нет связи (сохр. %s)", sta_ssid);
    ui.setColors(theme::ERR, theme::BG);
  } else {
    strlcpy(line, "Сейчас: не подключено", sizeof(line));
    ui.setColors(theme::TEXT_DIM, theme::BG);
  }

  ui.setTransparent(true);
  ui.setFont(UiFont::Tiny);
  ui.box(theme::MARGIN, Y, theme::SCREEN_W - 2 * theme::MARGIN, 22,
         ui.ellipsize(line, theme::SCREEN_W - 2 * theme::MARGIN),
         UiHAlign::Left, UiVAlign::Middle, true);
}

void displayWiFiSetupScreen() {
  static bool frameReady = false;
  static int8_t lastSelected = -1;
  static uint32_t lastStatusMs = 0;
  static bool lastScanProgress = false;
  static char lastStatusLine[64] = "";

  if (wifiUiFullRedraw || lastScanProgress != wifiScanInProgress) {
    frameReady = false;
    lastSelected = -1;
    lastStatusLine[0] = '\0';
    lastScanProgress = wifiScanInProgress;
    wifiUiFullRedraw = false;
  }

  if (!frameReady) {
    tft.fillScreen(theme::BG);

    tft.fillRoundRect(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24,
                      theme::RADIUS, theme::PANEL_ALT);
    ui.setTransparent(true);
    ui.setFont(UiFont::Med);
    ui.setColors(theme::ACCENT, theme::PANEL_ALT);
    ui.box(theme::MARGIN, theme::MARGIN - 2, theme::SCREEN_W - 2 * theme::MARGIN, 24, "ВЫБОР WI-FI",
           UiHAlign::Center, UiVAlign::Middle, false);

    ui.setFont(UiFont::Tiny);
    ui.setColors(theme::TEXT_DIM, theme::BG);
    ui.box(theme::MARGIN, 32, 280, 14, "КН1/КН2 — выбор,  КН3 — подключить",
           UiHAlign::Left, UiVAlign::Middle, false);
    ui.box(theme::MARGIN, 46, 280, 14, "КН4 — отмена   * = сохранённая сеть",
           UiHAlign::Left, UiVAlign::Middle, false);

    if (wifiScanInProgress) {
      ui.setFont(UiFont::Small);
      ui.setColors(theme::WARN, theme::BG);
      ui.box(theme::MARGIN, 90, 250, 18, "Сканирование...", UiHAlign::Left, UiVAlign::Middle, true);
      drawWiFiStatusBar();
      lastStatusLine[0] = '\0';
      frameReady = true;
      return;
    }

    for (uint8_t i = 0; i < wifi_scan_count; i++) {
      drawWiFiListRow(i, i == wifi_scan_selected);
    }
    lastSelected = static_cast<int8_t>(wifi_scan_selected);
    drawWiFiStatusBar();
    lastStatusMs = millis();
    lastStatusLine[0] = '\0';
    frameReady = true;
    return;
  }

  if (!wifiScanInProgress && lastSelected != static_cast<int8_t>(wifi_scan_selected)) {
    if (lastSelected >= 0 && lastSelected < static_cast<int8_t>(wifi_scan_count)) {
      drawWiFiListRow(static_cast<uint8_t>(lastSelected), false);
    }
    drawWiFiListRow(wifi_scan_selected, true);
    lastSelected = static_cast<int8_t>(wifi_scan_selected);
  }

  if (millis() - lastStatusMs >= 1000) {
    char line[64];
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(line, sizeof(line), "Сейчас: %s  %d dBm  %s",
               WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
    } else if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
      snprintf(line, sizeof(line), "Сейчас: AP %s  %s",
               wifi_ssid, WiFi.softAPIP().toString().c_str());
    } else if (sta_ssid[0] != '\0') {
      snprintf(line, sizeof(line), "Сейчас: нет связи (сохр. %s)", sta_ssid);
    } else {
      strlcpy(line, "Сейчас: не подключено", sizeof(line));
    }
    if (strcmp(line, lastStatusLine) != 0) {
      strlcpy(lastStatusLine, line, sizeof(lastStatusLine));
      drawWiFiStatusBar();
    }
    lastStatusMs = millis();
  }
}

void startFallbackAccessPoint() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  if (!WiFi.softAP(wifi_ssid, wifi_password)) {
    Serial.println("[WiFi] Не удалось запустить резервную точку доступа");
    wifiConnected = false;
    return;
  }
  wifiConnected = false;
  Serial.printf("[WiFi] AP fallback: %s, IP %s\n",
                wifi_ssid, WiFi.softAPIP().toString().c_str());
}

void connectConfiguredWiFi() {
  if (sta_ssid[0] == '\0') {
    Serial.println("[WiFi] SSID роутера не выбран — AP не поднимаем (только вручную из меню/OTA)");
    WiFi.mode(WIFI_STA);
    wifiConnected = false;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid, WIFI_STA_PASSWORD);
  Serial.printf("[WiFi] Подключение к \"%s\"\n", sta_ssid);
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("[WiFi] STA подключен, IP %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WiFi] STA не подключен — без fallback AP (ручной AP: OTA/настройки)");
    wifiConnected = false;
    WiFi.mode(WIFI_STA);
  }
}

void initializeDefaultCredentials() {
  uint64_t chipId = ESP.getEfuseMac();
  uint32_t suffix = static_cast<uint32_t>(chipId & 0xFFFFFF);
  if (wifi_password[0] == '\0' || strcmp(wifi_password, "12345678") == 0) {
    snprintf(wifi_password, sizeof(wifi_password), "KzWiFi-%06lX", suffix);
  }
  if (ota_password[0] == '\0' || strcmp(ota_password, "12345678") == 0) {
    snprintf(ota_password, sizeof(ota_password), "KzOTA-%06lX", suffix);
  }
}
