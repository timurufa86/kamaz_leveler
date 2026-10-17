#include "ota_net.h"
#include "ota_download_ca.h"
#include "logger.h"
#include "app_globals.h"
#include "task_pool.h"
#include "task_monitor.h"
#include "valve_ctrl.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <Update.h>
#include <new>

/* 10.15.0: проверка сертификата включена. CA-бандл ядра подключается, если
   заголовок доступен; иначе соединение остаётся незащищённым, но это
   фиксируется флагом otaTlsInsecure и громким сообщением в Serial. */
#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define KAMAZ_OTA_HAS_CERT_BUNDLE 1
#elif __has_include(<mbedtls/esp_crt_bundle.h>)
#include <mbedtls/esp_crt_bundle.h>
#define KAMAZ_OTA_HAS_CERT_BUNDLE 1
#else
#define KAMAZ_OTA_HAS_CERT_BUNDLE 0
#endif

volatile bool otaTlsInsecure = false;

void otaConfigureTls(WiFiClientSecure *client) {
  if (!client) return;
#if KAMAZ_OTA_HAS_CERT_BUNDLE
  extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
  extern const uint8_t rootca_crt_bundle_end[] asm("_binary_x509_crt_bundle_end");
  const size_t bundleLen = static_cast<size_t>(rootca_crt_bundle_end - rootca_crt_bundle_start);
  if (bundleLen > 0) {
    client->setCACertBundle(rootca_crt_bundle_start, bundleLen);
    otaTlsInsecure = false;
    return;
  }
#endif
  client->setInsecure();
  otaTlsInsecure = true;
  SERIAL_DBGLN("[OTA-TLS] ВНИМАНИЕ: CA-бандл недоступен, проверка сертификата ОТКЛЮЧЕНА");
}

/**
 * 10.15.4: TLS только для соединения скачивания образа — закреплённые корни
 * (Sectigo E46 для github.com, ISRG X1 для release-assets.githubusercontent.com).
 * Полный CA-бандл ESP-IDF после esp_ota_begin() уже не помещается в кучу: проверка
 * цепочки падала с MBEDTLS X509 fatal, и прерванная загрузка не могла
 * переподключиться и продолжиться с места обрыва. Закреплённые корни требуют
 * единиц КБ. Целостность образа по-прежнему гарантирует SHA-256 из digest,
 * полученного по отдельному проверенному API-соединению.
 */
void otaConfigureTlsDownload(WiFiClientSecure *client) {
  if (!client) return;
  client->setCACert(OTA_DOWNLOAD_CA_PEM);
  otaTlsInsecure = false;
}

/**
 * 10.15.5: image-download TLS without certificate verification (see header).
 */
void otaConfigureTlsInsecureDownload(WiFiClientSecure *client) {
  if (!client) return;
  client->setInsecure();
  otaTlsInsecure = true;
}

/** 10.15.0: прошивка и метаданные загружаются только по https. */
bool otaUrlIsHttps(const char *url) {
  return url != nullptr && strncmp(url, "https://", 8) == 0;
}

void githubOtaDefragHeap() {
  // Two-pass coalesce: grab every large free block, free LIFO.
  for (int pass = 0; pass < 2; pass++) {
    void *hold[48];
    int n = 0;
    size_t sz = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    while (n < 48 && sz >= 512) {
      hold[n] = heap_caps_malloc(sz, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
      if (!hold[n]) {
        sz /= 2;
        continue;
      }
      n++;
      sz = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    }
    while (n > 0) {
      heap_caps_free(hold[--n]);
    }
  }
}

// TLS-reserve disabled: parking 28KB split the free list so maxBlk stuck ~36KB
// and mbedTLS BIGNUM failed even with ~85KB free. Prefer stack trims + defrag.
void githubOtaReserveTlsHeap(const char *why) {
  (void)why;
}

void githubOtaReleaseTlsHeap(const char *why) {
  (void)why;
}

volatile bool s_otaWorkersPaused = false;
static uint16_t s_otaPauseDepth = 0;
static bool s_otaPauseHadValveLock = false;

void githubOtaPauseWorkers(bool pause) {
  if (pause) {
    if (s_otaPauseDepth == 0) {
      s_otaPauseHadValveLock = otaValveLock;
      // 10.15.0: сначала закрыть пневматику, и только потом морозить задачи —
      // раньше vTaskSuspend заставал открытые клапаны на всю TLS-сессию.
      otaQuiesceForNetwork();
    }
    s_otaPauseDepth++;
  } else {
    if (s_otaPauseDepth == 0) return;       // лишний resume игнорируем
    s_otaPauseDepth--;
    if (s_otaPauseDepth > 0) return;        // вложенная пауза ещё активна
    otaValveLock = s_otaPauseHadValveLock;  // вернуть прежнее состояние
  }
  s_otaWorkersPaused = pause;

  const uint8_t idxs[] = {taskIndex_Event,   taskIndex_Display, taskIndex_IMU,
                          taskIndex_Control, taskIndex_Valve,   taskIndex_Pressure,
                          taskIndex_ErrRec,  taskIndex_Button,  taskIndex_Calib};
  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  for (uint8_t idx : idxs) {
    if (idx == 0xFF) continue;
    // 10.15.0: задача, вызвавшая сеть (кнопки/дисплей), не усыпляет саму себя —
    // vTaskSuspend себя оставлял xDisplayMutex захваченным навсегда.
    if (TaskPool::getHandle(idx) == self) continue;
    if (pause) TaskPool::suspendTask(idx);
    else TaskPool::resumeTask(idx);
  }
}

void githubOtaPrintHeap(const char *tag) {
  SERIAL_DBG("[OTA-HEAP] %s free=%u maxBlk=%u int=%u\n", tag ? tag : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
}

void githubOtaHeartbeat() {
  extern uint8_t taskIndex_OTA;
  TaskPool::markRun(taskIndex_OTA);
  TaskMonitor::updateTaskStatus(TaskMonitor::TASK_OTA);
  // 10.15.0: метки остановленных задач больше не подделываются — монитор
  // честно пропускает проверку, пока пауза активна (см. TaskMonitor::checkTasks).
}

static esp_ota_handle_t s_ghOtaHandle = 0;
static bool s_ghOtaActive = false;
static const esp_partition_t *s_ghOtaPart = nullptr;

esp_ota_handle_t  getGhOtaHandle()   { return s_ghOtaHandle; }
bool              getGhOtaActive()    { return s_ghOtaActive; }
const esp_partition_t *getGhOtaPart() { return s_ghOtaPart; }
void setGhOtaHandle(esp_ota_handle_t h) { s_ghOtaHandle = h; }
void setGhOtaActive(bool v)             { s_ghOtaActive = v; }
void setGhOtaPart(const esp_partition_t *p) { s_ghOtaPart = p; }

void githubOtaClearUpdate(const char *why) {
  if (s_ghOtaActive) {
    SERIAL_DBG("[GH-OTA] abort esp_ota (%s) handle=%u\n", why ? why : "?",
                  static_cast<unsigned>(s_ghOtaHandle));
    esp_ota_abort(s_ghOtaHandle);
    s_ghOtaActive = false;
    s_ghOtaHandle = 0;
    s_ghOtaPart = nullptr;
  }
  if (Update.isRunning()) {
    SERIAL_DBG("[GH-OTA] abort Update (%s)\n", why ? why : "?");
    Update.abort();
  }
  Update.clearError();
}

void githubOtaEndInstallSession(const char *why) {
  otaInProgress = false;
  githubOtaReserveTlsHeap(why ? why : "ota-end");
  githubOtaPauseWorkers(false);
}

bool githubResolveDownloadUrl(const char *url, char *out, size_t outLen,
                              const char *acceptHdr) {
  if (!url || !out || outLen < 24) return false;
  strlcpy(out, url, outLen);
  // Only API asset URLs need a 302 → objects.githubusercontent.com hop.
  if (!strstr(url, "api.github.com/") || !strstr(url, "/releases/assets/")) {
    return true;
  }
  if (WiFi.status() != WL_CONNECTED) return false;
  if (strncmp(url, "https://", 8) != 0) return false;

  const char *p = url + 8;
  const char *slash = strchr(p, '/');
  if (!slash) return false;
  char host[96];
  const size_t hostLen = static_cast<size_t>(slash - p);
  if (hostLen == 0 || hostLen >= sizeof(host)) return false;
  memcpy(host, p, hostLen);
  host[hostLen] = '\0';
  const char *path = slash;

  WiFi.setSleep(false);
  githubOtaReleaseTlsHeap("resolve-url");
  if (!otaInProgress) githubOtaPauseWorkers(true);
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(40));

  bool ok = false;
  WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
  if (!client) {
    if (!otaInProgress) githubOtaPauseWorkers(false);
    return false;
  }
  otaConfigureTls(client);
  client->setHandshakeTimeout(25);
  client->setTimeout(20000);

  // Raw GET — HTTPClient often returns 302 with empty Location on ESP32.
  // Long GitHub Location (~900+ chars) must not use Arduino String (frag/fail → empty).
  if (client->connect(host, 443)) {
    client->printf(
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "User-Agent: kamaz-leveler/10.14.15\r\n"
        "Accept: %s\r\n"
        "X-GitHub-Api-Version: 2022-11-28\r\n"
        "Connection: close\r\n\r\n",
        path, host, acceptHdr ? acceptHdr : "application/octet-stream");

    // Slurp status+headers until CRLFCRLF (heap — avoid permanent BSS).
    constexpr size_t kHdrCap = 3072;
    char *hdrBuf = static_cast<char *>(malloc(kHdrCap));
    if (!hdrBuf) {
      SERIAL_DBGLN("[GH-OTA] resolve hdr malloc fail");
      client->stop();
      delete client;
      if (!otaInProgress) {
        githubOtaReserveTlsHeap("resolve-done");
        githubOtaPauseWorkers(false);
      }
      return false;
    }
    size_t hdrLen = 0;
    const uint32_t t0 = millis();
    bool hdrDone = false;
    while (!hdrDone && hdrLen + 1 < kHdrCap && (millis() - t0) < 25000) {
      githubOtaHeartbeat();
      if (client->available()) {
        const int b = client->read();
        if (b < 0) {
          delay(1);
          continue;
        }
        hdrBuf[hdrLen++] = static_cast<char>(b);
        if (hdrLen >= 4 && hdrBuf[hdrLen - 4] == '\r' && hdrBuf[hdrLen - 3] == '\n' &&
            hdrBuf[hdrLen - 2] == '\r' && hdrBuf[hdrLen - 1] == '\n') {
          hdrDone = true;
        }
      } else if (!client->connected() && !client->available()) {
        break;
      } else {
        delay(1);
      }
    }
    hdrBuf[hdrLen] = '\0';

    int code = -1;
    if (strncmp(hdrBuf, "HTTP/", 5) == 0) {
      const char *sp = strchr(hdrBuf, ' ');
      if (sp) code = atoi(sp + 1);
    }
    SERIAL_DBG("[GH-OTA] resolve GET > HTTP %d hdr=%u done=%d\n", code,
                  static_cast<unsigned>(hdrLen), hdrDone ? 1 : 0);

    char loc[1200];
    loc[0] = '\0';
    const char *pLoc = strstr(hdrBuf, "\nLocation:");
    if (!pLoc) pLoc = strstr(hdrBuf, "\nlocation:");
    if (pLoc) {
      pLoc = strchr(pLoc, ':');
      if (pLoc) {
        ++pLoc;
        while (*pLoc == ' ' || *pLoc == '\t') ++pLoc;
        size_t n = 0;
        while (pLoc[n] && pLoc[n] != '\r' && pLoc[n] != '\n' && n + 1 < sizeof(loc)) {
          loc[n] = pLoc[n];
          ++n;
        }
        loc[n] = '\0';
      }
    }
    SERIAL_DBG("[GH-OTA] resolve Location len=%u\n", static_cast<unsigned>(strlen(loc)));

    if ((code == 301 || code == 302 || code == 307 || code == 308) && strlen(loc) >= 12) {
      if (strlen(loc) + 1 <= outLen) {
        strlcpy(out, loc, outLen);
        SERIAL_DBG("[GH-OTA] resolve -> %.80s...\n", out);
        ok = true;
      } else {
        SERIAL_DBG("[GH-OTA] resolve Location too long %u >= %u\n",
                      static_cast<unsigned>(strlen(loc)), static_cast<unsigned>(outLen));
      }
    } else if (code == 200 || code == 206) {
      ok = true;  // no redirect needed
    } else if (code > 0) {
      SERIAL_DBG("[GH-OTA] resolve unexpected HTTP %d\n", code);
    }
    free(hdrBuf);
  } else {
    char errBuf[128];
    const int elen = client->lastError(errBuf, sizeof(errBuf));
    SERIAL_DBG("[GH-OTA] resolve TLS fail (%d): %s\n", elen, errBuf);
  }

  client->stop();
  delete client;
  if (!otaInProgress) {
    githubOtaReserveTlsHeap("resolve-done");
    githubOtaPauseWorkers(false);
  }
  return ok;
}

/** Raw HTTPS GET (no HTTPClient) — used for GitHub API JSON; HTTPClient truncates ~2–3KB. */
static int githubRawApiGetToFile(const char *url, const char *outPath, uint32_t timeoutMs,
                                 const char *acceptHdr) {
  if (!url || strncmp(url, "https://", 8) != 0) return -1;
  const char *p = url + 8;
  const char *slash = strchr(p, '/');
  if (!slash) return -1;
  char host[96];
  const size_t hostLen = static_cast<size_t>(slash - p);
  if (hostLen == 0 || hostLen >= sizeof(host)) return -1;
  memcpy(host, p, hostLen);
  host[hostLen] = '\0';
  const char *path = slash;

  for (int attempt = 1; attempt <= 3; attempt++) {
    githubOtaHeartbeat();
    githubOtaDefragHeap();
    SERIAL_DBG("[GH-OTA] raw API try %d/3 heap=%u maxBlk=%u\n", attempt,
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    otaConfigureTls(client);
    client->setHandshakeTimeout(30);
    client->setTimeout(timeoutMs > 5000 ? timeoutMs : 30000);

    if (!client->connect(host, 443)) {
      char errBuf[128];
      const int elen = client->lastError(errBuf, sizeof(errBuf));
      SERIAL_DBG("[GH-OTA] raw API TLS fail (%d): %s\n", elen, errBuf);
      client->stop();
      delete client;
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      continue;
    }

    client->print(String("GET ") + path + " HTTP/1.0\r\n" +
                  "Host: " + host + "\r\n" +
                  "User-Agent: kamaz-leveler/10.14.15\r\n" +
                  "Accept: " + String(acceptHdr ? acceptHdr : "application/vnd.github+json") +
                  "\r\n" +
                  "X-GitHub-Api-Version: 2022-11-28\r\n" +
                  "Accept-Encoding: identity\r\n" +
                  "Connection: close\r\n\r\n");

    // Status line
    String statusLine;
    const uint32_t t0 = millis();
    while (client->connected() && (millis() - t0) < 25000) {
      githubOtaHeartbeat();
      if (client->available()) {
        char c = static_cast<char>(client->read());
        if (c == '\n') break;
        if (c != '\r') statusLine += c;
      } else {
        delay(1);
      }
    }
    int code = -1;
    if (statusLine.startsWith("HTTP/")) {
      const int sp = statusLine.indexOf(' ');
      if (sp > 0) code = statusLine.substring(sp + 1).toInt();
    }
    SERIAL_DBG("[GH-OTA] raw API status %d (%s)\n", code, statusLine.c_str());

    int contentLen = -1;
    bool chunked = false;
    // Headers
    while ((millis() - t0) < timeoutMs) {
      githubOtaHeartbeat();
      String line;
      const uint32_t tLine = millis();
      while ((millis() - tLine) < 15000) {
        if (client->available()) {
          char c = static_cast<char>(client->read());
          if (c == '\n') break;
          if (c != '\r') line += c;
        } else if (!client->connected() && !client->available()) {
          break;
        } else {
          delay(1);
        }
      }
      if (line.length() == 0) break;
      String low = line;
      low.toLowerCase();
      if (low.startsWith("content-length:")) {
        contentLen = low.substring(15).toInt();
      } else if (low.startsWith("transfer-encoding:") && low.indexOf("chunked") >= 0) {
        chunked = true;
      }
    }
    SERIAL_DBG("[GH-OTA] raw API CL=%d chunked=%d heap=%u maxBlk=%u\n", contentLen,
                  chunked ? 1 : 0, static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));

    if (code != 200) {
      client->stop();
      delete client;
      if (code > 0) return code;
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      continue;
    }

    // Stream body → LittleFS (no large malloc; GitHub often omits CL / uses chunked).
    LittleFS.remove(outPath);
    File f = LittleFS.open(outPath, "w");
    if (!f) {
      SERIAL_DBGLN("[GH-OTA] raw API LittleFS open fail");
      client->stop();
      delete client;
      continue;
    }

    size_t total = 0;
    uint32_t lastDataMs = millis();
    uint8_t buf[512];
    bool wroteOk = true;

    if (chunked) {
      // Minimal chunked decoder: size line (hex), data, CRLF; ends at size 0.
      while ((millis() - t0) < timeoutMs) {
        githubOtaHeartbeat();
        String sizeLine;
        const uint32_t tSz = millis();
        while ((millis() - tSz) < 15000) {
          if (client->available()) {
            char c = static_cast<char>(client->read());
            if (c == '\n') break;
            if (c != '\r') sizeLine += c;
          } else if (!client->connected() && !client->available()) {
            break;
          } else {
            delay(1);
          }
        }
        // strip ;extensions
        const int semi = sizeLine.indexOf(';');
        if (semi >= 0) sizeLine = sizeLine.substring(0, semi);
        sizeLine.trim();
        if (sizeLine.length() == 0 && !client->connected()) break;
        char *endptr = nullptr;
        const unsigned long chunkSz = strtoul(sizeLine.c_str(), &endptr, 16);
        if (chunkSz == 0) {
          // consume trailing headers / final CRLF
          while (client->available()) (void)client->read();
          break;
        }
        size_t got = 0;
        while (got < chunkSz && (millis() - t0) < timeoutMs) {
          githubOtaHeartbeat();
          size_t want = chunkSz - got;
          if (want > sizeof(buf)) want = sizeof(buf);
          int n = 0;
          if (client->available()) {
            n = client->read(buf, want);
          } else if (!client->connected()) {
            break;
          } else {
            delay(1);
            continue;
          }
          if (n <= 0) {
            delay(1);
            continue;
          }
          if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
            wroteOk = false;
            break;
          }
          got += static_cast<size_t>(n);
          total += static_cast<size_t>(n);
          lastDataMs = millis();
        }
        // CRLF after chunk
        while (client->available()) {
          char c = static_cast<char>(client->read());
          if (c == '\n') break;
        }
        if (!wroteOk) break;
      }
    } else {
      while ((millis() - t0) < timeoutMs) {
        githubOtaHeartbeat();
        if (contentLen > 0 && static_cast<int>(total) >= contentLen) break;
        int avail = client->available();
        if (avail <= 0) {
          if (!client->connected() && (millis() - lastDataMs) > 500) break;
          if ((millis() - lastDataMs) > 25000) {
            SERIAL_DBGLN("[GH-OTA] raw API body stall");
            break;
          }
          delay(2);
          continue;
        }
        size_t want = static_cast<size_t>(avail);
        if (want > sizeof(buf)) want = sizeof(buf);
        if (contentLen > 0) {
          const size_t left = static_cast<size_t>(contentLen) - total;
          if (want > left) want = left;
        }
        const int n = client->read(buf, want);
        if (n > 0) {
          if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
            wroteOk = false;
            break;
          }
          total += static_cast<size_t>(n);
          lastDataMs = millis();
        } else {
          delay(2);
        }
      }
    }

    f.flush();
    f.close();
    client->stop();
    delete client;

    SERIAL_DBG("[GH-OTA] raw API body %u (CL=%d chunked=%d)\n", static_cast<unsigned>(total),
                  contentLen, chunked ? 1 : 0);
    if (!wroteOk || total < 16 || (contentLen > 0 && static_cast<int>(total) < contentLen)) {
      LittleFS.remove(outPath);
      vTaskDelay(pdMS_TO_TICKS(400 * attempt));
      continue;
    }
    SERIAL_DBG("[GH-OTA] saved %u байт > %s heap=%u\n", static_cast<unsigned>(total), outPath,
                  static_cast<unsigned>(ESP.getFreeHeap()));
    return HTTP_CODE_OK;
  }
  return -1;
}

int githubHttpsDownloadToFile(const char *url, const char *outPath,
                              uint32_t timeoutMs,
                              const char *acceptHdr,
                              const char *rangeHdr) {
  if (!url || !outPath) return -1;
  if (WiFi.status() != WL_CONNECTED) {
    SERIAL_DBGLN("[GH-OTA] download: WiFi не STA");
    return -1;
  }

  LittleFS.remove(outPath);
  WiFi.setSleep(false);

  static SemaphoreHandle_t s_ghMutex = nullptr;
  if (!s_ghMutex) s_ghMutex = xSemaphoreCreateMutex();
  if (s_ghMutex && xSemaphoreTake(s_ghMutex, pdMS_TO_TICKS(45000)) != pdTRUE) {
    SERIAL_DBGLN("[GH-OTA] download: mutex timeout");
    return -1;
  }

  const bool isApiJson =
      acceptHdr && (strstr(acceptHdr, "json") != nullptr) && strstr(url, "api.github.com");
  // Always pause workers for GitHub TLS — need ~40KB contiguous for mbedtls.
  const bool pauseWorkers = true;
  githubOtaReleaseTlsHeap("pre-GET");
  githubOtaPauseWorkers(true);
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(120));
  githubOtaDefragHeap();

  auto unlock = [&]() {
    if (pauseWorkers) {
      githubOtaPauseWorkers(false);
    }
    if (s_ghMutex) xSemaphoreGive(s_ghMutex);
  };

  if (isApiJson && !(rangeHdr && rangeHdr[0])) {
    SERIAL_DBG("[GH-OTA] TLS prep heap=%u maxBlk=%u internal=%u\n",
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()),
                  static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    {
      IPAddress ip;
      const bool dnsOk = WiFi.hostByName("api.github.com", ip);
      SERIAL_DBG("[GH-OTA] DNS api.github.com > %s (%s)\n",
                    dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
    }
    const int rawCode = githubRawApiGetToFile(url, outPath, timeoutMs, acceptHdr);
    if (rawCode == HTTP_CODE_OK) {
      unlock();
      return rawCode;
    }
    SERIAL_DBG("[GH-OTA] raw API fail code=%d — fallback HTTPClient\n", rawCode);
    githubOtaDefragHeap();
    vTaskDelay(pdMS_TO_TICKS(100));
    // fall through to HTTPClient path below
  }

  SERIAL_DBG("[GH-OTA] TLS prep heap=%u maxBlk=%u internal=%u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));

  {
    IPAddress ip;
    const bool dnsOk = WiFi.hostByName("api.github.com", ip);
    SERIAL_DBG("[GH-OTA] DNS api.github.com > %s (%s)\n",
                  dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  }

  int lastCode = -1;
  for (int attempt = 1; attempt <= 4; attempt++) {
    const uint32_t freeH = ESP.getFreeHeap();
    const uint32_t maxBlk = ESP.getMaxAllocHeap();
    SERIAL_DBG("[GH-OTA] GET try %d/4 heap=%u maxBlk=%u url=%s\n", attempt, freeH, maxBlk,
                  url);
    githubOtaDefragHeap();
    const uint32_t maxBlk2 = ESP.getMaxAllocHeap();
    if (maxBlk2 < maxBlk) {
      SERIAL_DBG("[GH-OTA] defrag maxBlk %u → %u\n", maxBlk, maxBlk2);
    }
    if (maxBlk2 < 24000u) {
      SERIAL_DBGLN("[GH-OTA] мало непрерывной RAM для TLS — пауза/defrag");
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      githubOtaDefragHeap();
      if (ESP.getMaxAllocHeap() < 20000u) {
        lastCode = -1;
        continue;
      }
    }

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      SERIAL_DBGLN("[GH-OTA] new WiFiClientSecure failed");
      lastCode = -1;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    otaConfigureTls(client);
    client->setHandshakeTimeout(30);
    client->setTimeout(timeoutMs);

    int code = -1;
    int contentLen = -1;
    size_t total = 0;
    bool wroteOk = false;

    {
      HTTPClient http;
      http.setConnectTimeout(20000);
      http.setTimeout(timeoutMs);
      if (!http.begin(*client, url)) {
        SERIAL_DBGLN("[GH-OTA] http.begin failed");
        lastCode = -1;
      } else {
        http.useHTTP10(true);
        http.setReuse(false);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.setUserAgent("kamaz-leveler/10.14.15");
        http.addHeader("Accept", acceptHdr ? acceptHdr : "*/*");
        http.addHeader("X-GitHub-Api-Version", "2022-11-28");
        http.addHeader("Accept-Encoding", "identity");
        http.addHeader("Connection", "close");
        if (rangeHdr && rangeHdr[0]) {
          http.addHeader("Range", rangeHdr);
          SERIAL_DBG("[GH-OTA] Range %s\n", rangeHdr);
        }

        SERIAL_DBGLN("[GH-OTA] GET…");
        const uint32_t tGet0 = millis();
        code = http.GET();
        contentLen = http.getSize();
        SERIAL_DBG("[GH-OTA] GET > HTTP %d (%s) size=%d in %lums heap=%u maxBlk=%u\n", code,
                      (code <= 0) ? HTTPClient::errorToString(code).c_str() : "ok", contentLen,
                      static_cast<unsigned long>(millis() - tGet0),
                      static_cast<unsigned>(ESP.getFreeHeap()),
                      static_cast<unsigned>(ESP.getMaxAllocHeap()));
        if (code <= 0) {
          char errBuf[128];
          const int elen = client->lastError(errBuf, sizeof(errBuf));
          SERIAL_DBG("[GH-OTA] TLS lastError(%d): %s\n", elen, errBuf);
        }
        lastCode = code;

        if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
          WiFiClient *stream = http.getStreamPtr();
          const uint32_t t0 = millis();
          uint32_t lastDataMs = millis();
          uint8_t buf[1024];
          const bool ranged = (rangeHdr && rangeHdr[0]);
          wroteOk = true;
          const bool smallApiBody = !ranged && contentLen > 0 && contentLen < 48000 &&
                                    acceptHdr && (strstr(acceptHdr, "json") != nullptr);

          if (smallApiBody) {
            // Full JSON into RAM first — LittleFS during TLS stalls the TCP window.
            githubOtaHeartbeat();
            String body;
            body.reserve(static_cast<unsigned>(contentLen) + 8);
            client->setTimeout(15000);
            while (static_cast<int>(body.length()) < contentLen &&
                   (millis() - t0) < timeoutMs) {
              githubOtaHeartbeat();
              const size_t left = static_cast<size_t>(contentLen) - body.length();
              const size_t want = left < sizeof(buf) ? left : sizeof(buf);
              const int n = stream->readBytes(buf, want);
              if (n > 0) {
                char tmp[sizeof(buf) + 1];
                memcpy(tmp, buf, static_cast<size_t>(n));
                tmp[n] = '\0';
                body += tmp;
                lastDataMs = millis();
              } else if (!client->connected()) {
                delay(40);
                if (!stream->available()) break;
              } else if ((millis() - lastDataMs) > 30000u && body.length() >= 16) {
                SERIAL_DBGLN("[GH-OTA] JSON body stall 30s");
                break;
              } else {
                delay(2);
              }
            }
            total = body.length();
            SERIAL_DBG("[GH-OTA] api body %u (CL=%d)\n", static_cast<unsigned>(total),
                          contentLen);
            if (contentLen > 0 && static_cast<int>(total) >= contentLen) {
              File f = LittleFS.open(outPath, "w");
              if (!f || f.write(reinterpret_cast<const uint8_t *>(body.c_str()), total) != total) {
                SERIAL_DBGLN("[GH-OTA] LittleFS write error");
                wroteOk = false;
                total = 0;
                if (f) f.close();
                LittleFS.remove(outPath);
              } else {
                f.flush();
                f.close();
              }
            } else {
              wroteOk = false;
              total = 0;
            }
          } else {
            File f = LittleFS.open(outPath, "w");
            if (!f) {
              SERIAL_DBGLN("[GH-OTA] LittleFS write open failed");
              lastCode = -1;
              wroteOk = false;
            } else if (ranged && contentLen > 0) {
              client->setTimeout(8000);
              while (static_cast<int>(total) < contentLen && (millis() - t0) < timeoutMs) {
                githubOtaHeartbeat();
                const size_t want =
                    min(sizeof(buf), static_cast<size_t>(contentLen) - total);
                const int n = stream->readBytes(buf, want);
                if (n <= 0) {
                  if (!client->connected() && total > 0) break;
                  delay(10);
                  continue;
                }
                if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                  SERIAL_DBGLN("[GH-OTA] LittleFS write error");
                  wroteOk = false;
                  break;
                }
                total += static_cast<size_t>(n);
                lastDataMs = millis();
              }
              f.flush();
              f.close();
              if (!wroteOk) LittleFS.remove(outPath);
            } else {
              client->setTimeout(8000);
              while ((millis() - t0) < timeoutMs) {
                githubOtaHeartbeat();
                if (contentLen > 0 && static_cast<int>(total) >= contentLen) break;
                size_t want = sizeof(buf);
                if (contentLen > 0) {
                  const size_t left = static_cast<size_t>(contentLen) - total;
                  if (left < want) want = left;
                }
                const int n = stream->readBytes(buf, want);
                if (n > 0) {
                  if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                    SERIAL_DBGLN("[GH-OTA] LittleFS write error");
                    wroteOk = false;
                    break;
                  }
                  total += static_cast<size_t>(n);
                  lastDataMs = millis();
                  continue;
                }
                const bool sockAlive = client->connected();
                const uint32_t quiet = millis() - lastDataMs;
                if (contentLen > 0) {
                  if (!sockAlive && quiet > 2500u) break;
                  if (sockAlive && quiet > 30000u) break;
                } else if (quiet > (sockAlive ? 8000u : 2000u) && total >= 16) {
                  break;
                }
                delay(5);
              }
              f.flush();
              f.close();
              if (!wroteOk) LittleFS.remove(outPath);
            }
          }
        }
        http.end();
      }
    }

    client->stop();
    delete client;
    client = nullptr;

    if (code > 0 && code != HTTP_CODE_OK && code != HTTP_CODE_PARTIAL_CONTENT) {
      unlock();
      return code;
    }
    if (wroteOk && (total >= 16 || ((rangeHdr && rangeHdr[0]) && total >= 1))) {
      if (contentLen > 0 && static_cast<int>(total) < contentLen) {
        SERIAL_DBG("[GH-OTA] incomplete body: got %u want %d — retry\n",
                      static_cast<unsigned>(total), contentLen);
        LittleFS.remove(outPath);
        lastCode = -1;
        vTaskDelay(pdMS_TO_TICKS(400 * attempt));
        continue;
      }
      SERIAL_DBG("[GH-OTA] saved %u байт > %s heap=%u\n", static_cast<unsigned>(total),
                    outPath, static_cast<unsigned>(ESP.getFreeHeap()));
      unlock();
      return (code == HTTP_CODE_PARTIAL_CONTENT) ? HTTP_CODE_PARTIAL_CONTENT : HTTP_CODE_OK;
    }

    LittleFS.remove(outPath);
    lastCode = (code > 0) ? code : -1;
    vTaskDelay(pdMS_TO_TICKS(400 * attempt));
  }

  unlock();
  return lastCode;
}

void githubOtaPingDiag() {
  SERIAL_DBGLN("[OTA-PING] begin");
  githubOtaPrintHeap("ping");
  SERIAL_DBG("[OTA-PING] wifi=%d IP=%s RSSI=%d\n", static_cast<int>(WiFi.status()),
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  IPAddress ip;
  const bool dnsOk = WiFi.hostByName("api.github.com", ip);
  SERIAL_DBG("[OTA-PING] DNS api.github.com > %s (%s)\n",
                dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  if (!dnsOk) {
    SERIAL_DBGLN("[OTA-PING] FAIL dns");
    return;
  }

  WiFiClient tcp;
  tcp.setTimeout(5000);
  const uint32_t tTcp = millis();
  const bool tcpOk = tcp.connect(ip, 443);
  SERIAL_DBG("[OTA-PING] TCP %s:443 > %s in %lums\n", ip.toString().c_str(),
                tcpOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTcp));
  if (tcpOk) tcp.stop();
  if (!tcpOk) {
    SERIAL_DBGLN("[OTA-PING] FAIL tcp");
    return;
  }

  WiFi.setSleep(false);
  githubOtaReleaseTlsHeap("ping-TLS");
  githubOtaPauseWorkers(true);
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(50));
  githubOtaPrintHeap("ping-pre-tls");

  WiFiClientSecure *sc = new (std::nothrow) WiFiClientSecure();
  if (!sc) {
    githubOtaReserveTlsHeap("ping-fail");
    githubOtaPauseWorkers(false);
    SERIAL_DBGLN("[OTA-PING] FAIL new secure");
    return;
  }
  otaConfigureTls(sc);
  sc->setHandshakeTimeout(30);
  sc->setTimeout(15000);
  const uint32_t tTls = millis();
  const bool tlsOk = sc->connect("api.github.com", 443);
  SERIAL_DBG("[OTA-PING] TLS api.github.com:443 > %s in %lums heap=%u\n",
                tlsOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTls),
                static_cast<unsigned>(ESP.getFreeHeap()));
  if (!tlsOk) {
    char errBuf[128];
    const int elen = sc->lastError(errBuf, sizeof(errBuf));
    SERIAL_DBG("[OTA-PING] TLS lastError(%d): %s\n", elen, errBuf);
  }
  if (tlsOk) {
    sc->print("GET /zen HTTP/1.0\r\nHost: api.github.com\r\nUser-Agent: kamaz-ping\r\n"
              "Connection: close\r\n\r\n");
    char line[128];
    size_t n = sc->readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = '\0';
    SERIAL_DBG("[OTA-PING] HTTP first-line: %s\n", line);
  }
  sc->stop();
  delete sc;
  githubOtaReserveTlsHeap("ping-post-tls");
  githubOtaPauseWorkers(false);

  constexpr char kUrl[] = "https://api.github.com/zen";
  constexpr char kPath[] = "/ota_ping.txt";
  const int code = githubHttpsDownloadToFile(kUrl, kPath, 20000, "*/*");
  if (code == HTTP_CODE_OK) {
    File f = LittleFS.open(kPath, "r");
    String body = f ? f.readString() : String();
    if (f) f.close();
    LittleFS.remove(kPath);
    SERIAL_DBG("[OTA-PING] HTTPS GET zen > HTTP %d body='%s'\n", code, body.c_str());
    SERIAL_DBGLN("[OTA-PING] OK");
  } else {
    SERIAL_DBG("[OTA-PING] HTTPS GET zen > HTTP %d FAIL\n", code);
  }
}
