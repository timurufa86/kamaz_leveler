#include "ota_net.h"
#include "app_globals.h"
#include "task_pool.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <Update.h>
#include <new>

#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define KAMAZ_OTA_CERT_BUNDLE_NET 0
#endif

void otaConfigureTls(WiFiClientSecure *client) {
  if (!client) return;
#if defined(KAMAZ_OTA_CERT_BUNDLE_NET) && KAMAZ_OTA_CERT_BUNDLE_NET
  extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
  extern const uint8_t rootca_crt_bundle_end[] asm("_binary_x509_crt_bundle_end");
  const size_t bundleLen = static_cast<size_t>(rootca_crt_bundle_end - rootca_crt_bundle_start);
  if (bundleLen > 0) {
    client->setCACertBundle(rootca_crt_bundle_start, bundleLen);
    return;
  }
#endif
  client->setInsecure();
}

void githubOtaDefragHeap() {
  void *hold[64];
  int n = 0;
  size_t sz = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  while (n < 64 && sz >= 1024) {
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

static void *s_tlsHeapReserve = nullptr;
static constexpr size_t kTlsHeapReserveBytes = 72 * 1024;

void githubOtaReserveTlsHeap(const char *why) {
  if (s_tlsHeapReserve) return;
  githubOtaDefragHeap();

  size_t maxBlk = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (maxBlk < 28 * 1024) {
    Serial.printf("[GH-OTA] TLS-reserve skip (%s) maxBlk=%u\n", why ? why : "?",
                  static_cast<unsigned>(maxBlk));
    return;
  }
  size_t want = (maxBlk > 1024) ? (maxBlk - 1024) : maxBlk;
  if (want > kTlsHeapReserveBytes) want = kTlsHeapReserveBytes;
  while (want >= 28 * 1024) {
    s_tlsHeapReserve = heap_caps_malloc(want, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (s_tlsHeapReserve) {
      Serial.printf("[GH-OTA] TLS-reserve +%u (%s) free=%u maxBlk=%u\n",
                    static_cast<unsigned>(want), why ? why : "?",
                    static_cast<unsigned>(ESP.getFreeHeap()),
                    static_cast<unsigned>(ESP.getMaxAllocHeap()));
      return;
    }
    want -= 1024;
  }
  Serial.printf("[GH-OTA] TLS-reserve FAIL (%s) free=%u maxBlk=%u\n", why ? why : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));
}

void githubOtaReleaseTlsHeap(const char *why) {
  if (!s_tlsHeapReserve) return;
  heap_caps_free(s_tlsHeapReserve);
  s_tlsHeapReserve = nullptr;
  Serial.printf("[GH-OTA] TLS-reserve free (%s) free=%u maxBlk=%u\n", why ? why : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));
}

volatile bool s_otaWorkersPaused = false;

void githubOtaPauseWorkers(bool pause) {
  s_otaWorkersPaused = pause;
  const uint8_t idxs[] = {taskIndex_Display, taskIndex_IMU, taskIndex_Control,
                          taskIndex_Valve,   taskIndex_Pressure, taskIndex_ErrRec,
                          taskIndex_Button};
  for (uint8_t idx : idxs) {
    if (idx == 0xFF) continue;
    if (pause) TaskPool::suspendTask(idx);
    else TaskPool::resumeTask(idx);
  }
}

void githubOtaPrintHeap(const char *tag) {
  Serial.printf("[OTA-HEAP] %s free=%u maxBlk=%u int=%u\n", tag ? tag : "?",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
}

void githubOtaHeartbeat() {
  extern uint8_t taskIndex_OTA;
  TaskPool::markRun(taskIndex_OTA);
  /* TaskMonitor::updateTaskStatus is in .ino; heartbeat from module just marks run. */
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
    Serial.printf("[GH-OTA] abort esp_ota (%s) handle=%u\n", why ? why : "?",
                  static_cast<unsigned>(s_ghOtaHandle));
    esp_ota_abort(s_ghOtaHandle);
    s_ghOtaActive = false;
    s_ghOtaHandle = 0;
    s_ghOtaPart = nullptr;
  }
  if (Update.isRunning()) {
    Serial.printf("[GH-OTA] abort Update (%s)\n", why ? why : "?");
    Update.abort();
  }
  Update.clearError();
}

void githubOtaEndInstallSession(const char *why) {
  otaInProgress = false;
  githubOtaReserveTlsHeap(why ? why : "ota-end");
  githubOtaPauseWorkers(false);
}

int githubHttpsDownloadToFile(const char *url, const char *outPath,
                              uint32_t timeoutMs,
                              const char *acceptHdr,
                              const char *rangeHdr) {
  if (!url || !outPath) return -1;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] download: WiFi не STA");
    return -1;
  }

  LittleFS.remove(outPath);
  WiFi.setSleep(false);

  static SemaphoreHandle_t s_ghMutex = nullptr;
  if (!s_ghMutex) s_ghMutex = xSemaphoreCreateMutex();
  if (s_ghMutex && xSemaphoreTake(s_ghMutex, pdMS_TO_TICKS(45000)) != pdTRUE) {
    Serial.println("[GH-OTA] download: mutex timeout");
    return -1;
  }

  githubOtaReleaseTlsHeap("pre-GET");
  if (!otaInProgress) {
    githubOtaPauseWorkers(true);
  } else if (!s_tlsHeapReserve) {
    githubOtaPauseWorkers(true);
  }
  githubOtaDefragHeap();
  vTaskDelay(pdMS_TO_TICKS(80));

  auto unlock = [&]() {
    if (otaInProgress) {
      if (s_ghMutex) xSemaphoreGive(s_ghMutex);
      return;
    }
    githubOtaPauseWorkers(false);
    if (s_ghMutex) xSemaphoreGive(s_ghMutex);
  };

  Serial.printf("[GH-OTA] TLS prep heap=%u maxBlk=%u internal=%u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()),
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));

  {
    IPAddress ip;
    const bool dnsOk = WiFi.hostByName("api.github.com", ip);
    Serial.printf("[GH-OTA] DNS api.github.com > %s (%s)\n",
                  dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  }

  int lastCode = -1;
  for (int attempt = 1; attempt <= 4; attempt++) {
    const uint32_t freeH = ESP.getFreeHeap();
    const uint32_t maxBlk = ESP.getMaxAllocHeap();
    Serial.printf("[GH-OTA] GET try %d/4 heap=%u maxBlk=%u url=%s\n", attempt, freeH, maxBlk,
                  url);
    githubOtaDefragHeap();
    const uint32_t maxBlk2 = ESP.getMaxAllocHeap();
    if (maxBlk2 < maxBlk) {
      Serial.printf("[GH-OTA] defrag maxBlk %u → %u\n", maxBlk, maxBlk2);
    }
    if (maxBlk2 < 24000u) {
      Serial.println("[GH-OTA] мало непрерывной RAM для TLS — пауза/defrag");
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      githubOtaDefragHeap();
      if (ESP.getMaxAllocHeap() < 20000u) {
        lastCode = -1;
        continue;
      }
    }

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      Serial.println("[GH-OTA] new WiFiClientSecure failed");
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
        Serial.println("[GH-OTA] http.begin failed");
        lastCode = -1;
      } else {
        http.useHTTP10(true);
        http.setReuse(false);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.addHeader("User-Agent", "kamaz-leveler/9.4.0");
        http.addHeader("Accept", acceptHdr ? acceptHdr : "*/*");
        http.addHeader("Accept-Encoding", "identity");
        http.addHeader("Connection", "close");
        if (rangeHdr && rangeHdr[0]) {
          http.addHeader("Range", rangeHdr);
          Serial.printf("[GH-OTA] Range %s\n", rangeHdr);
        }

        Serial.println("[GH-OTA] GET…");
        const uint32_t tGet0 = millis();
        code = http.GET();
        contentLen = http.getSize();
        Serial.printf("[GH-OTA] GET > HTTP %d (%s) size=%d in %lums heap=%u maxBlk=%u\n", code,
                      (code <= 0) ? HTTPClient::errorToString(code).c_str() : "ok", contentLen,
                      static_cast<unsigned long>(millis() - tGet0),
                      static_cast<unsigned>(ESP.getFreeHeap()),
                      static_cast<unsigned>(ESP.getMaxAllocHeap()));
        if (code <= 0) {
          char errBuf[128];
          const int elen = client->lastError(errBuf, sizeof(errBuf));
          Serial.printf("[GH-OTA] TLS lastError(%d): %s\n", elen, errBuf);
        }
        lastCode = code;

        if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
          File f = LittleFS.open(outPath, "w");
          if (!f) {
            Serial.println("[GH-OTA] LittleFS write open failed");
            lastCode = -1;
          } else {
            WiFiClient *stream = http.getStreamPtr();
            const uint32_t t0 = millis();
            uint32_t lastDataMs = millis();
            uint8_t buf[512];
            const bool ranged = (rangeHdr && rangeHdr[0]);
            wroteOk = true;
            if (ranged && contentLen > 0) {
              client->setTimeout(8000);
              while (static_cast<int>(total) < contentLen && (millis() - t0) < timeoutMs) {
                const size_t want =
                    min(sizeof(buf), static_cast<size_t>(contentLen) - total);
                const int n = stream->readBytes(buf, want);
                if (n <= 0) {
                  if (!client->connected() && total > 0) break;
                  delay(10);
                  continue;
                }
                if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                  Serial.println("[GH-OTA] LittleFS write error");
                  wroteOk = false;
                  break;
                }
                total += static_cast<size_t>(n);
                lastDataMs = millis();
              }
            } else {
              while ((millis() - t0) < timeoutMs) {
                int avail = stream->available();
                if (avail <= 0) {
                  const bool sockAlive = client->connected();
                  const uint32_t quiet = millis() - lastDataMs;
                  if (contentLen > 0 && static_cast<int>(total) >= contentLen) break;
                  if (quiet > (sockAlive ? 5000u : 1500u) && total >= 16) break;
                  delay(5);
                  continue;
                }
                while (stream->available()) {
                  const int n = stream->readBytes(buf, sizeof(buf));
                  if (n <= 0) break;
                  if (f.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
                    Serial.println("[GH-OTA] LittleFS write error");
                    wroteOk = false;
                    goto gh_rw_done;
                  }
                  total += static_cast<size_t>(n);
                  lastDataMs = millis();
                  if (contentLen > 0 && static_cast<int>(total) >= contentLen) goto gh_rw_done;
                }
              }
            }
          gh_rw_done:
            f.flush();
            f.close();
            if (!wroteOk) {
              LittleFS.remove(outPath);
              lastCode = -1;
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
      const bool ranged = (rangeHdr && rangeHdr[0]);
      if (ranged && contentLen > 0 && static_cast<int>(total) < contentLen) {
        Serial.printf("[GH-OTA] incomplete range: got %u want %d — retry\n",
                      static_cast<unsigned>(total), contentLen);
        LittleFS.remove(outPath);
        lastCode = -1;
        vTaskDelay(pdMS_TO_TICKS(400 * attempt));
        continue;
      }
      Serial.printf("[GH-OTA] saved %u байт > %s heap=%u\n", static_cast<unsigned>(total),
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
  Serial.println("[OTA-PING] begin");
  githubOtaPrintHeap("ping");
  Serial.printf("[OTA-PING] wifi=%d IP=%s RSSI=%d\n", static_cast<int>(WiFi.status()),
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  IPAddress ip;
  const bool dnsOk = WiFi.hostByName("api.github.com", ip);
  Serial.printf("[OTA-PING] DNS api.github.com > %s (%s)\n",
                dnsOk ? ip.toString().c_str() : "FAIL", dnsOk ? "ok" : "fail");
  if (!dnsOk) {
    Serial.println("[OTA-PING] FAIL dns");
    return;
  }

  WiFiClient tcp;
  tcp.setTimeout(5000);
  const uint32_t tTcp = millis();
  const bool tcpOk = tcp.connect(ip, 443);
  Serial.printf("[OTA-PING] TCP %s:443 > %s in %lums\n", ip.toString().c_str(),
                tcpOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTcp));
  if (tcpOk) tcp.stop();
  if (!tcpOk) {
    Serial.println("[OTA-PING] FAIL tcp");
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
    Serial.println("[OTA-PING] FAIL new secure");
    return;
  }
  otaConfigureTls(sc);
  sc->setHandshakeTimeout(30);
  sc->setTimeout(15000);
  const uint32_t tTls = millis();
  const bool tlsOk = sc->connect("api.github.com", 443);
  Serial.printf("[OTA-PING] TLS api.github.com:443 > %s in %lums heap=%u\n",
                tlsOk ? "OK" : "FAIL", static_cast<unsigned long>(millis() - tTls),
                static_cast<unsigned>(ESP.getFreeHeap()));
  if (!tlsOk) {
    char errBuf[128];
    const int elen = sc->lastError(errBuf, sizeof(errBuf));
    Serial.printf("[OTA-PING] TLS lastError(%d): %s\n", elen, errBuf);
  }
  if (tlsOk) {
    sc->print("GET /zen HTTP/1.0\r\nHost: api.github.com\r\nUser-Agent: kamaz-ping\r\n"
              "Connection: close\r\n\r\n");
    char line[128];
    size_t n = sc->readBytesUntil('\n', line, sizeof(line) - 1);
    line[n] = '\0';
    Serial.printf("[OTA-PING] HTTP first-line: %s\n", line);
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
    Serial.printf("[OTA-PING] HTTPS GET zen > HTTP %d body='%s'\n", code, body.c_str());
    Serial.println("[OTA-PING] OK");
  } else {
    Serial.printf("[OTA-PING] HTTPS GET zen > HTTP %d FAIL\n", code);
  }
}
