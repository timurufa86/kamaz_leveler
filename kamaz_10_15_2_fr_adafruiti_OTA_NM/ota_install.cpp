#include "ota_install.h"
#include "ota_net.h"
#include "ota_list.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_version.h"
#include "semver_utils.h"
#include "logger.h"
#include "valve_ctrl.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>
#include <new>

#ifndef HTTP_CODE_OK
#define HTTP_CODE_OK 200
#endif
#ifndef HTTP_CODE_PARTIAL_CONTENT
#define HTTP_CODE_PARTIAL_CONTENT 206
#endif

void emergencyStop();  // defined in .ino

/** Resolve github.com/releases/download → CDN Location; also probe Content-Length. */
static bool githubProbeDirectUrl(const char *url, char *finalUrl, size_t finalUrlLen,
                                 int *contentLen) {
  if (!url || !finalUrl || finalUrlLen < 24 || !contentLen) return false;
  strlcpy(finalUrl, url, finalUrlLen);
  *contentLen = -1;

  auto headOnce = [&](const char *u, bool follow, int *codeOut, int *sizeOut,
                      char *locOut, size_t locLen) -> bool {
    githubOtaHeartbeat();
    WiFiClientSecure *probe = new (std::nothrow) WiFiClientSecure();
    if (!probe) return false;
    otaConfigureTls(probe);
    probe->setHandshakeTimeout(30);
    probe->setTimeout(30000);
    HTTPClient http;
    http.setConnectTimeout(20000);
    http.setTimeout(30000);
    http.setReuse(false);
    http.setFollowRedirects(follow ? HTTPC_STRICT_FOLLOW_REDIRECTS
                                   : HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.setUserAgent("kamaz-leveler/10.14.15");
    if (!http.begin(*probe, u)) {
      delete probe;
      return false;
    }
    http.addHeader("Accept", "*/*");
    http.addHeader("Connection", "close");
    const int code = http.sendRequest("HEAD");
    if (codeOut) *codeOut = code;
    if (sizeOut) *sizeOut = http.getSize();
    if (locOut && locLen) {
      locOut[0] = '\0';
      const String &loc = http.getLocation();
      if (loc.length() > 8 && loc.length() + 1 < locLen) {
        strlcpy(locOut, loc.c_str(), locLen);
      }
    }
    SERIAL_DBG("[GH-OTA] HTTP HEAD > %d size=%d follow=%d heap=%u maxBlk=%u\n", code,
                  sizeOut ? *sizeOut : -1, follow ? 1 : 0,
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    http.end();
    probe->stop();
    delete probe;
    githubOtaDefragHeap();
    githubOtaHeartbeat();
    return true;
  };

  // Prefer no-follow so we can keep the long CDN URL after TLS is dropped.
  int code = 0, size = -1;
  char loc[1400];
  loc[0] = '\0';
  if (!headOnce(url, false, &code, &size, loc, sizeof(loc))) return false;

  if ((code == 301 || code == 302 || code == 303 || code == 307 || code == 308) &&
      loc[0] != '\0') {
    strlcpy(finalUrl, loc, finalUrlLen);
    SERIAL_DBG("[GH-OTA] CDN url len=%u\n", static_cast<unsigned>(strlen(finalUrl)));
    // Size from GET Content-Length — CDN HEAD often fails (-1) and wastes a TLS round.
  } else if (code == HTTP_CODE_OK && size > 100000) {
    *contentLen = size;
  }
  return finalUrl[0] != '\0';
}

static bool parseHttpsUrl(const char *url, char *host, size_t hostLen, char *path, size_t pathLen,
                          uint16_t *portOut) {
  if (!url || !host || !path || hostLen < 8 || pathLen < 2) return false;
  if (strncmp(url, "https://", 8) != 0) return false;
  const char *p = url + 8;
  const char *slash = strchr(p, '/');
  const char *colon = nullptr;
  size_t hlen = 0;
  if (slash) {
    colon = (const char *)memchr(p, ':', static_cast<size_t>(slash - p));
    hlen = static_cast<size_t>((colon ? colon : slash) - p);
  } else {
    colon = strchr(p, ':');
    hlen = strlen(p);
    if (colon) hlen = static_cast<size_t>(colon - p);
  }
  if (hlen == 0 || hlen >= hostLen) return false;
  memcpy(host, p, hlen);
  host[hlen] = '\0';
  uint16_t port = 443;
  if (colon && (!slash || colon < slash)) {
    port = static_cast<uint16_t>(atoi(colon + 1));
    if (port == 0) port = 443;
  }
  if (portOut) *portOut = port;
  if (!slash) {
    strlcpy(path, "/", pathLen);
  } else {
    strlcpy(path, slash, pathLen);
  }
  return path[0] == '/';
}

/** After Update.begin: raw TLS GET (no HTTPClient) — less peak RAM than HTTPClient::GET. */
static size_t rawHttpsStreamToUpdate(const char *url, int expectLen, mbedtls_sha256_context *sha) {
  if (!url || !sha) return 0;
  char host[128];
  static char path[1400];
  uint16_t port = 443;
  if (!parseHttpsUrl(url, host, sizeof(host), path, sizeof(path), &port)) {
    SERIAL_DBGLN("[GH-OTA] bad HTTPS url");
    return 0;
  }

  WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
  if (!client) return 0;
  otaConfigureTls(client);
  client->setHandshakeTimeout(30);
  client->setTimeout(120000);

  SERIAL_DBG("[GH-OTA] raw connect %s:%u heap=%u maxBlk=%u\n", host, static_cast<unsigned>(port),
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));
  githubOtaHeartbeat();
  if (!client->connect(host, port)) {
    SERIAL_DBG("[GH-OTA] raw connect fail wifi=%d\n", static_cast<int>(WiFi.status()));
    delete client;
    return 0;
  }

  client->print("GET ");
  client->print(path);
  client->print(" HTTP/1.0\r\nHost: ");
  client->print(host);
  client->print("\r\nUser-Agent: kamaz-leveler/10.14.15\r\nAccept: */*\r\n");
  client->print("Accept-Encoding: identity\r\nConnection: close\r\n\r\n");

  char line[192];
  size_t li = 0;
  int status = 0;
  int contentLen = expectLen;
  bool headersDone = false;
  const uint32_t t0 = millis();

  auto readLine = [&]() -> bool {
    li = 0;
    while ((millis() - t0) < 60000UL && li + 1 < sizeof(line)) {
      githubOtaHeartbeat();
      if (!client->available()) {
        if (!client->connected()) break;
        delay(2);
        continue;
      }
      const int ch = client->read();
      if (ch < 0) break;
      if (ch == '\n') {
        line[li] = '\0';
        if (li > 0 && line[li - 1] == '\r') line[--li] = '\0';
        return true;
      }
      line[li++] = static_cast<char>(ch);
    }
    line[li] = '\0';
    return li > 0;
  };

  if (!readLine()) {
    SERIAL_DBGLN("[GH-OTA] raw status timeout");
    client->stop();
    delete client;
    return 0;
  }
  if (sscanf(line, "HTTP/%*s %d", &status) != 1) {
    SERIAL_DBG("[GH-OTA] raw bad status: %.40s\n", line);
    client->stop();
    delete client;
    return 0;
  }
  while (readLine()) {
    if (line[0] == '\0') {
      headersDone = true;
      break;
    }
    if (strncasecmp(line, "Content-Length:", 15) == 0) {
      contentLen = atoi(line + 15);
    }
  }
  SERIAL_DBG("[GH-OTA] raw GET > %d size=%d hdrOk=%d heap=%u\n", status, contentLen,
                headersDone ? 1 : 0, static_cast<unsigned>(ESP.getFreeHeap()));
  if (status != HTTP_CODE_OK || contentLen < 100000) {
    client->stop();
    delete client;
    return 0;
  }

  uint8_t buf[1024];
  size_t written = 0;
  int lastPct = -1;
  uint32_t lastDataMs = millis();
  bool ok = true;
  while ((millis() - t0) < 360000UL) {
    githubOtaHeartbeat();
    if (static_cast<int>(written) >= contentLen) break;
    size_t want = sizeof(buf);
    const size_t left = static_cast<size_t>(contentLen) - written;
    if (left < want) want = left;
    const int n = client->readBytes(buf, want);
    if (n > 0) {
      if (Update.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
        SERIAL_DBG("[GH-OTA] Update.write fail: %s\n", Update.errorString());
        ok = false;
        break;
      }
      mbedtls_sha256_update(sha, buf, static_cast<size_t>(n));
      written += static_cast<size_t>(n);
      lastDataMs = millis();
      extern int otaProgress;
      otaProgress = static_cast<int>((written * 100u) / static_cast<size_t>(contentLen));
      if (otaProgress > 99) otaProgress = 99;
      if (otaProgress != lastPct && ((otaProgress % 5) == 0 || otaProgress >= 99)) {
        lastPct = otaProgress;
        displayDirty = true;
        SERIAL_DBG("[GH-OTA] download %d%% (%u) heap=%u\n", otaProgress,
                      static_cast<unsigned>(written),
                      static_cast<unsigned>(ESP.getFreeHeap()));
      }
    } else if (!client->connected() && !client->available()) {
      delay(40);
      if (!client->available()) break;
    } else if ((millis() - lastDataMs) > 30000u && written >= 16) {
      SERIAL_DBGLN("[GH-OTA] raw stall 30s");
      break;
    } else {
      delay(2);
    }
  }
  client->stop();
  delete client;
  if (!ok || written < 100000u) {
    SERIAL_DBG("[GH-OTA] raw incomplete written=%u\n", static_cast<unsigned>(written));
    return 0;
  }
  if (static_cast<int>(written) < contentLen) {
    SERIAL_DBG("[GH-OTA] raw short %u/%d\n", static_cast<unsigned>(written), contentLen);
    return 0;
  }
  return written;
}

/**
 * One TLS session + HTTP/1.1 keep-alive Range into a static 16KB buffer.
 * esp_ota_write only runs between chunks (socket idle) — flash ops must not
 * run while TLS is receiving, or WiFi drops on this no-PSRAM chip.
 *
 * 10.15.2: the CDN (release-assets.githubusercontent.com) closes long keep-alive
 * sessions mid-download (observed after ~540KB). A dropped socket is now
 * recovered by reconnecting and resuming from the byte offset already written;
 * before this, the whole attempt was aborted and restarted from zero, so a
 * 1.6MB image never completed. Chunks are also 16KB instead of 4KB (4x fewer
 * requests per image), and the resolved CDN URL is reused on the retry attempt
 * instead of asking GitHub for a fresh redirect.
 */
static size_t httpFollowStreamToUpdate(const char *url, mbedtls_sha256_context *sha,
                                       uint32_t knownSize) {
  if (!url || !sha) return 0;

  static char directUrl[1400];
  static constexpr size_t kChunk = 16 * 1024;
  static uint8_t s_chunk[kChunk];

  bool haveDirect = false;
  int probedLen = -1;
  for (int attempt = 1; attempt <= 2; attempt++) {
    githubOtaDefragHeap();
    githubOtaHeartbeat();
    SERIAL_DBG("[GH-OTA] keep-alive range try %d/2\n", attempt);

    if (!haveDirect) {
      if (!githubProbeDirectUrl(url, directUrl, sizeof(directUrl), &probedLen)) {
        vTaskDelay(pdMS_TO_TICKS(300 * attempt));
        continue;
      }
      haveDirect = true;
    }

    size_t total = 0;
    if (knownSize > 100000u) total = knownSize;
    else if (probedLen > 100000) total = static_cast<size_t>(probedLen);
    if (total < 100000u) {
      SERIAL_DBGLN("[GH-OTA] no known size for Range");
      continue;
    }

    char host[128];
    static char path[1400];
    uint16_t port = 443;
    if (!parseHttpsUrl(directUrl, host, sizeof(host), path, sizeof(path), &port)) {
      SERIAL_DBGLN("[GH-OTA] bad CDN url");
      continue;
    }

    githubOtaClearUpdate("pre-ota-begin");
    if (Update.isRunning()) Update.abort();
    Update.clearError();
    githubOtaDefragHeap();

    const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
    if (!part || total > part->size) {
      SERIAL_DBGLN("[GH-OTA] bad OTA partition/size");
      continue;
    }

    SERIAL_DBG("[GH-OTA] pre-esp_ota_begin size=%u heap=%u maxBlk=%u\n",
                  static_cast<unsigned>(total), static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    esp_ota_handle_t handle = 0;
    const uint32_t eraseT0 = millis();
    const esp_err_t be = esp_ota_begin(part, total, &handle);
    SERIAL_DBG("[GH-OTA] esp_ota_begin > %s in %lums heap=%u\n", esp_err_to_name(be),
                  static_cast<unsigned long>(millis() - eraseT0),
                  static_cast<unsigned>(ESP.getFreeHeap()));
    if (be != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(300 * attempt));
      continue;
    }
    setGhOtaPart(part);
    setGhOtaHandle(handle);
    setGhOtaActive(true);
    githubOtaDefragHeap();

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      githubOtaClearUpdate("no-client");
      continue;
    }
    otaConfigureTls(client);
    client->setHandshakeTimeout(30);
    client->setTimeout(60000);

    SERIAL_DBG("[GH-OTA] TLS connect %s heap=%u maxBlk=%u\n", host,
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    {
      IPAddress diagIp;
      const bool dnsOk = WiFi.hostByName(host, diagIp);
      SERIAL_DBG("[GH-OTA] DIAG dns %s > %s\n", host,
                    dnsOk ? diagIp.toString().c_str() : "FAIL");
      if (dnsOk) {
        WiFiClient tcpDiag;
        tcpDiag.setTimeout(5000);
        const uint32_t tTcp = millis();
        const bool tcpOk = tcpDiag.connect(diagIp, port);
        SERIAL_DBG("[GH-OTA] DIAG tcp %s:%u > %s in %lums\n",
                      diagIp.toString().c_str(), static_cast<unsigned>(port),
                      tcpOk ? "OK" : "FAIL",
                      static_cast<unsigned long>(millis() - tTcp));
        if (tcpOk) tcpDiag.stop();
      }
    }
    if (!client->connect(host, port)) {
      {
        char errBuf[128];
        const int elen = client->lastError(errBuf, sizeof(errBuf));
        SERIAL_DBG("[GH-OTA] DIAG tls lastError(%d)=%s\n", elen, errBuf);
      }
      SERIAL_DBG("[GH-OTA] TLS connect fail wifi=%d\n", static_cast<int>(WiFi.status()));
      delete client;
      githubOtaClearUpdate("tls-fail");
      vTaskDelay(pdMS_TO_TICKS(400 * attempt));
      continue;
    }

    size_t written = 0;
    int lastPct = -1;
    bool ok = true;
    int recoveries = 0;
    const uint32_t t0 = millis();
    char line[192];

    auto readLine = [&]() -> bool {
      size_t li = 0;
      const uint32_t lineT0 = millis();
      while ((millis() - lineT0) < 30000UL && li + 1 < sizeof(line)) {
        githubOtaHeartbeat();
        if (!client->available()) {
          if (!client->connected()) break;
          delay(2);
          continue;
        }
        const int ch = client->read();
        if (ch < 0) break;
        if (ch == '\n') {
          line[li] = '\0';
          if (li > 0 && line[li - 1] == '\r') line[--li] = '\0';
          return true;
        }
        line[li++] = static_cast<char>(ch);
      }
      line[li] = '\0';
      return li > 0;
    };

    while (written < total && ok && (millis() - t0) < 600000UL) {
      githubOtaHeartbeat();
      if (!client->connected()) {
        SERIAL_DBGLN("[GH-OTA] TLS dropped — reconnect");
        client->stop();
        githubOtaDefragHeap();
        bool reconnected = false;
        for (int r = 1; r <= 3 && !reconnected; r++) {
          if (recoveries >= 60) break;
          ++recoveries;
          vTaskDelay(pdMS_TO_TICKS(200 * r));
          reconnected = client->connect(host, port);
        }
        if (!reconnected) {
          ok = false;
          break;
        }
      }

      const size_t end = (written + kChunk - 1 < total) ? (written + kChunk - 1) : (total - 1);
      const size_t need = end - written + 1;

      client->print("GET ");
      client->print(path);
      client->print(" HTTP/1.1\r\nHost: ");
      client->print(host);
      client->printf("\r\nRange: bytes=%u-%u\r\n", static_cast<unsigned>(written),
                     static_cast<unsigned>(end));
      client->print("User-Agent: kamaz-leveler/10.14.15\r\nAccept: */*\r\n");
      client->print("Accept-Encoding: identity\r\nConnection: keep-alive\r\n\r\n");

      int status = 0;
      int contentLen = -1;
      if (!readLine() || sscanf(line, "HTTP/%*s %d", &status) != 1) {
        // 10.15.2: the CDN drops the keep-alive socket between chunks. That is
        // recoverable — reconnect and resume from `written` (see top of loop).
        SERIAL_DBG("[GH-OTA] bad status at %u (conn=%d) — resume\n",
                      static_cast<unsigned>(written), client->connected() ? 1 : 0);
        client->stop();
        if (++recoveries > 60) {
          ok = false;
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      while (readLine()) {
        if (line[0] == '\0') break;
        if (strncasecmp(line, "Content-Length:", 15) == 0) contentLen = atoi(line + 15);
      }
      if ((written % (kChunk * 8)) == 0) {
        SERIAL_DBG("[GH-OTA] Range %u-%u > %d cl=%d heap=%u\n",
                      static_cast<unsigned>(written), static_cast<unsigned>(end), status,
                      contentLen, static_cast<unsigned>(ESP.getFreeHeap()));
      }
      if (status != HTTP_CODE_OK && status != HTTP_CODE_PARTIAL_CONTENT) {
        SERIAL_DBG("[GH-OTA] Range HTTP %d\n", status);
        ok = false;
        break;
      }
      if (contentLen > 0 && static_cast<size_t>(contentLen) != need) {
        // Some CDNs omit CL; still read exactly `need` for 206.
      }

      size_t got = 0;
      uint32_t lastDataMs = millis();
      while (got < need && (millis() - lastDataMs) < 45000u) {
        githubOtaHeartbeat();
        size_t want = need - got;
        if (want > 1024) want = 1024;
        const int n = client->readBytes(s_chunk + got, want);
        if (n > 0) {
          got += static_cast<size_t>(n);
          lastDataMs = millis();
        } else if (!client->connected() && !client->available()) {
          delay(15);
          if (!client->available()) break;
        } else {
          delay(1);
        }
      }
      if (got != need) {
        // 10.15.2: treat a short chunk as a dropped socket too — discard the
        // partial chunk (nothing was written to flash yet) and resume.
        SERIAL_DBG("[GH-OTA] chunk short got=%u need=%u at %u — resume\n",
                      static_cast<unsigned>(got), static_cast<unsigned>(need),
                      static_cast<unsigned>(written));
        client->stop();
        if (++recoveries > 60) {
          ok = false;
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      // Flash write while TLS is idle (not receiving).
      const esp_err_t we = esp_ota_write(handle, s_chunk, need);
      if (we != ESP_OK) {
        SERIAL_DBG("[GH-OTA] esp_ota_write fail: %s\n", esp_err_to_name(we));
        ok = false;
        break;
      }
      mbedtls_sha256_update(sha, s_chunk, need);
      written += need;

      extern int otaProgress;
      otaProgress = static_cast<int>((written * 100u) / total);
      if (otaProgress > 99) otaProgress = 99;
      if (otaProgress != lastPct && ((otaProgress % 5) == 0 || otaProgress >= 99)) {
        lastPct = otaProgress;
        displayDirty = true;
        SERIAL_DBG("[GH-OTA] download %d%% (%u)\n", otaProgress,
                      static_cast<unsigned>(written));
      }
    }

    client->stop();
    delete client;

    if (!ok || written < total) {
      SERIAL_DBG("[GH-OTA] range incomplete written=%u/%u\n",
                    static_cast<unsigned>(written), static_cast<unsigned>(total));
      githubOtaClearUpdate("range-fail");
      vTaskDelay(pdMS_TO_TICKS(400 * attempt));
      continue;
    }
    Serial.printf("[GH-OTA] range OK %u\n", static_cast<unsigned>(written));
    return written;
  }
  return 0;
}

/* 10.15.0: функция httpUpdateFallback() удалена — она ставила образ через ядро
   HTTPUpdate без сверки SHA-256 (ядро проверяет только x-MD5, которого GitHub
   не отдаёт) и сразу активировала его. Верифицируемый путь остаётся единственным. */

bool installGitHubReleaseIndex(int8_t idx) {
  if (idx < 0 || idx >= static_cast<int8_t>(otaReleaseCount)) return false;
  const OtaRelease &r = otaReleases[idx];
  if (r.binUrl[0] == '\0') return false;
  if (r.sha256[0] == '\0' && r.shaUrl[0] == '\0') return false;

  SERIAL_DBG("[GH-OTA] Install release %s\n", r.tag);
  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(r.binUrl, r.shaUrl[0] ? r.shaUrl : nullptr, r.tag,
                              r.sha256[0] ? r.sha256 : nullptr, r.size)) {
    otaValveLock = false;
    resetEmergencyLatch(); // защёлку поднял сам OTA-путь — снимаем при неудаче
    return false;
  }
  ESP.restart();
  return true;
}

bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag, const char *preloadedSha256,
                            uint32_t knownSize) {
  if (WiFi.status() != WL_CONNECTED) {
    SERIAL_DBGLN("[GH-OTA] No Wi-Fi STA");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }
  if (!otaUrlIsHttps(firmwareUrl)) {
    // 10.15.0: http для прошивки запрещён (раньше проверялось только "http" в строке).
    strlcpy(otaListStatus, "нужен https", sizeof(otaListStatus));
    return false;
  }

  otaInProgress = true;
  extern int otaProgress;
  extern char otaStatus[32];
  otaProgress = 0;
  strlcpy(otaStatus, "SHA-256...", sizeof(otaStatus));
  displayDirty = true;
  githubOtaReleaseTlsHeap("install-begin");
  githubOtaPauseWorkers(true);
  githubOtaClearUpdate("before-start");
  githubOtaHeartbeat();
  githubOtaPrintHeap("start");
  WiFi.setSleep(false);

  {
    const esp_err_t mv = esp_ota_mark_app_valid_cancel_rollback();
    if (mv != ESP_OK && mv != ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
      SERIAL_DBG("[GH-OTA] mark_valid: %s\n", esp_err_to_name(mv));
    }
  }

  char expected[65] = "";
  if (preloadedSha256 && strlen(preloadedSha256) >= 64) {
    memcpy(expected, preloadedSha256, 64);
    expected[64] = '\0';
    for (char *p = expected; *p; ++p) {
      if (*p >= 'A' && *p <= 'F') *p = static_cast<char>(*p - 'A' + 'a');
    }
    SERIAL_DBGLN("[GH-OTA] sha256 from release digest");
  } else if (sha256Url && sha256Url[0]) {
    static constexpr char kShaPath[] = "/gh_sha.txt";
    const int shaCode = githubHttpsDownloadToFile(sha256Url, kShaPath, 45000, "*/*");
    githubOtaHeartbeat();
    if (shaCode == HTTP_CODE_OK) {
      File sf = LittleFS.open(kShaPath, "r");
      if (sf) {
        size_t n = sf.readBytes(expected, 64);
        expected[n < 64 ? n : 64] = '\0';
        sf.close();
      }
    }
    LittleFS.remove(kShaPath);
    for (char *p = expected; *p; ++p) {
      const char c = *p;
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
        *p = '\0';
        break;
      }
      if (c >= 'A' && c <= 'F') *p = static_cast<char>(c - 'A' + 'a');
    }
    if (strlen(expected) < 64) {
      SERIAL_DBG("[GH-OTA] SHA-256 bad (HTTP %d)\n", shaCode);
    }
  }
  if (strlen(expected) < 64) {
    strlcpy(otaListStatus, "нет sha256", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  expected[64] = '\0';
  SERIAL_DBG("[GH-OTA] expected sha256: %.16s... tag=%s\n", expected,
                releaseTag ? releaseTag : "?");
  SERIAL_DBG("[GH-OTA] firmware URL: %.80s\n", firmwareUrl);

  strlcpy(otaStatus, "Загрузка...", sizeof(otaStatus));
  displayDirty = true;

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);

  // Primary: esp32FOTA-style HTTPClient redirect follow (browser_download_url).
  SERIAL_DBG("[GH-OTA] knownSize=%u\n", static_cast<unsigned>(knownSize));
  size_t written = httpFollowStreamToUpdate(firmwareUrl, &sha, knownSize);

  // If caller still has API asset URL, try resolve+raw CDN once.
  if (written < 100000u && strstr(firmwareUrl, "api.github.com") &&
      strstr(firmwareUrl, "/releases/assets/")) {
    githubOtaClearUpdate("retry-cdn");
    mbedtls_sha256_free(&sha);
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    static char cdnUrl[1400];
    if (githubResolveDownloadUrl(firmwareUrl, cdnUrl, sizeof(cdnUrl),
                                 "application/octet-stream") &&
        strstr(cdnUrl, "http") && !strstr(cdnUrl, "api.github.com")) {
      SERIAL_DBG("[GH-OTA] CDN ready len=%u\n", static_cast<unsigned>(strlen(cdnUrl)));
      written = httpFollowStreamToUpdate(cdnUrl, &sha, knownSize);
    }
  }

  if (written < 100000u) {
    mbedtls_sha256_free(&sha);
    githubOtaClearUpdate("stream-fail");
    // 10.15.0: незащищённый фолбэк HTTPUpdate удалён — он активировал образ
    // БЕЗ сверки SHA-256 и вызывал ESP.restart() сразу после этого.
    strlcpy(otaListStatus, "ошибка загрузки", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  SERIAL_DBG("[GH-OTA] downloaded %u bytes\n", static_cast<unsigned>(written));
  strlcpy(otaStatus, "Проверка SHA...", sizeof(otaStatus));
  otaProgress = 99;
  displayDirty = true;

  unsigned char digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  char actual[65];
  for (uint8_t i = 0; i < sizeof(digest); i++) {
    snprintf(actual + i * 2, 3, "%02x", digest[i]);
  }
  actual[64] = '\0';
  if (strcmp(actual, expected) != 0) {
    Serial.printf("[GH-OTA] SHA-256 mismatch tag=%s\n", releaseTag ? releaseTag : "?");
    SERIAL_DBG("[GH-OTA] expected=%s\n", expected);
    SERIAL_DBG("[GH-OTA] actual  =%s\n", actual);
    if (Update.isRunning()) Update.abort();
    githubOtaClearUpdate("sha-mismatch");
    strlcpy(otaListStatus, "SHA не совпал", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  strlcpy(otaStatus, "Фиксация...", sizeof(otaStatus));
  displayDirty = true;
  if (getGhOtaActive()) {
    const esp_err_t ee = esp_ota_end(getGhOtaHandle());
    if (ee != ESP_OK) {
      Serial.printf("[GH-OTA] esp_ota_end fail: %s\n", esp_err_to_name(ee));
      githubOtaClearUpdate("ota-end");
      strlcpy(otaListStatus, "ошибка esp_ota_end", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }
    const esp_err_t se = esp_ota_set_boot_partition(getGhOtaPart());
    if (se != ESP_OK) {
      Serial.printf("[GH-OTA] set_boot fail: %s\n", esp_err_to_name(se));
      strlcpy(otaListStatus, "ошибка set_boot", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }
    Serial.printf("[GH-OTA] OK esp_ota_end %s > reboot\n", releaseTag ? releaseTag : "?");
  } else if (!Update.end(true)) {
    SERIAL_DBG("[GH-OTA] Update.end fail: %s\n", Update.errorString());
    strlcpy(otaListStatus, "ошибка Update.end", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  } else {
    SERIAL_DBG("[GH-OTA] OK Update.end %s > reboot\n", releaseTag ? releaseTag : "?");
  }
  setGhOtaActive(false);
  setGhOtaHandle(0);
  setGhOtaPart(nullptr);
  otaProgress = 100;
  strlcpy(otaStatus, "Готово, перезагрузка", sizeof(otaStatus));
  displayDirty = true;
  githubOtaEndInstallSession("dl-ok");
  return true;
}

bool checkGitHubUpdate(bool install) {
  if (WiFi.status() != WL_CONNECTED) {
    SERIAL_DBGLN("[GH-OTA] Connect device to Wi-Fi with internet");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }

  if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
    if (otaListStatus[0] == '\0') {
      strlcpy(otaListStatus, "нет ответа GitHub", sizeof(otaListStatus));
    }
    return false;
  }

  const OtaRelease &latest = otaReleases[0];
  static char tagBuf[16];
  static char binBuf[256];
  static char shaBuf[256];
  static char digBuf[65];
  strlcpy(tagBuf, latest.tag, sizeof(tagBuf));
  strlcpy(binBuf, latest.binUrl, sizeof(binBuf));
  strlcpy(shaBuf, latest.shaUrl, sizeof(shaBuf));
  strlcpy(digBuf, latest.sha256, sizeof(digBuf));
  strlcpy(otaLatestTag, tagBuf, sizeof(otaLatestTag));
  SERIAL_DBG("[GH-OTA] Latest release: %s (local %s)\n", tagBuf, VERSION);

  if (!install) {
    bool newer = isRemoteSemVerNewer(VERSION, tagBuf);
    if (newer) {
      snprintf(otaListStatus, sizeof(otaListStatus), "есть обновление %s", tagBuf);
    } else {
      SemVer local{}, remote{};
      if (parseSemVer(VERSION, local) && parseSemVer(tagBuf, remote) &&
          compareSemVer(local, remote) > 0) {
        snprintf(otaListStatus, sizeof(otaListStatus), "новее GitHub (%s)", tagBuf);
      } else {
        snprintf(otaListStatus, sizeof(otaListStatus), "актуально (%s)", tagBuf);
      }
    }
    SERIAL_DBG("[GH-OTA] remote newer: %s\n", newer ? "yes" : "no");
    return newer;
  }

  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(binBuf, shaBuf[0] ? shaBuf : nullptr, tagBuf,
                              digBuf[0] ? digBuf : nullptr, latest.size)) {
    otaValveLock = false;
    resetEmergencyLatch(); // защёлку поднял сам OTA-путь — снимаем при неудаче
    return false;
  }
  ESP.restart();
  return true;
}
