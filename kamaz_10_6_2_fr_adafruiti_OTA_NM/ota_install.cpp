#include "ota_install.h"
#include "ota_net.h"
#include "ota_list.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_version.h"
#include "semver_utils.h"
#include "logger.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include <Update.h>
#include <mbedtls/sha256.h>
#include <new>

void emergencyStop();   // defined in .ino

bool installGitHubReleaseIndex(int8_t idx) {
  if (idx < 0 || idx >= static_cast<int8_t>(otaReleaseCount)) return false;
  const OtaRelease &r = otaReleases[idx];
  if (r.binUrl[0] == '\0' || r.shaUrl[0] == '\0') return false;

  Serial.printf("[GH-OTA] Установка релиза %s\n", r.tag);
  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(r.binUrl, r.shaUrl, r.tag)) {
    otaValveLock = false;
    return false;
  }
  ESP.restart();
  return true;
}

bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Нет интернет-соединения");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }

  otaInProgress = true;
  extern int otaProgress;
  extern char otaStatus[32];
  otaProgress = 0;
  strlcpy(otaStatus, "SHA-256…", sizeof(otaStatus));
  displayDirty = true;
  githubOtaReleaseTlsHeap("install-begin");
  githubOtaPauseWorkers(true);
  githubOtaClearUpdate("before-start");
  githubOtaHeartbeat();
  githubOtaPrintHeap("start");

  {
    const esp_err_t mv = esp_ota_mark_app_valid_cancel_rollback();
    if (mv != ESP_OK && mv != ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
      Serial.printf("[GH-OTA] mark_valid: %s\n", esp_err_to_name(mv));
    }
  }

  static constexpr char kShaPath[] = "/gh_sha.txt";
  const int shaCode = githubHttpsDownloadToFile(sha256Url, kShaPath, 45000, "*/*");
  githubOtaHeartbeat();
  char expected[65] = "";
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
    Serial.printf("[GH-OTA] SHA-256 asset отсутствует или некорректен (HTTP %d, url=%s)\n",
                  shaCode, sha256Url);
    strlcpy(otaListStatus, "нет sha256", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  expected[64] = '\0';
  Serial.printf("[GH-OTA] expected sha256: %.16s… tag=%s\n", expected, releaseTag ? releaseTag : "?");
  githubOtaPrintHeap("after-sha");

  strlcpy(otaStatus, "OTA begin…", sizeof(otaStatus));
  displayDirty = true;
  WiFi.setSleep(false);
  vTaskDelay(pdMS_TO_TICKS(30));

  const esp_partition_t *ghOtaPart = esp_ota_get_next_update_partition(nullptr);
  if (!ghOtaPart) {
    Serial.println("[GH-OTA] нет OTA partition");
    strlcpy(otaListStatus, "нет OTA part", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  Serial.printf("[GH-OTA] part subtype=%u size=%u offset=0x%X\n",
                static_cast<unsigned>(ghOtaPart->subtype),
                static_cast<unsigned>(ghOtaPart->size),
                static_cast<unsigned>(ghOtaPart->address));
  githubOtaPrintHeap("pre-begin");

  esp_ota_handle_t ghOtaHandle = 0;
  esp_err_t beginErr = esp_ota_begin(ghOtaPart, OTA_WITH_SEQUENTIAL_WRITES, &ghOtaHandle);
  if (beginErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_begin(SEQ) fail %s — retry SIZE_UNKNOWN\n",
                  esp_err_to_name(beginErr));
    beginErr = esp_ota_begin(ghOtaPart, OTA_SIZE_UNKNOWN, &ghOtaHandle);
  }
  if (beginErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_begin fail %s heap=%u maxBlk=%u\n",
                  esp_err_to_name(beginErr),
                  static_cast<unsigned>(ESP.getFreeHeap()),
                  static_cast<unsigned>(ESP.getMaxAllocHeap()));
    strlcpy(otaListStatus, "OTA begin fail", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  setGhOtaHandle(ghOtaHandle);
  setGhOtaActive(true);
  setGhOtaPart(ghOtaPart);
  Serial.printf("[GH-OTA] esp_ota_begin OK handle=%u\n", static_cast<unsigned>(ghOtaHandle));
  githubOtaPrintHeap("post-begin");

  strlcpy(otaStatus, "Загрузка…", sizeof(otaStatus));
  displayDirty = true;

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);

  uint8_t stackBuf[1024];
  uint8_t *rdBuf = stackBuf;
  size_t rdCap = sizeof(stackBuf);

  size_t written = 0;
  size_t totalSize = 1455600;
  bool sizeKnown = true;
  int lastPct = -1;
  const uint32_t tStart = millis();
  static constexpr uint32_t kHardTimeoutMs = 900000UL;
  static constexpr uint32_t kStallMs = 12000;
  static constexpr uint32_t kStreamTimeoutMs = 120000;
  size_t streamFails = 0;

  Serial.printf("[GH-OTA] stream+resume total~%u heap=%u maxBlk=%u\n",
                static_cast<unsigned>(totalSize),
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getMaxAllocHeap()));

  while (written < totalSize) {
    if (millis() - tStart > kHardTimeoutMs) {
      Serial.printf("[GH-OTA] hard timeout after %u байт\n", static_cast<unsigned>(written));
      githubOtaClearUpdate("timeout");
      mbedtls_sha256_free(&sha);
      strlcpy(otaListStatus, "таймаут загрузки", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }

    if ((totalSize - written) > 0 && (totalSize - written) <= (256u * 1024u)) {
      const size_t from = written;
      const size_t to = totalSize - 1;
      char rangeHdr[48];
      snprintf(rangeHdr, sizeof(rangeHdr), "bytes=%u-%u", static_cast<unsigned>(from),
               static_cast<unsigned>(to));
      Serial.printf("[GH-OTA] tail FS %s\n", rangeHdr);
      bool tailOk = false;
      for (int attempt = 1; attempt <= 5; attempt++) {
        const int code =
            githubHttpsDownloadToFile(firmwareUrl, "/ota_tail.bin", 120000, "*/*", rangeHdr);
        if (code == HTTP_CODE_OK || code == HTTP_CODE_PARTIAL_CONTENT) {
          File tf = LittleFS.open("/ota_tail.bin", "r");
          if (tf && tf.size() > 0) {
            while (tf.available() && written < totalSize) {
              const int n = tf.read(rdBuf, rdCap);
              if (n <= 0) break;
              if (esp_ota_write(ghOtaHandle, rdBuf, static_cast<size_t>(n)) != ESP_OK) {
                tf.close();
                LittleFS.remove("/ota_tail.bin");
                githubOtaClearUpdate("tail-write");
                mbedtls_sha256_free(&sha);
                strlcpy(otaListStatus, "сбой записи", sizeof(otaListStatus));
                githubOtaEndInstallSession("dl-exit");
                return false;
              }
              mbedtls_sha256_update(&sha, rdBuf, static_cast<size_t>(n));
              written += static_cast<size_t>(n);
            }
            tf.close();
            LittleFS.remove("/ota_tail.bin");
            tailOk = (written >= totalSize);
            if (tailOk) break;
          }
        }
        LittleFS.remove("/ota_tail.bin");
        Serial.printf("[GH-OTA] tail attempt %d code fail, retry\n", attempt);
        vTaskDelay(pdMS_TO_TICKS(1000 * attempt));
        if (attempt >= 2) {
          WiFi.reconnect();
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
      }
      if (!tailOk) {
        githubOtaClearUpdate("tail-fail");
        mbedtls_sha256_free(&sha);
        strlcpy(otaListStatus, "обрыв хвоста", sizeof(otaListStatus));
        githubOtaEndInstallSession("dl-exit");
        return false;
      }
      break;
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[GH-OTA] WiFi lost — reconnect");
      WiFi.reconnect();
      for (int w = 0; w < 50 && WiFi.status() != WL_CONNECTED; w++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        githubOtaHeartbeat();
      }
    }

    WiFiClientSecure *client = new (std::nothrow) WiFiClientSecure();
    if (!client) {
      streamFails++;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    otaConfigureTls(client);
    client->setHandshakeTimeout(30);
    client->setTimeout(kStreamTimeoutMs);

    int httpCode = -1;
    int contentLen = -1;
    {
      HTTPClient http;
      http.setConnectTimeout(25000);
      http.setTimeout(kStreamTimeoutMs);
      if (!http.begin(*client, firmwareUrl)) {
        Serial.println("[GH-OTA] stream begin fail");
        delete client;
        streamFails++;
        vTaskDelay(pdMS_TO_TICKS(800));
        continue;
      }
      http.useHTTP10(true);
      http.setReuse(false);
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      http.addHeader("User-Agent", "kamaz-leveler/9.4.0");
      http.addHeader("Accept", "*/*");
      http.addHeader("Accept-Encoding", "identity");
      http.addHeader("Connection", "close");
      const char *hk[] = {"Content-Range", "Content-Length"};
      http.collectHeaders(hk, 2);
      if (written > 0) {
        char rangeHdr[48];
        snprintf(rangeHdr, sizeof(rangeHdr), "bytes=%u-", static_cast<unsigned>(written));
        http.addHeader("Range", rangeHdr);
        Serial.printf("[GH-OTA] resume %s\n", rangeHdr);
      }

      githubOtaHeartbeat();
      const uint32_t t0 = millis();
      httpCode = http.GET();
      contentLen = http.getSize();
      Serial.printf("[GH-OTA] stream GET > %d size=%d in %lums written=%u\n", httpCode, contentLen,
                    static_cast<unsigned long>(millis() - t0), static_cast<unsigned>(written));

      if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_PARTIAL_CONTENT) {
        http.end();
      } else if (written > 0 && httpCode != HTTP_CODE_PARTIAL_CONTENT) {
        Serial.println("[GH-OTA] resume: ожидали 206 Partial, получили 200 — abort chunk");
        http.end();
        httpCode = -1;
      } else {
        String cr = http.header("Content-Range");
        if (written > 0) {
          long rangeStart = -1;
          if (cr.startsWith("bytes ")) {
            rangeStart = cr.substring(6).toInt();
          }
          if (rangeStart < 0 || static_cast<size_t>(rangeStart) != written) {
            Serial.printf("[GH-OTA] resume: Content-Range start=%ld != written=%u — abort\n",
                          rangeStart, static_cast<unsigned>(written));
            http.end();
            httpCode = -1;
          }
        }
        if (httpCode < 0) {
          // already aborted
        } else if (cr.length() > 0) {
          const int slash = cr.lastIndexOf('/');
          if (slash >= 0) {
            const size_t ts = static_cast<size_t>(cr.substring(slash + 1).toInt());
            if (ts > 100000u) {
              totalSize = ts;
              sizeKnown = true;
            }
          }
        } else if (httpCode == HTTP_CODE_OK && contentLen > 100000) {
          totalSize = static_cast<size_t>(contentLen);
          sizeKnown = true;
        }

        if (httpCode < 0) {
          // skip body
        } else {
        WiFiClient *stream = http.getStreamPtr();
        size_t sessionGot = 0;
        const size_t expect =
            (contentLen > 0) ? static_cast<size_t>(contentLen) : (totalSize - written);
        client->setTimeout(8000);
        const uint32_t tRead0 = millis();
        uint32_t lastDataMs = millis();
        while (sessionGot < expect && written < totalSize &&
               (millis() - tRead0) < kStreamTimeoutMs) {
          githubOtaHeartbeat();
          if ((millis() - lastDataMs) > 25000u) {
            Serial.println("[GH-OTA] stream stall 25s — resume");
            break;
          }
          size_t want = rdCap;
          if (expect - sessionGot < want) want = expect - sessionGot;
          if (totalSize - written < want) want = totalSize - written;
          const int n = stream->readBytes(rdBuf, want);
          if (n <= 0) {
            if (!client->connected()) break;
            delay(20);
            continue;
          }
          lastDataMs = millis();
          const esp_err_t werr = esp_ota_write(ghOtaHandle, rdBuf, static_cast<size_t>(n));
          if (werr != ESP_OK) {
            Serial.printf("[GH-OTA] esp_ota_write fail at %u: %s\n",
                          static_cast<unsigned>(written), esp_err_to_name(werr));
            http.end();
            client->stop();
            delete client;
            githubOtaClearUpdate("write-fail");
            mbedtls_sha256_free(&sha);
            strlcpy(otaListStatus, "сбой записи", sizeof(otaListStatus));
            githubOtaEndInstallSession("dl-exit");
            return false;
          }
          mbedtls_sha256_update(&sha, rdBuf, static_cast<size_t>(n));
          written += static_cast<size_t>(n);
          sessionGot += static_cast<size_t>(n);

          otaProgress = static_cast<int>((written * 100u) / totalSize);
          if (otaProgress > 99) otaProgress = 99;
          if (otaProgress != lastPct) {
            lastPct = otaProgress;
            displayDirty = true;
            if ((otaProgress % 5) == 0 || otaProgress >= 99) {
              Serial.printf("[GH-OTA] download %d%% (%u/%u) heap=%u\n", otaProgress,
                            static_cast<unsigned>(written), static_cast<unsigned>(totalSize),
                            static_cast<unsigned>(ESP.getFreeHeap()));
            }
          }
        }
        Serial.printf("[GH-OTA] session +%u total=%u expect=%u connected=%d\n",
                      static_cast<unsigned>(sessionGot), static_cast<unsigned>(written),
                      static_cast<unsigned>(expect), client->connected() ? 1 : 0);
        http.end();
        if (sessionGot > 0) streamFails = 0;
        else streamFails++;
        }
      }
    }
    client->stop();
    delete client;

    if (written >= totalSize) break;

    if (streamFails >= 10) {
      githubOtaClearUpdate("stream-fail");
      mbedtls_sha256_free(&sha);
      strlcpy(otaListStatus, "обрыв потока", sizeof(otaListStatus));
      githubOtaEndInstallSession("dl-exit");
      return false;
    }
    if (httpCode <= 0) {
      Serial.println("[GH-OTA] stream fail > WiFi.reconnect()");
      WiFi.disconnect(false, false);
      vTaskDelay(pdMS_TO_TICKS(1000));
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(5000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(600));
    }
  }

  if (written < totalSize) {
    Serial.printf("[GH-OTA] incomplete %u/%u\n", static_cast<unsigned>(written),
                  static_cast<unsigned>(totalSize));
    githubOtaClearUpdate("incomplete");
    mbedtls_sha256_free(&sha);
    strlcpy(otaListStatus, "неполная загрузка", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  Serial.printf("[GH-OTA] скачано %u байт (chunked-FS)\n", static_cast<unsigned>(written));
  strlcpy(otaStatus, "Проверка SHA…", sizeof(otaStatus));
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
    Serial.printf("[GH-OTA] SHA-256 mismatch: %s\n", releaseTag);
    Serial.printf("[GH-OTA] expected=%s\n", expected);
    Serial.printf("[GH-OTA] actual  =%s\n", actual);
    githubOtaClearUpdate("sha-mismatch");
    strlcpy(otaListStatus, "SHA не совпал", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }

  strlcpy(otaStatus, "Фиксация…", sizeof(otaStatus));
  displayDirty = true;
  const esp_err_t endErr = esp_ota_end(ghOtaHandle);
  setGhOtaActive(false);
  setGhOtaHandle(0);
  if (endErr != ESP_OK) {
    Serial.printf("[GH-OTA] esp_ota_end fail: %s\n", esp_err_to_name(endErr));
    setGhOtaPart(nullptr);
    strlcpy(otaListStatus, "ошибка ota_end", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  const esp_err_t bootErr = esp_ota_set_boot_partition(ghOtaPart);
  if (bootErr != ESP_OK) {
    Serial.printf("[GH-OTA] set_boot fail: %s\n", esp_err_to_name(bootErr));
    setGhOtaPart(nullptr);
    strlcpy(otaListStatus, "ошибка boot part", sizeof(otaListStatus));
    githubOtaEndInstallSession("dl-exit");
    return false;
  }
  Serial.printf("[GH-OTA] OK set_boot %s > reboot\n", releaseTag ? releaseTag : "?");
  setGhOtaPart(nullptr);
  otaProgress = 100;
  strlcpy(otaStatus, "Готово, перезагрузка", sizeof(otaStatus));
  displayDirty = true;
  githubOtaEndInstallSession("dl-ok");
  return true;
}

bool checkGitHubUpdate(bool install) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GH-OTA] Подключите устройство к Wi-Fi с интернетом");
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    return false;
  }

  if (!fetchGitHubReleaseList() || otaReleaseCount == 0) {
    if (otaListStatus[0] == '\0') {
      strlcpy(otaListStatus, "нет ответа GitHub", sizeof(otaListStatus));
    }
    return false;
  }

  static char tagBuf[16];
  static char binBuf[176];
  static char shaBuf[176];
  strlcpy(tagBuf, otaReleases[0].tag, sizeof(tagBuf));
  strlcpy(binBuf, otaReleases[0].binUrl, sizeof(binBuf));
  strlcpy(shaBuf, otaReleases[0].shaUrl, sizeof(shaBuf));
  strlcpy(otaLatestTag, tagBuf, sizeof(otaLatestTag));
  Serial.printf("[GH-OTA] Последний release: %s (локально %s)\n", tagBuf, VERSION);

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
    Serial.printf("[GH-OTA] remote newer: %s\n", newer ? "yes" : "no");
    return newer;
  }

  otaValveLock = true;
  emergencyStop();
  if (!downloadGitHubFirmware(binBuf, shaBuf, tagBuf)) {
    otaValveLock = false;
    return false;
  }
  ESP.restart();
  return true;
}
