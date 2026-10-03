#include "ota_list.h"
#include "ota_net.h"
#include "app_globals.h"
#include "app_types.h"
#include "semver_utils.h"

#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#ifndef HTTP_CODE_OK
#define HTTP_CODE_OK 200
#endif

static constexpr char GITHUB_ASSET_NAME[] = "kamaz_leveler.bin";
static constexpr char GITHUB_SHA256_ASSET_NAME[] = "kamaz_leveler.bin.sha256";

static char otaListHdrBuf_ext[40] = "загрузка с GitHub...";
char *otaListHdrBuf_ptr() { return otaListHdrBuf_ext; }

extern char otaListHdrBuf[];

/** Extract 64 hex chars from GitHub asset digest ("sha256:aabb...") into out[65]. */
static bool copyDigestSha256(const char *digest, char *out, size_t outLen) {
  if (!digest || !out || outLen < 65) return false;
  out[0] = '\0';
  const char *p = digest;
  if (strncmp(p, "sha256:", 7) == 0) p += 7;
  size_t n = 0;
  while (p[n] && n < 64) {
    const char c = p[n];
    const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!hex) break;
    ++n;
  }
  if (n < 64) return false;
  for (size_t i = 0; i < 64; ++i) {
    char c = p[i];
    if (c >= 'A' && c <= 'F') c = static_cast<char>(c - 'A' + 'a');
    out[i] = c;
  }
  out[64] = '\0';
  return true;
}

static void githubListFilter(JsonDocument &filter) {
  filter[0]["tag_name"] = true;
  filter[0]["published_at"] = true;
  filter[0]["assets"][0]["name"] = true;
  filter[0]["assets"][0]["size"] = true;
  // API asset URL (api.github.com) — TLS на ESP32 стабильнее, чем github.com/releases/download
  filter[0]["assets"][0]["url"] = true;
  filter[0]["assets"][0]["browser_download_url"] = true;
  filter[0]["assets"][0]["digest"] = true;
}

static bool parseGitHubReleaseListFile(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f || f.size() < 16) {
    strlcpy(otaListStatus, "пустой ответ", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "пустой ответ", 40);
    if (f) f.close();
    return false;
  }
  Serial.printf("[GH-OTA] Список file %u байт heap=%u\n", static_cast<unsigned>(f.size()),
                static_cast<unsigned>(ESP.getFreeHeap()));

  JsonDocument filter;
  githubListFilter(filter);
  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, f, DeserializationOption::Filter(filter));
  f.close();
  LittleFS.remove(path);

  if (err) {
    snprintf(otaListStatus, sizeof(otaListStatus), "JSON %s", err.c_str());
    strlcpy(otaListHdrBuf, "ошибка JSON", 40);
    Serial.printf("[GH-OTA] Список: JSON %s\n", err.c_str());
    return false;
  }

  if (!doc.is<JsonArray>()) {
    strlcpy(otaListStatus, "не массив JSON", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "неверный JSON", 40);
    return false;
  }

  OtaRelease collected[OTA_FETCH_MAX];
  uint8_t collectedN = 0;
  for (JsonObject rel : doc.as<JsonArray>()) {
    if (collectedN >= OTA_FETCH_MAX) break;
    OtaRelease r{};
    strlcpy(r.tag, rel["tag_name"] | "", sizeof(r.tag));
    if (r.tag[0] == '\0') continue;

    const char *published = rel["published_at"] | "";
    if (strlen(published) >= 10) {
      memcpy(r.date, published, 10);
      r.date[10] = '\0';
    }

    for (JsonObject asset : rel["assets"].as<JsonArray>()) {
      const char *name = asset["name"] | "";
      if (name[0] == '\0') continue;
      const char *apiUrl = asset["url"] | "";
      const char *browserUrl = asset["browser_download_url"] | "";
      // Prefer API asset URL — same host as LIST (api.github.com), avoids github.com TLS BIGNUM fail
      const char *dl = (apiUrl[0] != '\0') ? apiUrl : browserUrl;
      if (dl[0] == '\0') continue;

      if (strcmp(name, GITHUB_ASSET_NAME) == 0) {
        strlcpy(r.binUrl, dl, sizeof(r.binUrl));
        r.size = asset["size"] | 0UL;
        copyDigestSha256(asset["digest"] | "", r.sha256, sizeof(r.sha256));
      } else if (strcmp(name, GITHUB_SHA256_ASSET_NAME) == 0) {
        strlcpy(r.shaUrl, dl, sizeof(r.shaUrl));
      }
    }
    if (r.binUrl[0] == '\0') {
      Serial.printf("[GH-OTA]  пропуск %s — нет %s\n", r.tag, GITHUB_ASSET_NAME);
      continue;
    }
    // sha: digest preferred; иначе отдельный .sha256 asset
    if (r.sha256[0] == '\0' && r.shaUrl[0] == '\0') {
      Serial.printf("[GH-OTA]  пропуск %s — нет digest/sha256\n", r.tag);
      continue;
    }
    collected[collectedN++] = r;
  }

  for (uint8_t i = 0; i + 1 < collectedN; i++) {
    for (uint8_t j = i + 1; j < collectedN; j++) {
      SemVer a{}, b{};
      parseSemVer(collected[i].tag, a);
      parseSemVer(collected[j].tag, b);
      if (!a.valid || !b.valid) continue;
      if (compareSemVer(b, a) > 0) {
        OtaRelease tmp = collected[i];
        collected[i] = collected[j];
        collected[j] = tmp;
      }
    }
  }

  uint8_t n = collectedN < OTA_LIST_MAX ? collectedN : OTA_LIST_MAX;
  for (uint8_t i = 0; i < n; i++) {
    otaReleases[i] = collected[i];
    Serial.printf("[GH-OTA]  #%u %s  %s  %lu Б  sha=%s digest=%s\n", i, otaReleases[i].tag,
                  otaReleases[i].date, static_cast<unsigned long>(otaReleases[i].size),
                  otaReleases[i].shaUrl[0] ? "file" : "-",
                  otaReleases[i].sha256[0] ? "yes" : "NO");
  }
  otaReleaseCount = n;
  if (n > 0) {
    strlcpy(otaLatestTag, otaReleases[0].tag, sizeof(otaLatestTag));
    snprintf(otaListStatus, sizeof(otaListStatus), "список: %u (актуал. %s)", n, otaLatestTag);
    snprintf(otaListHdrBuf, 40, "последние %u", n);
  } else {
    strlcpy(otaListStatus, "нет прошивок с bin", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет bin в релизе", 40);
  }
  Serial.printf("[GH-OTA] Получено релизов: %u\n", n);
  return n > 0;
}

bool fetchGitHubReleaseList() {
  otaReleaseCount = 0;
  otaSelectedIndex = -1;
  for (uint8_t i = 0; i < OTA_LIST_MAX; i++) {
    otaReleases[i] = OtaRelease{};
  }

  if (WiFi.status() != WL_CONNECTED) {
    strlcpy(otaListStatus, "нет Wi-Fi (STA)", sizeof(otaListStatus));
    strlcpy(otaListHdrBuf, "нет Wi-Fi (STA)", 40);
    Serial.println("[GH-OTA] Список: нет Wi-Fi");
    if (menuVisible) displayDirty = true;
    return false;
  }

  strlcpy(otaListHdrBuf, "связь с GitHub...", 40);
  if (menuVisible) displayDirty = true;
  // Releases API (не tags): даёт asset.url на api.github.com + digest sha256
  constexpr char kReleasesUrl[] =
      "https://api.github.com/repos/timurufa86/kamaz_leveler/releases?per_page=3";
  constexpr char kPath[] = "/gh_rels.json";
  Serial.printf("[GH-OTA] Список releases, heap %u\n", static_cast<unsigned>(ESP.getFreeHeap()));
  const int code = githubHttpsDownloadToFile(kReleasesUrl, kPath, 25000, "application/json");
  if (code != HTTP_CODE_OK) {
    if (code <= 0) {
      strlcpy(otaListStatus, "сеть/TLS ошибка", sizeof(otaListStatus));
      strlcpy(otaListHdrBuf, "нет связи TLS", 40);
    } else {
      snprintf(otaListStatus, sizeof(otaListStatus), "GitHub HTTP %d", code);
      snprintf(otaListHdrBuf, 40, "HTTP %d", code);
    }
    LittleFS.remove(kPath);
    if (menuVisible) displayDirty = true;
    return false;
  }

  strlcpy(otaListHdrBuf, "разбор списка...", 40);
  if (menuVisible) displayDirty = true;
  const bool ok = parseGitHubReleaseListFile(kPath);
  if (menuVisible) displayDirty = true;
  return ok;
}

bool fetchReleaseSha256(uint8_t idx) {
  if (idx >= otaReleaseCount) return false;
  OtaRelease &r = otaReleases[idx];
  if (r.sha256[0] != '\0') {
    strlcpy(otaListStatus, "sha256 (digest)", sizeof(otaListStatus));
    return true;
  }
  if (r.shaUrl[0] == '\0' || WiFi.status() != WL_CONNECTED) return false;

  static constexpr char kShaPath[] = "/gh_sha_chk.txt";
  const int code = githubHttpsDownloadToFile(r.shaUrl, kShaPath, 45000, "application/octet-stream");
  if (code != HTTP_CODE_OK) {
    LittleFS.remove(kShaPath);
    strlcpy(otaListStatus, "sha256 недоступен", sizeof(otaListStatus));
    return false;
  }
  File f = LittleFS.open(kShaPath, "r");
  char body[72] = "";
  if (f) {
    size_t n = f.readBytes(body, 64);
    body[n < 64 ? n : 64] = '\0';
    f.close();
  }
  LittleFS.remove(kShaPath);
  if (strlen(body) < 64) {
    strlcpy(otaListStatus, "sha256 недоступен", sizeof(otaListStatus));
    return false;
  }
  body[64] = '\0';
  strlcpy(r.sha256, body, sizeof(r.sha256));
  strlcpy(otaListStatus, "sha256 получен", sizeof(otaListStatus));
  Serial.printf("[GH-OTA] %s sha256: %s\n", r.tag, r.sha256);
  return true;
}
