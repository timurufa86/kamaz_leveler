#pragma once
#include <Arduino.h>
#include <WiFiClientSecure.h>

void otaConfigureTls(WiFiClientSecure *client);
/**
 * 10.15.4: TLS for the firmware-download connection only. Uses pinned roots
 * (Sectigo E46 + ISRG X1) instead of the full ESP-IDF CA bundle: after
 * esp_ota_begin()/partial writes the heap cannot carry bundle-based chain
 * verification, which broke resuming an interrupted download. Image integrity
 * stays guarded by the SHA-256 digest obtained over the verified API channel.
 */
void otaConfigureTlsDownload(WiFiClientSecure *client);
/**
 * 10.15.5: TLS for the image download only, WITHOUT certificate verification.
 * The image is authenticated by its SHA-256 digest, which is fetched over the
 * separately verified API connection; skipping chain verification here removes
 * the largest heap consumer exactly when a resume/reconnect must fit into the
 * 84-88KB left after a partial flash write.
 */
void otaConfigureTlsInsecureDownload(WiFiClientSecure *client);
/** true, если последнее TLS-подключение шло без проверки сертификата. */
extern volatile bool otaTlsInsecure;
/** true только для https-URL — http для OTA запрещён с 10.15.0. */
bool otaUrlIsHttps(const char *url);

void githubOtaDefragHeap();
void githubOtaReserveTlsHeap(const char *why);
void githubOtaReleaseTlsHeap(const char *why);
void githubOtaPauseWorkers(bool pause);
void githubOtaEndInstallSession(const char *why);
void githubOtaPingDiag();
void githubOtaPrintHeap(const char *tag);
void githubOtaHeartbeat();
void githubOtaClearUpdate(const char *why);

int githubHttpsDownloadToFile(const char *url, const char *outPath,
                              uint32_t timeoutMs = 25000,
                              const char *acceptHdr = "application/json",
                              const char *rangeHdr = nullptr);

/** Resolve API asset URL 302 Location → CDN URL (objects.githubusercontent.com).
 *  Writes final URL into out (may copy url unchanged). Returns true if out is usable. */
bool githubResolveDownloadUrl(const char *url, char *out, size_t outLen,
                              const char *acceptHdr = "application/octet-stream");

/**
 * 10.15.5: free heap for the install by deleting worker tasks whose stacks are
 * not needed during OTA (display/watchdog/OTA task are kept). Must be called
 * after githubOtaPauseWorkers(true), and paired with githubOtaRestoreWorkerTasks()
 * when the install fails. Measured need: a TLS reconnect after a partial flash
 * write fails below ~88KB free heap.
 */
void githubOtaFreeWorkerTasks();
void githubOtaRestoreWorkerTasks();

#include <esp_ota_ops.h>
esp_ota_handle_t       getGhOtaHandle();
bool                   getGhOtaActive();
const esp_partition_t *getGhOtaPart();
void setGhOtaHandle(esp_ota_handle_t h);
void setGhOtaActive(bool v);
void setGhOtaPart(const esp_partition_t *p);

extern volatile bool s_otaWorkersPaused;
