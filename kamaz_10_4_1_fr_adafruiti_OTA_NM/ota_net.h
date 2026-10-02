#pragma once
#include <Arduino.h>
#include <WiFiClientSecure.h>

void otaConfigureTls(WiFiClientSecure *client);

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

#include <esp_ota_ops.h>
esp_ota_handle_t       getGhOtaHandle();
bool                   getGhOtaActive();
const esp_partition_t *getGhOtaPart();
void setGhOtaHandle(esp_ota_handle_t h);
void setGhOtaActive(bool v);
void setGhOtaPart(const esp_partition_t *p);

extern volatile bool s_otaWorkersPaused;
