#pragma once
#include <Arduino.h>

/** sha256Url optional if preloadedSha256 is 64 hex chars (GitHub asset digest). */
bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag, const char *preloadedSha256 = nullptr);
bool checkGitHubUpdate(bool install);
bool installGitHubReleaseIndex(int8_t idx);
