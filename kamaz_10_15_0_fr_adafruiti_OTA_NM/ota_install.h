#pragma once
#include <Arduino.h>

/** sha256Url optional if preloadedSha256 is 64 hex chars (GitHub asset digest).
 *  knownSize from release list enables HTTP Range chunks (avoids WiFi stall during erase). */
bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag, const char *preloadedSha256 = nullptr,
                            uint32_t knownSize = 0);
bool checkGitHubUpdate(bool install);
bool installGitHubReleaseIndex(int8_t idx);
