#pragma once
#include <Arduino.h>

bool downloadGitHubFirmware(const char *firmwareUrl, const char *sha256Url,
                            const char *releaseTag);
bool checkGitHubUpdate(bool install);
bool installGitHubReleaseIndex(int8_t idx);
