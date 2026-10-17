#pragma once
#include <Arduino.h>

bool fetchGitHubReleaseList();
bool fetchReleaseSha256(uint8_t idx);
