#pragma once
#include <Arduino.h>

/** Async GitHub OTA request — processed by otaTask, not button/menu task. */
enum class GitHubOtaRequest : uint8_t {
  NONE = 0,
  CHECK_AND_INSTALL = 1
};

extern volatile GitHubOtaRequest g_githubOtaRequest;

void requestGitHubOtaCheckAndInstall();
GitHubOtaRequest takeGitHubOtaRequest();
