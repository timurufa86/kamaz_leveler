#pragma once
#include <Arduino.h>

/** Async GitHub OTA requests — processed by otaTask, not by button/menu task. */
enum class GitHubOtaRequest : uint8_t {
  NONE = 0,
  CHECK_AND_INSTALL = 1,  // проверить последний релиз и установить, если он новее
  FETCH_LIST = 2,         // получить список последних релизов (страница «Обновления»)
  INSTALL_INDEX = 3       // установить релиз из списка по индексу g_githubOtaIndex
};

extern volatile GitHubOtaRequest g_githubOtaRequest;
extern volatile int8_t g_githubOtaIndex;

void requestGitHubOtaCheckAndInstall();
void requestGitHubOtaFetchList();
void requestGitHubOtaInstallIndex(int8_t index);
GitHubOtaRequest takeGitHubOtaRequest();
int8_t takeGitHubOtaIndex();
