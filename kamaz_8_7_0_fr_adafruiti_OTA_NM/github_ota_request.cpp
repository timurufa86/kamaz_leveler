#include "github_ota_request.h"

volatile GitHubOtaRequest g_githubOtaRequest = GitHubOtaRequest::NONE;
volatile int8_t g_githubOtaIndex = -1;

void requestGitHubOtaCheckAndInstall() {
  g_githubOtaRequest = GitHubOtaRequest::CHECK_AND_INSTALL;
}

void requestGitHubOtaFetchList() {
  g_githubOtaRequest = GitHubOtaRequest::FETCH_LIST;
}

void requestGitHubOtaInstallIndex(int8_t index) {
  g_githubOtaIndex = index;
  g_githubOtaRequest = GitHubOtaRequest::INSTALL_INDEX;
}

GitHubOtaRequest takeGitHubOtaRequest() {
  GitHubOtaRequest req = g_githubOtaRequest;
  if (req != GitHubOtaRequest::NONE) {
    g_githubOtaRequest = GitHubOtaRequest::NONE;
  }
  return req;
}

int8_t takeGitHubOtaIndex() {
  const int8_t idx = g_githubOtaIndex;
  g_githubOtaIndex = -1;
  return idx;
}
