#include "github_ota_request.h"

volatile GitHubOtaRequest g_githubOtaRequest = GitHubOtaRequest::NONE;

void requestGitHubOtaCheckAndInstall() {
  g_githubOtaRequest = GitHubOtaRequest::CHECK_AND_INSTALL;
}

GitHubOtaRequest takeGitHubOtaRequest() {
  GitHubOtaRequest req = g_githubOtaRequest;
  if (req != GitHubOtaRequest::NONE) {
    g_githubOtaRequest = GitHubOtaRequest::NONE;
  }
  return req;
}
