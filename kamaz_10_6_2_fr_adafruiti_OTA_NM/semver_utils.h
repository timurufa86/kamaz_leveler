#pragma once
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

struct SemVer {
  int major = 0;
  int minor = 0;
  int patch = 0;
  bool valid = false;
};

inline const char *semverSkipPrefix(const char *s) {
  if (!s) return "";
  while (*s && (isspace(static_cast<unsigned char>(*s)) || *s == 'v' || *s == 'V')) {
    ++s;
  }
  return s;
}

inline bool parseSemVer(const char *text, SemVer &out) {
  out = SemVer{};
  if (!text) return false;
  const char *p = semverSkipPrefix(text);
  char *end = nullptr;
  long major = strtol(p, &end, 10);
  if (end == p || *end != '.') return false;
  p = end + 1;
  long minor = strtol(p, &end, 10);
  if (end == p || *end != '.') return false;
  p = end + 1;
  long patch = strtol(p, &end, 10);
  if (end == p) return false;
  out.major = static_cast<int>(major);
  out.minor = static_cast<int>(minor);
  out.patch = static_cast<int>(patch);
  out.valid = true;
  return true;
}

inline int compareSemVer(const SemVer &a, const SemVer &b) {
  if (a.major != b.major) return a.major < b.major ? -1 : 1;
  if (a.minor != b.minor) return a.minor < b.minor ? -1 : 1;
  if (a.patch != b.patch) return a.patch < b.patch ? -1 : 1;
  return 0;
}

/** true if remoteTag is newer than localVersionString (e.g. "V 8.5.7 ..." vs "v8.5.9"). */
inline bool isRemoteSemVerNewer(const char *localVersionString, const char *remoteTag) {
  SemVer local{}, remote{};
  if (!parseSemVer(localVersionString, local) || !parseSemVer(remoteTag, remote)) {
    return true;  // fail-open: allow update if parse fails
  }
  return compareSemVer(remote, local) > 0;
}
