# Modules (v8.5.9)

Extracted from the monolithic sketch for maintainability:

| File | Role |
|------|------|
| `mutex_guard.h` / `.cpp` | RAII `MutexGuard` + `takeMutexWithRetry` |
| `semver_utils.h` | SemVer parse/compare for GitHub OTA |
| `github_ota_request.h` / `.cpp` | Async OTA request flag for `otaTask` |

Next candidates (not yet split): `ConfigManager`, `AutoLevelingController`, valve/pressure tasks.
