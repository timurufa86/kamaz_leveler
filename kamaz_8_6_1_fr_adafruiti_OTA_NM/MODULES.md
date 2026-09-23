# Modules (v8.6.1)

Extracted from the monolithic sketch for maintainability:

| File | Role |
|------|------|
| `mutex_guard.h` / `.cpp` | RAII `MutexGuard` + `takeMutexWithRetry` |
| `semver_utils.h` | SemVer parse/compare for GitHub OTA |
| `github_ota_request.h` / `.cpp` | Async OTA request flag for `otaTask` |
| `ui_theme.h` | UI palette (RGB565), grid and metrics |
| `ui_fonts.h` | `UiFont` ladder → Cyrillic U8g2 fonts |
| `ui_text.h` / `.cpp` | `UiText` typography layer (baseline, align, box, printf) |

Typography: all screens except the GEM menu are rendered with
`U8g2_for_Adafruit_GFX` via the `ui` object. The GEM menu keeps its own
Adafruit-GFX fonts (`FontsRus/CourierCyr7.h`, `FontsRus/CourierCyr9.h`).

Next candidates (not yet split): `ConfigManager`, `AutoLevelingController`, valve/pressure tasks.
