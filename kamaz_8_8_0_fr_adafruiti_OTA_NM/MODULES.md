# Modules (v8.8.0)

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

## Menu and GitHub updates (8.7.0)

- **Menu entry from every mode:** hold the pair **КН3+КН4** (GPIO15 + GPIO17) for 2 s
  (`VirtButton menuCombo`). Emergency stop is unchanged and has priority: hold
  **КН4 + КН5** (GPIO17 + GPIO34).
- Menu keys: Кн1/Кн2 — move, Кн3 — OK, Кн4 — back (GEM inserts its own "Back" item
  because every sub-page is created with `setParentMenuPage`), Кн5 — save & exit.
- `github_ota_request.h` / `.cpp` now carry four request kinds —
  `CHECK_AND_INSTALL`, `FETCH_LIST`, `INSTALL_INDEX` — plus the selected release index.
- OTA pages live in the sketch: `otaPage` («Обновления»), `otaListPage`
  («Прошивки (GitHub)», up to 5 releases from `/releases?per_page=5`), `otaCardPage`
  (release card: date/size/status/SHA-256, «УСТАНОВИТЬ», «Проверить SHA-256»).
- Dynamic menu text («Информация», «Обновления», список и карточка релиза) is
  refreshed by `refreshDynamicMenu()` right before `gem.drawMenu()`.

Next candidates (not yet split): `ConfigManager`, `AutoLevelingController`, valve/pressure tasks.

## Tunable parameters + service screens (8.8.0)

- `CONFIG_FORMAT_VERSION` = **3**. 25 new fields in `ConfigManager` (single `Config`
  struct): `masterCheckSec`, `manualMaxTimeSec`, `pressureDeadband`,
  `coarseZoneRatio`, `fineZoneRatio`, `worseningRatio`, `movementDurationSec`,
  `movementSettleSec`, `movementCheckSec`, `movementTolerance`, `gyroThreshold`,
  `accelThreshold`, `backlightOffMin`, `frameMs`, `redrawAngleThr`,
  `redrawPressureThr`, `imuMotionDet`, `imuGyroOffX/Y/Z`, `imuAccelOffX/Y/Z`,
  `zeroAngleX/Y`. Old configs (v1/v2) load with defaults for the new keys.
- `ConfigManager::load()` — defaults come from the struct, every key is `constrain`ed;
  `save()` keeps the atomic `config.tmp` → `config.txt` swap, adds `/config.bak` and
  a read-back check; `dumpToSerial()` prints all values on boot and after saving.
- `applyRuntimeSettings()` (called after `load()` / `saveConfig()`) re-applies
  contrast, backlight policy, display timing and the MPU motion threshold without reboot.
- Hard-coded tuning constants replaced by config getters: master check interval,
  manual max time, pressure deadband, movement duration/settle/check/tolerance,
  gyro+accel motion thresholds, display frame period and redraw thresholds,
  auto-mode coarse/fine/worsening ratios, `setMotionDetectionThreshold`.
- Service screens (`enum class ServiceScreen`, drawn in `displayTask`, input handled in
  the button task): `VALVE_TEST` (ПЛ/ПП/ЗЛ/ЗП/НАКАЧКА/СБРОС, valve open while КН3 is
  held, 15 s safety timeout, all valves closed on exit/emergency), `IMU_ZERO_CONFIRM`,
  `IMU_CALIB` (`CalibrateGyro/CalibrateAccel`, DMP restart, offsets → config),
  `MPU_DIAG` (DeviceID, temperature, angles, raw `getMotion6`, offsets).
- New menu pages: `imuPage` («IMU / MPU6050»), `settingsViewPage` («Настройки»,
  live config snapshot via `refreshSettingsView()`).
