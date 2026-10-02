# Модули прошивки Kamaz-Leveler

## Симптом → файл

| Симптом | Куда смотреть |
|---|---|
| TLS / список прошивок / SHA / установка | ota_net, ota_list, ota_install |
| статус меню Обновления / dirty | ui_ota_menu, ui_marquee |
| MPU / углы / MOVEMENT / EMA | imu_dmp, imu_motion, task_imu |
| давление / JHM / калибровка | jhm1200, pressure_read, task_calib |
| клапаны / ручной / авто | valve_ctrl, auto_level, task_valve |
| ошибка / recovery / WDT | error_handler, task_recovery, task_watchdog |
| Wi-Fi скан / STA | wifi_setup |
| NVS / пороги / спиннеры | config_manager, ui_menu_build |

## Извлечённые модули (Phase 0)

| Модуль | Файлы | Статус |
|---|---|---|
| mutex_guard | mutex_guard.h, mutex_guard.cpp | ✅ extracted |
| semver_utils | semver_utils.h | ✅ extracted |
| github_ota_request | github_ota_request.h, github_ota_request.cpp | ✅ extracted |
| ui_theme | ui_theme.h | ✅ extracted |
| ui_fonts | ui_fonts.h | ✅ extracted |
| ui_text | ui_text.h, ui_text.cpp | ✅ extracted |
| icon | icon.h | ✅ extracted |
| jhm1200 | jhm1200.h, jhm1200.cpp | ✅ extracted |
| logger | logger.h, logger.cpp | ✅ extracted |
| task_pool | task_pool.h, task_pool.cpp | ✅ extracted |
| event_bus | event_bus.h, event_bus.cpp | ✅ extracted |
| app_version | app_version.h | ✅ extracted |
| app_pins | app_pins.h | ✅ extracted |
| app_types | app_types.h | ✅ extracted |
| app_globals | app_globals.h, app_globals.cpp | ✅ extracted |

## Извлечённые модули (Phase 2 — IMU)

| Модуль | Файлы | Статус |
|---|---|---|
| imu_dmp | imu_dmp.h, imu_dmp.cpp | ✅ extracted |
| imu_motion | imu_motion.h, imu_motion.cpp | ✅ extracted |
| task_imu | task_imu.h, task_imu.cpp | ✅ extracted |

## Извлечённые модули (Phase 3 — пневматика)

| Модуль | Файлы | Статус |
|---|---|---|
| pressure_read | pressure_read.h, pressure_read.cpp | ✅ extracted |
| valve_ctrl | valve_ctrl.h, valve_ctrl.cpp | ✅ extracted |
| auto_level | auto_level.h, auto_level.cpp | ✅ extracted |
| task_pressure | task_pressure.h, task_pressure.cpp | ✅ extracted |
| task_valve | task_valve.h, task_valve.cpp | ✅ extracted |
| task_control | task_control.h, task_control.cpp | ✅ extracted |
| task_calib | task_calib.h, task_calib.cpp | ✅ extracted |

## Планируемые модули (Phase 4+)

- error_handler, task_recovery, task_watchdog
- config_manager, ui_menu_build
