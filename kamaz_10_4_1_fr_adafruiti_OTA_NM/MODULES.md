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

## Планируемые модули (Phase 1+)

- ota_net, ota_list, ota_install
- ui_ota_menu, ui_marquee
- imu_dmp, imu_motion, task_imu
- pressure_read, task_calib
- valve_ctrl, auto_level, task_valve
- error_handler, task_recovery, task_watchdog
- wifi_setup
- config_manager, ui_menu_build
