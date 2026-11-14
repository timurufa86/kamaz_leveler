# Модули прошивки Kamaz-Leveler

## Симптом → файл

| Симптом | Куда смотреть |
|---|---|
| TLS / список прошивок / SHA / установка | ota_net, ota_list, ota_install |
| статус меню Обновления / dirty | ui_ota_menu, ui_marquee |
| MPU / углы / MOVEMENT / EMA | imu_dmp, imu_motion, task_imu |
| давление / JHM / калибровка | jhm1200, pressure_read, task_calib |
| клапаны / ручной / авто | valve_ctrl, auto_level, task_valve |
| плата v3: клапаны/кнопки/ST через расширитель | hw_map, pcal9555, app_pins, btn_io |
| /VLV_EN, /INT_MCU, /SPEED_PULSE, /PRES_SW | hw_map |
| ошибка / recovery / WDT | error_handler, task_recovery, task_watchdog, task_monitor |
| Wi-Fi скан / STA | wifi_setup |
| NVS / пороги / спиннеры | config_manager, ui_menu_build |
| serial TEST / симуляция | test_harness |
| тест клапанов / служебные экраны | service_ui, ui_screens |
| ArduinoOTA (AP) | arduino_ota |

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

## Извлечённые модули (Phase 1 — сеть и OTA)

| Модуль | Файлы | Статус |
|---|---|---|
| ota_net | ota_net.h, ota_net.cpp | ✅ extracted |
| ota_list | ota_list.h, ota_list.cpp | ✅ extracted |
| ota_install | ota_install.h, ota_install.cpp | ✅ extracted |
| ui_ota_menu | ui_ota_menu.h, ui_ota_menu.cpp | ✅ extracted |
| ui_marquee | ui_marquee.h, ui_marquee.cpp | ✅ extracted |
| wifi_setup | wifi_setup.h, wifi_setup.cpp | ✅ extracted |
| task_ota | task_ota.h, task_ota.cpp | ✅ extracted |

## 10.16.0 — Home Assistant (HTTP REST)

| Модуль | Файлы | Статус |
|---|---|---|
| ha_api | ha_api.h, ha_api.cpp | ✅ новый (сервер :80, `/api/*`, safety gate, Serial `HA …`) |
| mode_ctrl | mode_ctrl.h, mode_ctrl.cpp | ✅ новый (общее переключение АВТО/РУЧНОЙ) |
| HA-пакет | `ha/kamaz_leveler_package.yaml`, `ha/kamaz_leveler_dashboard.yaml` | ✅ для импорта в HA |
| Документация | `docs/HA_INTEGRATION.md` | ✅

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

## Извлечённые модули (Phase 4 — UI)

| Модуль | Файлы | Статус |
|---|---|---|
| ui_screens | ui_screens.h, ui_screens.cpp | ✅ extracted |
| ui_menu_build | ui_menu_build.h, ui_menu_build.cpp | ✅ extracted |
| task_button | task_button.h, task_button.cpp | ✅ extracted |
| task_display | task_display.h, task_display.cpp | ✅ extracted |

## Извлечённые модули (Phase 5 — инфраструктура)

| Модуль | Файлы | Статус |
|---|---|---|
| config_manager | config_manager.h, config_manager.cpp | ✅ extracted |
| error_handler | error_handler.h, error_handler.cpp | ✅ extracted |
| task_monitor | task_monitor.h, task_monitor.cpp | ✅ extracted |
| memory_monitor | memory_monitor.h, memory_monitor.cpp | ✅ extracted |
| task_watchdog | task_watchdog.h, task_watchdog.cpp | ✅ extracted |
| task_recovery | task_recovery.h, task_recovery.cpp | ✅ extracted |
| task_event | task_event.h, task_event.cpp | ✅ extracted |
| test_harness | test_harness.h, test_harness.cpp | ✅ extracted |
| service_ui | service_ui.h, service_ui.cpp | ✅ extracted |
| arduino_ota | arduino_ota.h, arduino_ota.cpp | ✅ extracted |

## 3.0.0 — новая плата (ESP32-S3 + PCAL9555A)

| Модуль / файл | Что изменилось |
|---|---|
| `pcal9555.h`, `pcal9555.cpp` | ✅ **новый** драйвер PCAL9555APW (I2C 0x20): init, конфиг портов, запись/чтение портов, теневые выходы, обработка INT, аварийный останов через /VLV_EN |
| `hw_map.h` | переписан под плату v3: GPIO 14/13/8/38; VLV_IN1..6 = P1_0..P1_5, BUT1..5 = P0_0..P0_4, ST_U4 = P0_5/P0_6, ST_U3 = P1_6/P1_7; виртуальные пины кнопок BTN_VPIN_* |
| `app_pins.h` | только реальные GPIO S3: I2C 21/47, дисплей 4/5/11/16/17/18; PIN_BUB*/PIN_BUT* удалены |
| `app_version.h` | `V 3.0.0 FreeRTOS OTA` |
| `btn_io.*`, `task_button.cpp` | чтение кнопок из кэша PCAL (`pcalPoll()` раз в цикл), инъекция расширена до КН1..КН5 |
| `valve_ctrl.cpp`, `task_valve.cpp` | клапаны пишутся в PCAL (`pcalValvePad/FlowSelect/FlowOff`), аварийный останов = /VLV_EN HIGH |
| `service_ui.cpp`, `ui_screens.h` | ручной тест клапанов по маскам PCAL (`manualValveMasks`) |
| `task_calib.cpp` | клапан сброса через `pcalDefl()` |
| `test_harness.cpp` | `TEST PCAL` — состояние расширителя, `TEST BTN` читает PCAL |
| `app_globals.*` | `bubPins[]` удалён (пинов клапанов больше нет) |
| `.ino` | переименован в `kamaz_3_0_0_fr_adafruiti_OTA_NM.ino`; SPIClass(FSPI) вместо VSPI; /VLV_EN=HIGH первым в setup(), `pcalBegin()` после Wire, буфер включается после обнуления выходов |

Порядок безопасности: /VLV_EN=HIGH (клапаны закрыты) → конфиг PCAL + выходы 0 →
проверка чтением → включение буфера U5. Любая ошибка I2C → буфер выключается,
повторное включение только через `resetEmergencyLatch()` / `pcalEnableValveBuffer(true)`.

## Роль .ino

`kamaz_3_0_0_fr_adafruiti_OTA_NM.ino` — только проводка: includes, объекты GEM/TFT/SPI/кнопок, зеркала пунктов меню, `setup()` / `loop()`. Бизнес-логика в модулях выше.
