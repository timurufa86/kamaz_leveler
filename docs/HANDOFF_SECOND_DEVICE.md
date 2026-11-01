# Handoff: добавить второе устройство Kamaz Leveler в Home Assistant

Кратко для агента, который продолжит работу. Всё проверено на первом устройстве.

## Что уже есть

- Репозиторий: `C:\my\for cursor\kamaz_leveler`, ветка `refactor/modular-firmware`, релиз **v10.16.8**.
- Устройство №1: ESP32, прошивка **10.16.8**, статический IP **192.168.0.204**, SSID `lesnaya1/1`.
- HA: **http://192.168.0.131:8123** (HA 2026.8.2 в Docker). SSH: `timur@192.168.0.131` (пароль у владельца).
- Конфиг HA: `/root/.homeassistant` (в контейнере `/config`), контейнер `homeassistant`.

## Как устроена интеграция (HTTP REST)

- Устройство поднимает HTTP-сервер на порту **80**; включается в меню «Система → Дом (HA)».
- Токен `X-Api-Key` (опциональный), задаётся по Serial: `HA TOKEN <строка>`.
- Эндпоинты: `GET /api/state`, `GET|POST /api/config`, `POST /api/mode`, `POST /api/manual`, `POST /api/action`, `GET /api/health`.
- Serial-команды: `HA ON|OFF|STATUS|TOKEN|STATE|CONFIG`, `WIFI STATUS|IP <ip> <gw> <mask> [dns]|IP DHCP`, `OTA VER`.
- Важно: с 10.16.8 при поднятом сервере вызывается `WiFi.setSleep(false)` — иначе HA получает таймауты 5–10 с.

## Файлы в репозитории

| Файл | Назначение |
|---|---|
| `ha/kamaz_leveler_package.yaml` | пакет HA: rest-сенсоры, binary_sensor, `rest_command`, хелперы `input_*`, `sensor.kamaz_config` |
| `ha/kamaz-tilt.js` | кастомная карточка `custom:kamaz-tilt` (зеркальная шкала крена/тангажа) → `/config/www/kamaz-tilt.js` |
| `ha/scripts_kamaz.yaml` | скрипт `kamaz_mode_toggle` (одна кнопка АВТО/РУЧНОЙ) |
| `docs/HA_INTEGRATION.md` | полная инструкция |

## Что нужно, чтобы добавить устройство №2

1. **Прошить** второе устройство 10.16.8 (`build.cmd`, esptool, COM-порт).
2. **Уникальный IP**: `WIFI IP 192.168.0.205 192.168.0.1 255.255.255.0 192.168.0.1` (адрес вне DHCP-пула).
3. **Уникальный токен**: `HA TOKEN <новый>` и включить «Дом (HA)».
4. **HA: отдельный пакет** `packages/kamaz2.yaml` — копия `kamaz_leveler_package.yaml`, где:
   - у каждого сенсора/бинарника своё `name` (напр. `Kamaz2 mode` → `sensor.kamaz2_mode`);
   - URL заменены на `192.168.0.205`;
   - токен из своего секрета `kamaz2_token` (добавить в `secrets.yaml`);
   - хелперы переименованы с префиксом `kamaz2_` (иначе конфликт `input_number.kamaz_*`);
   - `sensor.kamaz2_config` + свои `rest_command` (`kamaz2_*`).
5. **Скрипт** `kamaz2_mode_toggle` в `scripts.yaml` (по образцу `ha/scripts_kamaz.yaml`).
6. **Дашборд**: новый вид «Камазлевер-2» в «Кемпере» (или отдельный дашборд). Кастомная карточка `custom:kamaz-tilt` одна на всех — ей передаётся свой `entity`.
7. Перезапустить HA (пакеты/ресурсы), проверить: `sensor.kamaz2_*` доступны, команды работают.

## Грабли (проверено)

- Ресурс `/local/...` появляется только после **перезапуска HA** (каталог `/config/www` регистрируется на старте).
- Кастомная карточка должна быть добавлена как ресурс типа **module** (`lovelace/resources/create`, `res_type: module`).
- Шапка вида в HA берёт заголовок **первого вида** дашборда — держите нужный вид первым.
- Токен HA-сессии (login flow) живёт 30 минут; `/auth/token` требует **form-urlencoded**, не JSON.
- Дашборды хранятся в `/root/.homeassistant/.storage/lovelace.dashboard_kemper` (править через websocket `lovelace/config` + `lovelace/config/save`, не руками).
- Эндпоинты устройства требуют `X-Api-Key`, если задан токен; `GET /api/health` открыт.
