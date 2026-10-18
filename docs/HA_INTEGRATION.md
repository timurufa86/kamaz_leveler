# Интеграция Kamaz Leveler с Home Assistant (HTTP REST)

Прошивка **10.16.0+** умеет отдавать данные и принимать команды по HTTP в локальной
сети. Брокер MQTT не нужен: устройство поднимает встроенный веб-сервер на порту 80.

## Включение на устройстве

1. Меню → **Система** → **Дом (HA)** → **Интеграция** → включить.
2. Выйти из меню (Кн4/Кн5) — настройка сохранится, сервер поднимется в течение
   пары секунд (нужен подключённый Wi-Fi STA).
3. Там же в строке **Статус** будет адрес вида `http://192.168.0.204/api/state`.
   IP также виден в меню «Информация» и по Serial-команде `HA STATUS`.

Без меню (Serial 115200):

| Команда | Действие |
|---|---|
| `HA ON` / `HA OFF` | включить/выключить и сохранить |
| `HA STATUS` | состояние, IP, задан ли токен |
| `HA STATE` | напечатать JSON телеметрии |
| `HA CONFIG` | напечатать JSON настроек |
| `HA TOKEN <строка>` | задать токен API (X-Api-Key) |
| `HA TOKEN` без строки | сбросить токен |
| `WIFI IP <ip> <gw> <mask> [dns]` | **10.16.3:** статический IP (закрепление адреса) |
| `WIFI IP DHCP` | вернуться на DHCP |
| `WIFI STATUS` | текущий IP и настройки |

> Пароль/токен с экрана устройства не вводится — только по Serial.
> После `WIFI IP …` устройство переподключается; HA-сервер поднимется на новом адресе.
> Статический адрес должен быть **вне пула DHCP** роутера или зарезервирован — иначе
> возможен конфликт с другим клиентом.

## Безопасность

- Работает **только в локальной сети**, без TLS (памяти ESP32 не хватает на
  одновременный TLS и OTA). Не публикуйте порт 80 в интернет.
- Необязательный токен: если задан, все `/api/*` требуют заголовок `X-Api-Key`
  (или `?key=`). `/` и `/api/health` открыты.
- Рекомендуется закрепить IP за устройством в DHCP (по MAC), иначе адрес может
  измениться после перезагрузки роутера.

## API

Порт 80, JSON UTF-8, методы GET/POST, `Connection: close`.

| Метод | Путь | Назначение |
|---|---|---|
| GET | `/` | мини-страница статуса |
| GET | `/api/health` | `{"ok":true,"version":"10.16.0"}` |
| GET | `/api/state` | полный снимок телеметрии |
| GET | `/api/config` | все настройки (токен не отдаётся) |
| POST | `/api/config` | частичное обновление (безопасное подмножество) |
| POST | `/api/mode` | `{"mode":"auto"\|"manual"}` |
| POST | `/api/manual` | `start` / `stop` / `pulse` |
| POST | `/api/action` | `reset_errors`, `stop_all`, `emergency_stop`, `save`, `restart` |

Коды: `200` ок, `202` команда принята в очередь, `400` неверный запрос, `401` нет
токена, `403` поле не разрешено (в ответе `rejected`), `409` конфликт состояния,
`423` заблокировано (аварийная защёлка/OTA/тест), `503` нет Wi-Fi/не откалибровано.

### Примеры

```bash
curl http://192.168.0.204/api/state
curl http://192.168.0.204/api/config
curl -X POST http://192.168.0.204/api/mode \
     -H 'Content-Type: application/json' -d '{"mode":"manual"}'
curl -X POST http://192.168.0.204/api/manual \
     -H 'Content-Type: application/json' -d '{"action":"start","pad":"fl","inflate":true}'
curl -X POST http://192.168.0.204/api/manual \
     -H 'Content-Type: application/json' -d '{"action":"pulse","pad":"rr","inflate":false,"seconds":2}'
curl -X POST http://192.168.0.204/api/config \
     -H 'Content-Type: application/json' -d '{"pressureMin":1.2,"pressureMax":6.5}'
```

С токеном добавьте `-H 'X-Api-Key: <токен>'`.

### Поля `/api/state`

`version, uptime_s, mode (AUTO|MANUAL|MOVEMENT), state, test_state, angles{x,y},
temperature_c, pressure{fl,fr,rl,rr,master}, pressure_valid{...},
calibration{completed,valid}, mpu_ok, jhm_ready, moving, movement_mode_active,
leveling{stage,attempts_hour,niv_count}, manual{active,pad,inflate,elapsed_ms,max_ms,
targets[],target_set[]}, errors{active[],count}, wifi{connected,ssid,ip,rssi},
system{heap_free,heap_min,max_alloc,uptime_h}, ota{in_progress,progress,status},
ha{enabled,server,requests}`

### Какие настройки можно писать

`pressureMin/Max, masterLowBar, pressureDeadband, pressureStabilizeMs,
pressureIdleMin, inflateDelay, releaseDelay, manualMaxTimeSec, masterCheckSec,
tiltThresholdX/Y, nivCount, timeInterval, coarseZoneRatio, fineZoneRatio,
worseningRatio, stableThresholdDeg/HoldMs/TimeoutSec, movementEnabled,
movementPressureFront/Rear, movementDurationSec/SettleSec/CheckSec,
movementTolerance, parkingPressureBar, gyroThreshold, gyroBumpThreshold,
accelThreshold, imuMotionDet, backlightOffMin, frameMs, redrawAngleThr,
redrawPressureThr, haEnabled`.

Остальные (нули углов, офсеты IMU, фильтры Калмана/EMA) — **только чтение** (403).
Запись применяется в RAM сразу и сохраняется на флеш через 10 с (или по
`{"action":"save"}`).

## Home Assistant

1. Скопируйте `ha/kamaz_leveler_package.yaml` в `/config/packages/`.
2. В `configuration.yaml`:
   ```yaml
   homeassistant:
     packages: !include_dir_named packages
   ```
3. Замените `192.168.0.204` на IP устройства (или задайте статический через Serial).
4. Токен (если задан на устройстве) — в `secrets.yaml`:
   ```yaml
   kamaz_token: "ваш-токен"
   ```
   Пакет уже ссылается на `!secret kamaz_token`.
5. Перезагрузите HA. Появятся `sensor.kamaz_*`, `binary_sensor.kamaz_*`,
   `sensor.kamaz_config` (атрибуты — все настройки), хелперы `input_*`,
   а также `rest_command.kamaz_*`.
6. **Вид «Камазлевер» в дашборде «Кемпер»** (добавлен в 10.16.3): статус, углы,
   давления, кнопки АВТО/РУЧНОЙ/СТОП, по-подушечное управление и всплывающая
   панель «⚙ Настройки» (хелперы + «Применить»/«Сохранить»).
   Резервная копия дашборда: `.storage/lovelace.dashboard_kemper.bak-*`.

Логирование: все числовые сущности автоматически пишутся штатным HA Recorder —
отдельный код не нужен. Для долгого хранения добавьте `influxdb`/`statistics`.

## Безопасность пневматики

- Ручные команды запрещены, пока: идёт OTA, взведена аварийная защёлка, идёт тест
  клапанов, нет калибровки, есть критическая ошибка или отказ датчика (`423`).
- Удержание автоматически снимается по `manualMaxTimeSec` (меню «Клапаны»).
- `stop_all` доступен всегда; `emergency_stop` взводит защёлку (сброс — Кн1+Кн4).
- Импульс ставится в очередь (`202`) и выполняется задачей устройства, а не в
  обработчике HTTP.

## Во время OTA

При начале обновления сервер останавливается (освобождает сокеты и ~5 КБ кучи),
после неудачи — поднимается снова. После успешной установки устройство
перезагружается, и сервер поднимается автоматически.

## Ограничения

- Нет TLS/HTTPS (память ESP32), только LAN.
- Нет автообнаружения HA (сущности задаются пакетом).
- Нет статического IP в конфиге — используйте DHCP-резерв.
- MQTT не реализован (при необходимости — отдельная задача).
- В режиме ДВИЖЕНИЕ сервер не работает (безопасность); в АВТО/РУЧНОМ — работает.

## Диагностика

| Симптом | Проверить |
|---|---|
| `HA STATUS` показывает «нет Wi-Fi» | сеть STA в меню «Настроить WiFi», RSSI |
| Пустой ответ / таймаут | IP устройства, ПК в той же подсети, порт 80 не занят |
| `401` | токен задан: передайте `X-Api-Key` |
| `423` | защёлка (сброс Кн1+Кн4), идёт тест или OTA, нет калибровки |
| `409 config busy` | повторите запрос; конфиг занят другой операцией |
| Сервер не поднялся | `HA STATUS` по Serial, лог `[HA] server started` |
