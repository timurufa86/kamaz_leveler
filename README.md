# Kamaz-Leveler

Прошивка системы выравнивания пневмоподвески (ESP32 + FreeRTOS + GitHub OTA).

- Базовая ветка разработки: **`8.5.8`**
- Скетч: `kamaz_8_5_8_fr_adafruiti_OTA_NM/`
- Репозиторий: https://github.com/timurufa86/kamaz_leveler

## Toolchain

- **arduino-cli** 1.5.1+
- Ядро: **esp32:esp32 3.3.11+**
- Библиотеки: из `C:\my\libraries` (подключаются через `arduino-cli.yaml`)

## Сборка

Библиотеки подключаются через проектный `arduino-cli.yaml`, где
`directories.user: C:/my` — то есть sketchbook указывает на `C:\my`, а
библиотеки берутся из `C:\my\libraries`.

```powershell
# Компиляция (экспортирует *.bin в build/esp32.esp32.esp32/)
./build.cmd

# Компиляция + прошивка на подключённую плату
./build.cmd COM5
```

Или напрямую:

```powershell
arduino-cli compile --config-file arduino-cli.yaml `
  -b esp32:esp32:esp32 --warnings none --export-binaries `
  kamaz_8_5_8_fr_adafruiti_OTA_NM
```

> `--warnings none` обязателен: библиотека GEM 1.8.1 содержит функцию без
> `return`, которую ядро ESP32 превращает в ошибку через `-Werror=return-type`.

## Используемые библиотеки

Резолвятся из `C:\my\libraries`:

| Библиотека | Версия |
|---|---|
| ArduinoJson | 7.4.3 |
| Adafruit BusIO | 1.17.4 |
| Adafruit GFX Library | 1.12.6 |
| Adafruit ST7735 and ST7789 Library | 1.11.0 |
| Adafruit ADS1X15 | 2.6.2 |
| EncButton | 3.7.6 |
| GEM | 1.8.1 |
| GyverFilters | 3.2.0 |
| MPU6050 | 1.4.5 |
| U8g2 | 2.36.19 |

## Версионирование и релиз

- SemVer: см. `.cursor/rules/semantic-versioning.mdc` (стиль папок/веток `8.5.8`).
- OTA-релиз: см. `.cursor/rules/github-ota-release.mdc`
  (ассеты `kamaz_leveler.bin` + `kamaz_leveler.bin.sha256`, тег `v8.5.8`).
