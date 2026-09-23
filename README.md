# Kamaz-Leveler

Прошивка системы выравнивания пневмоподвески (ESP32 + FreeRTOS + GitHub OTA).

- Базовая ветка разработки: **`8.6.1`**
- Скетч: `kamaz_8_6_1_fr_adafruiti_OTA_NM/`
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
  -b "esp32:esp32:esp32:PartitionScheme=custom,FlashSize=16M" --warnings none --export-binaries `
  --build-property compiler.c.elf.extra_flags=-Wl,--allow-multiple-definition `
  kamaz_8_6_1_fr_adafruiti_OTA_NM
```

> `--warnings none` обязателен: библиотека GEM 1.8.1 содержит функцию без
> `return`, которую ядро ESP32 превращает в ошибку через `-Werror=return-type`.
> (Значение можно передавать как `-Wl,--allow-multiple-definition` или через
> `-D`, главное — попасть в `compiler.c.elf.extra_flags`.)

> `--build-property compiler.c.elf.extra_flags=-Wl,--allow-multiple-definition`
> обязателен, начиная с 8.6.0: библиотека GEM компилируется целиком (включая
> `GEM_u8g2.cpp`), из-за чего в сборку попадает библиотека `U8g2`, а её
> `src/clib/u8g2_fonts.c` дублирует символы шрифтов из
> `U8g2_for_Adafruit_GFX/src/u8g2_fonts.c`. Флаг разрешает дубликаты: линковщик
> оставляет первое определение, неиспользуемые секции удаляются `--gc-sections`.
> Без флага сборка падает на этапе линковки с `multiple definition of
> 'u8g2_font_...'`.

## Карта флеша (16 МБ, своя таблица разделов)

Плата оснащена **16 МБ** флеш-памяти, поэтому используется **своя схема
разделов** (`kamaz_8_6_1_fr_adafruiti_OTA_NM/partitions.csv`, включается
через `PartitionScheme=custom`):

| Раздел | Смещение | Размер | Назначение |
|---|---|---|---|
| `nvs` | 0x9000 | 20 КБ | Wi-Fi/настройки IDF |
| `otadata` | 0xe000 | 8 КБ | выбор OTA-слота |
| `app0` (ota_0) | 0x10000 | **6.25 МБ** | основной слот прошивки |
| `app1` (ota_1) | 0x650000 | **6.25 МБ** | слот для OTA-обновления |
| `spiffs` | 0xC90000 | 3.375 МБ | LittleFS (`/config.txt`) |
| `coredump` | 0xFF0000 | 64 КБ | дамп при сбое |

Скetch занимает ~1.31 МБ, то есть в каждом app-слоте остаётся **~80%**
свободно (ранее на дефолтной схеме 4 МБ было ~0.04%).

> ⚠️ **Смена карты флеша требует разовой прошивки по USB** (bootloader +
> partitions + app). OTA-обновление с прежней 4-мегабайтной схемы разделы
> НЕ переносит — такие устройства нужно один раз прошить кабелем.
> Устройства, уже собранные на этой схеме, дальше обновляются по воздуху как обычно.

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
| U8g2_for_Adafruit_GFX | 1.8.0 |
| U8g2 (только как источник шрифтов, напрямую не подключается) | 2.36.19 |

## Версионирование и релиз

- SemVer: см. `.cursor/rules/semantic-versioning.mdc` (стиль папок/веток `8.6.1`).
- OTA-релиз: см. `.cursor/rules/github-ota-release.mdc`
  (ассеты `kamaz_leveler.bin` + `kamaz_leveler.bin.sha256`, тег `v8.6.1`).

## Типографика и визуализация (появились в 8.6.0)

Весь текст на экранах выводится через **U8g2_for_Adafruit_GFX** (UTF-8, честные
размеры шрифтов, кириллица) — слой из трёх модулей:

| Модуль | Назначение |
|---|---|
| `ui_theme.h` | палитра RGB565, сетка и метрики экрана (отступы, радиусы, высоты строк) |
| `ui_fonts.h` | шрифтовая лестница `UiFont` (Tiny…Huge) — кириллические шрифты U8g2 |
| `ui_text.h` / `.cpp` | класс `UiText` (экземпляр `ui`): печать по базовой линии, выравнивание и вертикальное центрирование в прямоугольнике, `printf`-варианты без динамической памяти, усечение строк с `..` |

Шрифтовая сетка (все с кириллицей):

| Роль | Шрифт | Применение |
|---|---|---|
| Tiny | `u8g2_font_5x7_t_cyrillic` | подсказки, подписи шкал |
| Small | `u8g2_font_6x13_t_cyrillic` | основной мелкий текст, списки |
| SmallB | `u8g2_font_6x13B_t_cyrillic` | акценты и подписи карточек |
| Body / BodyWide | `u8g2_font_7x13` / `8x13_t_cyrillic` | основной текст |
| Med | `u8g2_font_10x20_t_cyrillic` | заголовки экранов, значения |
| Large / XLarge / Huge | `u8g2_font_inr24/33/46_t_cyrillic` | крупные значения, сплэш |

Важно: `U8g2_for_Adafruit_GFX` **не масштабирует** текст (аналога `setTextSize`
нет), поэтому размер задаётся выбором шрифта — глифы остаются пиксель-в-пиксель
вместо растягивания. Меню GEM (`GEM_adafruit_gfx`) по-прежнему работает на
Adafruit-GFX-шрифтах `CourierCyr7`/`CourierCyr9` из `FontsRus/` — только для
пунктов меню; все остальные экраны переведены на U8g2.
