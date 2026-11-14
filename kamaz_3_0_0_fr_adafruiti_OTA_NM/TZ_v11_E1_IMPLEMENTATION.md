# ТЗ v11 — ЭТАП E1 (детальная реализация): PCAL9555APW, клапаны, кнопки, VLV_EN

> Этот документ — пошаговая реализация **этапа E1** из `TZ_v11_PCAL9555_SENSORS.md`.
> База: `kamaz_10_15_6_fr_adafruiti_OTA_NM` (Arduino/ESP32, FreeRTOS).
> Готовый заголовок карты железа: **`hw_map.h`** (уже в проекте, не менять без согласования).
> Номера строк — по состоянию v10.15.6; перед правкой подтверждать поиском (`grep`/Find).

## 0. Результат E1 (definition of done)

- Все 6 клапанов управляются **только** через PCAL9555APW (P00…P05), прямых `digitalWrite` на GPIO клапанов не осталось.
- 5 кнопок читаются через PCAL (P12…P16), существующие EncButton-комбо и меню работают; KN5 добавлена.
- `VLV_EN` (GPIO32) инициализируется **первым** в `setup()`: при сбросе/ошибке PCAL клапаны закрыты; после успешной инициализации — разрешены.
- Сборка без ошибок; на стенде: переключение всех клапанов, индикаторы D11–D16, кнопки/комбо, аварийная защёлка.
- ST-диагностика, скорость и давление **в E1 не реализуются** (только заглушки/лог, этапы E2…E4).

## 1. Порядок работ (чек-лист)

1. Добавить `hw_map.h` (готов) — проверить компиляцию.
2. Создать `pcal9555.h/.cpp` (§3), собрать.
3. Правки `app_pins.h` (§4), `app_globals` (§5).
4. Правки `valve_ctrl.h/.cpp` (§6), `task_valve.cpp` (§7), `service_ui.cpp`/`task_calib.cpp` (§8).
5. Правки `btn_io.h/.cpp` (§9), `task_button.cpp` (§10), `.ino` (§11).
6. Сборка + стенд по §12; зафиксировать результаты, коммит `E1: PCAL9555 valves/buttons + VLV_EN`.

---

## 2. Что НЕЛЬЗЯ делать в E1

- Не трогать `pressure_read.*`, `task_pressure.*`, `jhm1200.*`, `imu_*`, OTA/Wi-Fi/дисплей — только перечисленные ниже файлы.
- Не менять публичный API `valve_ctrl.h` (`setValve`, `closeAllValves`, `emergencyStop`, `maneuver`-функции) — только **добавить** вспомогательные функции.
- Не убирать `xValveMutex`-логику и таймауты `task_valve`.
- Не инициализировать GPIO 13/12/15/17/34/25 как реальные входы/выходы (на плате это NC или новые функции). 13/12/15/17/34 — только «виртуальные ID» кнопок.

---

## 3. Новый драйвер `pcal9555.h/.cpp`

### 3.1 `pcal9555.h` (полный текст)

```cpp
#pragma once
/**
 * pcal9555.h — драйвер I2C-расширителя PCAL9555APW (U6, адрес 0x20).
 * Все обращения — под xI2CMutex (глобальный мьютекс шины).
 * Транзакции: запись/чтение пары байт регистров (auto-increment адреса).
 */
#include <Arduino.h>
#include "hw_map.h"

class Pcal9555 {
 public:
  bool begin();                       // polarity=0, config: P00..P05 выходы, остальные входы; outputs=0 + read-back
  bool isPresent() const { return present_; }

  bool readInputs(uint16_t &value);   // 0x00/0x01 → 16 бит (P00..P17)
  bool readOutputs(uint16_t &value);  // 0x02/0x03 (read-back)
  bool writeOutputs(uint16_t value);  // 0x02/0x03 + read-back
  bool setOutputBits(uint16_t mask);  // read-modify-write
  bool clearOutputBits(uint16_t mask);

  uint16_t lastInputs()  const { return lastInputs_; }
  uint16_t lastOutputs() const { return lastOutputs_; }
  uint32_t errorCount()  const { return errorCount_; }
  void clearErrorCount()       { errorCount_ = 0; }

 private:
  bool writeRegs(uint8_t reg, const uint8_t *data, uint8_t n);
  bool readRegs(uint8_t reg, uint8_t *data, uint8_t n);

  bool present_ = false;
  uint16_t lastInputs_  = 0xFFFF;
  uint16_t lastOutputs_ = 0x0000;
  uint32_t errorCount_  = 0;
};

extern Pcal9555 g_pcal;
```

### 3.2 `pcal9555.cpp` — требования к реализации

- `writeRegs(reg, data, n)`: взять `xI2CMutex` (`xSemaphoreTake(..., pdMS_TO_TICKS(20))`), `Wire.beginTransmission(PCAL_I2C_ADDR)`, `Wire.write(reg)`, затем `n` байт, `Wire.endTransmission()==0` → успех; отдать мьютекс; при ошибке `errorCount_++`.
- `readRegs(reg, data, n)`: `beginTransmission` → `write(reg)` → `endTransmission(false)` → `requestFrom(addr, n)` → читать байты; проверять количество; под тем же мьютексом.
- Повторы: до `PCAL_I2C_RETRIES` (3) попыток на операцию; между попытками `delay(1)`.
- `begin()`:
  1) `writeRegs(0x04, {0,0}, 2)` — polarity = 0 (не инвертировать P00…P17);
  2) `writeRegs(0x06, {0xC0, 0xFF}, 2)` — config: P00…P05 = выходы (0), P06,P07,P10…P17 = входы (1);
  3) `writeOutputs(0x0000)` — все клапаны закрыты;
  4) `readOutputs(v)` — read-back: `v == 0x0000` → `present_ = true`;
  5) `readInputs(lastInputs_)` — прогрев кэша кнопок/ST;
  6) лог: `[PCAL] ok addr=0x20 in=0x%04X out=0x%04X`.
  При любой ошибке: `present_ = false`, `errorCount_++`, лог `[PCAL] begin failed`.
- **Agile I/O регистры (0x40…0x4F) в E1 не писать** — не требуются (входы U5 имеют только утечку; push-pull 3,3 В достаточно). Задел на E2/E4: `configureAgileOutputs()` можно добавить позже.
- ISR/прерывания в E1 не обязательны: `btnPoll()` читает входы раз в 50 мс (см. §9). INT (IO33) подключить в E2 (ST).
- В `setup()` **до** `begin()`: `pinMode(PIN_VLV_EN, OUTPUT); digitalWrite(PIN_VLV_EN, HIGH);` (см. §11).

---

## 4. `app_pins.h` — новая версия (полная замена файла)

```cpp
#pragma once
#include <Arduino.h>
#include "hw_map.h"

/* I2C (JHM1200 / MPU6050 / PCAL9555APW / J4 / J6) */
constexpr uint8_t PIN_OLED_SDA = 21;
constexpr uint8_t PIN_OLED_SCL = 22;

/* TFT ST7789 (SPI) */
constexpr uint8_t PIN_TFT_MOSI = 23;
constexpr uint8_t PIN_TFT_SCLK = 18;
constexpr uint8_t PIN_TFT_CS   = 5;
constexpr uint8_t PIN_TFT_DC   = 16;
constexpr uint8_t PIN_TFT_RST  = 19;
constexpr uint8_t PIN_TFT_BL   = 4;

/* Клапаны: только через PCAL9555APW, см. hw_map.h
 *  VLV_EN (32) — OE буфера U5; PIN_SPEED_IN (35), PIN_PRES_SW (25) — датчики (E3/E4).
 *  GPIO 32/33/25/26/27/14 на плате v2 больше не клапаны. */

/* Кнопки: «виртуальные ID» для EncButton/btn_io (НЕ GPIO!) */
constexpr uint8_t PIN_BUT1 = BTN_VPIN_KN1;   // 13
constexpr uint8_t PIN_BUT2 = BTN_VPIN_KN2;   // 12
constexpr uint8_t PIN_BUT3 = BTN_VPIN_KN3;   // 15
constexpr uint8_t PIN_BUT4 = BTN_VPIN_KN4;   // 17
constexpr uint8_t PIN_BUT5 = BTN_VPIN_KN5;   // 34
```

Удалить: `PIN_BUB1..PIN_BUB4`, `PIN_INFL`, `PIN_DEFL`.
Проверка: `grep -rn "PIN_BUB\|PIN_INFL\|PIN_DEFL" *.cpp *.h *.ino` → пусто (после §6–§8).

---

## 5. `app_globals.h/.cpp`

- `app_globals.h`: удалить `extern const uint8_t bubPins[PAD_COUNT];`.
- `app_globals.cpp`: удалить строку
  `const uint8_t bubPins[PAD_COUNT] = { PIN_BUB1, PIN_BUB2, PIN_BUB3, PIN_BUB4 };`.
- (опционально) в `app_globals.h` добавить:
  ```cpp
  extern volatile bool g_valvesEnabled;   // true после успешного initValveHardware()
  ```
  Определение в `valve_ctrl.cpp` (§6).

---

## 6. `valve_ctrl.h/.cpp`

### 6.1 Добавить в `valve_ctrl.h` (в конец, после существующего API)

```cpp
/* ── Hardware helpers (плата v2, PCAL9555APW) ── */
/** Разрешение буфера U5: enable=false → VLV_EN=HIGH (клапаны принудительно закрыты). */
void hwValveEnable(bool enable);
/** Инициализация: VLV_EN=HIGH → PCAL begin → outputs=0 → VLV_EN=LOW (только при успехе). */
bool initValveHardware();
bool valveHardwareReady();

/** Прямые операции с битами PCAL (без xValveMutex — как прежний digitalWrite). */
void hwValvePad(Pad pad, bool on);
void hwValveInfl(bool on);
void hwValveDefl(bool on);
void hwValveAllOff();
```

### 6.2 `valve_ctrl.cpp` — вставить реализации в начало (после `#include`)

```cpp
#include "pcal9555.h"
#include "hw_map.h"

volatile bool g_valvesEnabled = false;

bool valveHardwareReady() { return g_pcal.isPresent(); }

void hwValveEnable(bool enable) {
  digitalWrite(PIN_VLV_EN, enable ? LOW : HIGH);
  g_valvesEnabled = enable;
}

bool initValveHardware() {
  pinMode(PIN_VLV_EN, OUTPUT);
  digitalWrite(PIN_VLV_EN, HIGH);           // fail-safe: буфер Hi-Z
  g_valvesEnabled = false;
  const bool ok = g_pcal.begin();
  if (ok) {
    hwValveAllOff();
    digitalWrite(PIN_VLV_EN, LOW);          // разрешаем буфер только после outputs=0
    g_valvesEnabled = true;
  } else {
    digitalWrite(PIN_VLV_EN, HIGH);
  }
  return ok;
}

void hwValvePad(Pad pad, bool on) {
  if (pad >= PAD_COUNT) return;
  const uint16_t m = pcalPadMask(pad);
  if (on) g_pcal.setOutputBits(m); else g_pcal.clearOutputBits(m);
}
void hwValveInfl(bool on) { if (on) g_pcal.setOutputBits(PCAL_VLV_INFL); else g_pcal.clearOutputBits(PCAL_VLV_INFL); }
void hwValveDefl(bool on) { if (on) g_pcal.setOutputBits(PCAL_VLV_DEFL); else g_pcal.clearOutputBits(PCAL_VLV_DEFL); }
void hwValveAllOff()      { g_pcal.clearOutputBits(PCAL_VLV_ALL); }
```

### 6.3 Замены в `valve_ctrl.cpp` (все прямые GPIO клапанов)

| Строка (v10.15.6) | Было | Стало |
|---|---|---|
| 33 | `for (...) digitalWrite(bubPins[i], LOW);` | `g_pcal.clearOutputBits(PCAL_VLV_ALL);` |
| 34 | `digitalWrite(PIN_INFL, LOW);` | (удалить — покрыто строкой 33) |
| 35 | `digitalWrite(PIN_DEFL, LOW);` | (удалить — покрыто строкой 33) |
| 71 | `digitalWrite(bubPins[pad], state);` | `hwValvePad(pad, state);` |
| 76-78 | `for ... bubPins` + INFL + DEFL | `hwValveAllOff();` |
| 94-96 | `for ... bubPins` + INFL + DEFL | `hwValveAllOff();` |
| 259 | `digitalWrite(PIN_INFL, HIGH);` | `hwValveInfl(true);` |
| 262 | `digitalWrite(PIN_INFL, LOW);` | `hwValveInfl(false);` |
| 358,359 | `digitalWrite(PIN_INFL/DEFL, LOW);` | `hwValveInfl(false); hwValveDefl(false);` |
| 497,498 | то же | то же |
| 632,633 | то же | то же |
| 783,784 | то же | то же |

**`forceAllValvePinsLow()`** (строки 32–36) — заменить тело на аварийный путь без I²C:

```cpp
static void forceAllValvePinsLow() {
  digitalWrite(PIN_VLV_EN, HIGH);   // аппаратно: буфер Hi-Z → все клапаны закрыты
  g_pcal.clearOutputBits(PCAL_VLV_ALL);  // best effort (если I²C жив)
  g_valvesEnabled = false;
}
```

**`emergencyStop()`** (строка ~82): в самое начало (после `s_emergencyLatched = true;` и `valveStopRequested = true;`) добавить `digitalWrite(PIN_VLV_EN, HIGH); g_valvesEnabled = false;`, далее — существующая логика. В блоке, где после ошибки mutex пишет `for (...) digitalWrite(bubPins[i], LOW)`, заменить на `hwValveAllOff();`.

**Замечание:** после успешного `emergencyStop` клапаны обесточены; разрешение буфера — только через `initValveHardware()`/`hwValveEnable(true)` по решению оператора (см. E2, сброс защёлки).

---

## 7. `task_valve.cpp` — замена 4 строк

| Строка | Было | Стало |
|---|---|---|
| 98 | `digitalWrite(PIN_INFL, LOW);` | `hwValveInfl(false);` |
| 99 | `digitalWrite(PIN_DEFL, LOW);` | `hwValveDefl(false);` |
| 126 | `digitalWrite(PIN_INFL, LOW);` | `hwValveInfl(false);` |
| 128 | `digitalWrite(PIN_DEFL, LOW);` | `hwValveDefl(false);` |

`setValve(...)` в этом файле уже маршрутизируется через §6 и менять вызовы не нужно.
Добавить в начало цикла (после `TaskPool::markRun`): проверку
`if (!valveHardwareReady()) { closeAllValves(); vTaskDelay(pdMS_TO_TICKS(50)); continue; }`
— при отсутствии PCAL клапаны закрыты, команды не выполняются (лог с rate-limit 1/с).

---

## 8. `service_ui.cpp`, `task_calib.cpp` — прямые GPIO

`service_ui.cpp` (13 мест) и `task_calib.cpp` (2 места) пишут `digitalWrite(PIN_INFL/PIN_DEFL, …)`.
Механическая замена:

- `digitalWrite(PIN_INFL, HIGH)` → `hwValveInfl(true)`
- `digitalWrite(PIN_INFL, LOW)`  → `hwValveInfl(false)`
- `digitalWrite(PIN_DEFL, HIGH)` → `hwValveDefl(true)`
- `digitalWrite(PIN_DEFL, LOW)`  → `hwValveDefl(false)`

Строки `service_ui.cpp`: 428, 439, 455, 478, 485, 499, 507, 562, 573, 589, 600, 619, 623.
`task_calib.cpp`: 108, 183.
Добавить `#include "valve_ctrl.h"` в оба файла, если ещё нет.

---

## 9. `btn_io.h/.cpp` — кнопки через PCAL

### 9.1 `btn_io.h` — добавить

```cpp
/** Прочитать входы PCAL в кэш (вызывать из ButtonTask перед чтением кнопок). */
void btnPoll();
```

### 9.2 `btn_io.cpp` — заменить начало файла (до тестовых функций)

```cpp
#include "btn_io.h"
#include "app_pins.h"
#include "hw_map.h"
#include "pcal9555.h"

static uint16_t s_inCache = 0xFFFF;   // активный низкий: 1 = отпущена
static bool     s_inValid = false;

static uint8_t s_injectMask = 0;      // bit0..4 = KN1..KN5
static uint8_t s_injectDown = 0;      // bit=1 → нажата (LOW)
static uint8_t s_forceEdge = 0;
static uint8_t s_forceRelease = 0;

void btnPoll() {
  uint16_t v = 0xFFFF;
  if (g_pcal.readInputs(v)) {
    s_inCache = v;
    s_inValid = true;
  } else {
    s_inValid = false;               // fail-safe: считаем все кнопки отпущенными
  }
}

static int knFromPin(uint8_t pin) {
  switch (pin) {
    case PIN_BUT1: return 0;
    case PIN_BUT2: return 1;
    case PIN_BUT3: return 2;
    case PIN_BUT4: return 3;
    case PIN_BUT5: return 4;
    default: return -1;
  }
}

uint8_t btnDigitalRead(uint8_t pin) {
  const int kn = knFromPin(pin);
  if (kn < 0) return HIGH;
  if (s_injectMask & (1u << kn)) {                 // инъекция автотеста
    return (s_injectDown & (1u << kn)) ? LOW : HIGH;
  }
  if (!s_inValid) return HIGH;                     // нет связи с PCAL → отпущено
  const uint16_t bit = (uint16_t)(PCAL_BTN_KN1 << kn);  // KN1..KN5 = P12..P16
  return (s_inCache & bit) ? HIGH : LOW;           // активный низкий
}
```

Остальные функции (`btnIsLow`, инъекция, `btnInjectSet/Clear`, `btnConsumeForceEdgeMask`, dry-run, тестовые) — **оставить**, расширив `btnInjectSet` на `kn = 1..5` (`bit = 1u << (kn - 1)`), а `btnInjectClear` — `s_forceRelease = 0x1F`.
`PIN_BUT*` брать из `app_pins.h` (виртуальные ID) — как сейчас.

---

## 10. `task_button.cpp`

1. В начале файла, рядом с `extern Button button0..3`, добавить:
   ```cpp
   extern Button button4;   // KN5
   ```
2. В `for(;;)` после интервальной проверки (`lastButtonCheck = now;`, строка ~137) добавить:
   ```cpp
   btnPoll();   // входы PCAL в кэш (I2C ~1 мс, 1 раз / 50 мс)
   ```
3. После тиков `button0..3` (строки 140–143) добавить:
   ```cpp
   static_cast<VirtButton &>(button4).tick(btnDigitalRead(PIN_BUT5));
   ```
4. После `syncBtnGpio(button3, PIN_BUT4);` добавить:
   ```cpp
   syncBtnGpio(button4, PIN_BUT5);
   ```
5. В блоке инициализации `buttonsArmed` (строки ~181–188) добавить `button4.reset();` рядом с `button0.reset();`.
6. KN5 = сброс аварийной защёлки: сразу после блока `emergencyButton` (строки ~295–316), добавить:
   ```cpp
   if (buttonsArmed && btn5Enabled() && button4.hold()) {
     if (emergencyIsLatched()) {
       resetEmergencyLatch();
       Logger::log(Logger::WARNING, "BTN", "KN5: сброс аварийной защёлки");
     }
   }
   ```
   `btn5Enabled()` — временно `return true;` (E5: из ConfigManager).
7. Строки лога `prev[4]/nowPress[4]` (строки ~196–210) — оставить для 4 кнопок; KN5 в лог добавить отдельно при желании.

---

## 11. `.ino` (kamaz_10_15_6_…ino)

### 11.1 Include
Добавить `#include "hw_map.h"` и `#include "pcal9555.h"` (после `app_pins.h`).

### 11.2 Кнопки (строки 186–193)
```cpp
Button button0(PIN_BUT1, INPUT_PULLUP, LOW);
Button button1(PIN_BUT2, INPUT_PULLUP, LOW);
Button button2(PIN_BUT3, INPUT_PULLUP, LOW);
Button button3(PIN_BUT4, INPUT_PULLUP, LOW);
Button button4(PIN_BUT5, INPUT, LOW);   // KN5: виртуальный пин, реальный GPIO не используется
```
Остальные объекты (`MultiButton …`) — без изменений.

### 11.3 `setup()` — самое начало (строки 198–205)
```cpp
  // Fail-safe: буфер U5 в Hi-Z ещё до инициализации I2C/PCAL.
  pinMode(PIN_VLV_EN, OUTPUT);
  digitalWrite(PIN_VLV_EN, HIGH);
```
(удалить `pinMode`-циклы по `bubPins` и `PIN_INFL/PIN_DEFL`, `closeAllValves()` до `Wire`).

### 11.4 После `Wire.begin(...)` и `Wire.setClock(...)` (строки ~288–290)
```cpp
  // PCAL9555APW: polarity/config, outputs=0, read-back; при успехе — VLV_EN=LOW.
  if (!initValveHardware()) {
    Serial.println("[ERROR] PCAL9555 init failed — valves disabled (VLV_EN=HIGH)");
  }
```

### 11.5 Второй блок «ВЫВОДЫ КЛАПАНОВ» (строки 449–453)
Заменить на:
```cpp
  // Клапаны уже инициализированы в initValveHardware(); дополнительных действий не нужно.
  if (!valveHardwareReady()) {
    hwValveEnable(false);   // остаёмся в safe state
  }
```

### 11.6 Конфигурация EncButton (строки 481–513)
- `cfgBtn(button4, PAD_HOLD_MS);`
- `button4.reset();` рядом с остальными.

### 11.7 Задачи
Новых задач в E1 нет: `btnPoll()` идёт из `ButtonTask` (50 мс), запись клапанов — из `ValveTask`/`valve_ctrl`.

---

## 12. Сборка и приёмка E1

### 12.1 Команда сборки

**Важно (проверено на v10.15.6, сборка проходит):**
- `--build-path "C:\my\fwbuild"` обязателен: без него сборка падает на `fatal error: opening dependency file …\arduino\sketches\…` из‑за длинного пути/временного каталога.
- `--build-property compiler.*.elf.extra_flags=-Wl,--allow-multiple-definition` обязателен: U8g2 и U8g2_for_Adafruit_GFX дают `multiple definition` шрифтов (в исходном `build.options.json` эта опция была).
- Baseline v10.15.6: **flash 1 607 230 Б (9 %), RAM globals 86 680 Б (26 %), свободно 241 000 Б** — новые проверки (E1 §12.2) сравнивать с этим.

```bat
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile ^
  --fqbn "esp32:esp32:esp32:PartitionScheme=custom,FlashSize=4M" ^
  --libraries "C:\my\libraries" ^
  --build-path "C:\my\fwbuild" ^
  --build-property "compiler.c.elf.extra_flags=-Wl,--allow-multiple-definition" ^
  --build-property "compiler.cpp.elf.extra_flags=-Wl,--allow-multiple-definition" ^
  "C:\my\for cursor\kamaz_leveler\kamaz_10_15_6_fr_adafruiti_OTA_NM"
```

Критерий: `Sketch uses ...`, без ошибок; предупреждения — сравнить с baseline v10.15.6 (не должно стать больше).

### 12.2 Ожидаемые логи Serial (115200)

```
[PCAL] begin ok addr=0x20 in=0xFFFF out=0x0000
[SETUP] ... 
[BTN] armed (idle settle)
```
Если PCAL не отвечает: `[PCAL] begin failed`, `[ERROR] PCAL9555 init failed — valves disabled (VLV_EN=HIGH)`; клапаны закрыты при любых командах.

### 12.3 Стенд (все пункты обязательны)

1. **Reset**: до и после загрузки — на `VLV_EN` HIGH до `initValveHardware()`, затем LOW; клапаны обесточены.
2. **Клапаны**: сервисный экран/существующие функции — поочерёдно ПЛ, ПП, ЗЛ, ЗП, INFL, DEFL;
   проверить щелчок/питание катушки и LED-индикаторы D11–D16 (LED под своим пином J2).
3. **Кнопки**: KN1…KN5 читаются в логе; combo: КН1+КН2 (АВТО/РУЧ), КН3+КН4 (меню), КН1+КН4 (авария);
   KN5 hold ≥ 2 с при активной защёлке — сбрасывает.
4. **Авария**: `emergencyStop()` → `VLV_EN` HIGH ≤ 1 мс, все клапаны закрыты, защёлка активна.
5. **Сбой PCAL**: отключить питание U6 или SDA → ошибка в логе, клапаны закрыты; восстановить — реинициализация (E2: автоматическая, в E1 допустим reset).
6. **Регресс**: OTA-меню, Wi-Fi, дисплей, калибровка JHM — не сломаны; `ValveTask` не зависает (WDT не срабатывает).

### 12.4 Отчёт по E1 (в PR)

- diff: список изменённых файлов и число замен `digitalWrite`→helpers;
- `grep`-подтверждение: `PIN_BUB|PIN_INFL|PIN_DEFL|bubPins` в коде — не встречаются;
- логи загрузки, результат стенда §12.3 (кратко), номер сборки;
- коммит: `E1: PCAL9555APW — клапаны/кнопки через I2C, VLV_EN fail-safe`.

---

## 13. Подводные камни (проверить обязательно)

1. **Порядок байт PCAL**: запись/чтение пары регистров — сначала младший адрес (0x00/0x02), порядок байт little-endian (P00…P07 = первый байт).
2. **Read-back обязателен**: PCAL может «принять» запись, но не сформировать уровень при проблеме питания буфера — сравнивать `readOutputs()` после каждой записи в `task_valve`/`emergencyStop` (E2 добавит ошибки).
3. **VLV_EN polarity**: `LOW` = буфер включён; `HIGH` = Hi-Z. Не перепутать с «1 = разрешено».
4. **Никакого I2C из ISR** (в E1 ISR вообще не нужны).
5. **EncButton**: `Button button4(PIN_BUT5, INPUT, LOW)` — для input-only пина 34 не использовать `INPUT_PULLUP` (предупреждение драйвера).
6. **xI2CMutex**: `pcal9555` берёт его сам; не вызывать `Wire` из-под чужого мьютекса повторно (не оборачивать `hwValve*` в `xI2CMutex` снаружи).
7. **`grep "digitalWrite(PIN_INFL"`** после правок должен давать 0 результатов по всему проекту.
8. **Стек**: `btnPoll()` в ButtonTask добавляет ~50 байт; новых задач нет — лимит не превышен.
