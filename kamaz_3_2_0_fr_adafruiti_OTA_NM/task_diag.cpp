/**
 *  task_diag.cpp — 3.0.0: /ST BTS724G, датчик скорости (PCNT), PRES_SW.
 *  Карта железа — hw_map.h; подробности контракта — task_diag.h.
 */
#include "task_diag.h"

#include "app_globals.h"
#include "config_manager.h"
#include "error_handler.h"
#include "logger.h"
#include "pcal9555.h"
#include "task_pool.h"
#include "valve_ctrl.h"
#include "mode_ctrl.h"   // 3.2.0: остановка АВТО при срабатывании СК

#include "driver/pulse_cnt.h"
#include "esp_err.h"

extern uint8_t taskIndex_Diag;

/* ======================================================================
 *  /ST BTS724G: 4 линии, активный низкий (0 = обрыв/КЗ/перегрев пары).
 *  pcalStFaults(): 1 = линия отпущена (норма), 0 = неисправность.
 * ==================================================================== */

const DiagStLine DIAG_ST_LINES[4] = {
    { PCAL_ST_U3_12, "U3_12" },
    { PCAL_ST_U3_34, "U3_34" },
    { PCAL_ST_U4_12, "U4_12" },
    { PCAL_ST_U4_34, "U4_34" },
};

/** Своя строка на каждую группу каналов (требование ТЗ). */
static const char *const ST_MSGS[4] = {
    "ST U3 1-2: обрыв/КЗ",   // BUB1+BUB2
    "ST U3 3-4: обрыв/КЗ",   // BUB3+BUB4
    "ST U4 1-2: обрыв/КЗ",   // INFL+DEFL
    "ST U4 3-4: обрыв/КЗ",   // неиспользуемые каналы U4
};

struct StTracker {
  bool     rawHealthy;     // последний отсчёт (1 = линия отпущена)
  bool     stableHealthy;  // после антидребезга
  uint8_t  agree;          // подряд одинаковых отсчётов
  uint32_t healthySince;   // момент подтверждённого перехода в «норма» (0 = ещё не было)
  bool     reported;       // ошибка по этой группе поднята
};

static StTracker s_st[4] = {
    { true, true, 0, 0, false },
    { true, true, 0, 0, false },
    { true, true, 0, 0, false },
    { true, true, 0, 0, false },
};
static bool     s_stErrRaised   = false;  // ошибка VALVE поднята этим модулем
static bool     s_stLatchOwned  = false;  // аварийную защёлку поставил именно ST-модуль
static uint32_t s_stRefreshMs   = 0;      // последнее обновление lastSeenTime ошибки

/* ── Датчик скорости: аппаратный PCNT, иначе ISR по нарастанию ── */
static pcnt_unit_handle_t    s_pcntUnit = nullptr;
static pcnt_channel_handle_t s_pcntChan = nullptr;
static bool                  s_pcntOk   = false;
static bool                  s_isrMode  = false;
static volatile uint32_t     s_isrPulses = 0;
static uint32_t              s_pulsesTotal = 0;
static uint32_t              s_lastSampleMs = 0;

float    speedHz       = 0.0f;
float    speedKmh      = 0.0f;
bool     speedValid    = false;
bool     movingBySpeed = false;

/* ── Сухой контакт давления (NC: LOW = норма, HIGH = авария) ── */
static bool     s_presRawHigh    = false;
static bool     s_presStableHigh = false;
static uint32_t s_presChangeMs   = 0;
static bool     s_presErrRaised  = false;
static uint32_t s_presRefreshMs  = 0;

bool presSwOk       = true;
bool presSwCritical = false;
bool presSwRawHigh  = false;

static constexpr const char *PRES_MSG =
    "Давление в ресивере критическое (<0,5-1 бар), не обязательно 0";

/* ====================  Датчик скорости: инициализация  ==================== */

void IRAM_ATTR diagSpeedIsr() {
  s_isrPulses = s_isrPulses + 1u;
}

static bool speedInitPcnt() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -32000;
  ucfg.high_limit = 32000;
  ucfg.flags.accum_count = 0;
  if (pcnt_new_unit(&ucfg, &s_pcntUnit) != ESP_OK) {
    s_pcntUnit = nullptr;
    return false;
  }
  pcnt_chan_config_t ccfg = {};
  ccfg.edge_gpio_num  = PIN_SPEED_PULSE;
  ccfg.level_gpio_num = -1;
  ccfg.flags.invert_edge_input = 0;
  if (pcnt_new_channel(s_pcntUnit, &ccfg, &s_pcntChan) != ESP_OK) {
    pcnt_del_unit(s_pcntUnit);
    s_pcntUnit = nullptr;
    return false;
  }
  /* Считаем по нарастающему фронту (спад игнорируем). */
  if (pcnt_channel_set_edge_action(s_pcntChan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) {
    return false;
  }
  /* Глитч-фильтр: максимум железа S3 (≈12.8 мкс). Шире нельзя — 1023 такта APB. */
  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = SPEED_GLITCH_NS;
  if (pcnt_unit_set_glitch_filter(s_pcntUnit, &fcfg) != ESP_OK) {
    SERIAL_DBGLN("[DIAG] SPEED: glitch-фильтр не включился (продолжаю без него)");
  }
  if (pcnt_unit_enable(s_pcntUnit) != ESP_OK) return false;
  if (pcnt_unit_start(s_pcntUnit) != ESP_OK) return false;
  return true;
}

static bool speedInitIsr() {
  pinMode(PIN_SPEED_PULSE, INPUT_PULLUP);
  const int irq = digitalPinToInterrupt(PIN_SPEED_PULSE);
  if (irq < 0) return false;
  attachInterrupt(irq, diagSpeedIsr, RISING);
  s_isrMode = true;
  return true;
}

/* ====================  Датчик скорости: выборка  ==================== */

/** Прочитать и обнулить счётчик импульсов. */
static uint32_t speedTakePulses() {
  uint32_t n = 0;
  if (s_pcntOk) {
    int v = 0;
    if (pcnt_unit_get_count(s_pcntUnit, &v) == ESP_OK) {
      n = (v > 0) ? (uint32_t)v : 0u;
    }
    pcnt_unit_clear_count(s_pcntUnit);
  } else if (s_isrMode) {
    noInterrupts();
    n = s_isrPulses;
    s_isrPulses = 0;
    interrupts();
  }
  return n;
}

static void speedPoll() {
  const uint32_t now = millis();
  if (s_lastSampleMs == 0) {
    s_lastSampleMs = now;
    return;
  }
  const uint32_t dt = now - s_lastSampleMs;
  if (dt < SPEED_WINDOW_MS) return;
  s_lastSampleMs = now;

  const uint32_t pulses = speedTakePulses();
  s_pulsesTotal += pulses;

  const float hzRaw = (float)pulses * 1000.0f / (float)dt;
  /* Помеха/дребезг: неправдоподобная частота отбрасывается как 0 Гц. */
  const float hzMeas = (hzRaw <= SPEED_MAX_HZ) ? hzRaw : 0.0f;
  speedHz = SPEED_EMA_ALPHA * hzMeas + (1.0f - SPEED_EMA_ALPHA) * speedHz;
  if (speedHz < SPEED_MIN_HZ) speedHz = 0.0f;   // мёртвая зона

  const bool enabled = ConfigManager::getSpeedEnabled();
  const float coeff = ConfigManager::getSpeedCoeff();
  speedKmh = enabled ? (speedHz * coeff) : 0.0f;
  speedValid = enabled && (s_pcntOk || s_isrMode);

  if (!enabled) {
    movingBySpeed = false;
  } else if (speedKmh >= SPEED_MOVING_ON_KPH) {
    movingBySpeed = true;
  } else if (speedKmh < SPEED_MOVING_OFF_KPH) {
    movingBySpeed = false;
  }
}

/* ====================  /ST: опрос с антидребезгом  ==================== */

static void stPoll() {
  const uint16_t in = pcalStFaults();
  const uint32_t now = millis();

  for (uint8_t i = 0; i < 4; i++) {
    StTracker &t = s_st[i];
    const bool healthy = (in & DIAG_ST_LINES[i].mask) != 0u;   // 1 = линия отпущена
    if (healthy == t.rawHealthy) {
      if (t.agree < 255u) t.agree++;
    } else {
      t.rawHealthy = healthy;
      t.agree = 1;
    }
    if (t.agree < ST_DEBOUNCE_SAMPLES || t.stableHealthy == t.rawHealthy) continue;

    t.stableHealthy = t.rawHealthy;
    if (!t.stableHealthy) {
      t.healthySince = 0;
      t.reported = true;
      s_stErrRaised = true;
      SERIAL_DBG("[DIAG] %s: линия ST прижата к 0 — %s\n", DIAG_ST_LINES[i].name,
                    ST_MSGS[i]);
      /* handleError(VALVE) — критическая ошибка: поднимает аварийную защёлку.
         Защёлку считаем «своей» только если она была снята ДО нашего вызова,
         иначе автоснятие ошибки отпустит ручной аварийный стоп оператора. */
      const bool wasLatched = emergencyIsLatched();
      ErrorHandler::handleError(ErrorHandler::Error::VALVE, ST_MSGS[i]);
      s_stLatchOwned = (!wasLatched && emergencyIsLatched());
    } else {
      t.healthySince = now;
      SERIAL_DBG("[DIAG] %s: линия ST отпущена (норма)\n", DIAG_ST_LINES[i].name);
    }
  }

  /* Автоснятие: каждая группа здорова ≥5 с → ошибка VALVE снимается. */
  bool reported = false;
  for (uint8_t i = 0; i < 4; i++) {
    StTracker &t = s_st[i];
    if (!t.reported) continue;
    if (t.stableHealthy && t.healthySince != 0 &&
        (uint32_t)(now - t.healthySince) >= ST_CLEAR_HEALTHY_MS) {
      t.reported = false;
      SERIAL_DBG("[DIAG] %s: 5 с норма — ошибка снимается\n", DIAG_ST_LINES[i].name);
    } else {
      reported = true;
    }
  }

  if (!reported && s_stErrRaised) {
    s_stErrRaised = false;
    ErrorHandler::markErrorCleared(ErrorHandler::Error::VALVE);
    /* Защёлку ставили мы (ST) — снимаем её сами, иначе пневматика остаётся
       мёртвой до ручного сброса после короткого сбоя линии. */
    if (s_stLatchOwned && emergencyIsLatched()) {
      resetEmergencyLatch();
      SERIAL_DBGLN("[DIAG] аварийная защёлка снята (линии ST здоровы)");
    }
    s_stLatchOwned = false;
  } else if (reported) {
    /* Держим метку «ошибка видна» свежей: lastSeenTime используется UI/телеметрией.
       updateErrorTime() не трогает критическую ветку — emergencyStop не повторяется. */
    if (s_stRefreshMs == 0 || (uint32_t)(now - s_stRefreshMs) >= DIAG_ERR_REFRESH_MS) {
      s_stRefreshMs = now;
      ErrorHandler::updateErrorTime(ErrorHandler::Error::VALVE);
    }
    return;
  }
  s_stRefreshMs = 0;
}

/* ====================  PRES_SW: антидребезг + ошибка  ==================== */

static void presPoll() {
  const uint32_t now = millis();
  presSwRawHigh = (digitalRead(PIN_PRES_SW) == HIGH);

  if (presSwRawHigh != s_presRawHigh) {
    s_presRawHigh = presSwRawHigh;
    s_presChangeMs = now;
  }
  const uint32_t debounce =
      (uint32_t)constrain(ConfigManager::getPresSwDebounceMs(), 10, 500);
  if (s_presStableHigh != s_presRawHigh &&
      (uint32_t)(now - s_presChangeMs) >= debounce) {
    s_presStableHigh = s_presRawHigh;
    if (s_presStableHigh) {
      SERIAL_DBGLN("[DIAG] PRES_SW: контакт разомкнут — критическое давление");
    } else {
      SERIAL_DBGLN("[DIAG] PRES_SW: контакт замкнут — норма");
    }
  }
  /* Публикуем состояние только когда опция включена: при выключенном контроле
     висящий в воздухе вход (нет датчика) не должен давать «критично» в HA. */
  const bool enabled = ConfigManager::getPresSwEnabled();
  presSwCritical = enabled && s_presStableHigh;
  presSwOk = !presSwCritical;

  const bool want = enabled && s_presStableHigh;

  /* 3.2.0: по флагу меню останавливаем АВТО при срабатывании сухого контакта
   * (один раз на переход «норма -> критично», чтобы не дёргать переключение). */
  static bool s_presAutoStopped = false;
  if (want) {
    if (!s_presAutoStopped && ConfigManager::getPresSwStopAuto()) {
      s_presAutoStopped = true;
      if (currentSystemMode == SystemMode::AUTO) {
        switchSystemMode(SystemMode::MANUAL, "PRES_SW");
        SERIAL_DBGLN("[DIAG] PRES_SW: сработал — АВТО остановлено, режим РУЧНОЙ");
      }
    }
  } else {
    s_presAutoStopped = false;
  }
  if (want) {
    if (!s_presErrRaised) {
      s_presErrRaised = true;
      s_presRefreshMs = now;
      ErrorHandler::handleError(ErrorHandler::Error::LOW_RECEIVER, PRES_MSG);
    } else if ((uint32_t)(now - s_presRefreshMs) >= DIAG_ERR_REFRESH_MS) {
      s_presRefreshMs = now;
      ErrorHandler::updateErrorTime(ErrorHandler::Error::LOW_RECEIVER);
    }
  } else if (s_presErrRaised) {
    s_presErrRaised = false;
    ErrorHandler::markErrorCleared(ErrorHandler::Error::LOW_RECEIVER);
  }
}

/* ====================  Публичное состояние  ==================== */

uint16_t diagValveFaultMask() {
  uint16_t m = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (s_st[i].reported) m |= DIAG_ST_LINES[i].mask;
  }
  return m;
}

const char *diagValveFaultsText() {
  static char buf[40];
  buf[0] = '\0';
  for (uint8_t i = 0; i < 4; i++) {
    if (!s_st[i].reported) continue;
    if (buf[0] != '\0') strlcat(buf, ",", sizeof(buf));
    strlcat(buf, DIAG_ST_LINES[i].name, sizeof(buf));
  }
  return buf;
}

/* 3.0.0: подсказка для экрана ошибки — конкретная группа каналов, а не имя линии. */
const char *diagValveFaultsHint() {
  static const char *const HINTS[4] = { "U3 1-2", "U3 3-4", "U4 1-2", "U4 3-4" };
  static char buf[40];
  buf[0] = '\0';
  for (uint8_t i = 0; i < 4; i++) {
    if (!s_st[i].reported) continue;
    if (buf[0] != '\0') strlcat(buf, ", ", sizeof(buf));
    strlcat(buf, HINTS[i], sizeof(buf));
  }
  return buf;
}

bool diagSpeedSensorOk() { return s_pcntOk || s_isrMode; }
bool diagSpeedUsingPcnt() { return s_pcntOk; }
uint32_t diagSpeedPulsesTotal() { return s_pulsesTotal; }

/* ====================  Инициализация / задача  ==================== */

void diagInit() {
  pinMode(PIN_PRES_SW, INPUT_PULLUP);

  s_pcntOk = speedInitPcnt();
  if (s_pcntOk) {
    SERIAL_DBGLN("[DIAG] SPEED: аппаратный PCNT unit0/ch0 на IO8 (фильтр 12.8 мкс)");
  } else if (speedInitIsr()) {
    SERIAL_DBGLN("[DIAG] SPEED: PCNT недоступен — счёт по прерыванию (RISING)");
  } else {
    SERIAL_DBGLN("[DIAG] SPEED: счётчик импульсов НЕ поднялся");
  }

  s_lastSampleMs = millis();
  s_presRawHigh = (digitalRead(PIN_PRES_SW) == HIGH);
  s_presStableHigh = s_presRawHigh;
  s_presChangeMs = millis();
  presSwRawHigh = s_presRawHigh;
  presSwCritical = ConfigManager::getPresSwEnabled() && s_presStableHigh;
  presSwOk = !presSwCritical;

  SERIAL_DBG("[DIAG] init: PRES_SW(IO38)=%s, скорость=%s\n",
                s_presStableHigh ? "HIGH (разомкнут)" : "LOW (замкнут)",
                s_pcntOk ? "PCNT" : (s_isrMode ? "ISR" : "нет"));
}

void diagTask(void *pvParameters) {
  (void)pvParameters;
  SERIAL_DBGLN("[DIAG] Diag Task started");

  for (;;) {
    TaskPool::markRun(taskIndex_Diag);
    stPoll();
    presPoll();
    speedPoll();
    vTaskDelay(pdMS_TO_TICKS(DIAG_TASK_PERIOD_MS));
  }
}
