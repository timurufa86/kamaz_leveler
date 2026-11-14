#include "pcal9555.h"
#include <Wire.h>
#include "app_globals.h"   // xI2CMutex
#include "logger.h"
#include "app_types.h"

/* ─────────────── Состояние драйвера ─────────────── */

static bool     s_ready     = false;   // последняя транзакция успешна
static bool     s_hold      = false;   // буфер U5 принудительно выключен (авария)
static uint16_t s_out       = 0;       // теневая копия выходных защёлок
static uint16_t s_in        = 0xFFFF;  // кэш входов: 1 = высокий уровень (отпущено)
static uint32_t s_okCount   = 0;
static uint32_t s_errCount  = 0;
static uint32_t s_lastTryMs = 0;

/* ─────────────── Низкий уровень: I2C ───────────────
 * Шина общая (JHM1200/MPU/PCAL) → короткая блокировка xI2CMutex.
 * В аварийном пути pcalEmergencyShutoff() шина НЕ блокируется. */

static bool busLock(TickType_t ticks) {
  if (xI2CMutex == nullptr) return true;
  return xSemaphoreTake(xI2CMutex, ticks) == pdTRUE;
}

static void busUnlock(bool locked) {
  if (locked && xI2CMutex != nullptr) xSemaphoreGive(xI2CMutex);
}

static bool busWrite(uint8_t reg, const uint8_t *buf, size_t n) {
  const bool locked = busLock(pdMS_TO_TICKS(PCAL_I2C_TIMEOUT_MS * 5));
  if (!locked) {
    s_errCount++;
    return false;
  }
  bool ok = false;
  for (uint8_t attempt = 0; attempt < PCAL_I2C_RETRIES && !ok; attempt++) {
    Wire.beginTransmission(PCAL_I2C_ADDR);
    Wire.write(reg);
    for (size_t i = 0; i < n; i++) Wire.write(buf[i]);
    ok = (Wire.endTransmission() == 0);
  }
  busUnlock(locked);
  if (ok) s_okCount++; else s_errCount++;
  return ok;
}

static bool busRead(uint8_t reg, uint8_t *buf, size_t n) {
  const bool locked = busLock(pdMS_TO_TICKS(PCAL_I2C_TIMEOUT_MS * 5));
  if (!locked) {
    s_errCount++;
    return false;
  }
  bool ok = false;
  for (uint8_t attempt = 0; attempt < PCAL_I2C_RETRIES && !ok; attempt++) {
    Wire.beginTransmission(PCAL_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) continue;
    const size_t got = Wire.requestFrom((uint8_t)PCAL_I2C_ADDR, (uint8_t)n);
    if (got != n) continue;
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    ok = true;
  }
  busUnlock(locked);
  if (ok) s_okCount++; else s_errCount++;
  return ok;
}

/** Ошибка записи → клапаны закрыты аппаратно, буфер ждёт явного включения. */
static void onBusFault(const char *where) {
  s_ready = false;
  s_out = 0;
  SERIAL_DBG("[PCAL] I2C fault (%s) — VLV_EN=HIGH (fail-safe)\n", where ? where : "?");
  pcalEmergencyShutoff();
}

/* ─────────────── Инициализация ─────────────── */

bool pcalBegin() {
  /* 1. Аппаратный стоп ДО любых обращений к шине. */
  pinMode(PIN_VLV_EN, OUTPUT);
  digitalWrite(PIN_VLV_EN, HIGH);
  pinMode(PIN_PCAL_INT, INPUT_PULLUP);

  s_lastTryMs = millis();
  s_out = 0;
  s_in = 0xFFFF;
  s_ready = false;

  /* 2. Выходные защёлки — в 0 (клапаны закрыты), затем полярность/направления. */
  const uint8_t out[2] = { PCAL_OUT0_INIT, PCAL_OUT1_INIT };
  if (!busWrite(PCAL_REG_OUTPUT0, out, 2)) {
    SERIAL_DBGLN("[PCAL] нет ответа на 0x20 — клапаны заблокированы (VLV_EN=HIGH)");
    return false;
  }

  const uint8_t pol[2] = { 0x00, 0x00 };
  busWrite(PCAL_REG_POLARITY0, pol, 2);

  const uint8_t cfg[2] = { PCAL_CFG_PORT0, PCAL_CFG_PORT1 };
  if (!busWrite(PCAL_REG_CONFIG0, cfg, 2)) {
    SERIAL_DBGLN("[PCAL] конфиг портов не записан");
    return false;
  }

  /* 3. Agile I/O — только PCAL9555A; на TCA9535 регистров нет (best effort). */
  const uint8_t drive0 = PCAL_DRIVE0;
  const uint8_t drive1 = PCAL_DRIVE1;
  const uint8_t im0 = PCAL_INTMASK0;
  const uint8_t im1 = PCAL_INTMASK1;
  busWrite(PCAL_REG_DRIVE0, &drive0, 1);
  busWrite(PCAL_REG_DRIVE1, &drive1, 1);
  busWrite(PCAL_REG_INTMASK0, &im0, 1);
  busWrite(PCAL_REG_INTMASK1, &im1, 1);

  /* 4. Проверка присутствия — чтение входов. */
  uint8_t in[2] = { 0xFF, 0xFF };
  if (!busRead(PCAL_REG_INPUT0, in, 2)) {
    SERIAL_DBGLN("[PCAL] расширитель не отвечает на чтение");
    return false;
  }
  s_in = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
  s_ready = true;

  /* 5. Буфер включается только если нет аварийной защёлки. */
  if (!s_hold) pcalEnableValveBuffer(true);
  SERIAL_DBGLN("[PCAL] PCAL9555A готов: P0=входы (кнопки/ST_U4), P1_0..P1_5=клапаны");
  return true;
}

bool pcalReady() { return s_ready; }

void pcalStats(uint32_t *okOpt, uint32_t *errOpt) {
  if (okOpt) *okOpt = s_okCount;
  if (errOpt) *errOpt = s_errCount;
}

/* ─────────────── Входы ─────────────── */

bool pcalReadInputsNow(uint16_t *outOpt) {
  uint8_t in[2] = { 0xFF, 0xFF };
  if (!busRead(PCAL_REG_INPUT0, in, 2)) {
    s_in = 0xFFFF;   // кнопки «отпущены», ST «нет fault»
    s_ready = false;
    return false;
  }
  s_in = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
  s_ready = true;
  if (outOpt) *outOpt = s_in;
  return true;
}

bool pcalPoll() {
  if (!pcalReadInputsNow(nullptr)) {
    /* Автоповтор инициализации не чаще PCAL_REINIT_PERIOD_MS. */
    if ((uint32_t)(millis() - s_lastTryMs) >= PCAL_REINIT_PERIOD_MS) pcalBegin();
    return false;
  }
  return true;
}

uint16_t pcalInputs() { return s_in; }
uint8_t pcalInputPort0() { return (uint8_t)(s_in & 0xFF); }
uint8_t pcalInputPort1() { return (uint8_t)(s_in >> 8); }

uint16_t pcalStFaults() { return (uint16_t)(s_in & PCAL_ST_ALL); }

uint8_t pcalButtonLevel(uint8_t idx) {
  if (idx > 4) return HIGH;
  return ((s_in >> idx) & 0x01u) ? HIGH : LOW;
}

uint8_t pcalButtonLevelVpin(uint8_t vpin) {
  switch (vpin) {
    case BTN_VPIN_KN1: return pcalButtonLevel(0);
    case BTN_VPIN_KN2: return pcalButtonLevel(1);
    case BTN_VPIN_KN3: return pcalButtonLevel(2);
    case BTN_VPIN_KN4: return pcalButtonLevel(3);
    case BTN_VPIN_KN5: return pcalButtonLevel(4);
    default: return HIGH;
  }
}

bool pcalIntAsserted() { return digitalRead(PIN_PCAL_INT) == LOW; }

/* ─────────────── Выходы ─────────────── */

bool pcalWriteOutputs(uint16_t word) {
  const uint8_t out[2] = { (uint8_t)(word & 0xFF), (uint8_t)(word >> 8) };
  if (!busWrite(PCAL_REG_OUTPUT0, out, 2)) {
    onBusFault("outputs");
    return false;
  }
  s_out = word;
  s_ready = true;
  return true;
}

uint16_t pcalOutputs() { return s_out; }

bool pcalSetBits(uint16_t mask, bool on) {
  if (mask == 0) return true;
  const uint16_t next = on ? (uint16_t)(s_out | mask) : (uint16_t)(s_out & ~mask);
  if (next == s_out) return true;
  return pcalWriteOutputs(next);
}

bool pcalValvePad(Pad pad, bool on) {
  if ((uint8_t)pad >= PAD_COUNT) return false;
  return pcalSetBits(pcalPadMask(pad), on);
}

bool pcalValveSet(uint16_t mask, bool on) {
  return pcalSetBits((uint16_t)(mask & PCAL_VLV_ALL), on);
}

bool pcalInfl(bool on) { return pcalSetBits(PCAL_VLV_INFL, on); }
bool pcalDefl(bool on) { return pcalSetBits(PCAL_VLV_DEFL, on); }

bool pcalFlowSelect(bool inflate) {
  uint16_t next = (uint16_t)(s_out & ~(PCAL_VLV_INFL | PCAL_VLV_DEFL));
  next = (uint16_t)(next | (inflate ? PCAL_VLV_INFL : PCAL_VLV_DEFL));
  return pcalWriteOutputs(next);
}

bool pcalFlowOff() {
  return pcalSetBits((uint16_t)(PCAL_VLV_INFL | PCAL_VLV_DEFL), false);
}

bool pcalValvesAllOff() { return pcalSetBits(PCAL_VLV_ALL, false); }

/* ─────────────── Буфер U5 / аварийный останов ─────────────── */

void pcalEnableValveBuffer(bool on) {
  if (!on) {
    s_hold = true;
    digitalWrite(PIN_VLV_EN, HIGH);
    return;
  }
  s_hold = false;
  if (!s_ready) {
    digitalWrite(PIN_VLV_EN, HIGH);
    return;
  }
  if (!pcalWriteOutputs(0)) return;   // fail-safe уже сработал
  digitalWrite(PIN_VLV_EN, LOW);
}

bool pcalValveBufferEnabled() {
  return !s_hold && (digitalRead(PIN_VLV_EN) == LOW);
}

void pcalEmergencyShutoff() {
  /* Сначала железо: Hi-Z буфера закрывает клапаны без участия I2C. */
  s_hold = true;
  pinMode(PIN_VLV_EN, OUTPUT);
  digitalWrite(PIN_VLV_EN, HIGH);

  /* Затем best-effort: обнулить защёлки, чтобы следующее включение было чистым. */
  const uint8_t zero[2] = { 0x00, 0x00 };
  const bool locked = (xI2CMutex == nullptr) ? true : (xSemaphoreTake(xI2CMutex, 0) == pdTRUE);
  Wire.beginTransmission(PCAL_I2C_ADDR);
  Wire.write((uint8_t)PCAL_REG_OUTPUT0);
  Wire.write(zero[0]);
  Wire.write(zero[1]);
  const bool ok = (Wire.endTransmission() == 0);
  if (locked && xI2CMutex != nullptr) xSemaphoreGive(xI2CMutex);
  if (ok) {
    s_out = 0;
    s_okCount++;
  } else {
    s_errCount++;
  }
}
