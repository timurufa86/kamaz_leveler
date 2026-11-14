#include "btn_io.h"
#include "hw_map.h"
#include "pcal9555.h"

static uint8_t s_injectMask = 0;  // bit0..4 = KN1..KN5 overridden
static uint8_t s_injectDown = 0;  // bit=1 → pressed (LOW)
static uint8_t s_forceEdge = 0;
static uint8_t s_forceRelease = 0;
static bool s_dryRun = false;
static uint8_t s_lastEdgePad = 0xFF;
static uint8_t s_lastStartPad = 0xFF;
static bool s_lastStartOk = false;

/** Плата v3: кнопки KN1..KN5 = PCAL P0_0..P0_4; «пин» — только ID (BTN_VPIN_*). */
static int knFromVpin(uint8_t vpin) {
  switch (vpin) {
    case BTN_VPIN_KN1: return 0;
    case BTN_VPIN_KN2: return 1;
    case BTN_VPIN_KN3: return 2;
    case BTN_VPIN_KN4: return 3;
    case BTN_VPIN_KN5: return 4;
    default: return -1;
  }
}

uint8_t btnDigitalRead(uint8_t pin) {
  const int kn = knFromVpin(pin);
  if (kn >= 0 && (s_injectMask & (1u << kn))) {
    return (s_injectDown & (1u << kn)) ? LOW : HIGH;
  }
  /* Кэш входов PCAL обновляется pcalPoll() в ButtonTask. */
  return pcalButtonLevelVpin(pin);
}

/** EncButton weak override (README: свой EB_read) — inject + PCAL. */
bool EB_read(uint8_t pin) {
  return btnDigitalRead(pin);
}

void btnInjectSet(uint8_t kn, bool down) {
  if (kn < 1 || kn > 5) return;
  const uint8_t bit = (uint8_t)(1u << (kn - 1));
  s_injectMask |= bit;
  if (down) {
    s_injectDown |= bit;
    s_forceEdge |= bit;  // гарантировать HIGH→LOW фронт в ButtonTask
  } else {
    s_injectDown &= (uint8_t)~bit;
    s_forceRelease |= bit;
  }
}

void btnInjectClear() {
  s_injectMask = 0;
  s_injectDown = 0;
  s_forceRelease = 0x1F;
  s_forceEdge = 0;
}

bool btnInjectActive() { return s_injectMask != 0; }

uint8_t btnInjectMask() { return s_injectMask; }

uint8_t btnInjectDownMask() { return s_injectDown; }

uint8_t btnConsumeForceEdgeMask() {
  const uint8_t m = s_forceEdge;
  s_forceEdge = 0;
  return m;
}

uint8_t btnConsumeForceReleaseMask() {
  const uint8_t m = s_forceRelease;
  s_forceRelease = 0;
  return m;
}

void btnSetDryRun(bool on) { s_dryRun = on; }

bool btnDryRun() { return s_dryRun; }

void btnTestNotePadEdge(uint8_t padIdx) { s_lastEdgePad = padIdx; }

void btnTestNotePadStart(uint8_t padIdx, bool ok) {
  s_lastStartPad = padIdx;
  s_lastStartOk = ok;
}

uint8_t btnTestLastEdgePad() { return s_lastEdgePad; }

uint8_t btnTestLastStartPad() { return s_lastStartPad; }

bool btnTestLastStartOk() { return s_lastStartOk; }

void btnTestResetStats() {
  s_lastEdgePad = 0xFF;
  s_lastStartPad = 0xFF;
  s_lastStartOk = false;
}
