#pragma once
/**
 *  valve_ctrl.h — valve open/close, manual command handling,
 *  pin control for bub/infl/defl solenoids.
 *  Manifold API: exclusive measurePad / measureMaster / pulsePad (canon A/B/C).
 */

#include <Arduino.h>
#include "app_types.h"

void setValve(Pad pad, bool state);
void closeAllValves();
void emergencyStop();
/** True while the emergency latch is set (emergencyStop() → until resetEmergencyLatch()). */
bool emergencyIsLatched();
/** Operator reset of the emergency latch — pneumatic work is allowed again. */
void resetEmergencyLatch();
/** Stop and close everything before a long network/TLS session (OTA). */
void otaQuiesceForNetwork();
/** Watchdog for the manual hold: stops the operation on over/under pressure. */
void enforceManualPressureSafety();
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs);
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs = 0);
/** @return ManualStartResult; Ok means hold started (manifold open). */
enum class ManualStartResult : uint8_t {
  Ok = 0,
  BadPad,
  AlreadyActive,
  Movement,
  Mutex,
  ReadFail,
  MaxP,
  MinP,
  MasterLow,
  /** Аварийная защёлка активна — нужен resetEmergencyLatch(). */
  Emergency
};

const char *manualStartResultStr(ManualStartResult r);

ManualStartResult startManualOperation(Pad padIdx, bool inflate);
void stopManualOperation();

/** True while any exclusive manifold sequence or manual hold owns the valves. */
bool manifoldIsBusy();

/** A: pad-only sample → updates pressure[pad]. Returns false on fail/busy. */
bool measurePad(Pad pad, float *outOpt = nullptr);

/** B: INFL-only sample → updates masterPressure. */
bool measureMaster(float *outOpt = nullptr);

/**
 * C: pad → t→P → P_before → INFL/DEFL → duration → close line → t→P → P_after → close pad.
 * Enforces pad min/max (+deadband) like MANUAL. Updates pressure[pad].
 * Returns false if blocked by limits, sensor fail, or mutex.
 */
bool pulsePad(Pad pad, bool inflate, uint32_t durationMs,
              float *pBeforeOpt = nullptr, float *pAfterOpt = nullptr);

/**
 * Axle pair: both selects open, one INFL/DEFL hold (menu duration), then per-pad A samples.
 * front=true → FL+FR; front=false → RL+RR.
 */
bool pulseAxlePair(bool front, bool inflate, uint32_t durationMs);

/**
 * Side pair (roll): both selects open, one INFL/DEFL hold, then per-pad A samples.
 * left=true → FL+RL; left=false → FR+RR.
 */
bool pulseSidePair(bool left, bool inflate, uint32_t durationMs);
