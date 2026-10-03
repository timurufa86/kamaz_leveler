#pragma once
/**
 *  valve_ctrl.h — valve open/close, manual command handling,
 *  pin control for bub/infl/defl solenoids.
 */

#include <Arduino.h>
#include "app_types.h"

void setValve(Pad pad, bool state);
void closeAllValves();
void emergencyStop();
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs);
bool sendValveCommandSync(Pad pad, bool inflate, uint32_t durationMs, uint32_t waitAfterMs = 0);
void startManualOperation(Pad padIdx, bool inflate);
void stopManualOperation();
