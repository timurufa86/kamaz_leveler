#pragma once
/**
 *  pressure_read.h — JHM1200 pressure reading, limits, master-pressure
 *  adjustment, leak monitor, manual target helpers.
 *
 *  Low-level I2C sits in jhm1200.h/.cpp (untouched).
 *  This layer adds calibration offset, EMA, limit checks,
 *  pressure request wakeup, and manual/movement target helpers.
 */

#include <Arduino.h>
#include "app_types.h"

float readPressure();
bool  initializeJhm1200();
bool  checkPressureLimits();
void  checkAndAdjustMasterPressure();
void  requestPressureMeasurement();
void  updateLeakMonitor();

void  setManualTargetPressure(Pad pad);
void  setAllManualTargetsFromCurrent();
void  setManualTargetsFromParkingPolicy();
void  maintainManualPressure();
void  maintainMovementPressure();
