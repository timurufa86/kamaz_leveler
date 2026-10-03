#pragma once
/**
 *  task_calib.h — FreeRTOS calibrationTask + ZeroCalibrator class.
 *  Runs JHM1200 init, sensor probe, MPU check, zero-pressure calibration.
 */

#include <Arduino.h>

void calibrationTask(void *pvParameters);
