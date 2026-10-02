#pragma once
/**
 *  task_imu.h — FreeRTOS imuTask and its shared stream/stats state.
 *
 *  g_imu* variables are written by imuTask, read/set by the serial
 *  test handler in the .ino (TEST IMU STREAM/STATS).
 */

#include <Arduino.h>

void imuTask(void *pvParameters);

/* ───── IMU stream / stats state ───── */
extern volatile bool     g_imuFilterResetReq;
extern volatile uint32_t g_imuStreamUntilMs;
extern volatile uint32_t g_imuStatsUntilMs;
extern volatile bool     g_imuStatsActive;
extern float g_imuRawX, g_imuRawY;
extern float g_imuAbsX, g_imuAbsY;
extern float g_imuOutX, g_imuOutY;

/* ───── STATS accumulator ───── */
extern uint32_t g_imuStatN;
extern uint32_t g_imuStatSpikes;
extern float    g_imuStatSumX, g_imuStatSumY;
extern float    g_imuStatSumXX, g_imuStatSumYY;
extern float    g_imuStatMinX, g_imuStatMaxX;
extern float    g_imuStatMinY, g_imuStatMaxY;
extern float    g_imuStatMaxStep;
extern float    g_imuStatPrevX, g_imuStatPrevY;
extern uint32_t g_imuStatT0;
