#pragma once
/**
 *  imu_motion.h — motion detection from IMU (gyro/accel EMA, bump, MOT).
 *
 *  detectMotionFromIMU() is the core algorithm.
 *  g_mot* variables expose detector state for debug streaming (TEST MOT …).
 */

#include <Arduino.h>

bool detectMotionFromIMU(uint8_t intStatus, uint8_t motStatus, uint32_t nowMs, float gyroRms,
                         float linAccRms, float gyroBump);

/* ───── Motion debug / stream state ───── */
extern volatile uint32_t g_motStreamUntilMs;
extern volatile uint32_t g_motStreamPeriodMs;
extern float g_motLastGyroRms;
extern float g_motLastGyroDelta;
extern float g_motLastGyroThr;
extern float g_motLastGyroBump;
extern float g_motLastGyroBumpDelta;
extern float g_motLastGyroBumpThr;
extern float g_motLastLinAccRms;
extern float g_motLastLinAccRaw;
extern float g_motLastLinAccThr;
extern uint8_t g_motLastIntStatus;
extern uint8_t g_motLastMotStatus;
extern uint8_t g_motDurMs;
extern bool g_motLastMotPulse;
extern bool g_motLastGyroBusy;
extern bool g_motLastGyroBumpBusy;
extern bool g_motLastLinAccBusy;
extern bool g_motLastHold;
extern uint32_t g_motLastActivityMs;
extern uint32_t g_motSampleCount;
extern uint32_t g_motMotPulseCount;
extern uint32_t g_motGyroBusyCount;
extern uint32_t g_motGyroBumpBusyCount;
extern uint32_t g_motLinAccBusyCount;
extern uint32_t g_motHoldEdgeCount;
extern uint32_t g_motModeEnterCount;
extern uint32_t g_motStatsSinceMs;
