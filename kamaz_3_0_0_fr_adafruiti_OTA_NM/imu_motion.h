#pragma once
/**
 *  imu_motion.h — motion detection from IMU (abs gyro/bump, lin EMA, MOT).
 *
 *  detectMotionFromIMU() is the core algorithm.
 *  g_mot* / g_dmp* expose detector + raw DMP state for TEST MOT / DMP streams.
 */

#include <Arduino.h>

bool detectMotionFromIMU(uint8_t intStatus, uint8_t motStatus, uint32_t nowMs, float gyroRms,
                         float linAccRms, float gyroBump);

/** Сброс EMA / lastActivity / hold (TEST MOT RESET). */
void resetMotionDetector();

/* ───── Motion debug / stream state ───── */
extern volatile uint32_t g_motStreamUntilMs;
extern volatile uint32_t g_dmpStreamUntilMs;
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
extern float g_motLastGyroEma;
extern float g_motLastBumpEma;
extern float g_motLastLinEma;
/* Last FIFO packet axes (DMP discovery stream). */
extern float g_dmpGx, g_dmpGy, g_dmpGz;
extern float g_dmpAx, g_dmpAy, g_dmpAz;  // world linAcc
extern float g_dmpBx, g_dmpBy, g_dmpBz;  // body linAcc (aaReal)
extern float g_dmpYaw, g_dmpPitch, g_dmpRoll;
extern int g_dmpNPkt;
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
