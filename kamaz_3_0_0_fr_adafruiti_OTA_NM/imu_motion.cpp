#include "imu_motion.h"
#include "app_globals.h"

/* ── ConfigManager bridge functions (defined in the .ino) ── */
extern int cfg_getGyroThreshold();
extern int cfg_getGyroBumpThreshold();
extern int cfg_getAccelThreshold();
extern int cfg_getMovementSettleSec();

/* ───── Motion debug / stream state definitions ───── */
volatile uint32_t g_motStreamUntilMs = 0;
volatile uint32_t g_dmpStreamUntilMs = 0;
volatile uint32_t g_motStreamPeriodMs = 100;  // период строк [MOT] / [DMP], мс
float g_motLastGyroRms = -1.0f;
float g_motLastGyroDelta = -1.0f;
float g_motLastGyroThr = 0.0f;
float g_motLastGyroBump = -1.0f;
float g_motLastGyroBumpDelta = -1.0f;
float g_motLastGyroBumpThr = 0.0f;
float g_motLastLinAccRms = -1.0f;
float g_motLastLinAccRaw = -1.0f;
float g_motLastLinAccThr = 0.0f;
float g_motLastGyroEma = 0.0f;
float g_motLastBumpEma = 0.0f;
float g_motLastLinEma = 0.0f;
float g_dmpGx = 0.0f, g_dmpGy = 0.0f, g_dmpGz = 0.0f;
float g_dmpAx = 0.0f, g_dmpAy = 0.0f, g_dmpAz = 0.0f;
float g_dmpBx = 0.0f, g_dmpBy = 0.0f, g_dmpBz = 0.0f;
float g_dmpYaw = 0.0f, g_dmpPitch = 0.0f, g_dmpRoll = 0.0f;
int g_dmpNPkt = 0;
uint8_t g_motLastIntStatus = 0;
uint8_t g_motLastMotStatus = 0;
uint8_t g_motDurMs = 40;         // MOT_DUR, мс (1 LSB = 1 ms)
bool g_motLastMotPulse = false;
bool g_motLastGyroBusy = false;
bool g_motLastGyroBumpBusy = false;
bool g_motLastLinAccBusy = false;
bool g_motLastHold = false;
uint32_t g_motLastActivityMs = 0;
uint32_t g_motSampleCount = 0;
uint32_t g_motMotPulseCount = 0;
uint32_t g_motGyroBusyCount = 0;
uint32_t g_motGyroBumpBusyCount = 0;
uint32_t g_motLinAccBusyCount = 0;
uint32_t g_motHoldEdgeCount = 0;
uint32_t g_motModeEnterCount = 0;
uint32_t g_motStatsSinceMs = 0;

static volatile bool s_motResetReq = false;

void resetMotionDetector() {
  s_motResetReq = true;
  g_motLastActivityMs = 0;
  g_motLastHold = false;
  g_motLastMotPulse = false;
  g_motLastGyroBusy = false;
  g_motLastGyroBumpBusy = false;
  g_motLastLinAccBusy = false;
}

/* ───── Motion detection algorithm ───── */

/**
 * Детектор движения (10.14.3, cabin/drive retest):
 *  - MOT INT 0x40 (ребро), без липкого MOT_DETECT_STATUS;
 *  - gyroRMS / bump абсолютные >= menu×8;
 *  - |linAccRMS − EMA|;
 *  - lastActivityMs только после ≥200 мс непрерывного busy (анти-спайк → settle).
 * Езда = сумма busy-импульсов >= durationSec; settleSec — хвост hold.
 */
bool detectMotionFromIMU(uint8_t intStatus, uint8_t motStatus, uint32_t nowMs, float gyroRms,
                         float linAccRms, float gyroBump) {
#if ENABLE_MPU6050
  static uint32_t lastActivityMs = 0;
  static uint32_t busyRunStartMs = 0;
  static float linEma = 0.0f;
  static bool linEmaInit = false;
  static float gyroEma = 0.0f;
  static bool gyroEmaInit = false;
  static float bumpEma = 0.0f;
  static bool bumpEmaInit = false;
  constexpr float kMotEmaAlpha = 0.05f;
  constexpr uint32_t kBusyArmMs = 200;

  if (s_motResetReq) {
    s_motResetReq = false;
    lastActivityMs = 0;
    busyRunStartMs = 0;
    linEmaInit = false;
    gyroEmaInit = false;
    bumpEmaInit = false;
    linEma = gyroEma = bumpEma = 0.0f;
  }

  // Только INT MOT (сбрасывается чтением). Осевой status липкий — не для activity.
  const bool motHwEdge = (intStatus & 0x40) != 0;
  (void)motStatus;
  const float gyroThr = (float)cfg_getGyroThreshold() * 8.0f;
  const float bumpThr = (float)cfg_getGyroBumpThreshold() * 8.0f;
  const float linAccThr = (float)cfg_getAccelThreshold();
  const bool gyroValid = (gyroRms >= 0.0f) && (gyroRms <= 2000.0f);
  const bool bumpValid = (gyroBump >= 0.0f) && (gyroBump <= 2000.0f);

  float gyroDelta = -1.0f;
  bool gyroBusy = false;
  if (gyroValid) {
    if (!gyroEmaInit) {
      gyroEma = gyroRms;
      gyroEmaInit = true;
      gyroDelta = 0.0f;
    } else {
      gyroDelta = fabsf(gyroRms - gyroEma);
      gyroEma += kMotEmaAlpha * (gyroRms - gyroEma);
    }
    gyroBusy = gyroRms >= gyroThr;
  }

  float bumpDelta = -1.0f;
  bool bumpBusy = false;
  if (bumpValid) {
    if (!bumpEmaInit) {
      bumpEma = gyroBump;
      bumpEmaInit = true;
      bumpDelta = 0.0f;
    } else {
      bumpDelta = fabsf(gyroBump - bumpEma);
      bumpEma += kMotEmaAlpha * (gyroBump - bumpEma);
    }
    bumpBusy = gyroBump >= bumpThr;
  }

  float linDelta = -1.0f;
  bool linAccBusy = false;
  if (linAccRms >= 0.0f) {
    if (!linEmaInit) {
      linEma = linAccRms;
      linEmaInit = true;
      linDelta = 0.0f;
    } else {
      linDelta = fabsf(linAccRms - linEma);
      linEma += kMotEmaAlpha * (linAccRms - linEma);
    }
    linAccBusy = linDelta >= linAccThr;
  }

  const bool rawBusy = motHwEdge || gyroBusy || bumpBusy || linAccBusy;
  if (rawBusy) {
    if (busyRunStartMs == 0) busyRunStartMs = nowMs;
  } else {
    busyRunStartMs = 0;
  }
  if (busyRunStartMs != 0 && (nowMs - busyRunStartMs) >= kBusyArmMs) {
    lastActivityMs = nowMs;
  }

  g_motLastIntStatus = intStatus;
  g_motLastMotStatus = motStatus;
  g_motLastMotPulse = motHwEdge;
  g_motLastGyroRms = gyroRms;
  g_motLastGyroDelta = gyroDelta;
  g_motLastGyroThr = gyroThr;
  g_motLastGyroBump = gyroBump;
  g_motLastGyroBumpDelta = bumpDelta;
  g_motLastGyroBumpThr = bumpThr;
  g_motLastLinAccRaw = linAccRms;
  g_motLastLinAccRms = linDelta;
  g_motLastLinAccThr = linAccThr;
  g_motLastGyroEma = gyroEma;
  g_motLastBumpEma = bumpEma;
  g_motLastLinEma = linEma;
  g_motLastGyroBusy = gyroBusy;
  g_motLastGyroBumpBusy = bumpBusy;
  g_motLastLinAccBusy = linAccBusy;
  g_motLastActivityMs = lastActivityMs;
  g_motSampleCount++;
  if (motHwEdge) g_motMotPulseCount++;
  if (gyroBusy) g_motGyroBusyCount++;
  if (bumpBusy) g_motGyroBumpBusyCount++;
  if (linAccBusy) g_motLinAccBusyCount++;

  if (lastActivityMs == 0) {
    g_motLastHold = false;
    return false;
  }
  const uint32_t settleMs = (uint32_t)cfg_getMovementSettleSec() * 1000UL;
  const bool hold = (nowMs - lastActivityMs) < settleMs;
  if (hold && !g_motLastHold) g_motHoldEdgeCount++;
  g_motLastHold = hold;
  return hold;
#else
  (void)intStatus;
  (void)motStatus;
  (void)nowMs;
  (void)gyroRms;
  (void)linAccRms;
  (void)gyroBump;
  return false;
#endif
}
