#include "task_imu.h"
#include "imu_dmp.h"
#include "imu_motion.h"
#include "app_globals.h"
#include "app_types.h"
#include "mutex_guard.h"
#include "event_bus.h"
#include "task_pool.h"
#include "logger.h"

#include <cmath>

/* ── ConfigManager bridge functions (defined in the .ino) ── */
extern int   cfg_getImuPollMs();
extern int   cfg_getImuFifoAvg();
extern float cfg_getImuKalmanMea();
extern float cfg_getImuKalmanEst();
extern float cfg_getImuKalmanQ();
extern float cfg_getImuEmaAlpha();
extern float cfg_getImuEmaSpikeAlpha();
extern float cfg_getImuEmaSpikeThr();
extern float cfg_getImuSlewDps();
extern float cfg_getZeroAngleX();
extern float cfg_getZeroAngleY();
extern int   cfg_getMovementDurationSec();
extern int   cfg_getMovementSettleSec();
extern float cfg_getMovementPressureFront();
extern float cfg_getMovementPressureRear();
extern float cfg_getMovementTolerance();

/* ── ErrorHandler / TaskMonitor bridge functions (defined in .ino) ── */
enum class EH_Error : uint8_t { NONE = 0, LOW_PRESSURE, MPU, SENSOR, VALVE, WATCHDOG, OTA, COUNT };
extern bool cfg_errorIsActive(uint8_t err);
extern void cfg_errorMarkCleared(uint8_t err);
extern void cfg_taskMonitorUpdate(uint8_t idx);
static constexpr uint8_t TASK_IMU_IDX = 3; // TaskMonitor::TASK_IMU

/* ── .ino functions used by imuTask ── */
void setDisplayDirty();
void sendValveCommand(Pad pad, bool inflate, uint32_t durationMs);
void stopManualOperation();
void setManualTargetsFromParkingPolicy();

/* ── .ino variables used by imuTask ── */
extern Mode currentMode;
extern bool manualControlActive;
extern float pressure[];
extern volatile uint32_t levelingAttemptsThisHour;
extern uint32_t lastLevelingCheckTime;
extern uint32_t lastLevelingAttemptTime;

#if ENABLE_SIMULATION
extern float simAngleX;
extern float simAngleY;
void updateSimulationData();
#endif

/* ───── IMU stream / stats state definitions ───── */
volatile bool     g_imuFilterResetReq  = false;
volatile uint32_t g_imuStreamUntilMs   = 0;
volatile uint32_t g_imuStatsUntilMs    = 0;
volatile bool     g_imuStatsActive     = false;
float g_imuRawX = 0.0f, g_imuRawY = 0.0f;
float g_imuAbsX = 0.0f, g_imuAbsY = 0.0f;
float g_imuOutX = 0.0f, g_imuOutY = 0.0f;

/* ───── STATS accumulator definitions ───── */
uint32_t g_imuStatN = 0;
uint32_t g_imuStatSpikes = 0;
float    g_imuStatSumX = 0, g_imuStatSumY = 0;
float    g_imuStatSumXX = 0, g_imuStatSumYY = 0;
float    g_imuStatMinX = 0, g_imuStatMaxX = 0;
float    g_imuStatMinY = 0, g_imuStatMaxY = 0;
float    g_imuStatMaxStep = 0;
float    g_imuStatPrevX = 0, g_imuStatPrevY = 0;
uint32_t g_imuStatT0 = 0;

/* ───── File-scope state ───── */
static uint8_t reinitAttempts = 0;

/* ═══════════════════════════════════════════════════════════
 *  imuTask — reads DMP FIFO, runs motion detection, publishes
 *  angle data via xIMUQueue / EventBus.
 * ═══════════════════════════════════════════════════════════ */

void imuTask(void *pvParameters) {

  UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
  if (stackHighWater < 300) {
    Serial.printf("[%s] ?? CRITICAL: Stack low! %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  } else if (stackHighWater < 500) {
    Serial.printf("[%s] ?? Stack low: %d bytes free\n",
                  pcTaskGetName(NULL), stackHighWater);
  }

  extern uint8_t taskIndex_IMU;
  uint8_t fifo[28];  // MotionApps 6.12 default packet
  uint8_t errCnt = 0;
  uint32_t lastIMU = 0;
  uint32_t lastReinitAttempt = 0;
  bool wasMoving = false;
  uint32_t motionStartTime = 0;
  uint32_t motionPulseAccumMs = 0;  // только busy-импульсы → вход в MOVEMENT
  uint32_t lastMotEvalMs = 0;
  bool localProlongedMovement = false;

  uint32_t lastDebugPrint = 0;

  for (;;) {
    TaskPool::markRun(taskIndex_IMU);
    cfg_taskMonitorUpdate(TASK_IMU_IDX);
    if (otaInProgress) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    uint32_t currentTime = millis();

    if (currentTime - lastDebugPrint > 10000) {
      lastDebugPrint = currentTime;
      Serial.printf("[IMU] Stats - IMU:%dms, Moving:%d, MPU:%d\n",
                    (currentTime - lastIMU),
                    wasMoving, mpuOk);
    }

#if ENABLE_SIMULATION
    updateSimulationData();
    static uint32_t lastImuDebug = 0;
    if (currentTime - lastImuDebug > 200) {
      lastImuDebug = currentTime;
      Serial.printf("[SIM_IMU] angleX=%.2f, angleY=%.2f\n", simAngleX, simAngleY);
    }

    {
      MutexGuard guard(xStateMutex);
      if (guard) {
        angleX = simAngleX;
        angleY = simAngleY;
        temperature = 25.0f + sin(currentTime * 0.001f) * 0.5f;
      }
    }

    IMUData imuData = { simAngleX, simAngleY, 25.0f };
    if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;

    Event event;
    event.type = EventType::IMU_UPDATE;
    event.timestamp = currentTime;
    event.data.imu.angleX = simAngleX;
    event.data.imu.angleY = simAngleY;
    event.data.imu.temperature = 25.0f;
    EventBus::publish(event, 0);

    if (currentState != SystemState::CALIBRATING) {
      setDisplayDirty();
    }

#elif ENABLE_MPU6050

    // ========== ? ПРОВЕРКА: ЕСЛИ MPU НЕ РАБОТАЕТ - ПРОПУСКАЕМ ==========
    if (!mpuOk) {
      // Пытаемся восстановить раз в 30 секунд
      if (currentTime - lastReinitAttempt > 30000) {
        lastReinitAttempt = currentTime;
        initializeDMP();
        Serial.println("[IMU] Attempting MPU reinitialization");
      }

      // Отправляем нулевые данные
      IMUData imuData = { 0, 0, 25.0f };
      if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;

      // Сбрасываем флаги движения
      wasMoving = false;
      motionPulseAccumMs = 0;
      lastMotEvalMs = 0;
      localProlongedMovement = false;
      if (movementModeActive) {
        movementModeActive = false;
        currentSystemMode = previousMode;
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ========== ДЕТЕКЦИЯ ДВИЖЕНИЯ + УГЛЫ (один getIntStatus за тик) ==========
    if (mpuOk) {
      if (currentTime - lastIMU > (uint32_t)cfg_getImuPollMs()) {
        MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(40));
        if (!i2c) {
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }

        // Один раз за цикл: INT_STATUS (latch) + MOT_DETECT_STATUS (оси).
        const uint8_t intStatus = mpu.getIntStatus();
        const uint8_t motStatus = mpu.getMotionStatus();

        // Сначала FIFO: углы + RMS gyro/linAcc (для непрерывного детекта езды).
        const int fifoMax = cfg_getImuFifoAvg();
        float sumX = 0.0f, sumY = 0.0f;
        int nPkt = 0;
        float gyroRmsMax = -1.0f;
        float gyroBumpMax = -1.0f;
        float linAccRmsMax = -1.0f;
        while (nPkt < fifoMax && mpu.dmpGetCurrentFIFOPacket(fifo)) {
          Quaternion q;
          VectorFloat gravity;
          VectorInt16 aa, aaReal, aaWorld;
          float ypr[3];
          int16_t gRaw[3];
          mpu.dmpGetQuaternion(&q, fifo);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          mpu.dmpGetGyro(gRaw, fifo);
          mpu.dmpGetAccel(&aa, fifo);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          // Мировая СК: линейное ускорение без привязки к наклону датчика (Electronic Cats / MotionApps 6.12).
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          const float gRms = sqrtf((float)gRaw[0] * gRaw[0] + (float)gRaw[1] * gRaw[1] +
                                  (float)gRaw[2] * gRaw[2]);
          if (gRms > gyroRmsMax) gyroRmsMax = gRms;
          // Неровности: кивок (pitch=gy) и крен (roll=gx); yaw (gz) не берём —
          // повороты руля не должны сами по себе держать «неровности».
          const float gxAbs = fabsf((float)gRaw[0]);
          const float gyAbs = fabsf((float)gRaw[1]);
          const float bump = (gxAbs > gyAbs) ? gxAbs : gyAbs;
          if (bump > gyroBumpMax) gyroBumpMax = bump;
          const float aRms = sqrtf((float)aaWorld.x * aaWorld.x + (float)aaWorld.y * aaWorld.y +
                                  (float)aaWorld.z * aaWorld.z);
          if (aRms > linAccRmsMax) linAccRmsMax = aRms;
          const float ax = degrees(ypr[2]);
          const float ay = degrees(ypr[1]);
          if (fabsf(ax) <= 90.0f && fabsf(ay) <= 90.0f && !isnan(ax) && !isnan(ay)) {
            sumX += ax;
            sumY += ay;
            nPkt++;
          }
        }

        const bool currentlyMoving = detectMotionFromIMU(intStatus, motStatus, currentTime,
                                                         gyroRmsMax, linAccRmsMax, gyroBumpMax);

        // Поток отладки движения (TEST MOT STREAM)
        if (g_motStreamUntilMs != 0 && currentTime < g_motStreamUntilMs) {
          static uint32_t lastMotLogMs = 0;
          const uint32_t period = g_motStreamPeriodMs < 50 ? 50 : g_motStreamPeriodMs;
          if (currentTime - lastMotLogMs >= period) {
            lastMotLogMs = currentTime;
            const char *ms = "MANUAL";
            if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
            else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
            float ageSec = 0.0f;
            float enterSec = -1.0f;
            float settleLeftSec = 0.0f;
            const uint32_t needMs = (uint32_t)cfg_getMovementDurationSec() * 1000UL;
            if (currentlyMoving && motionStartTime > 0) {
              ageSec = (currentTime - motionStartTime) * 0.001f;
              if (!movementModeActive) {
                enterSec = (motionPulseAccumMs >= needMs)
                               ? 0.0f
                               : (needMs - motionPulseAccumMs) * 0.001f;
              } else {
                enterSec = 0.0f;
              }
            }
            if (g_motLastActivityMs > 0) {
              const uint32_t settleMs = (uint32_t)cfg_getMovementSettleSec() * 1000UL;
              const uint32_t idle = currentTime - g_motLastActivityMs;
              settleLeftSec = (idle >= settleMs) ? 0.0f : (settleMs - idle) * 0.001f;
            }
            float exitSec = -1.0f;
            if (movementModeActive && movementEndTime > 0) {
              const uint32_t settleMs = (uint32_t)cfg_getMovementSettleSec() * 1000UL;
              const uint32_t elapsed = currentTime - movementEndTime;
              exitSec = (elapsed >= settleMs) ? 0.0f : (settleMs - elapsed) * 0.001f;
            }
            Serial.printf(
                "[MOT] t=%lu mot=%d motSt=0x%02X gyro=%.0f gdlt=%.0f thr=%.0f busy=%d "
                "bump=%.0f bdlt=%.0f bthr=%.0f bbusy=%d "
                "lin=%.0f ldlt=%.0f lthr=%.0f lbusy=%d hold=%d was=%d mode=%s "
                "age=%.1f enter=%.1f settleLeft=%.1f exit=%.1f int=0x%02X\n",
                (unsigned long)currentTime,
                g_motLastMotPulse ? 1 : 0,
                (unsigned)g_motLastMotStatus,
                g_motLastGyroRms,
                g_motLastGyroDelta,
                g_motLastGyroThr,
                g_motLastGyroBusy ? 1 : 0,
                g_motLastGyroBump,
                g_motLastGyroBumpDelta,
                g_motLastGyroBumpThr,
                g_motLastGyroBumpBusy ? 1 : 0,
                g_motLastLinAccRaw,
                g_motLastLinAccRms,
                g_motLastLinAccThr,
                g_motLastLinAccBusy ? 1 : 0,
                currentlyMoving ? 1 : 0,
                wasMoving ? 1 : 0,
                ms,
                ageSec,
                enterSec,
                settleLeftSec,
                exitSec,
                (unsigned)g_motLastIntStatus);
          }
        } else if (g_motStreamUntilMs != 0 && currentTime >= g_motStreamUntilMs) {
          g_motStreamUntilMs = 0;
          Serial.println("[MOT] STREAM end");
        }

        const bool pulseNow = g_motLastMotPulse || g_motLastGyroBusy || g_motLastGyroBumpBusy ||
                              g_motLastLinAccBusy;

        if (currentlyMoving) {
          if (!wasMoving) {
            wasMoving = true;
            motionStartTime = currentTime;
            motionPulseAccumMs = 0;
            lastMotEvalMs = currentTime;
            movementStartMs = currentTime;  // 8.9.0: отметка начала движения для экрана ДВИЖЕНИЯ
            localProlongedMovement = false;

            Serial.printf("[IMU] >>> ДВИЖЕНИЕ (MOT=%d gyroRms=%.0f bump=%.0f linAcc=%.0f) <<<\n",
                          (intStatus & 0x40) ? 1 : 0, gyroRmsMax, gyroBumpMax, linAccRmsMax);
            Event event;
            event.type = EventType::MOVEMENT_DETECTED;
            event.timestamp = millis();
            EventBus::publish(event);

          } else {
            // Duration: только реальное busy-время (не settle-afterglow).
            const uint32_t dt = (lastMotEvalMs > 0 && currentTime >= lastMotEvalMs)
                                    ? (currentTime - lastMotEvalMs)
                                    : 0;
            lastMotEvalMs = currentTime;
            if (pulseNow && dt > 0 && dt < 500) {
              motionPulseAccumMs += dt;
            }

            if (!movementModeActive &&
                (motionPulseAccumMs > (uint32_t)cfg_getMovementDurationSec() * 1000UL)) {
            localProlongedMovement = true;
            previousMode = currentSystemMode;

            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                currentSystemMode = SystemMode::MOVEMENT;
              }
            }

            movementModeActive = true;
            g_motModeEnterCount++;
            movementPressureLastCheck = currentTime;
            movementLastAdjustFront = currentTime;  // 8.9.0: корректировка при переходе в ДВИЖЕНИЕ
            movementLastAdjustRear = currentTime;
            movementEndTime = 0;

            Serial.printf("[MOVEMENT] Длительное движение! pulseAccum=%lu мс → MOVEMENT\n",
                          (unsigned long)motionPulseAccumMs);

            float targetPressureFront = cfg_getMovementPressureFront();
            float targetPressureRear = cfg_getMovementPressureRear();
            float avgPressureFront, avgPressureRear;
            {
              MutexGuard guard(xStateMutex);
              if (guard) {
                avgPressureFront = (pressure[PAD_FRONT_LEFT] + pressure[PAD_FRONT_RIGHT]) / 2.0f;
                avgPressureRear = (pressure[PAD_REAR_LEFT] + pressure[PAD_REAR_RIGHT]) / 2.0f;
              } else {
                avgPressureFront = avgPressureRear = 3.0f;
              }
            }

            float diffFront = targetPressureFront - avgPressureFront;
            if (abs(diffFront) > cfg_getMovementTolerance()) {  // 8.8.0: допуск из меню
              if (diffFront > 0) {
                sendValveCommand(PAD_FRONT_LEFT, true, 1500);
                sendValveCommand(PAD_FRONT_RIGHT, true, 1500);
              } else {
                sendValveCommand(PAD_FRONT_LEFT, false, 1200);
                sendValveCommand(PAD_FRONT_RIGHT, false, 1200);
              }
            }

            float diffRear = targetPressureRear - avgPressureRear;
            if (abs(diffRear) > cfg_getMovementTolerance()) {  // 8.8.0: допуск из меню
              if (diffRear > 0) {
                sendValveCommand(PAD_REAR_LEFT, true, 1500);
                sendValveCommand(PAD_REAR_RIGHT, true, 1500);
              } else {
                sendValveCommand(PAD_REAR_LEFT, false, 1200);
                sendValveCommand(PAD_REAR_RIGHT, false, 1200);
              }
            }

            if (manualControlActive) {
              stopManualOperation();
            }

            Event event;
            event.type = EventType::MOVEMENT_DETECTED;
            event.timestamp = millis();
            EventBus::publish(event);
            }
          }
        } else {
          if (wasMoving) {
            wasMoving = false;
            motionPulseAccumMs = 0;
            lastMotEvalMs = 0;
            Serial.printf("[IMU] >>> ДВИЖЕНИЕ ОКОНЧЕНО (пауза %d с) <<<\n",
                          cfg_getMovementSettleSec());

            Event event;
            event.type = EventType::MOVEMENT_ENDED;
            event.timestamp = millis();
            EventBus::publish(event);
          }

          if (movementModeActive) {
            if (movementEndTime == 0) {
              movementEndTime = currentTime;
              Serial.println("[MOVEMENT] Ожидание 30 секунд перед возвратом");
            } else if (currentTime - movementEndTime >= (uint32_t)cfg_getMovementSettleSec() * 1000UL) {  // 8.8.0: из меню
              movementModeActive = false;
              movementEndTime = 0;
              currentSystemMode = previousMode;
              localProlongedMovement = false;
              motionStartTime = 0;

              if (previousMode == SystemMode::AUTO) {
                currentMode = Mode::AUTO;
                lastLevelingCheckTime = currentTime;
                lastLevelingAttemptTime = currentTime;
                levelingAttemptsThisHour = 0;
              } else {
                currentMode = Mode::MANUAL;
                setManualTargetsFromParkingPolicy();
              }

              Event event;
              event.type = EventType::MOVEMENT_ENDED;
              event.timestamp = millis();
              EventBus::publish(event);
            }
          } else {
            motionStartTime = 0;
            localProlongedMovement = false;
            movementEndTime = 0;
          }
        }

        {
          MutexGuard guard(xStateMutex);
          if (guard) {
            bool oldIsMoving = isMoving;
            isMoving = wasMoving;
            prolongedMovementDetected = localProlongedMovement;

            if (oldIsMoving != wasMoving) {
              setDisplayDirty();
            }
          }
        }

        // Углы уже прочитаны из FIFO выше (sumX/Y, nPkt).
        (void)intStatus;

        if (nPkt > 0) {
          const float ax = sumX / (float)nPkt;
          const float ay = sumY / (float)nPkt;
          g_imuRawX = ax;
          g_imuRawY = ay;

          // EMA после Калмана. Выброс > thr — тянем слабее (глушим всплеск).
          static float emaX = 0.0f, emaY = 0.0f;
          static bool emaInit = false;
          static float limX = 0.0f, limY = 0.0f;
          static bool limInit = false;
          if (g_imuFilterResetReq) {
            emaInit = false;
            limInit = false;
            // setParameters + «прогрев» — без placement-new (гонка с applyRuntimeSettings).
            const float mea = cfg_getImuKalmanMea();
            const float est = cfg_getImuKalmanEst();
            const float q = cfg_getImuKalmanQ();
            filterX.setParameters(mea, est, q);
            filterY.setParameters(mea, est, q);
            for (int k = 0; k < 8; k++) {
              (void)filterX.filtered(ax);
              (void)filterY.filtered(ay);
            }
            g_imuFilterResetReq = false;
          }

          float newX = filterX.filtered(ax);
          float newY = filterY.filtered(ay);

          const float aNom = cfg_getImuEmaAlpha();
          const float aSpike = cfg_getImuEmaSpikeAlpha();
          const float spikeThr = cfg_getImuEmaSpikeThr();
          if (!emaInit) {
            emaX = newX;
            emaY = newY;
            emaInit = true;
          } else {
            float aX = aNom;
            float aY = aNom;
            if (fabsf(newX - emaX) > spikeThr) aX = aSpike;
            if (fabsf(newY - emaY) > spikeThr) aY = aSpike;
            emaX += aX * (newX - emaX);
            emaY += aY * (newY - emaY);
          }
          newX = emaX;
          newY = emaY;

          // Ограничение °/с + catch-up: при большой ошибке догоняем быстрее
          // (иначе рывок 2°/тик режется до ~0.6° при slew=40).
          const float dt = fmaxf((float)cfg_getImuPollMs() * 0.001f, 0.015f);
          float maxStepX = cfg_getImuSlewDps() * dt;
          float maxStepY = maxStepX;
          if (!limInit) {
            limX = newX;
            limY = newY;
            limInit = true;
          } else {
            const float errX = fabsf(newX - limX);
            const float errY = fabsf(newY - limY);
            if (errX > 1.0f) maxStepX = fmaxf(maxStepX, errX * 0.55f);
            if (errY > 1.0f) maxStepY = fmaxf(maxStepY, errY * 0.55f);
            limX += constrain(newX - limX, -maxStepX, maxStepX);
            limY += constrain(newY - limY, -maxStepY, maxStepY);
          }
          newX = limX;
          newY = limY;

          float newTemp = (mpu.getTemperature() / 340.0f) + 36.53f;

          if (isnan(newX) || isnan(newY) || isnan(newTemp) || fabsf(newX) > 90 ||
              fabsf(newY) > 90 || newTemp < -20 || newTemp > 120) {
            Serial.printf("[IMU] Invalid data: X=%.2f, Y=%.2f, T=%.2f\n", newX, newY, newTemp);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
          }

          // Нуль применяем здесь — один источник истины для UI и control
          g_imuAbsX = newX;
          g_imuAbsY = newY;
          const float outX = newX - cfg_getZeroAngleX();
          const float outY = newY - cfg_getZeroAngleY();
          g_imuOutX = outX;
          g_imuOutY = outY;

          // STREAM: сырой + фильтр на каждый тик
          const uint32_t nowMs = millis();
          if (g_imuStreamUntilMs != 0 && nowMs < g_imuStreamUntilMs) {
            Serial.printf("[IMU] S %lu raw=%.3f,%.3f out=%.3f,%.3f n=%d\n",
                          (unsigned long)nowMs, ax, ay, outX, outY, nPkt);
          } else if (g_imuStreamUntilMs != 0 && nowMs >= g_imuStreamUntilMs) {
            g_imuStreamUntilMs = 0;
            Serial.println("[IMU] STREAM end");
          }

          // STATS: копит метрики шума/всплесков
          if (g_imuStatsActive) {
            if (g_imuStatN == 0) {
              g_imuStatT0 = nowMs;
              g_imuStatPrevX = outX;
              g_imuStatPrevY = outY;
              g_imuStatMinX = g_imuStatMaxX = outX;
              g_imuStatMinY = g_imuStatMaxY = outY;
            } else {
              const float dx = fabsf(outX - g_imuStatPrevX);
              const float dy = fabsf(outY - g_imuStatPrevY);
              const float step = fmaxf(dx, dy);
              if (step > g_imuStatMaxStep) g_imuStatMaxStep = step;
              if (step > spikeThr) g_imuStatSpikes++;
              if (outX < g_imuStatMinX) g_imuStatMinX = outX;
              if (outX > g_imuStatMaxX) g_imuStatMaxX = outX;
              if (outY < g_imuStatMinY) g_imuStatMinY = outY;
              if (outY > g_imuStatMaxY) g_imuStatMaxY = outY;
              g_imuStatPrevX = outX;
              g_imuStatPrevY = outY;
            }
            g_imuStatSumX += outX;
            g_imuStatSumY += outY;
            g_imuStatSumXX += outX * outX;
            g_imuStatSumYY += outY * outY;
            g_imuStatN++;
            if (g_imuStatsUntilMs != 0 && nowMs >= g_imuStatsUntilMs) {
              const float n = (float)g_imuStatN;
              const float meanX = g_imuStatSumX / n;
              const float meanY = g_imuStatSumY / n;
              float varX = g_imuStatSumXX / n - meanX * meanX;
              float varY = g_imuStatSumYY / n - meanY * meanY;
              if (varX < 0) varX = 0;
              if (varY < 0) varY = 0;
              const float dtSec = fmaxf((nowMs - g_imuStatT0) * 0.001f, 0.001f);
              Serial.printf(
                  "[IMU] STATS n=%u hz=%.1f std=%.4f,%.4f pp=%.3f,%.3f spikes=%u maxStep=%.3f "
                  "mean=%.3f,%.3f\n",
                  (unsigned)g_imuStatN, (float)g_imuStatN / dtSec, sqrtf(varX), sqrtf(varY),
                  g_imuStatMaxX - g_imuStatMinX, g_imuStatMaxY - g_imuStatMinY,
                  (unsigned)g_imuStatSpikes, g_imuStatMaxStep, meanX, meanY);
              Serial.println("[TEST] OK IMU_STATS");
              g_imuStatsActive = false;
              g_imuStatsUntilMs = 0;
            }
          }

          {
            MutexGuard guard(xStateMutex);
            if (guard) {
              angleX = outX;
              angleY = outY;
              temperature = newTemp;
            }
          }

          Event event;
          event.type = EventType::IMU_UPDATE;
          event.timestamp = millis();
          event.data.imu.angleX = outX;
          event.data.imu.angleY = outY;
          event.data.imu.temperature = newTemp;
          EventBus::publish(event, 0);

          IMUData imuData = { outX, outY, newTemp };
          // Неблокирующая отправка: иначе при медленном controlTask очередь
          // забивается и imuTask тормозит до ~10 Гц.
          if (xQueueSend(xIMUQueue, &imuData, 0) != pdTRUE) {
            IMUData discard;
            if (xQueueReceive(xIMUQueue, &discard, 0) == pdTRUE) {
              xQueueSend(xIMUQueue, &imuData, 0);
            }
            imuQueueDropCount++;
          }

          lastIMU = currentTime;
          errCnt = 0;
          reinitAttempts = 0;

          if (cfg_errorIsActive((uint8_t)EH_Error::MPU)) {
            cfg_errorMarkCleared((uint8_t)EH_Error::MPU);
            Serial.println("[IMU] MPU восстановлен, ошибка будет удалена через 3 сек");
          }

        } else {
          lastIMU = currentTime;  // MOT обработан даже без пакета FIFO
          if (intStatus & 0x10) {
            Serial.println("[IMU] FIFO overflow — resetFIFO");
            mpu.resetFIFO();
            if (++errCnt > 10) {
              mpuOk = false;
              errCnt = 0;
              reinitAttempts++;
              Serial.printf("[IMU] Reinit scheduled after overflow (%d/5)\n", reinitAttempts);
            }
          } else {
            errCnt = 0;
          }
        }
      }
    }

    if (!mpuOk && reinitAttempts > 0 && reinitAttempts <= 5 &&
        currentTime - lastReinitAttempt > 2000) {
      lastReinitAttempt = currentTime;
      initializeDMP();
      (void)reinitAttempts;
    }

#else
    // ========== ТЕСТОВЫЙ РЕЖИМ (MPU отключен) ==========
    static uint32_t lastTestUpdate = 0;
    if (currentTime - lastTestUpdate > 1000) {
      lastTestUpdate = currentTime;

      MutexGuard guard(xStateMutex);
      if (guard) {
        if (angleX != 0 || angleY != 0) {
          angleX = 0;
          angleY = 0;
          setDisplayDirty();
        }
      }

      IMUData imuData = { 0, 0, 25.0f };
      if (xQueueSend(xIMUQueue, &imuData, pdMS_TO_TICKS(100)) != pdTRUE) imuQueueDropCount++;
    }

    wasMoving = false;
    motionPulseAccumMs = 0;
    lastMotEvalMs = 0;
    localProlongedMovement = false;
#endif

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
