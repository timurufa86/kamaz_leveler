#include "test_harness.h"
#include "config_manager.h"
#include "error_handler.h"
#include "task_monitor.h"
#include "task_button.h"
#include "task_display.h"
#include "task_imu.h"
#include "imu_dmp.h"
#include "imu_motion.h"
#include "ui_screens.h"
#include "ui_ota_menu.h"
#include "wifi_setup.h"
#include "github_ota_request.h"
#include "pressure_read.h"
#include "app_globals.h"
#include "app_pins.h"
#include "app_version.h"
#include "mutex_guard.h"
#include "jhm1200.h"
#include <WiFi.h>
#include <Wire.h>

extern bool backlightDimmed;
extern uint32_t lastUserActivityMs;
extern char sta_ssid[33];
extern int otaProgress;
extern bool errorScreenBlocking;

static volatile bool g_testHeartbeat = false;
static volatile bool g_testDebugDump = false;

/* ====================  SERIAL SELF-TEST HARNESS ====================
 * Команды (строка + \\n или \\r): TEST <CMD> [args]
 * Ответы: [TEST] ...  / маркеры TEST OK / TEST FAIL / TEST DONE
 * Хост-раннер: kamaz_leveler/_serial_selftest.py
 * ================================================================== */
static uint8_t g_selfTestPhase = 0;
static uint32_t g_selfTestAtMs = 0;
static uint8_t g_selfTestFails = 0;

static void testReplyOk(const char *msg) {
  Serial.printf("[TEST] OK %s\n", msg ? msg : "");
}

static void testReplyFail(const char *msg) {
  Serial.printf("[TEST] FAIL %s\n", msg ? msg : "");
}

static void testPrintHelp() {
  Serial.println("[TEST] HELP commands:");
  Serial.println("  TEST PING | STATUS | HELP | VER | STACK | CFG | HEAP | WDT");
  Serial.println("  TEST MENU OPEN|CLOSE|TOGGLE");
  Serial.println("  TEST BRIGHT <10..100> | DIM ON|OFF|TOGGLE");
  Serial.println("  TEST DIRTY | SAVE | RESETDISP");
  Serial.println("  TEST HEARTBEAT ON|OFF | DEBUG ON|OFF");
  Serial.println("  TEST PRESS | ADS | WIFI | WIFI SCAN | ERR | CALIB | BTN | I2C");
  Serial.println("  TEST IMU | IMU CFG | IMU STREAM [sec] | IMU STATS [sec]");
  Serial.println("  TEST IMU SET <key> <val> | IMU PRESET BAL|SMOOTH|FAST | IMU SAVE");
  Serial.println("  TEST MOT | MOT CFG | MOT STREAM [sec] | MOT STATS | MOT RESET | MOT SAVE");
  Serial.println("  TEST MOT SET gyro|bump|lin|det|dur|duration|settle|period <val>");
  Serial.println("  TEST MODE [MANUAL|AUTO|MOVEMENT]");
  Serial.println("  TEST OTA STATUS | OTA LIST");
  Serial.println("  TEST SELF | FULL   — smoke / multi-mode sequence");
}

static void testPrintPress() {
  float rawBar = -999.0f;
  bool ok = false;
  if (jhmReady) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
    if (i2c) ok = Jhm1200::readBar(rawBar);
  }
  Serial.printf("[TEST] PRESS master=%.2f pads=%.2f,%.2f,%.2f,%.2f zero=%.3f rawBar=%.3f jhm=%d st=0x%02X first=%d calib=%d\n",
                masterPressure, pressure[0], pressure[1], pressure[2], pressure[3],
                g_pressureZeroBar, ok ? rawBar : -1.0f, jhmReady ? 1 : 0, Jhm1200::lastStatus(),
                firstPressureMeasurementDone ? 1 : 0, calibrationCompleted ? 1 : 0);
}

static void testPrintImu() {
  Serial.printf("[TEST] IMU mpu=%d ang=%.3f,%.3f raw=%.3f,%.3f temp=%.1f\n",
                mpuOk ? 1 : 0, angleX, angleY, g_imuRawX, g_imuRawY, temperature);
  Serial.printf("[TEST] IMU_CFG mea=%.2f est=%.2f q=%.3f poll=%d fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f redraw=%.2f det=%d\n",
                ConfigManager::getImuKalmanMea(), ConfigManager::getImuKalmanEst(),
                ConfigManager::getImuKalmanQ(), ConfigManager::getImuPollMs(),
                ConfigManager::getImuFifoAvg(), ConfigManager::getImuEmaAlpha(),
                ConfigManager::getImuEmaSpikeAlpha(), ConfigManager::getImuEmaSpikeThr(),
                ConfigManager::getImuSlewDps(), ConfigManager::getRedrawAngleThr(),
                ConfigManager::getImuMotionDet());
}

/** Применить пресет фильтров IMU сразу (без меню). */
static void imuApplyPreset(const char *name) {
  if (strcasecmp(name, "BAL") == 0 || strcasecmp(name, "BALANCE") == 0 || strcasecmp(name, "BALANCED") == 0) {
    // 10.1.28: ослаблен (ближе к быстрой реакции)
    ConfigManager::setImuKalmanMea(5.5f);
    ConfigManager::setImuKalmanEst(3.5f);
    ConfigManager::setImuKalmanQ(0.008f);
    ConfigManager::setImuPollMs(20);
    ConfigManager::setImuFifoAvg(2);
    ConfigManager::setImuEmaAlpha(0.28f);
    ConfigManager::setImuEmaSpikeAlpha(0.10f);
    ConfigManager::setImuEmaSpikeThr(1.8f);
    ConfigManager::setImuSlewDps(70.0f);
    ConfigManager::setRedrawAngleThr(0.05f);
  } else if (strcasecmp(name, "SMOOTH") == 0) {
    // 10.1.28: ослаблен (меньше залипание)
    ConfigManager::setImuKalmanMea(8.0f);
    ConfigManager::setImuKalmanEst(5.0f);
    ConfigManager::setImuKalmanQ(0.004f);
    ConfigManager::setImuPollMs(25);
    ConfigManager::setImuFifoAvg(4);
    ConfigManager::setImuEmaAlpha(0.18f);
    ConfigManager::setImuEmaSpikeAlpha(0.07f);
    ConfigManager::setImuEmaSpikeThr(1.4f);
    ConfigManager::setImuSlewDps(25.0f);
    ConfigManager::setRedrawAngleThr(0.07f);
  } else if (strcasecmp(name, "FAST") == 0) {
    ConfigManager::setImuKalmanMea(3.5f);
    ConfigManager::setImuKalmanEst(3.0f);
    ConfigManager::setImuKalmanQ(0.015f);
    ConfigManager::setImuPollMs(15);
    ConfigManager::setImuFifoAvg(1);
    ConfigManager::setImuEmaAlpha(0.50f);
    ConfigManager::setImuEmaSpikeAlpha(0.18f);
    ConfigManager::setImuEmaSpikeThr(2.5f);
    ConfigManager::setImuSlewDps(90.0f);
    ConfigManager::setRedrawAngleThr(0.04f);
  } else {
    return;
  }
  editImuKalmanMea = ConfigManager::getImuKalmanMea();
  editImuKalmanEst = ConfigManager::getImuKalmanEst();
  editImuKalmanQ = ConfigManager::getImuKalmanQ();
  editImuPollMs = ConfigManager::getImuPollMs();
  editImuFifoAvg = ConfigManager::getImuFifoAvg();
  editImuEmaAlpha = ConfigManager::getImuEmaAlpha();
  editImuEmaSpikeAlpha = ConfigManager::getImuEmaSpikeAlpha();
  editImuEmaSpikeThr = ConfigManager::getImuEmaSpikeThr();
  editImuSlewDps = ConfigManager::getImuSlewDps();
  editRedrawAngle = ConfigManager::getRedrawAngleThr();
  applyRuntimeSettings();
}

/** TEST IMU SET key value — live-тюнинг. */
static bool imuSetParam(const char *key, const char *valStr) {
  if (!key || !valStr || !*valStr) return false;
  const float fv = atof(valStr);
  const int iv = atoi(valStr);
  if (strcasecmp(key, "mea") == 0) ConfigManager::setImuKalmanMea(constrain(fv, 0.5f, 25.0f));
  else if (strcasecmp(key, "est") == 0) ConfigManager::setImuKalmanEst(constrain(fv, 0.5f, 25.0f));
  else if (strcasecmp(key, "q") == 0) ConfigManager::setImuKalmanQ(constrain(fv, 0.001f, 0.100f));
  else if (strcasecmp(key, "poll") == 0) ConfigManager::setImuPollMs(constrain(iv, 15, 100));
  else if (strcasecmp(key, "fifo") == 0) ConfigManager::setImuFifoAvg(constrain(iv, 1, 8));
  else if (strcasecmp(key, "ema") == 0) ConfigManager::setImuEmaAlpha(constrain(fv, 0.05f, 0.50f));
  else if (strcasecmp(key, "spike") == 0) ConfigManager::setImuEmaSpikeAlpha(constrain(fv, 0.05f, 0.50f));
  else if (strcasecmp(key, "thr") == 0) ConfigManager::setImuEmaSpikeThr(constrain(fv, 0.5f, 5.0f));
  else if (strcasecmp(key, "slew") == 0) ConfigManager::setImuSlewDps(constrain(fv, 5.0f, 120.0f));
  else if (strcasecmp(key, "redraw") == 0) ConfigManager::setRedrawAngleThr(constrain(fv, 0.01f, 0.5f));
  else return false;
  applyRuntimeSettings();
  return true;
}

static void imuStartStats(uint32_t sec) {
  if (sec < 1) sec = 1;
  if (sec > 30) sec = 30;
  g_imuStatN = 0;
  g_imuStatSpikes = 0;
  g_imuStatSumX = g_imuStatSumY = 0;
  g_imuStatSumXX = g_imuStatSumYY = 0;
  g_imuStatMaxStep = 0;
  g_imuStatsActive = true;
  g_imuStatsUntilMs = millis() + sec * 1000UL;
  Serial.printf("[IMU] STATS start %lus\n", (unsigned long)sec);
}

static void testPrintWifi() {
  const bool sta = (WiFi.status() == WL_CONNECTED);
  Serial.printf("[TEST] WIFI mode=%d sta=%d ssid=%s ip=%s rssi=%d heap=%u maxBlk=%u\n",
                (int)WiFi.getMode(), sta ? 1 : 0,
                sta ? WiFi.SSID().c_str() : sta_ssid,
                sta ? WiFi.localIP().toString().c_str() : "-",
                sta ? (int)WiFi.RSSI() : 0,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

static void testPrintOta() {
  Serial.printf("[TEST] OTA wifi=%d releases=%u sel=%d status=%s hdr=%s latest=%s inProg=%d pct=%d\n",
                (WiFi.status() == WL_CONNECTED) ? 1 : 0,
                (unsigned)otaReleaseCount, (int)otaSelectedIndex,
                otaListStatus, otaListHdrBuf, otaLatestTag,
                otaInProgress ? 1 : 0, otaProgress);
  for (uint8_t i = 0; i < otaReleaseCount && i < OTA_LIST_MAX; i++) {
    Serial.printf("[TEST] OTA_REL %u %s size=%lu\n", (unsigned)i, otaReleases[i].tag,
                  (unsigned long)otaReleases[i].size);
  }
}

static void testPrintBtn() {
  Serial.printf("[TEST] BTN gpio KN1=%d KN2=%d KN3=%d KN4=%d | menuPair=%d modePair=%d emergPair=%d navIgnore=%lu\n",
                digitalRead(PIN_BUT1), digitalRead(PIN_BUT2), digitalRead(PIN_BUT3), digitalRead(PIN_BUT4),
                menuPairPressedNow() ? 1 : 0, modePairPressedNow() ? 1 : 0,
                emergencyPairPressedNow() ? 1 : 0,
                (unsigned long)(g_menuNavIgnoreUntilMs > millis()
                                    ? (g_menuNavIgnoreUntilMs - millis())
                                    : 0));
}

static void testPrintErr() {
  const int n = ErrorHandler::getActiveErrorCount();
  Serial.printf("[TEST] ERR count=%d blocking=%d\n", n, errorScreenBlocking ? 1 : 0);
  for (int i = 0; i < n; i++) {
    ErrorHandler::Error e = ErrorHandler::getActiveErrorAt(i);
    Serial.printf("[TEST] ERR_ITEM %d code=%d %s\n", i, (int)e, ErrorHandler::getErrorMessage(e));
  }
}

static void testPrintCalib() {
  Serial.printf("[TEST] CALIB done=%d state=%d zeroBar=%.3f firstPress=%d\n",
                calibrationCompleted ? 1 : 0, (int)getSystemState(), g_pressureZeroBar,
                firstPressureMeasurementDone ? 1 : 0);
}

static void testScanI2c() {
  Serial.print("[TEST] I2C");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(30));
    if (!i2c) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  Serial.printf(" n=%u\n", (unsigned)found);
}

static void testPrintStatus() {
  Serial.printf("[TEST] STATUS menu=%d dim=%d bright=%d editBright=%d dirty=%d "
                "svc=%d mode=%d mpu=%d heap=%u minHeap=%u maxBlk=%u uptime=%lus req=%u\n",
                menuVisible ? 1 : 0,
                backlightDimmed ? 1 : 0,
                ConfigManager::getContrast(),
                editContrast,
                settingsChanged ? 1 : 0,
                (int)serviceScreen,
                (int)currentSystemMode,
                mpuOk ? 1 : 0,
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMinFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(),
                (unsigned long)(millis() / 1000UL),
                (unsigned)g_menuReq);
  Serial.printf("[TEST] STATUS angles=%.2f,%.2f master=%.2f pads=%.2f/%.2f/%.2f/%.2f frameMs=%d blMin=%d idleMs=%lu\n",
                angleX, angleY, masterPressure,
                pressure[0], pressure[1], pressure[2], pressure[3],
                ConfigManager::getFrameMs(),
                ConfigManager::getBacklightOffMin(),
                (unsigned long)(millis() - lastUserActivityMs));
}

static void testPrintMot() {
  const char *ms = "MANUAL";
  if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
  else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
  const uint32_t now = millis();
  float settleLeft = 0.0f;
  if (g_motLastActivityMs > 0) {
    const uint32_t settleMs = (uint32_t)ConfigManager::getMovementSettleSec() * 1000UL;
    const uint32_t idle = now - g_motLastActivityMs;
    settleLeft = (idle >= settleMs) ? 0.0f : (settleMs - idle) * 0.001f;
  }
  Serial.printf(
      "[TEST] MOT hold=%d mot=%d motSt=0x%02X busy=%d gyro=%.0f gdlt=%.0f thr=%.0f "
      "bump=%.0f bdlt=%.0f bthr=%.0f bbusy=%d "
      "lin=%.0f ldlt=%.0f lthr=%.0f lbusy=%d int=0x%02X mode=%s "
      "movActive=%d settleLeft=%.1f\n",
      g_motLastHold ? 1 : 0, g_motLastMotPulse ? 1 : 0, (unsigned)g_motLastMotStatus,
      g_motLastGyroBusy ? 1 : 0,
      g_motLastGyroRms, g_motLastGyroDelta, g_motLastGyroThr,
      g_motLastGyroBump, g_motLastGyroBumpDelta, g_motLastGyroBumpThr,
      g_motLastGyroBumpBusy ? 1 : 0, g_motLastLinAccRaw, g_motLastLinAccRms, g_motLastLinAccThr,
      g_motLastLinAccBusy ? 1 : 0, (unsigned)g_motLastIntStatus, ms,
      movementModeActive ? 1 : 0, settleLeft);
  Serial.printf(
      "[TEST] MOT_CFG duration=%ds settle=%ds gyroMenu=%d gyroThr=%.0f bumpMenu=%d bumpThr=%.0f "
      "linThr=%d det=%d motDur=%ums period=%lums\n",
      ConfigManager::getMovementDurationSec(), ConfigManager::getMovementSettleSec(),
      ConfigManager::getGyroThreshold(), (float)ConfigManager::getGyroThreshold() * 8.0f,
      ConfigManager::getGyroBumpThreshold(), (float)ConfigManager::getGyroBumpThreshold() * 8.0f,
      ConfigManager::getAccelThreshold(),
      ConfigManager::getImuMotionDet(), (unsigned)g_motDurMs,
      (unsigned long)g_motStreamPeriodMs);
}

static void testPrintMotStats() {
  const uint32_t now = millis();
  const uint32_t dt = (g_motStatsSinceMs > 0 && now > g_motStatsSinceMs)
                          ? (now - g_motStatsSinceMs)
                          : 0;
  const float sec = dt > 0 ? dt * 0.001f : 0.0f;
  const float n = g_motSampleCount > 0 ? (float)g_motSampleCount : 1.0f;
  Serial.printf(
      "[TEST] MOT_STATS n=%lu sec=%.1f mot%%=%.1f busy%%=%.1f bump%%=%.1f lin%%=%.1f holdEdges=%lu modeEnter=%lu "
      "lastGyro=%.0f thr=%.0f lastBump=%.0f bthr=%.0f lastLin=%.0f lthr=%.0f\n",
      (unsigned long)g_motSampleCount, sec,
      100.0f * (float)g_motMotPulseCount / n, 100.0f * (float)g_motGyroBusyCount / n,
      100.0f * (float)g_motGyroBumpBusyCount / n,
      100.0f * (float)g_motLinAccBusyCount / n,
      (unsigned long)g_motHoldEdgeCount, (unsigned long)g_motModeEnterCount,
      g_motLastGyroRms, g_motLastGyroThr, g_motLastGyroBump, g_motLastGyroBumpThr,
      g_motLastLinAccRms, g_motLastLinAccThr);
}

static void testMotResetStats() {
  g_motSampleCount = 0;
  g_motMotPulseCount = 0;
  g_motGyroBusyCount = 0;
  g_motGyroBumpBusyCount = 0;
  g_motLinAccBusyCount = 0;
  g_motHoldEdgeCount = 0;
  g_motModeEnterCount = 0;
  g_motStatsSinceMs = millis();
}

void processTestCommandLine(char *line) {
  if (!line || line[0] == '\0') return;
  if (strncmp(line, "TEST", 4) != 0 && strncmp(line, "test", 4) != 0) return;
  char *cmd = line + 4;
  while (*cmd == ' ') cmd++;

      if (strcasecmp(cmd, "PING") == 0) {
        testReplyOk("PONG");
      } else if (strcasecmp(cmd, "HELP") == 0) {
        testPrintHelp();
        testReplyOk("HELP");
      } else if (strcasecmp(cmd, "VER") == 0) {
        Serial.printf("[TEST] VER %s\n", VERSION);
        testReplyOk("VER");
      } else if (strcasecmp(cmd, "STATUS") == 0) {
        testPrintStatus();
        testReplyOk("STATUS");
      } else if (strcasecmp(cmd, "HEAP") == 0) {
        Serial.printf("[TEST] HEAP free=%u min=%u\n",
                      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
        testReplyOk("HEAP");
      } else if (strcasecmp(cmd, "STACK") == 0) {
        printTaskInfo();
        testReplyOk("STACK");
      } else if (strcasecmp(cmd, "CFG") == 0) {
        ConfigManager::dumpToSerial();
        testReplyOk("CFG");
      } else if (strcasecmp(cmd, "MENU OPEN") == 0) {
        requestMenuOpen("TEST");
        testReplyOk("MENU_OPEN_REQ");
      } else if (strcasecmp(cmd, "MENU CLOSE") == 0) {
        requestMenuClose("TEST");
        testReplyOk("MENU_CLOSE_REQ");
      } else if (strcasecmp(cmd, "MENU TOGGLE") == 0) {
        if (menuVisible) requestMenuClose("TEST");
        else requestMenuOpen("TEST");
        testReplyOk("MENU_TOGGLE_REQ");
      } else if (strncasecmp(cmd, "BRIGHT ", 7) == 0) {
        int v = atoi(cmd + 7);
        v = constrain(v, CONTRAST_MIN, CONTRAST_MAX);
        editContrast = v;
        ConfigManager::setContrast(v);
        applyBacklightPwm(v);
        backlightDimmed = false;
        settingsChanged = true;
        lastUserActivityMs = millis();
        Serial.printf("[TEST] BRIGHT %d\n", v);
        testReplyOk("BRIGHT");
      } else if (strcasecmp(cmd, "DIM ON") == 0) {
        backlightDimmed = true;
        applyBacklightPwm(CONTRAST_DIM);
        testReplyOk("DIM_ON");
      } else if (strcasecmp(cmd, "DIM OFF") == 0) {
        backlightDimmed = false;
        applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        lastUserActivityMs = millis();
        testReplyOk("DIM_OFF");
      } else if (strcasecmp(cmd, "DIM TOGGLE") == 0) {
        if (backlightDimmed) {
          backlightDimmed = false;
          applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
          lastUserActivityMs = millis();
          testReplyOk("DIM_OFF");
        } else {
          backlightDimmed = true;
          applyBacklightPwm(CONTRAST_DIM);
          testReplyOk("DIM_ON");
        }
      } else if (strcasecmp(cmd, "DIRTY") == 0) {
        settingsChanged = true;
        testReplyOk("DIRTY");
      } else if (strcasecmp(cmd, "SAVE") == 0) {
        saveMenuSettings();
        testReplyOk("SAVE");
      } else if (strcasecmp(cmd, "RESETDISP") == 0) {
        // Безопасно: только флаг — DisplayTask подхватит через dirty;
        // forceDisplayReset без мьютекса не вызываем.
        if (!menuVisible) {
          displayDirty = true;
          // Запрос полного сброса через close-path нельзя — меню закрыто.
          // Ставим dirty; для hard reset — краткий take.
          MutexGuard guard(xDisplayMutex, pdMS_TO_TICKS(200));
          if (guard) {
            forceDisplayReset(true);
            testReplyOk("RESETDISP");
          } else {
            testReplyFail("RESETDISP_NO_MUTEX");
          }
        } else {
          testReplyFail("RESETDISP_MENU_OPEN");
        }
      } else if (strcasecmp(cmd, "HEARTBEAT ON") == 0) {
        g_testHeartbeat = true;
        testReplyOk("HEARTBEAT_ON");
      } else if (strcasecmp(cmd, "HEARTBEAT OFF") == 0) {
        g_testHeartbeat = false;
        testReplyOk("HEARTBEAT_OFF");
      } else if (strcasecmp(cmd, "DEBUG ON") == 0) {
        g_testDebugDump = true;
        testReplyOk("DEBUG_ON");
      } else if (strcasecmp(cmd, "DEBUG OFF") == 0) {
        g_testDebugDump = false;
        testReplyOk("DEBUG_OFF");
      } else if (strcasecmp(cmd, "WDT") == 0) {
        Serial.printf("[TEST] WDT alive t=%lu\n", (unsigned long)millis());
        testReplyOk("WDT");
      } else if (strcasecmp(cmd, "PRESS") == 0) {
        testPrintPress();
        testReplyOk("PRESS");
      } else if (strcasecmp(cmd, "ADS") == 0) {
        testPrintPress();
        testReplyOk("ADS");
      } else if (strncasecmp(cmd, "MOT", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        const char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg == '\0' || strcasecmp(arg, "CFG") == 0) {
          testPrintMot();
          testReplyOk(*arg ? "MOT_CFG" : "MOT");
        } else if (strncasecmp(arg, "STREAM", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 30;
          if (sec < 1) sec = 1;
          if (sec > 600) sec = 600;  // до 10 мин — тест в машине
          if (g_motStatsSinceMs == 0) testMotResetStats();
          g_motStreamUntilMs = millis() + (uint32_t)sec * 1000UL;
          Serial.printf("[MOT] STREAM start %ds period=%lums (gyro thr=%.0f = menu%d×8, linThr=%d)\n",
                        sec, (unsigned long)g_motStreamPeriodMs, g_motLastGyroThr > 0
                            ? g_motLastGyroThr
                            : (float)ConfigManager::getGyroThreshold() * 8.0f,
                        ConfigManager::getGyroThreshold(),
                        ConfigManager::getAccelThreshold());
          testReplyOk("MOT_STREAM");
        } else if (strcasecmp(arg, "STATS") == 0) {
          testPrintMotStats();
          testReplyOk("MOT_STATS");
        } else if (strcasecmp(arg, "RESET") == 0) {
          testMotResetStats();
          Serial.println("[MOT] stats reset");
          testReplyOk("MOT_RESET");
        } else if (strcasecmp(arg, "SAVE") == 0) {
          // Пишем live ConfigManager в LittleFS (не через устаревшие edit*).
          editGyroThreshold = ConfigManager::getGyroThreshold();
          editGyroBumpThreshold = ConfigManager::getGyroBumpThreshold();
          editAccelThreshold = ConfigManager::getAccelThreshold();
          editMoveSettle = ConfigManager::getMovementSettleSec();
          editMoveDuration = ConfigManager::getMovementDurationSec();
          const bool saved = saveConfig();
          if (saved) {
            settingsChanged = false;
            Serial.println("[MOT] config.txt сохранён");
            testPrintMot();
            testReplyOk("MOT_SAVE");
          } else {
            testReplyFail("MOT_SAVE");
          }
        } else if (strncasecmp(arg, "SET", 3) == 0) {
          char key[16] = {0};
          char val[24] = {0};
          if (sscanf(arg + 3, " %15s %23s", key, val) == 2) {
            const int iv = atoi(val);
            bool ok = true;
            if (strcasecmp(key, "gyro") == 0) {
              ConfigManager::setGyroThreshold(constrain(iv, 10, 200));
              editGyroThreshold = ConfigManager::getGyroThreshold();
            } else if (strcasecmp(key, "bump") == 0 || strcasecmp(key, "pitch") == 0) {
              ConfigManager::setGyroBumpThreshold(constrain(iv, 10, 200));
              editGyroBumpThreshold = ConfigManager::getGyroBumpThreshold();
            } else if (strcasecmp(key, "lin") == 0 || strcasecmp(key, "accel") == 0) {
              ConfigManager::setAccelThreshold(constrain(iv, 100, 3000));
              editAccelThreshold = ConfigManager::getAccelThreshold();
            } else if (strcasecmp(key, "det") == 0) {
              ConfigManager::setImuMotionDet(constrain(iv, 20, 255));
              editImuMotionDet = ConfigManager::getImuMotionDet();
              if (mpuOk) {
                MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(80));
                if (i2c) {
                  mpu.setMotionDetectionThreshold((uint8_t)ConfigManager::getImuMotionDet());
                }
              }
            } else if (strcasecmp(key, "dur") == 0) {
              // MOT_DUR (мс), не путать с durationSec входа в MOVEMENT.
              g_motDurMs = (uint8_t)constrain(iv, 1, 255);
              if (mpuOk) {
                MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(80));
                if (i2c) {
                  mpu.setMotionDetectionDuration(g_motDurMs);
                }
              }
            } else if (strcasecmp(key, "duration") == 0) {
              ConfigManager::setMovementDurationSec(constrain(iv, 10, 120));
              editMoveDuration = ConfigManager::getMovementDurationSec();
            } else if (strcasecmp(key, "settle") == 0) {
              ConfigManager::setMovementSettleSec(constrain(iv, 10, 120));
              editMoveSettle = ConfigManager::getMovementSettleSec();
            } else if (strcasecmp(key, "period") == 0) {
              g_motStreamPeriodMs = (uint32_t)constrain(iv, 50, 1000);
            } else {
              ok = false;
            }
            if (ok) {
              settingsChanged = true;
              testPrintMot();
              testReplyOk("MOT_SET");
            } else {
              testReplyFail("MOT_SET");
            }
          } else {
            testReplyFail("MOT_SET");
          }
        } else {
          testReplyFail("MOT_UNKNOWN");
        }
      } else if (strncasecmp(cmd, "IMU", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        const char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        if (*arg == '\0' || strcasecmp(arg, "CFG") == 0) {
          testPrintImu();
          testReplyOk(*arg ? "IMU_CFG" : "IMU");
        } else if (strncasecmp(arg, "STREAM", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 3;
          if (sec < 1) sec = 1;
          if (sec > 20) sec = 20;
          g_imuStreamUntilMs = millis() + (uint32_t)sec * 1000UL;
          Serial.printf("[IMU] STREAM start %ds\n", sec);
          testReplyOk("IMU_STREAM");
        } else if (strncasecmp(arg, "STATS", 5) == 0) {
          const char *p = arg + 5;
          while (*p == ' ') p++;
          int sec = (*p) ? atoi(p) : 5;
          imuStartStats((uint32_t)sec);
          // OK придёт из imuTask по завершении; здесь ACK старта
          testReplyOk("IMU_STATS_START");
        } else if (strncasecmp(arg, "PRESET", 6) == 0) {
          const char *p = arg + 6;
          while (*p == ' ') p++;
          if (*p == '\0') {
            testReplyFail("IMU_PRESET_ARG");
          } else {
            imuApplyPreset(p);
            testPrintImu();
            testReplyOk("IMU_PRESET");
          }
        } else if (strncasecmp(arg, "SET", 3) == 0) {
          char key[16] = {0};
          char val[24] = {0};
          if (sscanf(arg + 3, " %15s %23s", key, val) == 2 && imuSetParam(key, val)) {
            testPrintImu();
            testReplyOk("IMU_SET");
          } else {
            testReplyFail("IMU_SET");
          }
        } else if (strcasecmp(arg, "SAVE") == 0) {
          // синхронизируем edit* из live-конфига и пишем в LittleFS
          editImuKalmanMea = ConfigManager::getImuKalmanMea();
          editImuKalmanEst = ConfigManager::getImuKalmanEst();
          editImuKalmanQ = ConfigManager::getImuKalmanQ();
          editImuPollMs = ConfigManager::getImuPollMs();
          editImuFifoAvg = ConfigManager::getImuFifoAvg();
          editImuEmaAlpha = ConfigManager::getImuEmaAlpha();
          editImuEmaSpikeAlpha = ConfigManager::getImuEmaSpikeAlpha();
          editImuEmaSpikeThr = ConfigManager::getImuEmaSpikeThr();
          editImuSlewDps = ConfigManager::getImuSlewDps();
          editRedrawAngle = ConfigManager::getRedrawAngleThr();
          saveMenuSettings();
          testReplyOk("IMU_SAVE");
        } else {
          testReplyFail("IMU_UNKNOWN");
        }
      } else if (strcasecmp(cmd, "WIFI") == 0) {
        testPrintWifi();
        testReplyOk("WIFI");
      } else if (strcasecmp(cmd, "WIFI SCAN") == 0) {
        requestWiFiSetup();
        testReplyOk("WIFI_SCAN");
      } else if (strcasecmp(cmd, "BTN") == 0) {
        testPrintBtn();
        testReplyOk("BTN");
      } else if (strcasecmp(cmd, "ERR") == 0) {
        testPrintErr();
        testReplyOk("ERR");
      } else if (strcasecmp(cmd, "CALIB") == 0) {
        testPrintCalib();
        testReplyOk("CALIB");
      } else if (strcasecmp(cmd, "I2C") == 0) {
        testScanI2c();
        testReplyOk("I2C");
      } else if (strcasecmp(cmd, "OTA STATUS") == 0) {
        testPrintOta();
        testReplyOk("OTA_STATUS");
      } else if (strcasecmp(cmd, "OTA LIST") == 0) {
        if (WiFi.status() != WL_CONNECTED) {
          testReplyFail("OTA_LIST_NO_WIFI");
        } else {
          strlcpy(otaListStatus, "загрузка списка…", sizeof(otaListStatus));
          requestGitHubOtaFetchList();
          Serial.println("[TEST] OTA LIST req");
          testReplyOk("OTA_LIST_REQ");
        }
      } else if (strncasecmp(cmd, "MODE", 4) == 0) {
        const char *arg = cmd + 4;
        while (*arg == ' ') arg++;
        if (*arg == '\0') {
          const char *ms = "MANUAL";
          if (currentSystemMode == SystemMode::AUTO) ms = "AUTO";
          else if (currentSystemMode == SystemMode::MOVEMENT) ms = "MOVEMENT";
          Serial.printf("[TEST] MODE %s\n", ms);
          testReplyOk("MODE");
        } else if (strcasecmp(arg, "MANUAL") == 0) {
          {
            MutexGuard guard(xStateMutex);
            if (guard) currentSystemMode = SystemMode::MANUAL;
          }
          currentMode = Mode::MANUAL;
          setAllManualTargetsFromCurrent();
          forceDisplayReset(true);
          setDisplayDirty();
          Serial.println("[TEST] MODE > MANUAL");
          testReplyOk("MODE_MANUAL");
        } else if (strcasecmp(arg, "AUTO") == 0) {
          currentSystemMode = SystemMode::AUTO;
          currentMode = Mode::AUTO;
          lastLevelingCheckTime = millis();
          forceDisplayReset(true);
          setDisplayDirty();
          Serial.println("[TEST] MODE > AUTO");
          testReplyOk("MODE_AUTO");
        } else if (strcasecmp(arg, "MOVEMENT") == 0) {
          if (!mpuOk) {
            testReplyFail("MODE_MOVEMENT_NO_MPU");
          } else {
            previousMode = currentSystemMode;
            currentSystemMode = SystemMode::MOVEMENT;
            movementModeActive = true;
            movementStartTime = millis();
            movementStartMs = movementStartTime;
            forceDisplayReset(true);
            setDisplayDirty();
            Serial.println("[TEST] MODE > MOVEMENT");
            testReplyOk("MODE_MOVEMENT");
          }
        } else {
          testReplyFail("MODE_BAD_ARG");
        }
      } else if (strcasecmp(cmd, "SELF") == 0) {
        if (g_selfTestPhase != 0) {
          testReplyFail("SELF_BUSY");
        } else {
          g_selfTestPhase = 1;
          g_selfTestAtMs = millis();
          g_selfTestFails = 0;
          Serial.println("[TEST] SELF begin");
          testReplyOk("SELF_START");
        }
      } else if (strcasecmp(cmd, "FULL") == 0) {
        if (g_selfTestPhase != 0) {
          testReplyFail("FULL_BUSY");
        } else {
          // Расширенная последовательность: SELF + режимы + датчики
          g_selfTestPhase = 10;
          g_selfTestAtMs = millis();
          g_selfTestFails = 0;
          Serial.println("[TEST] FULL begin");
          testReplyOk("FULL_START");
        }
      } else {
        Serial.printf("[TEST] UNKNOWN '%s'\n", cmd);
        testReplyFail("UNKNOWN");
      }
}

static void processSerialTestCommands() {
  // Serial линии обычно забирает OTA-task → processTestCommandLine.
  // Здесь — запасной путь + фоновый SELF/FULL.
  static char line[96];
  static uint8_t len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      len = 0;
      if (line[0] == 0) continue;
      processTestCommandLine(line);
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;
    }
  }

  // Фоновый smoke: OPEN > wait > CLOSE > bright/dim > STATUS
  // FULL (phase>=10): + PRESS/IMU/WIFI + MODE AUTO/MANUAL/MOVEMENT
  if (g_selfTestPhase != 0 && (int32_t)(millis() - g_selfTestAtMs) >= 0) {
    switch (g_selfTestPhase) {
      case 1:
        requestMenuOpen("SELF");
        g_selfTestPhase = 2;
        g_selfTestAtMs = millis() + 400;
        break;
      case 2:
        if (!menuVisible) {
          testReplyFail("SELF_OPEN");
          g_selfTestFails++;
        } else {
          testReplyOk("SELF_OPEN");
        }
        requestMenuClose("SELF");
        g_selfTestPhase = 3;
        g_selfTestAtMs = millis() + 600;
        break;
      case 3:
        if (menuVisible) {
          testReplyFail("SELF_CLOSE");
          g_selfTestFails++;
        } else {
          testReplyOk("SELF_CLOSE");
        }
        editContrast = 40;
        ConfigManager::setContrast(40);
        applyBacklightPwm(40);
        g_selfTestPhase = 4;
        g_selfTestAtMs = millis() + 200;
        break;
      case 4:
        backlightDimmed = true;
        applyBacklightPwm(CONTRAST_DIM);
        g_selfTestPhase = 5;
        g_selfTestAtMs = millis() + 200;
        break;
      case 5:
        backlightDimmed = false;
        applyBacklightPwm(constrain(ConfigManager::getContrast(), CONTRAST_MIN, CONTRAST_MAX));
        testPrintStatus();
        printTaskInfo();
        if (g_selfTestFails == 0) {
          Serial.println("[TEST] SELF DONE PASS");
          testReplyOk("SELF_PASS");
        } else {
          Serial.printf("[TEST] SELF DONE FAIL fails=%u\n", (unsigned)g_selfTestFails);
          testReplyFail("SELF_FAIL");
        }
        g_selfTestPhase = 0;
        break;

      // ----- FULL sequence -----
      case 10:
        testPrintPress();
        testPrintImu();
        testPrintWifi();
        testPrintCalib();
        testPrintErr();
        testPrintBtn();
        testScanI2c();
        testReplyOk("FULL_SENSORS");
        g_selfTestPhase = 11;
        g_selfTestAtMs = millis() + 100;
        break;
      case 11:
        requestMenuOpen("FULL");
        g_selfTestPhase = 12;
        g_selfTestAtMs = millis() + 500;
        break;
      case 12:
        if (!menuVisible) { testReplyFail("FULL_MENU_OPEN"); g_selfTestFails++; }
        else testReplyOk("FULL_MENU_OPEN");
        requestMenuClose("FULL");
        g_selfTestPhase = 13;
        g_selfTestAtMs = millis() + 700;
        break;
      case 13:
        if (menuVisible) { testReplyFail("FULL_MENU_CLOSE"); g_selfTestFails++; }
        else testReplyOk("FULL_MENU_CLOSE");
        currentSystemMode = SystemMode::AUTO;
        currentMode = Mode::AUTO;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 14;
        g_selfTestAtMs = millis() + 400;
        break;
      case 14:
        if (currentSystemMode != SystemMode::AUTO) { testReplyFail("FULL_AUTO"); g_selfTestFails++; }
        else testReplyOk("FULL_AUTO");
        {
          MutexGuard guard(xStateMutex);
          if (guard) currentSystemMode = SystemMode::MANUAL;
        }
        currentMode = Mode::MANUAL;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 15;
        g_selfTestAtMs = millis() + 400;
        break;
      case 15:
        if (currentSystemMode != SystemMode::MANUAL) { testReplyFail("FULL_MANUAL"); g_selfTestFails++; }
        else testReplyOk("FULL_MANUAL");
        if (mpuOk) {
          previousMode = SystemMode::MANUAL;
          currentSystemMode = SystemMode::MOVEMENT;
          movementModeActive = true;
          movementStartTime = millis();
          movementStartMs = movementStartTime;
          forceDisplayReset(true);
          setDisplayDirty();
          g_selfTestPhase = 16;
          g_selfTestAtMs = millis() + 500;
        } else {
          testReplyOk("FULL_MOVEMENT_SKIP");
          g_selfTestPhase = 17;
          g_selfTestAtMs = millis() + 100;
        }
        break;
      case 16:
        if (currentSystemMode != SystemMode::MOVEMENT) { testReplyFail("FULL_MOVEMENT"); g_selfTestFails++; }
        else testReplyOk("FULL_MOVEMENT");
        movementModeActive = false;
        currentSystemMode = SystemMode::MANUAL;
        currentMode = Mode::MANUAL;
        forceDisplayReset(true);
        setDisplayDirty();
        g_selfTestPhase = 17;
        g_selfTestAtMs = millis() + 400;
        break;
      case 17:
        testPrintStatus();
        testPrintOta();
        testPrintPress();
        if (g_selfTestFails == 0) {
          Serial.println("[TEST] FULL DONE PASS");
          testReplyOk("FULL_PASS");
        } else {
          Serial.printf("[TEST] FULL DONE FAIL fails=%u\n", (unsigned)g_selfTestFails);
          testReplyFail("FULL_FAIL");
        }
        g_selfTestPhase = 0;
        break;
      default:
        g_selfTestPhase = 0;
        break;
    }
  }
}


void testHarnessPoll() {
  processSerialTestCommands();
  if (g_testHeartbeat) {
    static uint32_t lastHb = 0;
    if (millis() - lastHb >= 1000) {
      lastHb = millis();
      Serial.printf("[TEST] HB t=%lu heap=%u maxBlk=%u menu=%d mode=%d\n",
                    (unsigned long)millis(), (unsigned)ESP.getFreeHeap(),
                    (unsigned)ESP.getMaxAllocHeap(),
                    menuVisible ? 1 : 0, (int)currentSystemMode);
    }
  }

  if (g_testDebugDump) {
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg >= 2000) {
      lastDbg = millis();
      testPrintStatus();
      testPrintPress();
      testPrintImu();
    }
  }
}

#define SIMULATE_AUTO_MODE 0
#define SIMULATE_ERRORS 0
#define SIMULATE_MENU_AUTO_ENTER 0

/* ====================  ПЕРЕМЕННЫЕ ДЛЯ СИМУЛЯЦИИ ==================== */
// Эти переменные используются даже когда симуляция выключена
float simAngleX = 0;
float simAngleY = 0;
float simPressures[PAD_COUNT] = { 2.0f, 2.5f, 3.0f, 3.5f };
float simMasterPressure = 4.0f;
int simDirectionX = 1;
int simDirectionY = 1;
uint32_t lastSimUpdate = 0;

// ? ДОБАВЛЕНО: Эти переменные теперь всегда объявлены
// чтобы их можно было использовать в forceDisplayReset()
static uint32_t menuAutoEnterTime = 0;
static bool menuAutoEnterDone = false;
static bool errorSimulated = false;
static ErrorHandler::Error simulatedError = ErrorHandler::Error::NONE;

#if ENABLE_SIMULATION
// Эти переменные нужны только когда симуляция включена
static uint32_t lastAutoSimTime = 0;
static uint32_t autoSimPhase = 0;
static uint32_t lastErrorSimTime = 0;
#endif

#if ENABLE_SIMULATION
void updateSimulationData() {
  uint32_t now = millis();

  static ErrorHandler::Error lastError = ErrorHandler::Error::NONE;
  ErrorHandler::Error currentErr = ErrorHandler::getCurrentActiveError();
  if (currentErr != lastError) {
    Serial.printf("[SIM] ERROR CHANGED! Old=%d, New=%d\n", (int)lastError, (int)currentErr);
    lastError = currentErr;
  }

  static uint32_t lastCall = 0;
  if (now - lastCall > 100) {
    lastCall = now;
    Serial.printf("[SIM] >>> updateSimulationData called, mode=%d, state=%d, autoMode=%d\n",
                  (int)currentSystemMode, (int)currentState, SIMULATE_AUTO_MODE);
  }

#if SIMULATE_AUTO_MODE
  if (currentState == SystemState::RUNNING) {
    currentSystemMode = SystemMode::AUTO;
    currentMode = Mode::AUTO;
  }

  uint32_t phaseDuration = 10000;
  uint32_t currentPhase = (now / phaseDuration) % 4;

  static uint32_t autoSimPhase = 0;
  static float simAngleXTarget = 0;
  static float simAngleYTarget = 0;

  if (currentPhase != autoSimPhase) {
    autoSimPhase = currentPhase;
    switch (autoSimPhase) {
      case 0:
        simAngleXTarget = 0;
        simAngleYTarget = 4.5f;
        break;
      case 1:
        simAngleXTarget = 0;
        simAngleYTarget = -4.5f;
        break;
      case 2:
        simAngleXTarget = 4.5f;
        simAngleYTarget = 0;
        break;
      case 3:
        simAngleXTarget = -4.5f;
        simAngleYTarget = 0;
        break;
    }
  }

  simAngleX += (simAngleXTarget - simAngleX) * 0.1f;
  simAngleY += (simAngleYTarget - simAngleY) * 0.1f;

#else
  simAngleX += simDirectionX * 0.08f;
  if (simAngleX > 3.0f) {
    simAngleX = 3.0f;
    simDirectionX = -1;
  } else if (simAngleX < -3.0f) {
    simAngleX = -3.0f;
    simDirectionX = 1;
  }

  simAngleY += simDirectionY * 0.08f;
  if (simAngleY > 3.5f) {
    simAngleY = 3.5f;
    simDirectionY = -1;
  } else if (simAngleY < -3.5f) {
    simAngleY = -3.5f;
    simDirectionY = 1;
  }
#endif

  simPressures[PAD_FRONT_LEFT] = 2.5f + (simAngleX < 0 ? -simAngleX * 0.5f : 0) + (simAngleY < 0 ? -simAngleY * 0.3f : 0);
  simPressures[PAD_FRONT_RIGHT] = 2.5f + (simAngleX > 0 ? simAngleX * 0.5f : 0) + (simAngleY < 0 ? -simAngleY * 0.3f : 0);
  simPressures[PAD_REAR_LEFT] = 3.0f + (simAngleX < 0 ? -simAngleX * 0.5f : 0) + (simAngleY > 0 ? simAngleY * 0.3f : 0);
  simPressures[PAD_REAR_RIGHT] = 3.0f + (simAngleX > 0 ? simAngleX * 0.5f : 0) + (simAngleY > 0 ? simAngleY * 0.3f : 0);

  for (int i = 0; i < PAD_COUNT; i++) {
    simPressures[i] = constrain(simPressures[i], 0.5f, 7.0f);
  }

  simMasterPressure = 4.5f + sin(now * 0.001f) * 0.3f;

#if SIMULATE_MENU_AUTO_ENTER
  // Используем переменные только если они объявлены
  if (ErrorHandler::isErrorActive(ErrorHandler::Error::LOW_PRESSURE)) {
    ErrorHandler::removeError(ErrorHandler::Error::LOW_PRESSURE);
    Serial.println("[SIM] Forced reset of LOW_PRESSURE error");
  }
#endif

#if SIMULATE_ERRORS
  static uint32_t lastErrorDebug = 0;
  static uint32_t lastErrorGenTime = 0;
  static uint32_t errorActiveTime = 0;
  static bool firstErrorScheduled = false;

  if (now - lastErrorDebug > 5000) {
    lastErrorDebug = now;
    Serial.printf("[SIM_ERROR] errorSimulated=%d, lastErrorGenTime=%d, currentState=%d, hasError=%d\n",
                  errorSimulated, lastErrorGenTime, (int)currentState,
                  ErrorHandler::hasActiveErrors());
  }

  if (currentState == SystemState::RUNNING && !firstErrorScheduled) {
    lastErrorGenTime = now;
    firstErrorScheduled = true;
    Serial.println("[SIM_ERROR] Ошибки начнутся через 20 секунд");
  }

  if (currentState == SystemState::RUNNING && !errorSimulated && firstErrorScheduled) {
    if (now - lastErrorGenTime > 20000) {
      lastErrorGenTime = now;
      errorSimulated = true;
      errorActiveTime = now;

      int errType = random(0, 3);
      switch (errType) {
        case 0:
          Serial.println("[SIM_ERROR] Симуляция: НИЗКОЕ ДАВЛЕНИЕ!");
          ErrorHandler::handleError(ErrorHandler::Error::LOW_PRESSURE,
                                    "Симуляция низкого давления");
          break;
        case 1:
          Serial.println("[SIM_ERROR] Симуляция: ОШИБКА MPU6050!");
          ErrorHandler::handleError(ErrorHandler::Error::MPU,
                                    "Симуляция ошибки MPU");
          break;
        case 2:
          Serial.println("[SIM_ERROR] Симуляция: WATCHDOG!");
          ErrorHandler::handleError(ErrorHandler::Error::WATCHDOG,
                                    "Симуляция Watchdog");
          break;
      }
    }
  }

  if (errorSimulated && (now - errorActiveTime > 10000)) {
    errorSimulated = false;
    ErrorHandler::forceClearAllErrors();
    displayDirty = true;
    forceDisplayReset(true);
    Serial.println("[SIM_ERROR] Ошибка сброшена, возврат к главному экрану");
    lastErrorGenTime = now;
  }
#endif

  static uint32_t lastDebug = 0;
  if (now - lastDebug > 5000) {
    lastDebug = now;
    Serial.printf("[SIM] X=%.2f, Y=%.2f, Режим=%d, Ошибка=%d\n",
                  simAngleX, simAngleY, (int)currentSystemMode,
                  (int)ErrorHandler::getCurrentActiveError());
  }
}
#endif

void simulationOnDisplayReset() {
#if ENABLE_SIMULATION
  menuAutoEnterTime = 0;
  menuAutoEnterDone = false;
  errorSimulated = false;
  simulatedError = ErrorHandler::Error::NONE;
#endif
}
