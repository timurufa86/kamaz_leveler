#pragma once
/**
 *  config_manager.h — LittleFS config, format version, runtime apply.
 *  Methods stay inline (cut from the sketch). Free functions in the .cpp.
 */
#include <Arduino.h>
#include "logger.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "mutex_guard.h"
#include "app_globals.h"
#include "event_bus.h"
#include "ui_menu_build.h"

constexpr uint32_t CONFIG_FORMAT_VERSION = 11;

class ConfigManager {
private:
  struct Config {
    uint32_t formatVersion = CONFIG_FORMAT_VERSION;
    // ===== прежние параметры (8.6.x / 8.7.x) =====
    float pressureMin = 1.0f;
    float pressureMax = 7.0f;
    float masterLowBar = 0.5f;       // LOW_PRESSURE если МП < порога; 0.00 = выкл., бар (0…4)
    float tiltThresholdX = 0.5f;
    float tiltThresholdY = 0.5f;
    int contrast = 50;
    int nivCount = 5;
    int timeInterval = 5;
    int inflateDelay = 4;            // импульс накачки, с (AUTO и ручной)
    int releaseDelay = 7;            // импульс сброса, с (точная зона AUTO = половина)
    float movementPressureFront = 3.5f;
    float movementPressureRear = 4.0f;
    float parkingPressureBar = 1.5f; // после ДВИЖЕНИЕ→РУЧ; 0.00 = взять перед/зад движения

    // ===== 8.8.0: магистраль / клапаны =====
    int masterCheckSec = 240;        // период проверки магистрали, с (меню: минуты ×60)
    int manualMaxTimeSec = 10;       // максимальное время ручной операции, с
    float pressureDeadband = 0.2f;   // зона нечувствительности по давлению, бар
    int pressureStabilizeMs = 500;   // пауза до замера P после клапана, мс
    int pressureIdleMin = 2;         // пауза между полными опросами подушек в простое, мин (2…30)

    // ===== 8.8.0: авторежим =====
    float coarseZoneRatio = 0.4f;    // грубая зона = доля порога
    float fineZoneRatio = 0.15f;     // точная зона = доля порога
    float worseningRatio = 1.2f;     // порог «стало хуже» (множитель)
    float stableThresholdDeg = 0.10f; // waitForStability: допуск угла, °
    int stableHoldMs = 1000;         // waitForStability: удержание, мс
    int stableTimeoutSec = 5;        // waitForStability: таймаут, с

    // ===== 8.8.0: движение =====
    bool movementEnabled = true;     // разрешить переход в режим MOVEMENT
    int movementDurationSec = 20;    // вход в MOVEMENT через, с (20..300); retest enter~22s
    int movementSettleSec = 20;      // выход из MOVEMENT через, с (20..300)
    int movementCheckSec = 120;      // проверка P после движения, с (20..600)
    float movementTolerance = 0.2f;  // гистерезис P после движения, бар (0.1..0.5)
    int gyroThreshold = 2;           // 10.11: абс. gRms, меню×8 → thr (2→16; idle_p95~3)
    int gyroBumpThreshold = 2;       // 10.11: абс. max(|gx|,|gy|), меню×8
    int accelThreshold = 100;        // |linAcc − EMA|; 100 ≈ idle<<drive по ldlt

    // ===== 8.8.0: дисплей =====
    int backlightOffMin = 0;         // гашение подсветки, мин (0 = никогда)
    int frameMs = 50;                // интервал кадра дисплея, мс
    float redrawAngleThr = 0.06f;    // порог перерисовки по углу, °
    float redrawPressureThr = 0.03f; // порог перерисовки по давлению, бар

    // ===== 8.8.0: IMU / MPU6050 =====
    int imuMotionDet = 40;           // аппаратный порог MOT (LSB=2mg; меньше = чувствительнее)
    // DLPF_CFG 0..6: 256/188/98/42/20/10/5 Hz.
    // Дефолт 6 = MPU6050_DLPF_BW_5 (5 Hz) — сильнее режет вибрацию двигателя/дороги.
    int imuDlpfMode = 6;
    // AFS_SEL 0..3 → ±2/±4/±8/±16G. Дефолт 2 = MPU6050_ACCEL_FS_8 (меньше clipping от вибрации).
    int imuAccelFs = 2;
    // Заводские офсеты из примеров i2cdevlib / MotionApps (не нули).
    int imuGyroOffX = 220;
    int imuGyroOffY = 76;
    int imuGyroOffZ = -85;
    int imuAccelOffX = 0;
    int imuAccelOffY = 0;
    int imuAccelOffZ = 1788;
    float zeroAngleX = 0.0f;         // программный нуль углов (поперечный), °
    float zeroAngleY = 0.0f;         // программный нуль углов (продольный), °
    // ===== 10.1.28: баланс ослаблен (быстрее реакция, меньше залипание) =====
    float imuKalmanMea = 5.5f;       // mea_e — шум измерения (^ = плавнее)
    float imuKalmanEst = 3.5f;       // est_e — шум оценки
    float imuKalmanQ = 0.008f;       // q — шум процесса (v = плавнее)
    int imuPollMs = 20;              // период чтения DMP, мс (v = быстрее)
    int imuFifoAvg = 2;              // пакетов FIFO усреднять (^ = тише, v = быстрее)
    float imuEmaAlpha = 0.28f;       // EMA после Калмана
    float imuEmaSpikeAlpha = 0.10f;  // EMA при выбросе
    float imuEmaSpikeThr = 1.8f;     // порог выброса, °
    float imuSlewDps = 70.0f;        // макс. °/с (шум режет EMA; catch-up догоняет рывки)

    // ===== 10.16.0: интеграция с Home Assistant (HTTP REST) =====
    bool haEnabled = false;          // тумблер в меню «Дом (HA)»
    char haToken[33] = "";           // опциональный X-Api-Key; пусто = без авторизации
  };

  static Config currentConfig;

public:
  static bool load() {
    SERIAL_DBGLN("[CFG] Загрузка конфигурации");

    if (!LittleFS.totalBytes()) {
      SERIAL_DBGLN("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      SERIAL_DBGLN("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    File file = LittleFS.open("/config.txt", "r");
    if (!file) {
      SERIAL_DBGLN("[CFG] Нет /config.txt — пробую /config.bak");
      file = LittleFS.open("/config.bak", "r");
      if (!file) {
        SERIAL_DBGLN("[CFG] Нет конфигурации — значения по умолчанию");
        currentConfig = Config{};
        currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
        return true;
      }
      SERIAL_DBGLN("[CFG] Загрузка из /config.bak");
    }

    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      SERIAL_DBGLN("[CFG] Ошибка десериализации JSON — пробую /config.bak");
      File bak = LittleFS.open("/config.bak", "r");
      if (!bak) {
        SERIAL_DBGLN("[CFG] Нет /config.bak");
        return false;
      }
      error = deserializeJson(doc, bak);
      bak.close();
      if (error) {
        SERIAL_DBGLN("[CFG] /config.bak тоже повреждён");
        return false;
      }
      SERIAL_DBGLN("[CFG] Восстановлено из /config.bak");
    }

    uint32_t formatVersion = doc["formatVersion"] | 1;
    if (formatVersion > CONFIG_FORMAT_VERSION) {
      SERIAL_DBG("[CFG] Неподдерживаемая версия конфигурации: %u\n", formatVersion);
      return false;
    }

    // 8.8.0: стартуем с дефолтов — поля, которых нет в старом файле (форматы 1 и 2),
    // останутся значениями по умолчанию из структуры Config
    currentConfig = Config{};
    currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
    float val = doc["pressureMin"] | 1.0f;
    currentConfig.pressureMin = constrain(val, 0.1f, 5.0f);

    val = doc["pressureMax"] | 7.0f;
    currentConfig.pressureMax = constrain(val, 1.0f, 8.0f);

    val = doc["masterLowBar"] | 0.5f;
    currentConfig.masterLowBar = constrain(val, 0.0f, 4.0f);

    val = doc["tiltThresholdX"] | 0.5f;
    currentConfig.tiltThresholdX = constrain(val, 0.0f, 5.0f);

    val = doc["tiltThresholdY"] | 0.5f;
    currentConfig.tiltThresholdY = constrain(val, 0.0f, 5.0f);

    currentConfig.contrast = constrain(doc["contrast"] | 50, CONTRAST_MIN, CONTRAST_MAX);
    currentConfig.nivCount = constrain(doc["nivCount"] | 5, 1, 20);
    currentConfig.timeInterval = constrain(doc["timeInterval"] | 5, 1, 60);
    currentConfig.inflateDelay = constrain(doc["inflateDelay"] | 4, 1, 10);
    currentConfig.releaseDelay = constrain(doc["releaseDelay"] | 7, 1, 10);

    val = doc["movementPressureFront"] | 3.5f;
    currentConfig.movementPressureFront = constrain(val, 1.0f, 6.0f);

    val = doc["movementPressureRear"] | 4.0f;
    currentConfig.movementPressureRear = constrain(val, 1.0f, 6.0f);

    val = doc["parkingPressureBar"] | 1.5f;
    currentConfig.parkingPressureBar = constrain(val, 0.0f, 6.0f);

    // ===== 8.8.0: магистраль / клапаны =====
    currentConfig.masterCheckSec = constrain((int)(doc["masterCheckSec"] | 240), 30, 600);
    currentConfig.manualMaxTimeSec = constrain((int)(doc["manualMaxTimeSec"] | 10), 1, 30);
    val = doc["pressureDeadband"] | 0.2f;
    currentConfig.pressureDeadband = constrain(val, 0.05f, 0.5f);
    currentConfig.pressureStabilizeMs = constrain((int)(doc["pressureStabilizeMs"] | 500), 100, 2000);
    // v7: pressureIdleMin (мин). Старое pressureCycleMs было в мс (500…10000) — берём дефолт 2 мин.
    if (doc.containsKey("pressureIdleMin")) {
      currentConfig.pressureIdleMin = constrain((int)(doc["pressureIdleMin"] | 2), 2, 30);
    } else {
      const int legacyCycleMs = (int)(doc["pressureCycleMs"] | 2000);
      currentConfig.pressureIdleMin = (legacyCycleMs > 30) ? 2 : constrain(legacyCycleMs, 2, 30);
    }

    // ===== 8.8.0: авторежим =====
    val = doc["coarseZoneRatio"] | 0.4f;
    currentConfig.coarseZoneRatio = constrain(val, 0.2f, 0.9f);
    val = doc["fineZoneRatio"] | 0.15f;
    currentConfig.fineZoneRatio = constrain(val, 0.05f, 0.3f);
    val = doc["worseningRatio"] | 1.2f;
    currentConfig.worseningRatio = constrain(val, 1.05f, 2.0f);
    val = doc["stableThresholdDeg"] | 0.10f;
    currentConfig.stableThresholdDeg = constrain(val, 0.01f, 0.50f);
    currentConfig.stableHoldMs = constrain((int)(doc["stableHoldMs"] | 1000), 500, 3000);
    currentConfig.stableTimeoutSec = constrain((int)(doc["stableTimeoutSec"] | 5), 2, 15);

    // ===== 8.8.0: движение =====
    currentConfig.movementEnabled = (bool)(doc["movementEnabled"] | true);
    currentConfig.movementDurationSec = constrain((int)(doc["movementDurationSec"] | 20), 20, 300);
    currentConfig.movementSettleSec = constrain((int)(doc["movementSettleSec"] | 20), 20, 300);
    currentConfig.movementCheckSec = constrain((int)(doc["movementCheckSec"] | 120), 20, 600);
    val = doc["movementTolerance"] | 0.2f;
    currentConfig.movementTolerance = constrain(val, 0.1f, 0.5f);
    currentConfig.gyroThreshold = constrain((int)(doc["gyroThreshold"] | 2), 1, 25);
    currentConfig.gyroBumpThreshold = constrain((int)(doc["gyroBumpThreshold"] | 2), 1, 25);
    currentConfig.accelThreshold = constrain((int)(doc["accelThreshold"] | 100), 100, 3000);

    // ===== 8.8.0: дисплей =====
    currentConfig.backlightOffMin = constrain((int)(doc["backlightOffMin"] | 0), 0, 30);
    currentConfig.frameMs = constrain((int)(doc["frameMs"] | 50), 20, 200);
    val = doc["redrawAngleThr"] | 0.05f;
    currentConfig.redrawAngleThr = constrain(val, 0.01f, 0.5f);
    val = doc["redrawPressureThr"] | 0.03f;
    currentConfig.redrawPressureThr = constrain(val, 0.01f, 0.5f);

    // ===== 8.8.0: IMU / MPU6050 =====
    currentConfig.imuMotionDet = constrain((int)(doc["imuMotionDet"] | 40), 20, 255);
    currentConfig.imuDlpfMode = constrain((int)(doc["imuDlpfMode"] | 6), 0, 6);
    currentConfig.imuAccelFs = constrain((int)(doc["imuAccelFs"] | 2), 0, 3);
    currentConfig.imuGyroOffX = constrain((int)(doc["imuGyroOffX"] | 220), -32768, 32767);
    currentConfig.imuGyroOffY = constrain((int)(doc["imuGyroOffY"] | 76), -32768, 32767);
    currentConfig.imuGyroOffZ = constrain((int)(doc["imuGyroOffZ"] | -85), -32768, 32767);
    currentConfig.imuAccelOffX = constrain((int)(doc["imuAccelOffX"] | 0), -32768, 32767);
    currentConfig.imuAccelOffY = constrain((int)(doc["imuAccelOffY"] | 0), -32768, 32767);
    currentConfig.imuAccelOffZ = constrain((int)(doc["imuAccelOffZ"] | 1788), -32768, 32767);
    // Старые конфиги с нулевыми офсетами гиро → заводские i2cdevlib
    if (!doc.containsKey("imuGyroOffX") && !doc.containsKey("imuGyroOffY") &&
        !doc.containsKey("imuGyroOffZ")) {
      currentConfig.imuGyroOffX = 220;
      currentConfig.imuGyroOffY = 76;
      currentConfig.imuGyroOffZ = -85;
    } else if (currentConfig.imuGyroOffX == 0 && currentConfig.imuGyroOffY == 0 &&
               currentConfig.imuGyroOffZ == 0) {
      currentConfig.imuGyroOffX = 220;
      currentConfig.imuGyroOffY = 76;
      currentConfig.imuGyroOffZ = -85;
      if (currentConfig.imuAccelOffZ == 0) currentConfig.imuAccelOffZ = 1788;
      SERIAL_DBGLN("[CFG] Офсеты гиро были 0 — применены заводские (220,76,-85)");
    }
    val = doc["zeroAngleX"] | 0.0f;
    currentConfig.zeroAngleX = constrain(val, -45.0f, 45.0f);
    val = doc["zeroAngleY"] | 0.0f;
    currentConfig.zeroAngleY = constrain(val, -45.0f, 45.0f);
    val = doc["imuKalmanMea"] | 7.0f;
    currentConfig.imuKalmanMea = constrain(val, 0.5f, 25.0f);
    val = doc["imuKalmanEst"] | 4.5f;
    currentConfig.imuKalmanEst = constrain(val, 0.5f, 25.0f);
    val = doc["imuKalmanQ"] | 0.005f;
    currentConfig.imuKalmanQ = constrain(val, 0.001f, 0.100f);
    currentConfig.imuPollMs = constrain((int)(doc["imuPollMs"] | 20), 15, 100);
    currentConfig.imuFifoAvg = constrain((int)(doc["imuFifoAvg"] | 3), 1, 8);
    val = doc["imuEmaAlpha"] | 0.20f;
    currentConfig.imuEmaAlpha = constrain(val, 0.05f, 0.50f);
    val = doc["imuEmaSpikeAlpha"] | 0.07f;
    currentConfig.imuEmaSpikeAlpha = constrain(val, 0.05f, 0.50f);
    val = doc["imuEmaSpikeThr"] | 1.4f;
    currentConfig.imuEmaSpikeThr = constrain(val, 0.5f, 5.0f);
    val = doc["imuSlewDps"] | 70.0f;
    currentConfig.imuSlewDps = constrain(val, 5.0f, 120.0f);
    // ===== 10.16.0: интеграция с Home Assistant =====
    currentConfig.haEnabled = (bool)(doc["haEnabled"] | false);
    {
      const char *haTok = doc["haToken"] | "";
      strlcpy(currentConfig.haToken, haTok, sizeof(currentConfig.haToken));
    }
    // Миграция старых заводских пресетов > ослабленный баланс 10.1.28
    const bool oldFactoryBal =
        (fabsf(currentConfig.imuKalmanMea - 7.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.005f) < 0.0005f &&
         currentConfig.imuFifoAvg == 3 &&
         fabsf(currentConfig.imuEmaAlpha - 0.20f) < 0.01f);
    const bool oldLegacy =
        (fabsf(currentConfig.imuKalmanMea - 3.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.012f) < 0.0005f) ||
        (fabsf(currentConfig.imuKalmanMea - 8.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.004f) < 0.0005f && formatVersion < 5) ||
        (fabsf(currentConfig.imuKalmanMea - 6.0f) < 0.05f &&
         fabsf(currentConfig.imuKalmanQ - 0.006f) < 0.0005f && formatVersion < 5);
    if (oldFactoryBal || oldLegacy) {
      currentConfig.imuKalmanMea = 5.5f;
      currentConfig.imuKalmanEst = 3.5f;
      currentConfig.imuKalmanQ = 0.008f;
      currentConfig.imuPollMs = 20;
      currentConfig.imuFifoAvg = 2;
      currentConfig.imuEmaAlpha = 0.28f;
      currentConfig.imuEmaSpikeAlpha = 0.10f;
      currentConfig.imuEmaSpikeThr = 1.8f;
      currentConfig.imuSlewDps = 70.0f;
      currentConfig.redrawAngleThr = 0.05f;
      SERIAL_DBGLN("[CFG] IMU плавность: миграция > баланс 5.5/3.5/0.008 poll=20");
    }

    SERIAL_DBG("[CFG] Конфигурация загружена успешно (формат файла: %u)\n", formatVersion);
    // Cross-field: min < max, цели движения в пределах.
    if (currentConfig.pressureMin >= currentConfig.pressureMax) {
      SERIAL_DBGLN("[CFG] pressureMin >= pressureMax — исправляю");
      currentConfig.pressureMin = 1.0f;
      currentConfig.pressureMax = 7.0f;
    }
    currentConfig.movementPressureFront =
        constrain(currentConfig.movementPressureFront, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    currentConfig.movementPressureRear =
        constrain(currentConfig.movementPressureRear, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    if (currentConfig.fineZoneRatio >= currentConfig.coarseZoneRatio) {
      currentConfig.fineZoneRatio = currentConfig.coarseZoneRatio * 0.4f;
    }
    return true;
  }

  static bool save() {
    SERIAL_DBGLN("[CFG] Сохранение конфигурации");

    if (!LittleFS.totalBytes()) {
      SERIAL_DBGLN("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      SERIAL_DBGLN("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    StaticJsonDocument<2048> doc;  // 8.8.0: увеличено под новые параметры
    toJson(doc.to<JsonObject>(), true);

    File file = LittleFS.open("/config.tmp", "w");
    if (!file) {
      // Повтор: иногда FD заняты коротким листингом/OTA-хвостом
      LittleFS.remove("/config.tmp");
      delay(20);
      file = LittleFS.open("/config.tmp", "w");
    }
    if (!file) {
      SERIAL_DBGLN("[CFG] Не удалось создать временный файл (нет FD LittleFS)");
      return false;
    }

    size_t bytesWritten = serializeJson(doc, file);
    file.close();

    if (bytesWritten == 0) {
      SERIAL_DBGLN("[CFG] Ошибка записи JSON");
      LittleFS.remove("/config.tmp");
      return false;
    }

    // 8.8.0: сохраняем предыдущую версию как резервную копию
    if (LittleFS.exists("/config.txt")) {
      LittleFS.remove("/config.bak");
      if (!LittleFS.rename("/config.txt", "/config.bak")) {
        // 10.15.6: не игнорируем сбой резервного копирования.
        SERIAL_DBGLN("[CFG] ВНИМАНИЕ: не удалось создать /config.bak");
      }
    }
    if (LittleFS.rename("/config.tmp", "/config.txt")) {
      // 8.8.0: проверяем, что записанный файл читается и содержит нужную версию формата
      bool verified = false;
      File check = LittleFS.open("/config.txt", "r");
      if (check) {
        String content = check.readString();
        check.close();
        verified = (content.length() > 0) &&
                   (content.indexOf(String("\"formatVersion\":") + String(CONFIG_FORMAT_VERSION)) >= 0);
      }
      SERIAL_DBG("[CFG] Конфигурация сохранена (%u Б, проверка чтением: %s)\n",
                    (unsigned)bytesWritten, verified ? "OK" : "ОШИБКА");
      if (!verified) {
        // 10.15.6: раньше возвращали true даже при непройденной проверке.
        SERIAL_DBGLN("[CFG] ОШИБКА: файл конфигурации не прошёл проверку (копия /config.bak)");
        return false;
      }

      Event event;
      event.type = EventType::CONFIG_CHANGED;
      event.timestamp = millis();
      EventBus::publish(event);
      return true;
    } else {
      SERIAL_DBGLN("[CFG] Ошибка переименования файла");
      LittleFS.remove("/config.tmp");
      return false;
    }
  }

  /** 10.16.0: единая сериализация всех полей — используется file save() и HTTP-API.
   *  includeSecrets=false: токен не отдаётся, вместо него haTokenSet. */
  static void toJson(JsonObject dst, bool includeSecrets) {
    dst["formatVersion"] = CONFIG_FORMAT_VERSION;
    dst["pressureMin"] = currentConfig.pressureMin;
    dst["pressureMax"] = currentConfig.pressureMax;
    dst["masterLowBar"] = currentConfig.masterLowBar;
    dst["tiltThresholdX"] = currentConfig.tiltThresholdX;
    dst["tiltThresholdY"] = currentConfig.tiltThresholdY;
    dst["contrast"] = currentConfig.contrast;
    dst["nivCount"] = currentConfig.nivCount;
    dst["timeInterval"] = currentConfig.timeInterval;
    dst["inflateDelay"] = currentConfig.inflateDelay;
    dst["releaseDelay"] = currentConfig.releaseDelay;
    dst["movementPressureFront"] = currentConfig.movementPressureFront;
    dst["movementPressureRear"] = currentConfig.movementPressureRear;
    dst["parkingPressureBar"] = currentConfig.parkingPressureBar;
    dst["masterCheckSec"] = currentConfig.masterCheckSec;
    dst["manualMaxTimeSec"] = currentConfig.manualMaxTimeSec;
    dst["pressureDeadband"] = currentConfig.pressureDeadband;
    dst["pressureStabilizeMs"] = currentConfig.pressureStabilizeMs;
    dst["pressureIdleMin"] = currentConfig.pressureIdleMin;
    dst["coarseZoneRatio"] = currentConfig.coarseZoneRatio;
    dst["fineZoneRatio"] = currentConfig.fineZoneRatio;
    dst["worseningRatio"] = currentConfig.worseningRatio;
    dst["stableThresholdDeg"] = currentConfig.stableThresholdDeg;
    dst["stableHoldMs"] = currentConfig.stableHoldMs;
    dst["stableTimeoutSec"] = currentConfig.stableTimeoutSec;
    dst["movementEnabled"] = currentConfig.movementEnabled;
    dst["movementDurationSec"] = currentConfig.movementDurationSec;
    dst["movementSettleSec"] = currentConfig.movementSettleSec;
    dst["movementCheckSec"] = currentConfig.movementCheckSec;
    dst["movementTolerance"] = currentConfig.movementTolerance;
    dst["gyroThreshold"] = currentConfig.gyroThreshold;
    dst["gyroBumpThreshold"] = currentConfig.gyroBumpThreshold;
    dst["accelThreshold"] = currentConfig.accelThreshold;
    dst["backlightOffMin"] = currentConfig.backlightOffMin;
    dst["frameMs"] = currentConfig.frameMs;
    dst["redrawAngleThr"] = currentConfig.redrawAngleThr;
    dst["redrawPressureThr"] = currentConfig.redrawPressureThr;
    dst["imuMotionDet"] = currentConfig.imuMotionDet;
    dst["imuDlpfMode"] = currentConfig.imuDlpfMode;
    dst["imuAccelFs"] = currentConfig.imuAccelFs;
    dst["imuGyroOffX"] = currentConfig.imuGyroOffX;
    dst["imuGyroOffY"] = currentConfig.imuGyroOffY;
    dst["imuGyroOffZ"] = currentConfig.imuGyroOffZ;
    dst["imuAccelOffX"] = currentConfig.imuAccelOffX;
    dst["imuAccelOffY"] = currentConfig.imuAccelOffY;
    dst["imuAccelOffZ"] = currentConfig.imuAccelOffZ;
    dst["zeroAngleX"] = currentConfig.zeroAngleX;
    dst["zeroAngleY"] = currentConfig.zeroAngleY;
    dst["imuKalmanMea"] = currentConfig.imuKalmanMea;
    dst["imuKalmanEst"] = currentConfig.imuKalmanEst;
    dst["imuKalmanQ"] = currentConfig.imuKalmanQ;
    dst["imuPollMs"] = currentConfig.imuPollMs;
    dst["imuFifoAvg"] = currentConfig.imuFifoAvg;
    dst["imuEmaAlpha"] = currentConfig.imuEmaAlpha;
    dst["imuEmaSpikeAlpha"] = currentConfig.imuEmaSpikeAlpha;
    dst["imuEmaSpikeThr"] = currentConfig.imuEmaSpikeThr;
    dst["imuSlewDps"] = currentConfig.imuSlewDps;
    dst["haEnabled"] = currentConfig.haEnabled;
    if (includeSecrets) {
      dst["haToken"] = currentConfig.haToken;
    } else {
      dst["haTokenSet"] = currentConfig.haToken[0] != '\0';
    }
  }

  /** 10.16.0: поля, разрешённые к записи из HTTP-API (безопасное подмножество). */
  static bool isSafeConfigKey(const char *key) {
    static const char *keys[] = {
        "pressureMin", "pressureMax", "masterLowBar", "pressureDeadband",
        "pressureStabilizeMs", "pressureIdleMin", "inflateDelay", "releaseDelay",
        "manualMaxTimeSec", "masterCheckSec", "tiltThresholdX", "tiltThresholdY",
        "nivCount", "timeInterval", "coarseZoneRatio", "fineZoneRatio",
        "worseningRatio", "stableThresholdDeg", "stableHoldMs", "stableTimeoutSec",
        "movementEnabled", "movementPressureFront", "movementPressureRear",
        "movementDurationSec", "movementSettleSec", "movementCheckSec",
        "movementTolerance", "parkingPressureBar", "gyroThreshold",
        "gyroBumpThreshold", "accelThreshold", "imuMotionDet", "backlightOffMin",
        "frameMs", "redrawAngleThr", "redrawPressureThr", "haEnabled"};
    for (const char *k : keys) {
      if (strcmp(k, key) == 0) return true;
    }
    return false;
  }

  /** 10.16.0: применить поля из JSON с теми же ограничениями, что и load().
   *  safeOnly=true — только whitelist (иначе поле уходит в rejected).
   *  На флеш не сохраняет; caller решает, когда вызвать save(). */
  static void applyJsonObject(JsonObjectConst src, bool safeOnly, String *applied,
                              String *rejected, String *err) {
    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(300));
    if (!guard) {
      if (err) *err = "config busy";
      return;
    }
    if (applied) applied->remove(0);
    if (rejected) rejected->remove(0);
    auto ok = [&](const char *k) {
      if (applied) {
        if (applied->length()) *applied += ",";
        *applied += k;
      }
    };
    auto reject = [&](const char *k) {
      if (rejected) {
        if (rejected->length()) *rejected += ",";
        *rejected += k;
      }
    };
    auto takeF = [&](const char *k, float &dst, float lo, float hi) {
      if (!src.containsKey(k)) return;
      if (safeOnly && !isSafeConfigKey(k)) { reject(k); return; }
      dst = constrain(static_cast<float>(src[k]), lo, hi);
      ok(k);
    };
    auto takeI = [&](const char *k, int &dst, int lo, int hi) {
      if (!src.containsKey(k)) return;
      if (safeOnly && !isSafeConfigKey(k)) { reject(k); return; }
      dst = constrain(static_cast<int>(src[k]), lo, hi);
      ok(k);
    };
    auto takeB = [&](const char *k, bool &dst) {
      if (!src.containsKey(k)) return;
      if (safeOnly && !isSafeConfigKey(k)) { reject(k); return; }
      dst = static_cast<bool>(src[k]);
      ok(k);
    };

    takeF("pressureMin", currentConfig.pressureMin, 0.1f, 5.0f);
    takeF("pressureMax", currentConfig.pressureMax, 1.0f, 8.0f);
    takeF("masterLowBar", currentConfig.masterLowBar, 0.0f, 4.0f);
    takeF("pressureDeadband", currentConfig.pressureDeadband, 0.05f, 0.5f);
    takeI("pressureStabilizeMs", currentConfig.pressureStabilizeMs, 100, 2000);
    takeI("pressureIdleMin", currentConfig.pressureIdleMin, 2, 30);
    takeI("inflateDelay", currentConfig.inflateDelay, 1, 10);
    takeI("releaseDelay", currentConfig.releaseDelay, 1, 10);
    takeI("manualMaxTimeSec", currentConfig.manualMaxTimeSec, 1, 30);
    takeI("masterCheckSec", currentConfig.masterCheckSec, 30, 600);
    takeF("tiltThresholdX", currentConfig.tiltThresholdX, 0.0f, 5.0f);
    takeF("tiltThresholdY", currentConfig.tiltThresholdY, 0.0f, 5.0f);
    takeI("nivCount", currentConfig.nivCount, 1, 20);
    takeI("timeInterval", currentConfig.timeInterval, 1, 60);
    takeF("coarseZoneRatio", currentConfig.coarseZoneRatio, 0.2f, 0.9f);
    takeF("fineZoneRatio", currentConfig.fineZoneRatio, 0.05f, 0.3f);
    takeF("worseningRatio", currentConfig.worseningRatio, 1.05f, 2.0f);
    takeF("stableThresholdDeg", currentConfig.stableThresholdDeg, 0.01f, 0.50f);
    takeI("stableHoldMs", currentConfig.stableHoldMs, 500, 3000);
    takeI("stableTimeoutSec", currentConfig.stableTimeoutSec, 2, 15);
    takeB("movementEnabled", currentConfig.movementEnabled);
    takeF("movementPressureFront", currentConfig.movementPressureFront, 1.0f, 6.0f);
    takeF("movementPressureRear", currentConfig.movementPressureRear, 1.0f, 6.0f);
    takeI("movementDurationSec", currentConfig.movementDurationSec, 20, 300);
    takeI("movementSettleSec", currentConfig.movementSettleSec, 20, 300);
    takeI("movementCheckSec", currentConfig.movementCheckSec, 20, 600);
    takeF("movementTolerance", currentConfig.movementTolerance, 0.1f, 0.5f);
    takeF("parkingPressureBar", currentConfig.parkingPressureBar, 0.0f, 6.0f);
    takeI("gyroThreshold", currentConfig.gyroThreshold, 1, 25);
    takeI("gyroBumpThreshold", currentConfig.gyroBumpThreshold, 1, 25);
    takeI("accelThreshold", currentConfig.accelThreshold, 100, 3000);
    takeI("imuMotionDet", currentConfig.imuMotionDet, 20, 255);
    takeI("backlightOffMin", currentConfig.backlightOffMin, 0, 30);
    takeI("frameMs", currentConfig.frameMs, 20, 200);
    takeF("redrawAngleThr", currentConfig.redrawAngleThr, 0.01f, 0.5f);
    takeF("redrawPressureThr", currentConfig.redrawPressureThr, 0.01f, 0.5f);
    takeB("haEnabled", currentConfig.haEnabled);

    // Cross-field (как в load()): min < max, цели движения в пределах, fine < coarse.
    if (currentConfig.pressureMin >= currentConfig.pressureMax) {
      currentConfig.pressureMin = 1.0f;
      currentConfig.pressureMax = 7.0f;
    }
    currentConfig.movementPressureFront =
        constrain(currentConfig.movementPressureFront, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    currentConfig.movementPressureRear =
        constrain(currentConfig.movementPressureRear, currentConfig.pressureMin,
                  currentConfig.pressureMax);
    if (currentConfig.fineZoneRatio >= currentConfig.coarseZoneRatio) {
      currentConfig.fineZoneRatio = currentConfig.coarseZoneRatio * 0.4f;
    }
  }

  static float getPressureMin() {
    return currentConfig.pressureMin;
  }
  static float getPressureMax() {
    return currentConfig.pressureMax;
  }
  static float getMasterLowBar() {
    return currentConfig.masterLowBar;
  }
  static float getTiltThresholdX() {
    return currentConfig.tiltThresholdX;
  }
  static float getTiltThresholdY() {
    return currentConfig.tiltThresholdY;
  }
  static int getContrast() {
    return currentConfig.contrast;
  }
  static int getNivCount() {
    return currentConfig.nivCount;
  }
  static int getTimeInterval() {
    return currentConfig.timeInterval;
  }
  static int getInflateDelay() {
    return currentConfig.inflateDelay;
  }
  static int getReleaseDelay() {
    return currentConfig.releaseDelay;
  }
  static float getMovementPressureFront() {
    return currentConfig.movementPressureFront;
  }
  static float getMovementPressureRear() {
    return currentConfig.movementPressureRear;
  }
  static float getParkingPressureBar() {
    return currentConfig.parkingPressureBar;
  }

  static float getMovementPressure(int pad) {
    return (pad == PAD_FRONT_LEFT || pad == PAD_FRONT_RIGHT)
             ? currentConfig.movementPressureFront
             : currentConfig.movementPressureRear;
  }

  static void setPressureMin(float v) {
    currentConfig.pressureMin = v;
  }
  static void setPressureMax(float v) {
    currentConfig.pressureMax = v;
  }
  static void setMasterLowBar(float v) {
    if (v < 0.0f) v = 0.0f;
    v = roundf(v * 10.0f) / 10.0f;
    currentConfig.masterLowBar = constrain(v, 0.0f, 4.0f);
  }
  static void setTiltThresholdX(float v) {
    currentConfig.tiltThresholdX = v;
  }
  static void setTiltThresholdY(float v) {
    currentConfig.tiltThresholdY = v;
  }
  static void setContrast(int v) {
    currentConfig.contrast = constrain(v, CONTRAST_MIN, CONTRAST_MAX);
  }
  static void setNivCount(int v) {
    currentConfig.nivCount = v;
  }
  static void setTimeInterval(int v) {
    currentConfig.timeInterval = v;
  }
  static void setInflateDelay(int v) {
    currentConfig.inflateDelay = v;
  }
  static void setReleaseDelay(int v) {
    currentConfig.releaseDelay = v;
  }
  static void setMovementPressureFront(float v) {
    currentConfig.movementPressureFront = v;
  }
  static void setMovementPressureRear(float v) {
    currentConfig.movementPressureRear = v;
  }
  static void setParkingPressureBar(float v) {
    currentConfig.parkingPressureBar = constrain(v, 0.0f, 6.0f);
  }

  /* ===== 8.8.0: аксессоры новых параметров ===== */
#define CFG_INT_ACCESSOR(Name, field)                     \
  static int get##Name() { return currentConfig.field; }  \
  static void set##Name(int v) { currentConfig.field = v; }
#define CFG_FLOAT_ACCESSOR(Name, field)                     \
  static float get##Name() { return currentConfig.field; }  \
  static void set##Name(float v) { currentConfig.field = v; }
#define CFG_BOOL_ACCESSOR(Name, field)                      \
  static bool get##Name() { return currentConfig.field; }   \
  static void set##Name(bool v) { currentConfig.field = v; }

  CFG_INT_ACCESSOR(MasterCheckSec, masterCheckSec)
  CFG_INT_ACCESSOR(ManualMaxTimeSec, manualMaxTimeSec)
  CFG_FLOAT_ACCESSOR(PressureDeadband, pressureDeadband)
  CFG_INT_ACCESSOR(PressureStabilizeMs, pressureStabilizeMs)
  CFG_INT_ACCESSOR(PressureIdleMin, pressureIdleMin)
  CFG_FLOAT_ACCESSOR(CoarseZoneRatio, coarseZoneRatio)
  CFG_FLOAT_ACCESSOR(FineZoneRatio, fineZoneRatio)
  CFG_FLOAT_ACCESSOR(WorseningRatio, worseningRatio)
  CFG_FLOAT_ACCESSOR(StableThresholdDeg, stableThresholdDeg)
  CFG_INT_ACCESSOR(StableHoldMs, stableHoldMs)
  CFG_INT_ACCESSOR(StableTimeoutSec, stableTimeoutSec)
  CFG_BOOL_ACCESSOR(MovementEnabled, movementEnabled)
  CFG_INT_ACCESSOR(MovementDurationSec, movementDurationSec)
  CFG_INT_ACCESSOR(MovementSettleSec, movementSettleSec)
  CFG_INT_ACCESSOR(MovementCheckSec, movementCheckSec)
  CFG_FLOAT_ACCESSOR(MovementTolerance, movementTolerance)
  CFG_INT_ACCESSOR(GyroThreshold, gyroThreshold)
  CFG_INT_ACCESSOR(GyroBumpThreshold, gyroBumpThreshold)
  CFG_INT_ACCESSOR(AccelThreshold, accelThreshold)
  CFG_INT_ACCESSOR(BacklightOffMin, backlightOffMin)
  CFG_INT_ACCESSOR(FrameMs, frameMs)
  CFG_FLOAT_ACCESSOR(RedrawAngleThr, redrawAngleThr)
  CFG_FLOAT_ACCESSOR(RedrawPressureThr, redrawPressureThr)
  CFG_INT_ACCESSOR(ImuMotionDet, imuMotionDet)
  CFG_INT_ACCESSOR(ImuDlpfMode, imuDlpfMode)
  CFG_INT_ACCESSOR(ImuAccelFs, imuAccelFs)
  CFG_INT_ACCESSOR(ImuGyroOffX, imuGyroOffX)
  CFG_INT_ACCESSOR(ImuGyroOffY, imuGyroOffY)
  CFG_INT_ACCESSOR(ImuGyroOffZ, imuGyroOffZ)
  CFG_INT_ACCESSOR(ImuAccelOffX, imuAccelOffX)
  CFG_INT_ACCESSOR(ImuAccelOffY, imuAccelOffY)
  CFG_INT_ACCESSOR(ImuAccelOffZ, imuAccelOffZ)
  CFG_FLOAT_ACCESSOR(ZeroAngleX, zeroAngleX)
  CFG_FLOAT_ACCESSOR(ZeroAngleY, zeroAngleY)
  CFG_FLOAT_ACCESSOR(ImuKalmanMea, imuKalmanMea)
  CFG_FLOAT_ACCESSOR(ImuKalmanEst, imuKalmanEst)
  CFG_FLOAT_ACCESSOR(ImuKalmanQ, imuKalmanQ)
  CFG_INT_ACCESSOR(ImuPollMs, imuPollMs)
  CFG_INT_ACCESSOR(ImuFifoAvg, imuFifoAvg)
  CFG_FLOAT_ACCESSOR(ImuEmaAlpha, imuEmaAlpha)
  CFG_FLOAT_ACCESSOR(ImuEmaSpikeAlpha, imuEmaSpikeAlpha)
  CFG_FLOAT_ACCESSOR(ImuEmaSpikeThr, imuEmaSpikeThr)
  CFG_FLOAT_ACCESSOR(ImuSlewDps, imuSlewDps)
#undef CFG_INT_ACCESSOR
#undef CFG_FLOAT_ACCESSOR

  /** 8.8.0: разовый перенос офсетов IMU (после калибровки) — без отдельных сеттеров. */
  static void setImuOffsets(int gx, int gy, int gz, int ax, int ay, int az) {
    currentConfig.imuGyroOffX = gx;
    currentConfig.imuGyroOffY = gy;
    currentConfig.imuGyroOffZ = gz;
    currentConfig.imuAccelOffX = ax;
    currentConfig.imuAccelOffY = ay;
    currentConfig.imuAccelOffZ = az;
  }

  /** On-demand (TEST CFG) — always prints; not gated by DEBUG. */
  static void dumpToSerial() {
    Serial.printf("[CFG] v%u | Pmin=%.2fбар Pmax=%.2fбар Наклон X=%.2f° Y=%.2f° Ярк=%d%% Попыток=%d Инт=%dмин СБРОС=%dс НАКАЧ=%dс\n",
                  (unsigned)currentConfig.formatVersion, currentConfig.pressureMin, currentConfig.pressureMax,
                  currentConfig.tiltThresholdX, currentConfig.tiltThresholdY, currentConfig.contrast,
                  currentConfig.nivCount, currentConfig.timeInterval,
                  currentConfig.releaseDelay, currentConfig.inflateDelay);
    Serial.printf("[CFG] Движение: en=%d перед=%.2f зад=%.2f стоянка=%.2f длит=%dс усп=%dс пров=%dс доп=%.2f гиро=%d акс=%d\n",
                  currentConfig.movementEnabled ? 1 : 0,
                  currentConfig.movementPressureFront, currentConfig.movementPressureRear,
                  currentConfig.parkingPressureBar,
                  currentConfig.movementDurationSec, currentConfig.movementSettleSec, currentConfig.movementCheckSec,
                  currentConfig.movementTolerance, currentConfig.gyroThreshold, currentConfig.accelThreshold);
    Serial.printf("[CFG] Магистраль=%dс | Клапаны: макс=%dс зона=%.2f выравн=%dмс пауза=%dмин | Авто: грубая=%.2f точная=%.2f хуже=%.2f\n",
                  currentConfig.masterCheckSec, currentConfig.manualMaxTimeSec, currentConfig.pressureDeadband,
                  currentConfig.pressureStabilizeMs, currentConfig.pressureIdleMin,
                  currentConfig.coarseZoneRatio, currentConfig.fineZoneRatio, currentConfig.worseningRatio);
    Serial.printf("[CFG] Дисплей: подсветка=%dмин кадр=%dмс углы=%.2f давл=%.2f | IMU: det=%d dlpf=%d fs=%d гиро=%d,%d,%d акс=%d,%d,%d нуль=%.2f,%.2f\n",
                  currentConfig.backlightOffMin, currentConfig.frameMs, currentConfig.redrawAngleThr,
                  currentConfig.redrawPressureThr, currentConfig.imuMotionDet, currentConfig.imuDlpfMode,
                  currentConfig.imuAccelFs,
                  currentConfig.imuGyroOffX, currentConfig.imuGyroOffY, currentConfig.imuGyroOffZ,
                  currentConfig.imuAccelOffX, currentConfig.imuAccelOffY, currentConfig.imuAccelOffZ,
                  currentConfig.zeroAngleX, currentConfig.zeroAngleY);
    Serial.printf("[CFG] IMU плавность: mea=%.2f est=%.2f q=%.3f poll=%dмс fifo=%d ema=%.2f/%.2f thr=%.1f slew=%.1f\n",
                  currentConfig.imuKalmanMea, currentConfig.imuKalmanEst, currentConfig.imuKalmanQ,
                  currentConfig.imuPollMs, currentConfig.imuFifoAvg,
                  currentConfig.imuEmaAlpha, currentConfig.imuEmaSpikeAlpha,
                  currentConfig.imuEmaSpikeThr, currentConfig.imuSlewDps);
  }
  /** 10.16.0: Home Assistant — включение и токен API. */
  static bool getHaEnabled() { return currentConfig.haEnabled; }
  static void setHaEnabled(bool v) { currentConfig.haEnabled = v; }
  static const char *getHaToken() { return currentConfig.haToken; }
  static void setHaToken(const char *t) {
    strlcpy(currentConfig.haToken, t ? t : "", sizeof(currentConfig.haToken));
  }
};


bool initFileSystem();
bool loadConfig();
bool saveConfig();
bool loadWiFiConfig();
bool saveWiFiConfig();
void saveMenuSettings();
void applyRuntimeSettings();

int   cfg_getImuGyroOffX();
int   cfg_getImuGyroOffY();
int   cfg_getImuGyroOffZ();
int   cfg_getImuAccelOffX();
int   cfg_getImuAccelOffY();
int   cfg_getImuAccelOffZ();
int   cfg_getImuMotionDet();
int   cfg_getImuDlpfMode();
int   cfg_getImuAccelFs();
int   cfg_getGyroThreshold();
int   cfg_getGyroBumpThreshold();
int   cfg_getAccelThreshold();
bool  cfg_getMovementEnabled();
int   cfg_getMovementSettleSec();
int   cfg_getMovementDurationSec();
int   cfg_getImuPollMs();
int   cfg_getImuFifoAvg();
float cfg_getImuKalmanMea();
float cfg_getImuKalmanEst();
float cfg_getImuKalmanQ();
float cfg_getImuEmaAlpha();
float cfg_getImuEmaSpikeAlpha();
float cfg_getImuEmaSpikeThr();
float cfg_getImuSlewDps();
float cfg_getZeroAngleX();
float cfg_getZeroAngleY();
float cfg_getMovementPressureFront();
float cfg_getMovementPressureRear();
float cfg_getMovementTolerance();
void cfg_setMasterLowBar(float v);
void cfg_setBacklightOffMin(int v);
float cfg_getPressureMin();
float cfg_getPressureMax();
float cfg_getPressureDeadband();
float cfg_getMasterLowBar();
float cfg_getParkingPressureBar();
int   cfg_getInflateDelay();
int   cfg_getReleaseDelay();
int   cfg_getMasterCheckSec();
int   cfg_getMovementCheckSec();
int   cfg_getManualMaxTimeSec();
int   cfg_getPressureStabilizeMs();
int   cfg_getPressureIdleMin();
float cfg_getRedrawPressureThr();
float cfg_getTiltThresholdX();
float cfg_getTiltThresholdY();
float cfg_getCoarseZoneRatio();
float cfg_getWorseningRatio();
float cfg_getFineZoneRatio();
float cfg_getStableThresholdDeg();
int   cfg_getStableHoldMs();
int   cfg_getStableTimeoutSec();
int   cfg_getNivCount();
int cfg_getBacklightOffMin();
int cfg_getContrast();
int cfg_getFrameMs();
float cfg_getRedrawAngleThr();
int cfg_getTimeInterval();
/* 10.16.0: Home Assistant */
bool  cfg_getHaEnabled();
void  cfg_setHaEnabled(bool v);
const char *cfg_getHaToken();
void  cfg_setHaToken(const char *t);
