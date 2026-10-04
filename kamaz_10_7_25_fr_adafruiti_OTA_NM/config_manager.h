#pragma once
/**
 *  config_manager.h — LittleFS config, format version, runtime apply.
 *  Methods stay inline (cut from the sketch). Free functions in the .cpp.
 */
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "mutex_guard.h"
#include "app_globals.h"
#include "event_bus.h"
#include "ui_menu_build.h"

constexpr uint32_t CONFIG_FORMAT_VERSION = 10;

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
    int pressureStabilizeMs = 500;   // выравнивание давления после клапана, мс
    int pressureIdleMin = 2;         // пауза между полными опросами подушек в простое, мин (2…30)

    // ===== 8.8.0: авторежим =====
    float coarseZoneRatio = 0.4f;    // грубая зона = доля порога
    float fineZoneRatio = 0.15f;     // точная зона = доля порога
    float worseningRatio = 1.2f;     // порог «стало хуже» (множитель)

    // ===== 8.8.0: движение =====
    bool movementEnabled = true;     // разрешить переход в режим MOVEMENT
    int movementDurationSec = 30;    // длительность ожидания движения, с
    int movementSettleSec = 10;      // успокоение; duration > settle → нужен повторный шум
    int movementCheckSec = 120;      // период проверки давления после движения, с
    float movementTolerance = 0.2f;  // допуск давления после движения, бар
    int gyroThreshold = 25;          // чувств. вибрации 1..25 (меню×8 → порог)
    int gyroBumpThreshold = 25;      // чувств. неровностей 1..25 (меню×8 → порог)
    int accelThreshold = 500;        // порог |linAcc − EMA| (не абсолютный RMS)

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
  };

  static Config currentConfig;

public:
  static bool load() {
    Serial.println("[CFG] Загрузка конфигурации");

    if (!LittleFS.totalBytes()) {
      Serial.println("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      Serial.println("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    File file = LittleFS.open("/config.txt", "r");
    if (!file) {
      Serial.println("[CFG] Нет /config.txt — пробую /config.bak");
      file = LittleFS.open("/config.bak", "r");
      if (!file) {
        Serial.println("[CFG] Нет конфигурации — значения по умолчанию");
        currentConfig = Config{};
        currentConfig.formatVersion = CONFIG_FORMAT_VERSION;
        dumpToSerial();
        return true;
      }
      Serial.println("[CFG] Загрузка из /config.bak");
    }

    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      Serial.println("[CFG] Ошибка десериализации JSON — пробую /config.bak");
      File bak = LittleFS.open("/config.bak", "r");
      if (!bak) {
        Serial.println("[CFG] Нет /config.bak");
        return false;
      }
      error = deserializeJson(doc, bak);
      bak.close();
      if (error) {
        Serial.println("[CFG] /config.bak тоже повреждён");
        return false;
      }
      Serial.println("[CFG] Восстановлено из /config.bak");
    }

    uint32_t formatVersion = doc["formatVersion"] | 1;
    if (formatVersion > CONFIG_FORMAT_VERSION) {
      Serial.printf("[CFG] Неподдерживаемая версия конфигурации: %u\n", formatVersion);
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

    // ===== 8.8.0: движение =====
    currentConfig.movementEnabled = (bool)(doc["movementEnabled"] | true);
    currentConfig.movementDurationSec = constrain((int)(doc["movementDurationSec"] | 30), 10, 120);
    currentConfig.movementSettleSec = constrain((int)(doc["movementSettleSec"] | 10), 10, 120);
    currentConfig.movementCheckSec = constrain((int)(doc["movementCheckSec"] | 120), 30, 300);
    val = doc["movementTolerance"] | 0.2f;
    currentConfig.movementTolerance = constrain(val, 0.1f, 1.0f);
    currentConfig.gyroThreshold = constrain((int)(doc["gyroThreshold"] | 25), 1, 25);
    currentConfig.gyroBumpThreshold = constrain((int)(doc["gyroBumpThreshold"] | 25), 1, 25);
    currentConfig.accelThreshold = constrain((int)(doc["accelThreshold"] | 500), 100, 3000);

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
      Serial.println("[CFG] Офсеты гиро были 0 — применены заводские (220,76,-85)");
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
      Serial.println("[CFG] IMU плавность: миграция > баланс 5.5/3.5/0.008 poll=20");
    }

    Serial.printf("[CFG] Конфигурация загружена успешно (формат файла: %u)\n", formatVersion);
    // Cross-field: min < max, цели движения в пределах.
    if (currentConfig.pressureMin >= currentConfig.pressureMax) {
      Serial.println("[CFG] pressureMin >= pressureMax — исправляю");
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
    dumpToSerial();
    return true;
  }

  static bool save() {
    Serial.println("[CFG] Сохранение конфигурации");

    if (!LittleFS.totalBytes()) {
      Serial.println("[CFG] LittleFS не смонтирована");
      return false;
    }

    MutexGuard guard(xConfigMutex, pdMS_TO_TICKS(500));
    if (!guard) {
      Serial.println("[CFG] Не удалось получить доступ к мьютексу конфигурации!");
      return false;
    }

    StaticJsonDocument<2048> doc;  // 8.8.0: увеличено под новые параметры
    doc["pressureMin"] = currentConfig.pressureMin;
    doc["formatVersion"] = CONFIG_FORMAT_VERSION;
    doc["pressureMax"] = currentConfig.pressureMax;
    doc["masterLowBar"] = currentConfig.masterLowBar;
    doc["tiltThresholdX"] = currentConfig.tiltThresholdX;
    doc["tiltThresholdY"] = currentConfig.tiltThresholdY;
    doc["contrast"] = currentConfig.contrast;
    doc["nivCount"] = currentConfig.nivCount;
    doc["timeInterval"] = currentConfig.timeInterval;
    doc["inflateDelay"] = currentConfig.inflateDelay;
    doc["releaseDelay"] = currentConfig.releaseDelay;
    doc["movementPressureFront"] = currentConfig.movementPressureFront;
    doc["movementPressureRear"] = currentConfig.movementPressureRear;
    doc["parkingPressureBar"] = currentConfig.parkingPressureBar;

    // ===== 8.8.0 =====
    doc["masterCheckSec"] = currentConfig.masterCheckSec;
    doc["manualMaxTimeSec"] = currentConfig.manualMaxTimeSec;
    doc["pressureDeadband"] = currentConfig.pressureDeadband;
    doc["pressureStabilizeMs"] = currentConfig.pressureStabilizeMs;
    doc["pressureIdleMin"] = currentConfig.pressureIdleMin;
    doc["coarseZoneRatio"] = currentConfig.coarseZoneRatio;
    doc["fineZoneRatio"] = currentConfig.fineZoneRatio;
    doc["worseningRatio"] = currentConfig.worseningRatio;
    doc["movementEnabled"] = currentConfig.movementEnabled;
    doc["movementDurationSec"] = currentConfig.movementDurationSec;
    doc["movementSettleSec"] = currentConfig.movementSettleSec;
    doc["movementCheckSec"] = currentConfig.movementCheckSec;
    doc["movementTolerance"] = currentConfig.movementTolerance;
    doc["gyroThreshold"] = currentConfig.gyroThreshold;
    doc["gyroBumpThreshold"] = currentConfig.gyroBumpThreshold;
    doc["accelThreshold"] = currentConfig.accelThreshold;
    doc["backlightOffMin"] = currentConfig.backlightOffMin;
    doc["frameMs"] = currentConfig.frameMs;
    doc["redrawAngleThr"] = currentConfig.redrawAngleThr;
    doc["redrawPressureThr"] = currentConfig.redrawPressureThr;
    doc["imuMotionDet"] = currentConfig.imuMotionDet;
    doc["imuDlpfMode"] = currentConfig.imuDlpfMode;
    doc["imuAccelFs"] = currentConfig.imuAccelFs;
    doc["imuGyroOffX"] = currentConfig.imuGyroOffX;
    doc["imuGyroOffY"] = currentConfig.imuGyroOffY;
    doc["imuGyroOffZ"] = currentConfig.imuGyroOffZ;
    doc["imuAccelOffX"] = currentConfig.imuAccelOffX;
    doc["imuAccelOffY"] = currentConfig.imuAccelOffY;
    doc["imuAccelOffZ"] = currentConfig.imuAccelOffZ;
    doc["zeroAngleX"] = currentConfig.zeroAngleX;
    doc["zeroAngleY"] = currentConfig.zeroAngleY;
    doc["imuKalmanMea"] = currentConfig.imuKalmanMea;
    doc["imuKalmanEst"] = currentConfig.imuKalmanEst;
    doc["imuKalmanQ"] = currentConfig.imuKalmanQ;
    doc["imuPollMs"] = currentConfig.imuPollMs;
    doc["imuFifoAvg"] = currentConfig.imuFifoAvg;
    doc["imuEmaAlpha"] = currentConfig.imuEmaAlpha;
    doc["imuEmaSpikeAlpha"] = currentConfig.imuEmaSpikeAlpha;
    doc["imuEmaSpikeThr"] = currentConfig.imuEmaSpikeThr;
    doc["imuSlewDps"] = currentConfig.imuSlewDps;

    File file = LittleFS.open("/config.tmp", "w");
    if (!file) {
      // Повтор: иногда FD заняты коротким листингом/OTA-хвостом
      LittleFS.remove("/config.tmp");
      delay(20);
      file = LittleFS.open("/config.tmp", "w");
    }
    if (!file) {
      Serial.println("[CFG] Не удалось создать временный файл (нет FD LittleFS)");
      return false;
    }

    size_t bytesWritten = serializeJson(doc, file);
    file.close();

    if (bytesWritten == 0) {
      Serial.println("[CFG] Ошибка записи JSON");
      LittleFS.remove("/config.tmp");
      return false;
    }

    // 8.8.0: сохраняем предыдущую версию как резервную копию
    if (LittleFS.exists("/config.txt")) {
      LittleFS.remove("/config.bak");
      LittleFS.rename("/config.txt", "/config.bak");
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
      Serial.printf("[CFG] Конфигурация сохранена (%u Б, проверка чтением: %s)\n",
                    (unsigned)bytesWritten, verified ? "OK" : "ОШИБКА");
      if (!verified) {
        Serial.println("[CFG] ВНИМАНИЕ: файл конфигурации не прошёл проверку — оставлена копия /config.bak");
      }
      dumpToSerial();

      Event event;
      event.type = EventType::CONFIG_CHANGED;
      event.timestamp = millis();
      EventBus::publish(event);
      return true;
    } else {
      Serial.println("[CFG] Ошибка переименования файла");
      LittleFS.remove("/config.tmp");
      return false;
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

  /** 8.8.0: печать всех параметров конфигурации в Serial (проверка после загрузки и сохранения). */
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
int   cfg_getNivCount();
int cfg_getBacklightOffMin();
int cfg_getContrast();
int cfg_getFrameMs();
float cfg_getRedrawAngleThr();
int cfg_getTimeInterval();
