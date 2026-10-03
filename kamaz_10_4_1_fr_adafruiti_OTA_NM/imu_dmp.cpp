#include "imu_dmp.h"
#include "mutex_guard.h"
#include "app_globals.h"
#include "app_pins.h"
#include "logger.h"
#include "imu_motion.h"   // g_motDurMs

#include <Wire.h>
#include <new>

/* ── ConfigManager bridge functions (defined in the .ino) ── */
extern int   cfg_getImuGyroOffX();
extern int   cfg_getImuGyroOffY();
extern int   cfg_getImuGyroOffZ();
extern int   cfg_getImuAccelOffX();
extern int   cfg_getImuAccelOffY();
extern int   cfg_getImuAccelOffZ();
extern int   cfg_getImuMotionDet();

/* ── ErrorHandler bridge function (defined in the .ino) ── */
extern void cfg_errorRemove(uint8_t err);
static constexpr uint8_t EH_MPU = 2; // ErrorHandler::Error::MPU

/* ───── MPU6050 & Kalman filters ───── */
MPU6050 mpu;
GKalman filterX(7.0f, 4.5f, 0.005f);
GKalman filterY(7.0f, 4.5f, 0.005f);

/* ───── I2C helpers (file-scope) ───── */

/** Восстановление I2C, если ведомый держит SDA (типичный зависон MPU). */
static void i2cBusRecover() {
  Wire.end();
  pinMode(PIN_OLED_SDA, INPUT_PULLUP);
  pinMode(PIN_OLED_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_OLED_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_OLED_SCL, LOW);
    delayMicroseconds(5);
  }
  pinMode(PIN_OLED_SDA, OUTPUT);
  digitalWrite(PIN_OLED_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_OLED_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_OLED_SDA, HIGH);
  delayMicroseconds(5);
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(100000L);
  Wire.setTimeOut(50);
}

static void i2cScanLog(const char* tag) {
  uint8_t found = 0;
  Serial.printf("[I2C] scan (%s):", tag);
  // Включая 0x78 (JHM) — иначе при «пустом» логе кажется, что шина мертва.
  for (uint8_t addr = 0x08; addr <= 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (!found) Serial.print(" (пусто)");
  Serial.println();
}

/* ───── DMP initialization ───── */

void initializeDMP() {
#if ENABLE_MPU6050
  MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(1500));
  if (!i2c && xI2CMutex != nullptr) {
    Serial.println("[MPU] I2C busy, откладываю инициализацию");
    return;
  }

  Logger::log(Logger::INFO, "MPU", "Инициализация");

  auto probe = [](uint8_t addr) -> bool {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  };

  // Мягкий старт: не рвём шину Wire.end(), пока JHM/MPU отвечают.
  // Recover — только если 0x68/0x69 молчат (типичный зависон SDA у MPU).
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(100000L);
  Wire.setTimeOut(50);

  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    if (attempt > 1) {
      i2cBusRecover();
      delay(50 * attempt);
    } else {
      delay(20);
    }

    if (attempt == 1 || attempt == 5) {
      i2cScanLog(attempt == 1 ? "до MPU" : "после fail");
      Serial.printf("[MPU] idle SDA=%d SCL=%d\n",
                    digitalRead(PIN_OLED_SDA), digitalRead(PIN_OLED_SCL));
    }

    const bool a68 = probe(0x68);
    const bool a69 = probe(0x69);
    Serial.printf("[MPU] attempt %u: ACK 0x68=%d 0x69=%d heap=%u\n", attempt, a68 ? 1 : 0,
                  a69 ? 1 : 0, static_cast<unsigned>(ESP.getFreeHeap()));

    if (!a68 && !a69) {
      continue;
    }

    // AD0=HIGH > 0x69; иначе 0x68
    if (a69 && !a68) {
      new (&mpu) MPU6050(0x69);
    } else {
      new (&mpu) MPU6050(0x68);
    }

    mpu.initialize();
    delay(20);
    if (!mpu.testConnection()) {
      Serial.printf("[MPU] testConnection fail (attempt %u)\n", attempt);
      continue;
    }

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setSleepEnabled(false);
    mpu.setInterruptMode(false);

    // MOT/INT до dmpInitialize() сбрасываются прошивкой DMP — только после setDMPEnabled.
    const uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
      Logger::logf(Logger::ERROR, "MPU", "DMP error %d (attempt %u)", devStatus, attempt);
      Serial.printf("[MPU] DMP error code: %d\n", devStatus);
      continue;
    }

    mpu.setDMPEnabled(true);

    // Офсеты после DMP (dmpInitialize перезаписывает регистры).
    mpu.setXGyroOffset(cfg_getImuGyroOffX());
    mpu.setYGyroOffset(cfg_getImuGyroOffY());
    mpu.setZGyroOffset(cfg_getImuGyroOffZ());
    mpu.setXAccelOffset(cfg_getImuAccelOffX());
    mpu.setYAccelOffset(cfg_getImuAccelOffY());
    mpu.setZAccelOffset(cfg_getImuAccelOffZ());

    // Аппаратный MOT + FIFO + DMP_INT (0x52). HPF нужен, иначе MOT с DMP почти мёртв.
    // MOT_DUR=40 мс, counter decrement=1 — типичный гайд для стабильных импульсов.
    mpu.setDHPFMode(MPU6050_DHPF_1P25);
    mpu.setMotionDetectionThreshold((uint8_t)cfg_getImuMotionDet());
    mpu.setMotionDetectionDuration(g_motDurMs);
    mpu.setAccelerometerPowerOnDelay(3);
    mpu.setMotionDetectionCounterDecrement(1);
    mpu.setZeroMotionDetectionThreshold(156);
    mpu.setZeroMotionDetectionDuration(0);
    mpu.setIntEnabled(0x52);  // MOT | FIFO_OFLOW | DMP_INT
    (void)mpu.getIntStatus();  // очистка latched INT

    Serial.printf("[MPU] DMP OK MotionApps 6.12 packet=%u | MOT_THR=%u MOT_DUR=%u INT_EN=0x%02X gyroOff=(%d,%d,%d)\n",
                  (unsigned)mpu.dmpGetFIFOPacketSize(),
                  (unsigned)cfg_getImuMotionDet(),
                  (unsigned)g_motDurMs,
                  (unsigned)mpu.getIntEnabled(),
                  cfg_getImuGyroOffX(), cfg_getImuGyroOffY(),
                  cfg_getImuGyroOffZ());

    mpuOk = true;
    cfg_errorRemove(EH_MPU);
    Wire.setClock(400000L);
    Logger::log(Logger::INFO, "MPU", "DMP готов");
    return;
  }

  Logger::log(Logger::ERROR, "MPU", "Ошибка подключения!");
  mpuOk = false;
  Wire.setClock(100000L);  // оставляем 100 кГц — надёжнее для JHM, пока MPU мёртв
  cfg_errorRemove(EH_MPU);
  displayDirty = true;
  Serial.println("[MPU] нет ACK 0x68/0x69 — проверьте 3.3V/GND/SDA21/SCL22/AD0");
  Serial.println("[MPU] IMU отсутствует — углы/движение отключены, давление работает");
#endif
}
