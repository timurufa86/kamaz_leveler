#include "jhm1200.h"

namespace Jhm1200 {
namespace {
TwoWire *bus_ = nullptr;
bool ready_ = false;
uint8_t lastStatus_ = 0xFF;

void useSafeClock() {
  if (bus_) bus_->setClock(kBusHz);
}

bool writeCmd(uint8_t cmd) {
  useSafeClock();
  bus_->beginTransmission(kAddr);
  bus_->write(cmd);
  return bus_->endTransmission() == 0;
}

/**
 * Ждём готовности: бит 5 == 0. Короткий таймаут — дальше всё равно читаем кадр
 * (многие модули отдают 0x64 с бит5=1 при уже готовых данных).
 */
bool waitDataReady() {
  useSafeClock();
  delay(kMeasureSettleMs);
  const uint32_t start = millis();
  while (millis() - start < kMeasureTimeoutMs) {
    if (bus_->requestFrom(static_cast<uint8_t>(kAddr), static_cast<uint8_t>(1)) == 1) {
      const uint8_t status = bus_->read();
      lastStatus_ = status;
      if ((status & kStatusBusyBit) == 0) {
        return true;
      }
    }
    delay(kMeasurePollMs);
  }
  return false;
}

bool read6(uint8_t *buf) {
  useSafeClock();
  const size_t n = bus_->requestFrom(static_cast<int>(kAddr), 6);
  if (n != 6) return false;
  for (int i = 0; i < 6; i++) buf[i] = bus_->read();
  return true;
}

bool decode(const uint8_t *buf, float &barOut, float *tempC) {
  lastStatus_ = buf[0];
  const uint32_t pressRaw = (static_cast<uint32_t>(buf[1]) << 16) |
                            (static_cast<uint32_t>(buf[2]) << 8) |
                            static_cast<uint32_t>(buf[3]);
  const uint16_t tempRaw =
      (static_cast<uint16_t>(buf[4]) << 8) | static_cast<uint16_t>(buf[5]);

  const double pressNorm = static_cast<double>(pressRaw) / 16777216.0;
  const double pa = pressNorm * (kCalH - kCalL) + kCalL;
  barOut = static_cast<float>(pa / 100000.0);  // Pa → bar

  if (tempC) {
    const double tNorm = static_cast<double>(tempRaw) / 65536.0;
    *tempC = static_cast<float>((tNorm * 19000.0 - 4000.0) / 100.0);
  }
  return true;
}
}  // namespace

bool begin(TwoWire &wire) {
  bus_ = &wire;
  ready_ = false;
  useSafeClock();
  bus_->beginTransmission(kAddr);
  if (bus_->endTransmission() != 0) {
    Serial.println("[JHM1200] нет ответа на 0x78");
    return false;
  }
  float discard = 0.0f;
  ready_ = true;
  if (!readBar(discard, nullptr)) {
    ready_ = false;
    Serial.println("[JHM1200] не удалось прочитать после прогрева");
    return false;
  }
  Serial.printf("[JHM1200] OK status=0x%02X\n", lastStatus_);
  return true;
}

bool isReady() { return ready_; }

uint8_t lastStatus() { return lastStatus_; }

bool readBar(float &barOut, float *tempC) {
  if (!ready_ || !bus_) return false;

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (!writeCmd(kCmdMeasure)) {
      delay(2);
      continue;
    }

    (void)waitDataReady();  // предпочитаем бит5=0; иначе читаем кадр сразу
    uint8_t buf[6];
    if (!read6(buf)) {
      delay(2);
      continue;
    }
    return decode(buf, barOut, tempC);
  }
  return false;
}
}  // namespace Jhm1200
