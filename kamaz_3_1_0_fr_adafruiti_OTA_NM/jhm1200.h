#pragma once
#include <Arduino.h>
#include <Wire.h>

/** JHM1200 / KY-3V3-IIC — цифровой датчик 0..1 MPa (0..10 бар), I2C. */
namespace Jhm1200 {
constexpr uint8_t kAddr = 0x78;
constexpr uint8_t kCmdMeasure = 0xAC;
// Заводская шкала модуля 0–1 MPa (Pa), как в доке продавца / ESPHome.
constexpr double kCalL = -125000.0;   // Pa
constexpr double kCalH = 1125000.0;   // Pa
/** Бит 5 статуса: 1 = busy, 0 = данные готовы (JHM1200 / ESPHome). */
constexpr uint8_t kStatusBusyBit = 0x20;
/** Пауза после 0xAC перед первым опросом статуса. */
constexpr uint32_t kMeasureSettleMs = 5;
/**
 * Короткий опрос готовности (бит 5 → 0).
 * У KY-клонов status часто остаётся 0x64 (бит5=1) даже при валидных данных —
 * долгий wait держит I2C и провоцирует ложный SENSOR рядом с MPU.
 */
constexpr uint32_t kMeasureTimeoutMs = 20;
constexpr uint32_t kMeasurePollMs = 2;
/** JHM нестабилен на 400 кГц рядом с MPU — всегда 100 кГц на транзакцию. */
constexpr uint32_t kBusHz = 100000L;

bool begin(TwoWire &wire = Wire);
bool isReady();
/** Давление в барах (избыточное). false при I2C / полном провале чтения. */
bool readBar(float &barOut, float *tempC = nullptr);
uint8_t lastStatus();
}  // namespace Jhm1200
