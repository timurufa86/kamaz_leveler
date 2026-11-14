#pragma once
#include <Arduino.h>

/**
 * app_pins.h — пины ESP32-S3 (плата v3), которые действительно есть как GPIO.
 * Клапаны, кнопки и ST-статусы GPIO НЕ имеют: только PCAL9555APW → hw_map.h / pcal9555.h.
 */

/* I2C (PCAL9555A U6 / JHM1200 0x78 / MPU6050 0x68): IO21-SDA, IO47-SCL (S3 не имеет GPIO22) */
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 47;

/* Исторические имена (imu_dmp.cpp) — тот же физический I2C. */
constexpr uint8_t PIN_OLED_SDA = PIN_I2C_SDA;
constexpr uint8_t PIN_OLED_SCL = PIN_I2C_SCL;

/* TFT ST7789 (SPI, J5) */
constexpr uint8_t PIN_TFT_MOSI = 11;
constexpr uint8_t PIN_TFT_SCLK = 18;
constexpr uint8_t PIN_TFT_CS   = 5;
constexpr uint8_t PIN_TFT_DC   = 16;
constexpr uint8_t PIN_TFT_RST  = 17;
constexpr uint8_t PIN_TFT_BL   = 4;
