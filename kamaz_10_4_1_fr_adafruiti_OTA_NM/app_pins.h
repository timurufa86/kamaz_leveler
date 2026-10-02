#pragma once
#include <Arduino.h>

/* I2C (OLED / JHM1200 / MPU6050) */
constexpr uint8_t PIN_OLED_SDA = 21;
constexpr uint8_t PIN_OLED_SCL = 22;

/* TFT ST7789 (SPI) */
constexpr uint8_t PIN_TFT_MOSI = 23;
constexpr uint8_t PIN_TFT_SCLK = 18;
constexpr uint8_t PIN_TFT_CS = 5;
constexpr uint8_t PIN_TFT_DC = 16;
constexpr uint8_t PIN_TFT_RST = 19;
constexpr uint8_t PIN_TFT_BL = 4;

/* Valves */
constexpr uint8_t PIN_BUB1 = 32;
constexpr uint8_t PIN_BUB2 = 33;
constexpr uint8_t PIN_BUB3 = 25;
constexpr uint8_t PIN_BUB4 = 26;
constexpr uint8_t PIN_INFL = 27;
constexpr uint8_t PIN_DEFL = 14;

/* Buttons */
constexpr uint8_t PIN_BUT1 = 13;
constexpr uint8_t PIN_BUT2 = 12;
constexpr uint8_t PIN_BUT3 = 15;
constexpr uint8_t PIN_BUT4 = 17;
// GPIO34 is input-only and has no internal pull-up. Use an external pull-up.
constexpr uint8_t PIN_BUT5 = 34;
