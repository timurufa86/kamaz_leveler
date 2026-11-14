#pragma once
/**
 * hw_map.h — карта железа платы v2 (Kamaz-Leveler).
 *
 * Плата v2: ESP32 DevKitC + PCAL9555APW (I2C 0x20) + U5 74HCT541 (буфер, OE = VLV_EN)
 *           + 2 x BTS724G (high-side клапаны) + J7 (датчик скорости) + J8 (аварийное давление).
 *
 * ПРАВИЛА:
 *  1) VLV_EN = HIGH  → буфер U5 в Hi-Z → входы BTS прижаты к GND → ВСЕ клапаны закрыты (fail-safe).
 *     VLV_EN = LOW   → буфер активен, клапаны управляются битами PCAL P00..P05.
 *  2) Клапаны и кнопки НЕ управляются GPIO ESP32 напрямую — только через PCAL9555APW.
 *  3) ST-выводы BTS724G — open-drain, активный низкий: 0 = неисправность пары каналов.
 *  4) Кнопки активны низким уровнем (нажата = 0), на плате внешние подтяжки 10 к.
 */

#include <Arduino.h>
#include "app_types.h"   // Pad, PAD_COUNT

/* ======================= ESP32 GPIO (плата v2) ======================= */

/** U5 OE (буфер клапанов). HIGH = Hi-Z = клапаны закрыты (fail-safe). */
constexpr uint8_t PIN_VLV_EN   = 32;

/** PCAL9555APW INT (open-drain, активный низкий). */
constexpr uint8_t PIN_PCAL_INT = 33;

/** Импульсный вход датчика скорости (input-only, PCNT). */
constexpr uint8_t PIN_SPEED_IN = 35;

/** Дискретный датчик аварийно низкого давления в рессивере (активный низкий). */
constexpr uint8_t PIN_PRES_SW  = 25;

/* ======================= PCAL9555APW ======================= */

constexpr uint8_t PCAL_I2C_ADDR = 0x20;   // A0..A2 = GND

/* Регистры PCAL9555A (базовые, совместимы с PCA9555) */
constexpr uint8_t PCAL_REG_INPUT0     = 0x00;
constexpr uint8_t PCAL_REG_INPUT1     = 0x01;
constexpr uint8_t PCAL_REG_OUTPUT0    = 0x02;
constexpr uint8_t PCAL_REG_OUTPUT1    = 0x03;
constexpr uint8_t PCAL_REG_POLARITY0  = 0x04;
constexpr uint8_t PCAL_REG_POLARITY1  = 0x05;
constexpr uint8_t PCAL_REG_CONFIG0    = 0x06;   // 1 = вход, 0 = выход
constexpr uint8_t PCAL_REG_CONFIG1    = 0x07;

/* Agile I/O (PCAL9555A). На TCA9535PWR этих регистров нет — писать «best effort». */
constexpr uint8_t PCAL_REG_DRIVE0     = 0x40;   // drive strength port 0
constexpr uint8_t PCAL_REG_DRIVE1     = 0x41;
constexpr uint8_t PCAL_REG_LATCH0     = 0x44;
constexpr uint8_t PCAL_REG_LATCH1     = 0x45;
constexpr uint8_t PCAL_REG_INTMASK0   = 0x4A;
constexpr uint8_t PCAL_REG_INTMASK1   = 0x4B;

/* --- Номера бит PCAL (P00..P17) --- */
constexpr uint8_t PCAL_P00 = 0;   // VLV_IN1 -> U5.A0 -> BTS U3.IN1 -> клапан ПЛ (BUB1)
constexpr uint8_t PCAL_P01 = 1;   // VLV_IN2 -> ... ПП (BUB2)
constexpr uint8_t PCAL_P02 = 2;   // VLV_IN3 -> ... ЗЛ (BUB3)
constexpr uint8_t PCAL_P03 = 3;   // VLV_IN4 -> ... ЗП (BUB4)
constexpr uint8_t PCAL_P04 = 4;   // VLV_IN5 -> INFL (общий впуск)
constexpr uint8_t PCAL_P05 = 5;   // VLV_IN6 -> DEFL (общий выпуск)
constexpr uint8_t PCAL_P06 = 6;   // ST_U3_12  (open-drain, 0 = fault)
constexpr uint8_t PCAL_P07 = 7;   // ST_U3_34
constexpr uint8_t PCAL_P10 = 8;   // ST_U4_12
constexpr uint8_t PCAL_P11 = 9;   // ST_U4_34 (неиспользуемые каналы U4)
constexpr uint8_t PCAL_P12 = 10;  // BUT1 / KN1
constexpr uint8_t PCAL_P13 = 11;  // BUT2 / KN2
constexpr uint8_t PCAL_P14 = 12;  // BUT3 / KN3
constexpr uint8_t PCAL_P15 = 13;  // BUT4 / KN4
constexpr uint8_t PCAL_P16 = 14;  // BUT5 / KN5
constexpr uint8_t PCAL_P17 = 15;  // резерв (не использовать)

/* --- Маски клапанов --- */
constexpr uint16_t PCAL_VLV_PAD0  = (uint16_t)(1u << PCAL_P00);
constexpr uint16_t PCAL_VLV_PAD1  = (uint16_t)(1u << PCAL_P01);
constexpr uint16_t PCAL_VLV_PAD2  = (uint16_t)(1u << PCAL_P02);
constexpr uint16_t PCAL_VLV_PAD3  = (uint16_t)(1u << PCAL_P03);
constexpr uint16_t PCAL_VLV_PADS  = (uint16_t)(PCAL_VLV_PAD0 | PCAL_VLV_PAD1 |
                                               PCAL_VLV_PAD2 | PCAL_VLV_PAD3);
constexpr uint16_t PCAL_VLV_INFL  = (uint16_t)(1u << PCAL_P04);
constexpr uint16_t PCAL_VLV_DEFL  = (uint16_t)(1u << PCAL_P05);
constexpr uint16_t PCAL_VLV_ALL   = (uint16_t)(PCAL_VLV_PADS | PCAL_VLV_INFL | PCAL_VLV_DEFL);

/* --- Маски ST-флагов (активный низкий) --- */
constexpr uint16_t PCAL_ST_U3_12  = (uint16_t)(1u << PCAL_P06);
constexpr uint16_t PCAL_ST_U3_34  = (uint16_t)(1u << PCAL_P07);
constexpr uint16_t PCAL_ST_U4_12  = (uint16_t)(1u << PCAL_P10);
constexpr uint16_t PCAL_ST_U4_34  = (uint16_t)(1u << PCAL_P11);
constexpr uint16_t PCAL_ST_ALL    = (uint16_t)(PCAL_ST_U3_12 | PCAL_ST_U3_34 |
                                               PCAL_ST_U4_12 | PCAL_ST_U4_34);

/* --- Маски кнопок (активный низкий) --- */
constexpr uint16_t PCAL_BTN_KN1 = (uint16_t)(1u << PCAL_P12);
constexpr uint16_t PCAL_BTN_KN2 = (uint16_t)(1u << PCAL_P13);
constexpr uint16_t PCAL_BTN_KN3 = (uint16_t)(1u << PCAL_P14);
constexpr uint16_t PCAL_BTN_KN4 = (uint16_t)(1u << PCAL_P15);
constexpr uint16_t PCAL_BTN_KN5 = (uint16_t)(1u << PCAL_P16);
constexpr uint16_t PCAL_BTN_ALL = (uint16_t)(PCAL_BTN_KN1 | PCAL_BTN_KN2 | PCAL_BTN_KN3 |
                                             PCAL_BTN_KN4 | PCAL_BTN_KN5);

/* --- Направления (config-регистр: 1 = вход) --- */
constexpr uint16_t PCAL_OUTPUT_MASK = PCAL_VLV_ALL;                       // P00..P05 — выходы
constexpr uint16_t PCAL_DIR_VALUE   = (uint16_t)(~PCAL_OUTPUT_MASK);      // остальное — входы

/* --- Адресация «подушка → бит» --- */
constexpr uint16_t pcalPadMask(Pad pad) {
  return (uint16_t)(1u << (uint8_t)pad);   // Pad 0..3 -> P00..P03
}

/* --- Виртуальные пины кнопок (ID для EncButton; НЕ GPIO) --- */
constexpr uint8_t BTN_VPIN_KN1 = 13;
constexpr uint8_t BTN_VPIN_KN2 = 12;
constexpr uint8_t BTN_VPIN_KN3 = 15;
constexpr uint8_t BTN_VPIN_KN4 = 17;
constexpr uint8_t BTN_VPIN_KN5 = 34;

/* ======================= ST-диагностика ======================= */

struct StGroup {
  uint16_t    stMask;    // бит ST в PCAL
  uint16_t    chanMask;  // биты клапанов, входящих в группу
  const char *name;      // для лога/UI
};

constexpr StGroup ST_GROUPS[] = {
    { PCAL_ST_U3_12, (uint16_t)(PCAL_VLV_PAD0 | PCAL_VLV_PAD1), "BUB1/BUB2" },
    { PCAL_ST_U3_34, (uint16_t)(PCAL_VLV_PAD2 | PCAL_VLV_PAD3), "BUB3/BUB4" },
    { PCAL_ST_U4_12, (uint16_t)(PCAL_VLV_INFL | PCAL_VLV_DEFL), "INFL/DEFL" },
};
constexpr uint8_t ST_GROUP_COUNT = (uint8_t)(sizeof(ST_GROUPS) / sizeof(ST_GROUPS[0]));

/* ======================= Тайминги/константы ======================= */

constexpr uint32_t PCAL_I2C_TIMEOUT_MS   = 3;      // на одну транзакцию
constexpr uint8_t  PCAL_I2C_RETRIES      = 3;      // попыток на операцию
constexpr uint32_t PCAL_REINIT_PERIOD_MS = 1000;   // backoff при отсутствии/сбое
constexpr uint32_t ST_POLL_MS            = 20;     // опрос ST/кнопок
constexpr uint32_t ST_BLOCK_MS           = 5000;   // пауза между попытками идентификации канала
constexpr uint32_t ST_PROBE_MS           = 30;     // включение канала при идентификации
constexpr uint32_t PRES_SW_DEBOUNCE_MS   = 50;
constexpr uint32_t PRES_SW_MISMATCH_MS   = 2000;   // расхождение с JHM1200
constexpr float    PRES_LOW_BAR_DEFAULT  = 4.9f;   // 5 ± 0,5 кгс/см²
constexpr uint32_t SPEED_WINDOW_MS       = 500;
constexpr uint32_t SPEED_TIMEOUT_MS      = 1000;
constexpr float    SPEED_MIN_VALID_KPH   = 0.5f;
constexpr float    SPEED_MAX_KPH         = 200.0f;
constexpr float    SPEED_EMA_ALPHA       = 0.3f;
constexpr uint32_t ODO_SAVE_PERIOD_MS    = 60000;
