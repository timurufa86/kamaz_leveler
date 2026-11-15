#pragma once
/**
 * hw_map.h — карта железа платы v3 (Kamaz-Leveler 3.0.0), ESP32-S3.
 * Источник фактов: hardware/docs/CHECKUP_v3.md (снято с платы esp32s3_ruichi158).
 *
 * ПЛАТА v3: ESP32-S3-DevKitC-1 (U7) + PCAL9555APW (U6, I2C 0x20)
 *           + U5 74HCT541 (буфер, OE = /VLV_EN) + U3/U4 BTS724G + ST7789.
 *
 * ПРАВИЛА:
 *  1) /VLV_EN (IO14) = HIGH → буфер U5 в Hi-Z → входы BTS прижаты к GND →
 *     ВСЕ клапаны закрыты (fail-safe).
 *     /VLV_EN = LOW → буфер активен, клапаны = биты PCAL P1_0..P1_5.
 *  2) Клапаны, кнопки и ST-статусы НЕ имеют GPIO на S3 — только через
 *     PCAL9555APW (см. pcal9555.h).
 *  3) ST-выводы BTS724G — open-drain, активный низкий: 0 = неисправность пары каналов.
 *  4) Кнопки активны низким уровнем (нажата = 0), подтяжки 10 к на плате.
 *  5) GPIO 22/23/24/25/26/27/32/33 на ESP32-S3 НЕ СУЩЕСТВУЮТ — не использовать.
 */

#include <Arduino.h>
#include "app_types.h"   // Pad, PAD_COUNT

/* ======================= ESP32-S3 GPIO (плата v3) ======================= */

/** /VLV_EN — OE буфера клапанов U5. HIGH = Hi-Z = клапаны закрыты (fail-safe). */
constexpr uint8_t PIN_VLV_EN      = 14;

/** /INT_MCU — выход INT PCAL9555APW (open-drain, активный низкий). */
constexpr uint8_t PIN_PCAL_INT    = 13;

/** /SPEED_PULSE — импульсы датчика скорости через 74LVC1G14 (обычный GPIO на S3). */
constexpr uint8_t PIN_SPEED_PULSE = 8;

/** /PRES_SW — дискретный датчик аварийно низкого давления (активный низкий). */
constexpr uint8_t PIN_PRES_SW     = 38;

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

/* --- Номера бит PCAL: 0..7 = P0_x, 8..15 = P1_x (совпадает с номером ножки) --- */
constexpr uint8_t PCAL_P00 = 0;    // /BUT1  (KN1), вход, активный низкий
constexpr uint8_t PCAL_P01 = 1;    // /BUT2  (KN2)
constexpr uint8_t PCAL_P02 = 2;    // /BUT3  (KN3)
constexpr uint8_t PCAL_P03 = 3;    // /BUT4  (KN4)
constexpr uint8_t PCAL_P04 = 4;    // /BUT5  (KN5)
constexpr uint8_t PCAL_P05 = 5;    // /ST_U4_12 — статус U4, каналы 1–2
constexpr uint8_t PCAL_P06 = 6;    // /ST_U4_34 — статус U4, каналы 3–4
constexpr uint8_t PCAL_P07 = 7;    // не задействован (не использовать)
constexpr uint8_t PCAL_P10 = 8;    // /VLV_IN1 → клапан ПЛ  (BUB1)
constexpr uint8_t PCAL_P11 = 9;    // /VLV_IN2 → клапан ПП  (BUB2)
constexpr uint8_t PCAL_P12 = 10;   // /VLV_IN3 → клапан ЗЛ  (BUB3)
constexpr uint8_t PCAL_P13 = 11;   // /VLV_IN4 → клапан ЗП  (BUB4)
constexpr uint8_t PCAL_P14 = 12;   // /VLV_IN5 → INFL (общий впуск)
constexpr uint8_t PCAL_P15 = 13;   // /VLV_IN6 → DEFL (общий выпуск)
constexpr uint8_t PCAL_P16 = 14;   // /ST_U3_12 — статус U3, каналы 1–2
constexpr uint8_t PCAL_P17 = 15;   // /ST_U3_34 — статус U3, каналы 3–4

/* --- Маски клапанов (P1_0..P1_5) --- */
constexpr uint16_t PCAL_VLV_PAD0  = (uint16_t)(1u << PCAL_P10);
constexpr uint16_t PCAL_VLV_PAD1  = (uint16_t)(1u << PCAL_P11);
constexpr uint16_t PCAL_VLV_PAD2  = (uint16_t)(1u << PCAL_P12);
constexpr uint16_t PCAL_VLV_PAD3  = (uint16_t)(1u << PCAL_P13);
constexpr uint16_t PCAL_VLV_PADS  = (uint16_t)(PCAL_VLV_PAD0 | PCAL_VLV_PAD1 |
                                               PCAL_VLV_PAD2 | PCAL_VLV_PAD3);
constexpr uint16_t PCAL_VLV_INFL  = (uint16_t)(1u << PCAL_P14);
constexpr uint16_t PCAL_VLV_DEFL  = (uint16_t)(1u << PCAL_P15);
constexpr uint16_t PCAL_VLV_ALL   = (uint16_t)(PCAL_VLV_PADS | PCAL_VLV_INFL | PCAL_VLV_DEFL);

/* --- Маски ST-флагов (активный низкий) --- */
constexpr uint16_t PCAL_ST_U4_12  = (uint16_t)(1u << PCAL_P05);
constexpr uint16_t PCAL_ST_U4_34  = (uint16_t)(1u << PCAL_P06);
constexpr uint16_t PCAL_ST_U3_12  = (uint16_t)(1u << PCAL_P16);
constexpr uint16_t PCAL_ST_U3_34  = (uint16_t)(1u << PCAL_P17);
constexpr uint16_t PCAL_ST_ALL    = (uint16_t)(PCAL_ST_U4_12 | PCAL_ST_U4_34 |
                                               PCAL_ST_U3_12 | PCAL_ST_U3_34);

/* --- Маски кнопок (P0_0..P0_4, активный низкий) --- */
constexpr uint16_t PCAL_BTN_KN1 = (uint16_t)(1u << PCAL_P00);
constexpr uint16_t PCAL_BTN_KN2 = (uint16_t)(1u << PCAL_P01);
constexpr uint16_t PCAL_BTN_KN3 = (uint16_t)(1u << PCAL_P02);
constexpr uint16_t PCAL_BTN_KN4 = (uint16_t)(1u << PCAL_P03);
constexpr uint16_t PCAL_BTN_KN5 = (uint16_t)(1u << PCAL_P04);
constexpr uint16_t PCAL_BTN_ALL = (uint16_t)(PCAL_BTN_KN1 | PCAL_BTN_KN2 | PCAL_BTN_KN3 |
                                             PCAL_BTN_KN4 | PCAL_BTN_KN5);

/* --- Маски направлений и начальных уровней (config: 1 = вход) --- */
constexpr uint16_t PCAL_OUTPUT_MASK = PCAL_VLV_ALL;                    // P1_0..P1_5 — выходы
constexpr uint16_t PCAL_DIR_VALUE   = (uint16_t)(~PCAL_OUTPUT_MASK);   // остальное — входы
constexpr uint8_t  PCAL_CFG_PORT0   = 0xFF;                            // порт 0 целиком вход
constexpr uint8_t  PCAL_CFG_PORT1   = (uint8_t)(~0x3Fu);               // P1_0..P1_5 = выходы
constexpr uint8_t  PCAL_OUT0_INIT   = 0x00;                            // все клапаны закрыты
constexpr uint8_t  PCAL_OUT1_INIT   = 0x00;
/* INT_MASK: 1 = прерывание по биту запрещено */
constexpr uint8_t  PCAL_INTMASK0    = 0x80;   // P0_7 не задействован
constexpr uint8_t  PCAL_INTMASK1    = 0x3F;   // выходы P1_0..P1_5 не дают INT
/* Agile I/O: драйв выходов (2 бита на вывод; 01 = 0.25x) — best effort */
constexpr uint8_t  PCAL_DRIVE0      = 0x00;
constexpr uint8_t  PCAL_DRIVE1      = 0x55;   // все 8 — 0.25x

/* --- Адресация «подушка → бит»: Pad 0..3 → P1_0..P1_3 --- */
constexpr uint16_t pcalPadMask(Pad pad) {
  return (uint16_t)(1u << (PCAL_P10 + (uint8_t)pad));
}

/* --- Виртуальные пины кнопок (ID для EncButton; НЕ GPIO) --- */
constexpr uint8_t BTN_VPIN_KN1 = 0xE0;
constexpr uint8_t BTN_VPIN_KN2 = 0xE1;
constexpr uint8_t BTN_VPIN_KN3 = 0xE2;
constexpr uint8_t BTN_VPIN_KN4 = 0xE3;
constexpr uint8_t BTN_VPIN_KN5 = 0xE4;

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

/* ======================= 3.0.0: диагностика ======================= */

/** Период task_diag: 50 мс. 3 отсчёта ST = 150 мс, скорость — раз в SPEED_WINDOW_MS. */
constexpr uint32_t DIAG_TASK_PERIOD_MS  = 50;
/** Антидребезг /ST: столько подряд одинаковых отсчётов (3×50 мс ≈ 150 мс). */
constexpr uint8_t  ST_DEBOUNCE_SAMPLES  = 3;
/** Столько непрерывного здоровья линии → ошибка VALVE снимается автоматически. */
constexpr uint32_t ST_CLEAR_HEALTHY_MS  = 5000;
/** Период обновления lastSeenTime ошибки (ErrorHandler снимает её через 3 с тишины). */
constexpr uint32_t DIAG_ERR_REFRESH_MS  = 1000;

/** PCNT: максимум аппаратного glitch-фильтра S3 = 1023 такта APB (80 МГц) ≈ 12.8 мкс. */
constexpr uint32_t SPEED_GLITCH_NS      = 12780;
/** Мёртвая зона датчика скорости, Гц (ниже — считаем 0). */
constexpr float    SPEED_MIN_HZ         = 0.5f;
/** Отсечка помех: частота выше — отсчёт отбрасывается (а не «скорость 500 км/ч»). */
constexpr float    SPEED_MAX_HZ         = 2000.0f;
/** «Автомобиль движется по скорости»: включение и отпускание (гистерезис), км/ч. */
constexpr float    SPEED_MOVING_ON_KPH  = 1.0f;
constexpr float    SPEED_MOVING_OFF_KPH = 0.5f;
