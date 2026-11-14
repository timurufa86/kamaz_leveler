#pragma once
/**
 * pcal9555 — драйвер расширителя PCAL9555APW (U6, I2C 0x20) платы v3.
 *
 * Зачем: на ESP32-S3 клапаны/кнопки/ST-статусы НЕ имеют собственных GPIO —
 * всё идёт через PCAL9555A (карта — hw_map.h):
 *   P0_0..P0_4 = /BUT1../BUT5   (входы, активный низкий)
 *   P0_5/P0_6  = /ST_U4_12, /ST_U4_34 (входы, open-drain, активный низкий)
 *   P1_0..P1_5 = /VLV_IN1../VLV_IN6 (выходы → U5 74HCT541 → BTS724G)
 *   P1_6/P1_7  = /ST_U3_12, /ST_U3_34 (входы)
 *
 * БЕЗОПАСНОСТЬ (fail-safe):
 *  - /VLV_EN (IO14) = HIGH принудительно закрывает ВСЕ клапаны независимо от
 *    состояния PCAL (буфер U5 уходит в Hi-Z). Это аппаратный «стоп».
 *  - Любая ошибка записи в PCAL → pcalEmergencyShutoff(): буфер выключен,
 *    повторное включение — только явным pcalEnableValveBuffer(true).
 *  - Ошибка чтения входов → кнопки считаются отпущенными, ST-флаг «нет fault».
 *  - Порядок инициализации: 1) /VLV_EN = HIGH, 2) конфиг PCAL, 3) выходы = 0,
 *    4) проверка чтением, 5) включение буфера.
 *
 * Шина I2C (Wire.begin) поднимается в setup() ДО pcalBegin().
 */
#include <Arduino.h>
#include "app_types.h"
#include "hw_map.h"

/* ── Инициализация / состояние ─────────────────────────────────────────── */

/** Полная инициализация: /VLV_EN→HIGH, конфиг портов, выходы 0, включает буфер. */
bool pcalBegin();

/** true, если последняя операция с PCAL прошла успешно. */
bool pcalReady();

/** Диагностика: количество удачных/неудачных транзакций. */
void pcalStats(uint32_t *okOpt, uint32_t *errOpt);

/* ── Входы ─────────────────────────────────────────────────────────────── */

/** Опросить оба порта, обновить кэш (вызывать из ButtonTask 1 раз в цикл). */
bool pcalPoll();

/** Прямое чтение входов без кэша (для тестов и диагностики). */
bool pcalReadInputsNow(uint16_t *outOpt);

/** Кэш входов: bit0..7 = P0, bit8..15 = P1 (1 = высокий уровень). */
uint16_t pcalInputs();
uint8_t pcalInputPort0();
uint8_t pcalInputPort1();

/** Уровень кнопки 0..4 из кэша: LOW = нажата. */
uint8_t pcalButtonLevel(uint8_t idx);

/** Уровень виртуального пина кнопки (BTN_VPIN_KN1..KN5): LOW = нажата. */
uint8_t pcalButtonLevelVpin(uint8_t vpin);

/** Кэш ST-линий (PCAL_ST_*): 1 = линия отпущена, 0 = неисправность пары каналов. */
uint16_t pcalStFaults();

/** /INT_MCU == LOW (PCAL просит внимания). */
bool pcalIntAsserted();

/* ── Выходы (клапаны P1_0..P1_5) ───────────────────────────────────────── */

/** Записать оба порта целиком (теневая копия обновляется по успеху). */
bool pcalWriteOutputs(uint16_t word);

/** Теневая копия выходов. */
uint16_t pcalOutputs();

/** Изменить группу бит (read-modify-write по тени). */
bool pcalSetBits(uint16_t mask, bool on);

/** Клапан подушки: PAD_* → P1_0..P1_3. */
bool pcalValvePad(Pad pad, bool on);

/** Клапаны по маске (PCAL_VLV_*). */
bool pcalValveSet(uint16_t mask, bool on);

/** Общий впуск / выпуск. */
bool pcalInfl(bool on);
bool pcalDefl(bool on);

/**
 * Открыть одну линию потока и закрыть вторую за одну транзакцию:
 * inflate=true → INFL открыт, DEFL закрыт; inflate=false → наоборот.
 */
bool pcalFlowSelect(bool inflate);

/** Закрыть обе линии потока. */
bool pcalFlowOff();

/** Закрыть все 6 клапанов (нормальный путь, буфер остаётся активным). */
bool pcalValvesAllOff();

/* ── Буфер U5 (/VLV_EN) ────────────────────────────────────────────────── */

/**
 * on=false → /VLV_EN = HIGH: аппаратный стоп, клапаны закрыты при любом PCAL.
 * on=true  → выходы в 0, затем /VLV_EN = LOW (снимает и аварийную защёлку инициализации).
 */
void pcalEnableValveBuffer(bool on);
bool pcalValveBufferEnabled();

/* ── Аварийный останов ─────────────────────────────────────────────────── */

/**
 * Немедленный останов БЕЗ зависимостей от I2C: /VLV_EN = HIGH первым,
 * затем best-effort обнуление выходов. Буфер остаётся выключенным до
 * pcalEnableValveBuffer(true).
 */
void pcalEmergencyShutoff();
