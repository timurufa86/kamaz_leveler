#pragma once
/**
 *  task_diag.h — периодическая аппаратная диагностика платы v3 (3.0.0).
 *
 *  Одна задача (TaskPool, период DIAG_TASK_PERIOD_MS = 50 мс) закрывает три узла:
 *   1) 4 линии /ST BTS724G через PCAL9555A (активный низкий, 0 = неисправность
 *      пары каналов): антидребезг 3×50 мс ≈ 150 мс, автоснятие ошибки после
 *      5 с здоровья линии;
 *   2) датчик скорости SPEED_PULSE (IO8) на аппаратном PCNT (fallback — ISR
 *      по нарастанию): выборка каждые SPEED_WINDOW_MS, EMA α=0.3, мёртвая зона
 *      0.5 Гц, км/ч = Гц × speedCoeff;
 *   3) сухой контакт PRES_SW (IO38, NC): замкнут → LOW = норма,
 *      разомкнут → HIGH = авария критического давления.
 *
 *  Задача ничего не пишет в PCAL (только читает кэш входов pcalPoll()),
 *  поэтому не влияет на клапаны/кнопки.
 */
#include <Arduino.h>
#include "hw_map.h"

/** Период задачи: 50 мс (3 отсчёта ST = 150 мс; скорость — раз в SPEED_WINDOW_MS). */
void diagInit();                    // из setup() после pcalBegin()
void diagTask(void *pvParameters);  // тело задачи для TaskPool

/* ── Линии /ST: 1 = неисправность группы каналов (после антидребезга) ── */
struct DiagStLine {
  uint16_t    mask;   // бит PCAL_ST_*
  const char *name;   // "U3_12" — для HA / лога
};
extern const DiagStLine DIAG_ST_LINES[4];

/** Битовая маска подтверждённых неисправностей (биты PCAL_ST_*). */
uint16_t diagValveFaultMask();
/** Текст «U3_12,U4_12» (пустая строка = все линии здоровы). */
const char *diagValveFaultsText();

/** Человекочитаемая подсказка для экрана ошибки: "U3 1-2, U4 3-4" (пусто — нет). */
const char *diagValveFaultsHint();

/* ── Датчик скорости ── */
extern float speedHz;        // сглаженная частота импульсов, Гц
extern float speedKmh;       // speedHz × speedCoeff (0, если источник выключен)
extern bool  speedValid;     // источник включён И счётчик инициализирован
extern bool  movingBySpeed;  // скорость выше порога (гистерезис 1.0/0.5 км/ч)
bool diagSpeedSensorOk();    // счётчик импульсов поднялся
bool diagSpeedUsingPcnt();   // true = аппаратный PCNT, false = ISR-фолбэк
uint32_t diagSpeedPulsesTotal();

/* ── Сухой контакт давления (PRES_SW, NC) ── */
extern bool presSwOk;        // контакт замкнут (LOW) — давление в норме
extern bool presSwCritical;  // контакт разомкнут (HIGH) — критическое давление
extern bool presSwRawHigh;   // сырой уровень входа (для TEST PRES)
