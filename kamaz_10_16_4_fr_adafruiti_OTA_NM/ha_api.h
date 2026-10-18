#pragma once
/**
 *  ha_api.h — Home Assistant integration over HTTP REST (10.16.0).
 *
 *  Сервер поднимается только при haEnabled=true и подключённом STA Wi-Fi.
 *  Включение/выключение — тумблер в меню «Дом (HA)»; токен задаётся по Serial.
 */
#include <Arduino.h>

/** Задача HTTP-сервера (TaskPool, стек 6 КБ). */
void haTask(void *pvParameters);

/** Остановить сервер перед OTA (освобождает сокеты/кучу), поднять после. */
void haOtaBegin();
void haOtaEnd();

/** Serial-консоль: обрабатывает строки "HA ...". @return true, если строка наша. */
bool haHandleSerialCommand(const char *line);

/** Тумблер меню: применить и сохранить. */
void haSetEnabled(bool enabled);
bool haIsEnabled();

/** Отложенное сохранение конфига (после POST /api/config) и авто-стоп ручного. */
void haTick();

/** Строка статуса для меню/инфо-страницы. */
const char *haStatusText();

/** 10.16.2: сервер реально работает (включено + Wi-Fi + не OTA) — для индикатора. */
bool haIsOnline();

/** Количество подключений, обслуженных сервером (для статуса). */
uint32_t haRequestsServed();
