#pragma once
/**
 *  mode_ctrl.h — единая точка переключения АВТО/РУЧНОЙ.
 *
 *  Раньше логика жила в static-функции ButtonTask; теперь её используют и кнопки,
 *  и HTTP-API (Home Assistant), чтобы оба пути выполняли одинаковые действия
 *  (сброс таймеров авторежима, цели ручного режима, перерисовка экрана).
 */
#include "app_types.h"

/**
 * Переключает режим AUTO <-> MANUAL.
 * MOVEMENT не поддерживается (нужен отдельный вход) — возвращает false.
 * @param via короткая метка источника для лога ("buttons", "ha", ...)
 */
bool switchSystemMode(SystemMode target, const char *via);
