#pragma once
/**
 * btn_io — единая точка чтения кнопок + Serial-инъекция для автотеста.
 * Плата v3 (ESP32-S3): кнопки НЕ GPIO — КН1..КН5 = PCAL9555A P0_0..P0_4,
 * читаются через pcal9555.h. «Пин» кнопки — только ID (BTN_VPIN_KN1..KN5, hw_map.h).
 */
#include <Arduino.h>

/** Как digitalRead: LOW = нажата (внешняя подтяжка 10 к). */
uint8_t btnDigitalRead(uint8_t pin);

/** true если кнопка логически нажата (уровень LOW). */
inline bool btnIsLow(uint8_t pin) { return btnDigitalRead(pin) == LOW; }

/** kn = 1..5. down=true — зажать, false — отпустить. */
void btnInjectSet(uint8_t kn, bool down);

/** Снять все инъекции (снова только реальные входы PCAL). */
void btnInjectClear();

/** Есть ли активная инъекция хотя бы одной кнопки. */
bool btnInjectActive();

/** Текущая маска инъекции: bit0=КН1 … bit4=КН5 (1=перекрыт). */
uint8_t btnInjectMask();

/** Уровни инъекции: bit=1 → LOW (нажата). */
uint8_t btnInjectDownMask();

/**
 * ButtonTask: биты pad 0..3 — на следующем цикле prevDown=false (гарантированный фронт).
 * ButtonTask сам сбрасывает биты после применения.
 */
uint8_t btnConsumeForceEdgeMask();

/** ButtonTask: сбросить lastState/prevDown (после CLEAR). */
uint8_t btnConsumeForceReleaseMask();

/** DRY: ручной цикл не открывает клапаны, только фиксирует «старт подушки». */
void btnSetDryRun(bool on);
bool btnDryRun();

/** Результат последнего намерения старта подушки (для TEST BTN SELF). */
void btnTestNotePadEdge(uint8_t padIdx);
void btnTestNotePadStart(uint8_t padIdx, bool ok);
uint8_t btnTestLastEdgePad();
uint8_t btnTestLastStartPad();
bool btnTestLastStartOk();
void btnTestResetStats();
