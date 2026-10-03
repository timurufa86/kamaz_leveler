#pragma once
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include "ui_fonts.h"

/*
 * UiText — тонкая типографская обёртка над U8g2_for_Adafruit_GFX.
 *
 * Задачи слоя:
 *   - одна точка настройки шрифта/цвета для всего UI;
 *   - печать UTF-8 (кириллица) через drawUTF8/print;
 *   - правильное позиционирование: в U8g2 точка привязки — ЛЕВАЯ НИЖНЯЯ
 *     (базовая линия), а не левый верхний угол, как в Adafruit GFX;
 *   - выравнивание текста в прямоугольнике (box/boxf) с вертикальным
 *     центрированием по ascent/descent — надписи всегда стоят ровно
 *     относительно своего кегля, без «магических» координат;
 *   - форматированные варианты (drawf/boxf) без динамической памяти.
 */
enum class UiHAlign : uint8_t { Left, Center, Right };
enum class UiVAlign : uint8_t { Top, Middle, Bottom };

class UiText {
 public:
  /** Привязать слой к конкретному Adafruit-совместимому дисплею. */
  void begin(Adafruit_GFX &gfx);

  bool ready() const { return gfx_ != nullptr; }

  /* ------------------------- шрифт и цвет ------------------------- */
  void setFont(UiFont f);
  UiFont font() const { return cur_; }

  void setColors(uint16_t fg, uint16_t bg);
  void setFg(uint16_t fg);
  uint16_t fg() const { return fg_; }
  uint16_t bg() const { return bg_; }

  /** true — прозрачный фон глифов (по умолчанию), false — заливка bg_. */
  void setTransparent(bool on);

  /* --------------------------- метрики ---------------------------- */
  int16_t ascent();
  int16_t descent();               // отрицательный (как в U8g2)
  int16_t lineHeight();            // ascent - descent
  int16_t width(const char *s);
  int16_t widthf(const char *fmt, ...);

  /* --------------------------- заливки ---------------------------- */
  void clear(int16_t x, int16_t y, int16_t w, int16_t h);
  void clear(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  /* ------------------ печать по базовой линии ---------------------- */
  void draw(int16_t x, int16_t baselineY, const char *s);
  void drawf(int16_t x, int16_t baselineY, const char *fmt, ...);
  void drawCenter(int16_t cx, int16_t baselineY, const char *s);
  void drawCenterf(int16_t cx, int16_t baselineY, const char *fmt, ...);
  void drawRight(int16_t rightX, int16_t baselineY, const char *s);
  void drawRightf(int16_t rightX, int16_t baselineY, const char *fmt, ...);

  /* ------------------ печать в прямоугольнике --------------------- */
  void box(int16_t x, int16_t y, int16_t w, int16_t h, const char *s,
           UiHAlign ha = UiHAlign::Left, UiVAlign va = UiVAlign::Middle,
           bool erase = true);
  void boxf(int16_t x, int16_t y, int16_t w, int16_t h, UiHAlign ha,
            UiVAlign va, bool erase, const char *fmt, ...);

  /** Усечение строки под ширину maxW (в buf_, с суффиксом ".."). */
  const char *ellipsize(const char *s, int16_t maxW);

  /* ------------------------- вычисления --------------------------- */
  int16_t baselineIn(int16_t y, int16_t h, UiVAlign va);
  int16_t leftIn(int16_t x, int16_t w, int16_t textW, UiHAlign ha);

 private:
  U8G2_FOR_ADAFRUIT_GFX u8_;
  Adafruit_GFX *gfx_ = nullptr;
  UiFont cur_ = UiFont::Small;
  uint16_t fg_ = 0xFFFF;
  uint16_t bg_ = 0x0000;
  char buf_[128];
};

/** Единственный экземпляр слоя для всего проекта. */
extern UiText ui;
