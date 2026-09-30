#include "ui_text.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

UiText ui;

/* ============================== init ============================== */

void UiText::begin(Adafruit_GFX &gfx) {
  gfx_ = &gfx;
  u8_.begin(gfx);
  u8_.setFontDirection(0);   // слева направо
  u8_.setFontMode(1);        // прозрачный фон глифов
  u8_.setForegroundColor(fg_);
  u8_.setBackgroundColor(bg_);
  setFont(UiFont::Small);
}

/* =========================== шрифт/цвет =========================== */

void UiText::setFont(UiFont f) {
  cur_ = f;
  u8_.setFont(uiFontPtr(f));
}

void UiText::setColors(uint16_t fg, uint16_t bg) {
  fg_ = fg;
  bg_ = bg;
  u8_.setForegroundColor(fg_);
  u8_.setBackgroundColor(bg_);
}

void UiText::setFg(uint16_t fg) {
  fg_ = fg;
  u8_.setForegroundColor(fg_);
}

void UiText::setTransparent(bool on) {
  u8_.setFontMode(on ? 1 : 0);
}

/* ============================= метрики ============================ */

int16_t UiText::ascent() { return gfx_ ? u8_.getFontAscent() : 0; }
int16_t UiText::descent() { return gfx_ ? u8_.getFontDescent() : 0; }
int16_t UiText::lineHeight() { return static_cast<int16_t>(ascent() - descent()); }

int16_t UiText::width(const char *s) {
  if (!gfx_ || !s) return 0;
  return u8_.getUTF8Width(s);
}

int16_t UiText::widthf(const char *fmt, ...) {
  if (!gfx_ || !fmt) return 0;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_, sizeof(buf_), fmt, ap);
  va_end(ap);
  return width(buf_);
}

/* ============================= заливки ============================ */

void UiText::clear(int16_t x, int16_t y, int16_t w, int16_t h) {
  if (gfx_) gfx_->fillRect(x, y, w, h, bg_);
}

void UiText::clear(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (gfx_) gfx_->fillRect(x, y, w, h, color);
}

/* =========================== вычисления =========================== */

int16_t UiText::baselineIn(int16_t y, int16_t h, UiVAlign va) {
  const int16_t a = ascent();
  const int16_t d = descent();
  int16_t top = y;
  switch (va) {
    case UiVAlign::Top:    top = y; break;
    case UiVAlign::Middle: top = static_cast<int16_t>(y + (h - (a - d)) / 2); break;
    case UiVAlign::Bottom: top = static_cast<int16_t>(y + h - (a - d)); break;
  }
  return static_cast<int16_t>(top + a);
}

int16_t UiText::leftIn(int16_t x, int16_t w, int16_t textW, UiHAlign ha) {
  switch (ha) {
    case UiHAlign::Left:   return x;
    case UiHAlign::Center: return static_cast<int16_t>(x + (w - textW) / 2);
    case UiHAlign::Right:  return static_cast<int16_t>(x + w - textW);
  }
  return x;
}

const char *UiText::ellipsize(const char *s, int16_t maxW) {
  if (!s) return "";
  if (width(s) <= maxW) {
    strncpy(buf_, s, sizeof(buf_) - 1);
    buf_[sizeof(buf_) - 1] = '\0';
    return buf_;
  }
  const int16_t dotsW = width("..");
  size_t len = strlen(s);
  if (len > sizeof(buf_) - 1) len = sizeof(buf_) - 1;

  // Идём от конца к началу, отбрасывая целые UTF-8 символы, пока
  // «обрезанная строка + ..» не поместится в maxW.
  size_t cut = len;
  while (cut > 0) {
    while (cut > 0 && (static_cast<uint8_t>(s[cut]) & 0xC0) == 0x80) cut--;  // к началу символа
    memcpy(buf_, s, cut);
    buf_[cut] = '\0';
    if (static_cast<int16_t>(width(buf_) + dotsW) <= maxW) {
      strncat(buf_, "..", sizeof(buf_) - strlen(buf_) - 1);
      return buf_;
    }
    cut--;
  }
  buf_[0] = '\0';
  return buf_;
}

/* ====================== печать по базовой линии =================== */

void UiText::draw(int16_t x, int16_t baselineY, const char *s) {
  if (!gfx_ || !s) return;
  u8_.drawUTF8(x, baselineY, s);
}

void UiText::drawf(int16_t x, int16_t baselineY, const char *fmt, ...) {
  if (!gfx_ || !fmt) return;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_, sizeof(buf_), fmt, ap);
  va_end(ap);
  draw(x, baselineY, buf_);
}

void UiText::drawCenter(int16_t cx, int16_t baselineY, const char *s) {
  if (!gfx_ || !s) return;
  draw(static_cast<int16_t>(cx - width(s) / 2), baselineY, s);
}

void UiText::drawCenterf(int16_t cx, int16_t baselineY, const char *fmt, ...) {
  if (!gfx_ || !fmt) return;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_, sizeof(buf_), fmt, ap);
  va_end(ap);
  drawCenter(cx, baselineY, buf_);
}

void UiText::drawRight(int16_t rightX, int16_t baselineY, const char *s) {
  if (!gfx_ || !s) return;
  draw(static_cast<int16_t>(rightX - width(s)), baselineY, s);
}

void UiText::drawRightf(int16_t rightX, int16_t baselineY, const char *fmt, ...) {
  if (!gfx_ || !fmt) return;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_, sizeof(buf_), fmt, ap);
  va_end(ap);
  drawRight(rightX, baselineY, buf_);
}

/* ===================== печать в прямоугольнике =================== */

void UiText::box(int16_t x, int16_t y, int16_t w, int16_t h, const char *s,
                 UiHAlign ha, UiVAlign va, bool erase) {
  if (!gfx_ || !s) return;
  if (erase) clear(x, y, w, h);
  const int16_t tw = width(s);
  draw(leftIn(x, w, tw, ha), baselineIn(y, h, va), s);
}

void UiText::boxf(int16_t x, int16_t y, int16_t w, int16_t h, UiHAlign ha,
                  UiVAlign va, bool erase, const char *fmt, ...) {
  if (!gfx_ || !fmt) return;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_, sizeof(buf_), fmt, ap);
  va_end(ap);
  box(x, y, w, h, buf_, ha, va, erase);
}
