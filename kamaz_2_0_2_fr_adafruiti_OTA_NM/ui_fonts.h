#pragma once
#include <stdint.h>
#include <U8g2_for_Adafruit_GFX.h>

/*
 * Шрифтовая сетка UI на базе U8g2.
 *
 * ВАЖНО: U8g2_for_Adafruit_GFX не умеет масштабировать текст (аналога
 * Adafruit setTextSize нет), поэтому «размер» надписи задаётся выбором
 * шрифта — глифы остаются пиксель-в-пиксель и читаются ровно так, как
 * нарисованы дизайнером шрифта, без растягивания.
 *
 * Кириллицу в бандле библиотеки содержат только варианты *_cyrillic.
 */
enum class UiFont : uint8_t {
  Tiny,      // 5x7      — подсказки, подписи шкал, плотные строки
  Small,     // 6x13     — основной мелкий текст
  SmallB,    // 6x13B    — акцент/заголовок секции
  Body,      // 7x13     — основной текст
  BodyWide,  // 8x13     — основной текст «пошире»
  Uni16,     // unifont  — широкая кириллица, 16 px
  Med,       // 10x20    — заголовки экранов и значения
  Large,     // inr24    — крупные значения (давление, угол)
  XLarge,    // inr33    — очень крупные показатели
  Huge       // inr46    — сплэш/акцентная цифра
};

inline const uint8_t *uiFontPtr(UiFont f) {
  switch (f) {
    case UiFont::Tiny:     return u8g2_font_5x8_t_cyrillic;   // 2.0.2: 5x8 читаемее 5x7 (+86 Б)
    case UiFont::Small:    return u8g2_font_6x13_t_cyrillic;
    case UiFont::SmallB:   return u8g2_font_6x13B_t_cyrillic;
    case UiFont::Body:     return u8g2_font_7x13_t_cyrillic;
    case UiFont::BodyWide: return u8g2_font_8x13_t_cyrillic;
    case UiFont::Uni16:    return u8g2_font_10x20_t_cyrillic;   // 2.0.1: unifont не используется (−6,6 КБ)
    case UiFont::Med:      return u8g2_font_10x20_t_cyrillic;
    case UiFont::Large:    return u8g2_font_inr27_t_cyrillic;   // 2.0.2: главное значение крупнее
    case UiFont::XLarge:   return u8g2_font_10x20_t_cyrillic;   // 2.0.2: inr33/inr24 не используются
    case UiFont::Huge:     return u8g2_font_10x20_t_cyrillic;   // 2.0.2: inr46 не используется
  }
  return u8g2_font_6x13_t_cyrillic;
}
