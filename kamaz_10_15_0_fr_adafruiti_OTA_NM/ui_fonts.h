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
    case UiFont::Tiny:     return u8g2_font_5x7_t_cyrillic;
    case UiFont::Small:    return u8g2_font_6x13_t_cyrillic;
    case UiFont::SmallB:   return u8g2_font_6x13B_t_cyrillic;
    case UiFont::Body:     return u8g2_font_7x13_t_cyrillic;
    case UiFont::BodyWide: return u8g2_font_8x13_t_cyrillic;
    case UiFont::Uni16:    return u8g2_font_unifont_t_cyrillic;
    case UiFont::Med:      return u8g2_font_10x20_t_cyrillic;
    case UiFont::Large:    return u8g2_font_inr24_t_cyrillic;
    case UiFont::XLarge:   return u8g2_font_inr33_t_cyrillic;
    case UiFont::Huge:     return u8g2_font_inr46_t_cyrillic;
  }
  return u8g2_font_6x13_t_cyrillic;
}
