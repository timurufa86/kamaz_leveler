#pragma once

#include <Adafruit_GFX.h>
#include <stddef.h>

/** Print UTF-8 menu text for CourierCyr (FontsRus) via Adafruit_GFX.
 *  - ASCII as-is
 *  - Cyrillic U+0400…U+04FF → glyph = UTF-8 low byte (FontsRus hack)
 *  - «» ° × … — Δ → private slots 0xC2…0xC8
 *  maxGlyphs / startGlyph count visible glyphs, not raw UTF-8 bytes.
 */
void gemPrintUtf8(Adafruit_GFX &gfx, const char *str, size_t maxGlyphs,
                  size_t startGlyph = 0);
