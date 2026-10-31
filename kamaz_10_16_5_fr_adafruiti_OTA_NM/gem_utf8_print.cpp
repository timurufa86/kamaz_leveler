#include "gem_utf8_print.h"

#include <stdint.h>

static size_t utf8Next(const char *s, size_t i, uint32_t *cp) {
  const uint8_t c0 = static_cast<uint8_t>(s[i]);
  if (c0 == 0) {
    *cp = 0;
    return 0;
  }
  if (c0 < 0x80) {
    *cp = c0;
    return 1;
  }
  const uint8_t c1 = static_cast<uint8_t>(s[i + 1]);
  if ((c0 & 0xE0) == 0xC0 && (c1 & 0xC0) == 0x80) {
    *cp = (static_cast<uint32_t>(c0 & 0x1F) << 6) | (c1 & 0x3F);
    return 2;
  }
  const uint8_t c2 = static_cast<uint8_t>(s[i + 2]);
  if ((c0 & 0xF0) == 0xE0 && (c1 & 0xC0) == 0x80 && (c2 & 0xC0) == 0x80) {
    *cp = (static_cast<uint32_t>(c0 & 0x0F) << 12) |
          (static_cast<uint32_t>(c1 & 0x3F) << 6) | (c2 & 0x3F);
    return 3;
  }
  const uint8_t c3 = static_cast<uint8_t>(s[i + 3]);
  if ((c0 & 0xF8) == 0xF0 && (c1 & 0xC0) == 0x80 && (c2 & 0xC0) == 0x80 &&
      (c3 & 0xC0) == 0x80) {
    *cp = (static_cast<uint32_t>(c0 & 0x07) << 18) |
          (static_cast<uint32_t>(c1 & 0x3F) << 12) |
          (static_cast<uint32_t>(c2 & 0x3F) << 6) | (c3 & 0x3F);
    return 4;
  }
  *cp = c0;  // invalid lead — skip one byte
  return 1;
}

/** Map Unicode codepoint to CourierCyr glyph index, or -1 to skip. */
static int mapToGlyph(uint32_t cp) {
  if (cp < 0x80) return static_cast<int>(cp);
  if (cp >= 0x0400 && cp <= 0x04FF) {
    // FontsRus: draw UTF-8 continuation byte (D0/D1 lead is out of font range).
    return static_cast<int>(0x80 | (cp & 0x3F));
  }
  switch (cp) {
    case 0x00AB: return 0xC2;  // «
    case 0x00BB: return 0xC3;  // »
    case 0x00B0: return 0xC4;  // °
    case 0x00D7: return 0xC5;  // ×
    case 0x2026: return 0xC6;  // …
    case 0x2014: return 0xC7;  // —
    case 0x0394: return 0xC8;  // Δ
    default: return -1;
  }
}

void gemPrintUtf8(Adafruit_GFX &gfx, const char *str, size_t maxGlyphs,
                  size_t startGlyph) {
  if (!str || maxGlyphs == 0) return;

  size_t i = 0;
  size_t glyphIndex = 0;
  size_t printed = 0;

  while (str[i] != '\0' && printed < maxGlyphs) {
    uint32_t cp = 0;
    const size_t step = utf8Next(str, i, &cp);
    if (step == 0) break;
    i += step;

    const int glyph = mapToGlyph(cp);
    if (glyph < 0) continue;

    if (glyphIndex < startGlyph) {
      glyphIndex++;
      continue;
    }
    glyphIndex++;
    gfx.write(static_cast<uint8_t>(glyph));
    printed++;
  }
}
