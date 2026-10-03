#include "ui_marquee.h"
#include "app_globals.h"
#include <Adafruit_ST7789.h>
#include "FontsRus/CourierCyr9.h"

extern Adafruit_ST7789 tft;
extern GEM_adafruit_gfx gem;

extern GEMPage otaPage;
extern GEMPage otaListPage;
extern GEMPage otaCardPage;

constexpr int16_t SCREEN_WIDTH_MQ  = 320;
constexpr int16_t SCREEN_HEIGHT_MQ = 240;
constexpr uint8_t MENU_TOP_OFFSET_MQ  = 14;
constexpr uint8_t ROW_HEIGHT_MQ       = 22;
constexpr uint16_t MENU_BG_COLOR_MQ   = 0x0000;   // ST77XX_BLACK
constexpr uint16_t MENU_TEXT_COLOR_MQ  = 0xFFFF;   // ST77XX_WHITE
constexpr size_t MQ_MAX_GLYPHS = 28;

void otaCheckUpdates();          // ui_ota_menu
void refreshOtaPage();           // ui_ota_menu

static MarqueeSlot g_mq[MQ_COUNT];
static GEMPage *s_prevMenuPage = nullptr;

static uint8_t mqTextStep(const char *s, size_t i) {
  const uint8_t c = static_cast<uint8_t>(s[i]);
  if (c == 0) return 0;
  if (c < 0x80) return 1;
  if ((c & 0xE0) == 0xC0 && (static_cast<uint8_t>(s[i + 1]) & 0xC0) == 0x80) return 2;
  if ((c & 0xF0) == 0xE0 && (static_cast<uint8_t>(s[i + 1]) & 0xC0) == 0x80 &&
      (static_cast<uint8_t>(s[i + 2]) & 0xC0) == 0x80)
    return 3;
  return 1;
}

static size_t mqGlyphCount(const char *s) {
  size_t n = 0, i = 0;
  while (s[i]) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    i += st;
    n++;
  }
  return n;
}

static size_t mqAlignOffset(const char *s, size_t off) {
  size_t i = 0;
  while (s[i] && i < off) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    if (i + st > off) return i;
    i += st;
  }
  return i;
}

static size_t mqCopyGlyphs(char *dst, size_t dstCap, const char *src, size_t maxGlyphs) {
  size_t w = 0, g = 0, i = 0;
  while (src[i] && g < maxGlyphs && w + 1 < dstCap) {
    const uint8_t st = mqTextStep(src, i);
    if (st == 0 || w + st >= dstCap) break;
    memcpy(dst + w, src + i, st);
    w += st;
    i += st;
    g++;
  }
  dst[w] = '\0';
  return w;
}

static void mqRender(MarqueeSlot &m) {
  if (!m.item) return;
  const size_t labelGlyphs = mqGlyphCount(m.label);
  if (labelGlyphs >= MQ_MAX_GLYPHS) {
    mqCopyGlyphs(m.shown, sizeof(m.shown), m.label, MQ_MAX_GLYPHS);
    m.item->setTitle(m.shown);
    m.scrolling = false;
    return;
  }

  const size_t availGlyphs = MQ_MAX_GLYPHS - labelGlyphs;
  const size_t labelBytes = mqCopyGlyphs(m.shown, sizeof(m.shown), m.label, labelGlyphs);
  const size_t vlenGlyphs = mqGlyphCount(m.value);

  if (vlenGlyphs <= availGlyphs) {
    mqCopyGlyphs(m.shown + labelBytes, sizeof(m.shown) - labelBytes, m.value, availGlyphs);
    m.scrolling = false;
  } else if (availGlyphs == 0) {
    m.shown[labelBytes] = '\0';
    m.scrolling = false;
  } else {
    m.scrolling = true;
    char loopBuf[160];
    snprintf(loopBuf, sizeof(loopBuf), "%s   ", m.value);
    const size_t loopLen = strlen(loopBuf);
    size_t pos = (loopLen > 0) ? mqAlignOffset(loopBuf, m.offset % loopLen) : 0;
    size_t wpos = labelBytes;
    size_t copied = 0;
    while (copied < availGlyphs && wpos + 1 < sizeof(m.shown) && loopLen > 0) {
      if (!loopBuf[pos]) pos = 0;
      const uint8_t st = mqTextStep(loopBuf, pos);
      if (st == 0 || wpos + st >= sizeof(m.shown)) break;
      memcpy(m.shown + wpos, loopBuf + pos, st);
      wpos += st;
      pos += st;
      copied++;
    }
    m.shown[wpos] = '\0';
  }
  m.item->setTitle(m.shown);
}

void mqBind(MarqueeId id, GEMItem &item, const char *label, const char *value) {
  MarqueeSlot &m = g_mq[id];
  const bool changed = !m.enabled || m.item != &item || strcmp(m.label, label ? label : "") != 0 ||
                       strcmp(m.value, value ? value : "") != 0;
  m.item = &item;
  m.enabled = true;
  strlcpy(m.label, label ? label : "", sizeof(m.label));
  strlcpy(m.value, value ? value : "", sizeof(m.value));
  if (changed) m.offset = 0;
  mqRender(m);
}

static bool mqItemOnCurrentPage(GEMItem *item) {
  GEMPage *page = gem.getCurrentMenuPage();
  if (!page || !item) return false;
  GEMItem *it = page->getMenuItem(0);
  while (it != nullptr) {
    if (it == item) return true;
    it = it->getMenuItemNext();
  }
  return false;
}

void gemRestoreMenuItemFont() {
  tft.setFont(&CourierCyr9pt8b);
  tft.setTextSize(1);
  tft.setTextWrap(false);
}

static int16_t mqItemRowTop(GEMItem *item, bool *selectedOut) {
  if (selectedOut) *selectedOut = false;
  if (!item || !menuVisible) return -1;
  GEMPage *page = gem.getCurrentMenuPage();
  if (!page) return -1;

  const byte perScreen = static_cast<byte>((SCREEN_HEIGHT_MQ - MENU_TOP_OFFSET_MQ) / ROW_HEIGHT_MQ);
  const byte focusIdx = page->getCurrentMenuItemIndex();
  const byte screenNum = focusIdx / perScreen;
  GEMItem *it = page->getMenuItem(screenNum * perScreen);
  byte i = 0;
  int16_t y = MENU_TOP_OFFSET_MQ;
  while (it != nullptr && i < perScreen) {
    if (it == item) {
      if (selectedOut) *selectedOut = (it == page->getCurrentMenuItem());
      return y;
    }
    it = it->getMenuItemNext();
    y += ROW_HEIGHT_MQ;
    i++;
  }
  return -1;
}

static void mqSoftRedrawRow(MarqueeSlot &m) {
  bool selected = false;
  const int16_t rowTop = mqItemRowTop(m.item, &selected);
  if (rowTop < 0) return;

  gemRestoreMenuItemFont();
  const uint16_t bg = selected ? MENU_TEXT_COLOR_MQ : MENU_BG_COLOR_MQ;
  const uint16_t textFg = selected ? MENU_BG_COLOR_MQ : MENU_TEXT_COLOR_MQ;
  tft.fillRect(0, rowTop, SCREEN_WIDTH_MQ - 2, ROW_HEIGHT_MQ, bg);

  const int16_t textY = rowTop + ((ROW_HEIGHT_MQ > 18) ? (ROW_HEIGHT_MQ * 3) / 4 : ROW_HEIGHT_MQ - 4);
  tft.setTextColor(textFg);
  tft.setCursor(5, textY);
  const char *s = m.shown;
  size_t glyphs = 0;
  for (size_t i = 0; s[i] != '\0' && glyphs < MQ_MAX_GLYPHS; ) {
    const uint8_t st = mqTextStep(s, i);
    if (st == 0) break;
    for (uint8_t b = 0; b < st; b++) tft.write(static_cast<uint8_t>(s[i + b]));
    i += st;
    glyphs++;
  }
}

bool mqTick() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < 280) return false;
  last = now;

  bool any = false;
  for (uint8_t id = 0; id < MQ_COUNT; id++) {
    MarqueeSlot &m = g_mq[id];
    if (!m.enabled || !m.item || !m.scrolling) continue;
    if (!mqItemOnCurrentPage(m.item)) continue;
    char loopBuf[160];
    snprintf(loopBuf, sizeof(loopBuf), "%s   ", m.value);
    const size_t loopLen = strlen(loopBuf);
    if (loopLen == 0) continue;
    size_t pos = mqAlignOffset(loopBuf, m.offset % loopLen);
    const uint8_t st = mqTextStep(loopBuf, pos);
    m.offset = static_cast<uint16_t>(pos + (st ? st : 1));
    mqRender(m);
    mqSoftRedrawRow(m);
    any = true;
  }
  return any;
}

void mqOnMenuPageEnter() {
  GEMPage *page = gem.getCurrentMenuPage();
  if (page == &otaPage && s_prevMenuPage != &otaPage) {
    otaCheckUpdates();
  }
  s_prevMenuPage = page;
}
