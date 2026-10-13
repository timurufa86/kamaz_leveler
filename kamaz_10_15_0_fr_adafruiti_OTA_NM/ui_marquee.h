#pragma once
#include <Arduino.h>
#include <GEM_adafruit_gfx.h>

enum MarqueeId : uint8_t {
  MQ_OTA_VER = 0,
  MQ_OTA_ST,
  MQ_LIST_HDR,
  MQ_CARD_TAG,
  MQ_CARD_INFO,
  MQ_CARD_ST,
  MQ_CARD_SHA,
  MQ_INFO_VER,
  MQ_INFO_WIFI,
  MQ_INFO_SYS,
  MQ_COUNT
};

struct MarqueeSlot {
  GEMItem *item = nullptr;
  char label[20] = "";
  char value[72] = "";
  char shown[72] = "";
  uint16_t offset = 0;
  bool enabled = false;
  bool scrolling = false;
};

void mqBind(MarqueeId id, GEMItem &item, const char *label, const char *value);
bool mqTick();
void mqOnMenuPageEnter();
void gemRestoreMenuItemFont();
