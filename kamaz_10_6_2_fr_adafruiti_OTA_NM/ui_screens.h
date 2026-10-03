#pragma once
/**
 *  ui_screens.h — main, movement, error, calibration and service screens.
 *  Drawing moved out of the sketch; GEM/tft stay defined in the .ino.
 */

#include <Arduino.h>
#include "ui_text.h"

constexpr int16_t SCREEN_WIDTH = 320;
constexpr int16_t SCREEN_HEIGHT = 240;

enum class ServiceScreen : uint8_t {
  NONE = 0,
  VALVE_TEST,
  IMU_ZERO_CONFIRM,
  IMU_CALIB,
  MPU_DIAG
};

extern volatile ServiceScreen serviceScreen;

constexpr uint8_t MANUAL_VALVE_COUNT = 6;
extern const char *const manualValveNames[MANUAL_VALVE_COUNT];
extern const uint8_t manualValvePins[MANUAL_VALVE_COUNT];

extern bool valveTestUiFullRedraw;
extern bool imuZeroUiFullRedraw;
extern bool imuCalibUiFullRedraw;
extern bool mpuDiagUiFullRedraw;

extern const uint16_t COLOR_BG;
extern const uint16_t COLOR_TEXT;
extern const uint16_t COLOR_HIGHLIGHT;
extern const uint16_t COLOR_ERROR;
extern const uint16_t COLOR_SUCCESS;
extern const uint16_t COLOR_WARNING;
extern const uint16_t COLOR_OTA;
extern const uint16_t COLOR_WHITE;

void uiRedrawLabel(int16_t x, int16_t y, int16_t w, int16_t h, char *prev, size_t prevSz,
                   const char *next, uint16_t fg, uint16_t bg, UiFont font, UiHAlign ha);
void uiInfoRow(int16_t y, const char *label, const char *value);
void uiHeader(const char *title, uint16_t titleColor);
void uiHintBar(const char *text, uint16_t color);

void setDisplayDirty();
void forceDisplayReset(bool force = false);
void drawProgressBar(int16_t x, int16_t y, int16_t width, int16_t height, uint8_t percent, uint16_t color);
void drawIcon(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);
void drawIconL(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);
void drawIconB(int16_t x, int16_t y, const unsigned char *icon, uint16_t color);
void blinkErrorIcon();

void drawServiceScreen();
void displayMovementScreen();
void displayErrorScreen();
void displayCalibrationScreen();
void displayMainScreen();
void updateMainTiltLive();
void updateMainInfoLine(bool movingNow);
void updateMainStatBadge();
