#pragma once
/**
 *  auto_level.h — AutoLevelingController class and instance.
 *  Provides automatic angle-based leveling via coarse/fine valve adjustments.
 */

#include <Arduino.h>

class AutoLevelingController {
public:
  void process();
  bool isBusy() const;
  uint8_t fineIterations() const;
  const char *stageName() const;
};

extern AutoLevelingController autoLevelingController;

/** Thin bridge so other TUs can query isBusy() without the class header. */
bool autoLevelIsBusy();
