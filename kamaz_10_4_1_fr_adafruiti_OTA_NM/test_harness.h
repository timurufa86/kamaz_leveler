#pragma once
/**
 *  test_harness.h — serial commands `TEST ...` and the SELF/FULL sequencer.
 */
#include <Arduino.h>

void processTestCommandLine(char *line);
void testHarnessPoll();
