#pragma once
/**
 * EncButton config — macros MUST appear before #include <EncButton.h>
 * (see https://github.com/GyverLibs/EncButton README).
 * Do not define EB_*_TIME here: that freezes timeouts and disables setXxxTimeout().
 */
#define EB_NO_CALLBACK
#define EB_NO_COUNTER
#define EB_NO_BUFFER
#include <EncButton.h>
