#pragma once

#include <Arduino.h>

class Logger {
public:
  enum Level {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
  };

  static void setMinLevel(Level level);
  static Level getMinLevel();
  /** True when TEST DEBUG ON (minLevel <= DEBUG). */
  static bool verbose();

  static void log(Level level, const char *tag, const char *message);
  static void logf(Level level, const char *tag, const char *format, ...);
};

/** Verbose Serial — only when Logger::verbose() (TEST DEBUG ON). */
#define SERIAL_DBG(...)                                                         \
  do {                                                                          \
    if (Logger::verbose()) {                                                    \
      Serial.printf(__VA_ARGS__);                                               \
    }                                                                           \
  } while (0)

#define SERIAL_DBGLN(msg)                                                       \
  do {                                                                          \
    if (Logger::verbose()) {                                                    \
      Serial.println(msg);                                                      \
    }                                                                           \
  } while (0)

#define SERIAL_DBG_PRINT(...)                                                   \
  do {                                                                          \
    if (Logger::verbose()) {                                                    \
      Serial.print(__VA_ARGS__);                                                \
    }                                                                           \
  } while (0)
