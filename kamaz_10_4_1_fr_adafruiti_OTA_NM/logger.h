#pragma once

class Logger {
public:
  enum Level {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3
  };

  static void log(Level level, const char *tag, const char *message);
  static void logf(Level level, const char *tag, const char *format, ...);
};
