#include "logger.h"
#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

void Logger::log(Level level, const char *tag, const char *message) {
  const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
  Serial.printf("[%s][%s] %s\n", levelStr[level], tag, message);
}

void Logger::logf(Level level, const char *tag, const char *format, ...) {
  const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
  Serial.printf("[%s][%s] ", levelStr[level], tag);
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  Serial.println();
}
