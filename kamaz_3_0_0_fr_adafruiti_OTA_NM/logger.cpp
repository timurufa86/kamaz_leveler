#include "logger.h"
#include <cstdarg>
#include <cstdio>

static Logger::Level s_minLevel = Logger::ERROR;

void Logger::setMinLevel(Level level) { s_minLevel = level; }

Logger::Level Logger::getMinLevel() { return s_minLevel; }

bool Logger::verbose() { return s_minLevel <= DEBUG; }

void Logger::log(Level level, const char *tag, const char *message) {
  if (level < s_minLevel) return;
  const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
  Serial.printf("[%s][%s] %s\n", levelStr[level], tag ? tag : "?", message ? message : "");
}

void Logger::logf(Level level, const char *tag, const char *format, ...) {
  if (level < s_minLevel) return;
  const char *levelStr[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
  Serial.printf("[%s][%s] ", levelStr[level], tag ? tag : "?");
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  Serial.println();
}
