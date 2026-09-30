#include "mutex_guard.h"

uint32_t MutexGuard::failCount_ = 0;
uint32_t MutexGuard::timeoutCount_ = 0;
