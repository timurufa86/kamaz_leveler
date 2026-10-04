#include "memory_monitor.h"

uint32_t MemoryMonitor::lastCheckTime = 0;
uint32_t MemoryMonitor::minFreeHeap = 0;
uint32_t MemoryMonitor::lastWarningTime = 0;
bool MemoryMonitor::lowMemoryReported = false;
bool MemoryMonitor::criticalMemoryReported = false;

