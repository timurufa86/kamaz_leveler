#include "task_monitor.h"
#include "task_pool.h"

TaskMonitor::TaskInfo TaskMonitor::tasks[TASK_COUNT];

void cfg_taskMonitorUpdate(uint8_t idx) { TaskMonitor::updateTaskStatus(static_cast<TaskMonitor::TaskIndex>(idx)); }

void printTaskInfo() {
  Serial.println("\n=== TASK INFO ===");
  for (uint8_t i = 0; i < TaskPool::getTaskCount(); i++) {
    const char *name = TaskPool::getTaskName(i);
    UBaseType_t stackFree = TaskPool::getTaskMinStack(i);
    bool enabled = TaskPool::isTaskEnabled(i);

    Serial.printf("%s: stack=%d bytes, enabled=%d\n",
                  name ? name : "Unknown",
                  stackFree,
                  enabled ? 1 : 0);

    if (stackFree < 200) {
      Serial.printf("  ?? WARNING: Task '%s' has very low stack!\n",
                    name ? name : "Unknown");
    }
  }

  Serial.printf("Total tasks: %d\n", TaskPool::getTaskCount());
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.println("==================\n");
}

