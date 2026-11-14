#pragma once
/**
 *  task_control.h — FreeRTOS controlTask: main control loop
 *  consuming IMU/pressure queues, running auto-leveling,
 *  manual maintenance, leak monitor, valve health check.
 */

void controlTask(void *pvParameters);
