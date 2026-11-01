#pragma once
/**
 *  task_valve.h — FreeRTOS valveTask: receives valve commands from queue,
 *  drives solenoid pins with timing and ACK.
 */

void valveTask(void *pvParameters);
