#pragma once
/**
 *  task_pressure.h — FreeRTOS pressureTask: reads pads + master,
 *  publishes PressureData via queue / EventBus.
 */

void pressureTask(void *pvParameters);
