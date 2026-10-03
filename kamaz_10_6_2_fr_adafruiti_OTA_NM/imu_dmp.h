#pragma once
/**
 *  imu_dmp.h — DMP initialization, MPU6050 / Kalman filter access.
 *
 *  MPU6050 instance and GKalman filters live in imu_dmp.cpp.
 *  Extern declarations here allow the .ino (calibration, runtime settings)
 *  and task_imu.cpp (FIFO read loop) to access them directly.
 */

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <GyverFilters.h>

void initializeDMP();

extern MPU6050 mpu;
extern GKalman filterX;
extern GKalman filterY;
