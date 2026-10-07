#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "FC_config.h"

extern Adafruit_MPU6050 mpu;

// Create a custom TwoWire instance for the ESP32-S3
extern TwoWire I2C_MPU;

typedef struct{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
}AccGyro_t;

void MpuInit();
void MpuCalibration();
void printSensorData();
void updateSensorData();
AccGyro_t getSensorData();

#endif