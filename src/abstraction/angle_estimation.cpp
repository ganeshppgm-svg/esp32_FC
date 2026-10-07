#include "angle_estimation.h"
#include <cmath>

attitude current_att ={};

const float gyroyaw_weight= 0.98f; // Weight for gyro yaw integration
const float mag_weight = (1.0f - gyroyaw_weight); // Weight for magnetometer correction

float wrapAngle180(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void updateAttitude(){

    updateSensorData();
    static uint32_t previous_update_us = 0;
    static bool initialized = false;

    const uint32_t now_us = micros();
    const float dt = initialized
        ? (float)(now_us - previous_update_us) / 1000000.0f
        : 0.0f;
    previous_update_us = now_us;

    // Weight gyro integration over short intervals and accel correction over longer ones.
    const float gyro_weight = FILTER_TIME_CONSTANT / (FILTER_TIME_CONSTANT + dt);
    const float accel_weight = 1.0f - gyro_weight;

    const AccGyro_t sensor = getSensorData();
    // Accelerometer angles use gravity to estimate roll and pitch, in degrees.
    const float accel_roll = atan2f(sensor.accelY, sensor.accelZ) * radians_to_degrees;
    const float accel_pitch = atan2f(-sensor.accelX,
        sqrtf(sensor.accelY * sensor.accelY + sensor.accelZ * sensor.accelZ)
    ) * radians_to_degrees;
    const float gyro_roll = sensor.gyroX * radians_to_degrees;
    const float gyro_pitch = sensor.gyroY * radians_to_degrees;
    const float gyro_yaw = sensor.gyroZ * radians_to_degrees;

    // Establish a gravity-based starting orientation before integrating gyro rates.
    if (!initialized) {
        current_att.rollangle = accel_roll;
        current_att.pitchangle = accel_pitch;
        current_att.yawangle = 0.0f;
        initialized = true;
        return;
    }

    current_att.rollangle = gyro_weight * (current_att.rollangle + gyro_roll * dt)
        + accel_weight * accel_roll;
    current_att.pitchangle = gyro_weight * (current_att.pitchangle + gyro_pitch * dt)
        + accel_weight * accel_pitch;
    
    float mag_heading = getHeading();
    float gyroyaw = current_att.yawangle + gyro_yaw * dt;
    float yaw_error = wrapAngle180(mag_heading - gyroyaw);
    current_att.yawangle = gyroyaw + mag_weight * yaw_error;
}

float getRollAngle(){
    return current_att.rollangle;
}

float getPitchAngle(){
    return current_att.pitchangle;
}

float getYawAngle(){
    return current_att.yawangle;
}
