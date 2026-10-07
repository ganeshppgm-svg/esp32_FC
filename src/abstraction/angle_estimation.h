#ifndef ANGLE_ESTIMATION_H
#define ANGLE_ESTIMATION_H

#include "drivers\mpu6050_driver.h"
#include "FC_config.h"
#include "drivers/RCreciever_driver.h"
#include "math.h"
#include "drivers/hmc5883l_driver.h"

typedef struct{
    float rollangle;
    float pitchangle;
    float yawangle;
}attitude;

const float radians_to_degrees = 57.2957795f;
const float FILTER_TIME_CONSTANT = 0.5f;

void updateAttitude();
float getRollAngle();
float getPitchAngle();
float getYawAngle();

#endif