#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H
#include "control_params.h"
#include "abstraction/angle_estimation.h"
#include "drivers/bmp280_driver.h"
#include "drivers/RCreciever_driver.h"

typedef struct{
    float thrust;
    float rollThrust;
    float pitchThrust;
    float yawThrust;
}final_thrust_t;

void updatePID_altitude(float ,float);
void updatePID_roll(float ,float);
void updatePID_pitch(float ,float);
void updatePID_yaw(float ,float);

float getThrust();
float getRollThrust();
float getPitchThrust();
float getYawThrust();

#endif