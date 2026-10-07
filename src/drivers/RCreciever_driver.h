#ifndef RCRECIEVER_DRIVER_H
#define RCRECIEVER_DRIVER_H

#include <Arduino.h>
#include "FC_config.h"

typedef struct{
    float throttle;
    float roll;
    float pitch;
    float yaw;
}RC_cmd_t;

void rx_setup();
void updateRCCMD();
float getThrottleCMD();
float getRollCMD();
float getPitchCMD();
float getYawCMD();

#endif