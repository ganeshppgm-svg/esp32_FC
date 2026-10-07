#ifndef ESC_OUTPUT_H
#define ESC_OUTPUT_H

#include <ESP32Servo.h>
#include "FC_config.h"

void setupESC();

void sendESC(float m1 ,
             float m2,
             float m3,
             float m4,
             float m5,
             float m6);

#endif