#include "esc_output.h"

Servo xrotorESC[6];

// setup of esc throttle pins
void setupESC(){
    xrotorESC[0].attach(ESC1, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
    xrotorESC[1].attach(ESC2, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
    xrotorESC[2].attach(ESC3, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
    xrotorESC[3].attach(ESC4, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
    xrotorESC[4].attach(ESC5, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
    xrotorESC[5].attach(ESC6, MAX_PULSE_WIDTH_ESC, MIN_PULSE_WIDTH_ESC); 
}

//update the PWM duty cycles for the ESCs
void sendESC(float m1 ,
             float m2,
             float m3,
             float m4,
             float m5,
             float m6){

float e1 = (m1*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m1)); 
float e2 = (m2*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m2)); 
float e3 = (m3*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m3)); 
float e4 = (m4*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m4)); 
float e5 = (m5*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m5)); 
float e6 = (m6*(MAX_PULSE_WIDTH_ESC)) + (MIN_PULSE_WIDTH_ESC*(1-m6)); 

xrotorESC[0].writeMicroseconds(e1);
xrotorESC[1].writeMicroseconds(e2);
xrotorESC[2].writeMicroseconds(e3); 
xrotorESC[3].writeMicroseconds(e4);
xrotorESC[4].writeMicroseconds(e5);
xrotorESC[5].writeMicroseconds(e6); 

}