#include "drivers/RCreciever_driver.h"

// Volatile variables to store raw pulse widths in microseconds
volatile uint32_t throttlePulse = 1500;
volatile uint32_t rollPulse     = 1500;
volatile uint32_t pitchPulse    = 1500;
volatile uint32_t yawPulse      = 1500;

// Variables to track rising edge timestamps inside ISRs
volatile uint32_t throttleStart = 0;
volatile uint32_t rollStart     = 0;
volatile uint32_t pitchStart    = 0;
volatile uint32_t yawStart      = 0;

RC_cmd_t cmd = {};

// Interrupt Service Routines (ISRs) for each channel
void IRAM_ATTR handleThrottle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    throttleStart = micros();
  } else {
    throttlePulse = micros() - throttleStart;
  }
}

void IRAM_ATTR handleRoll() {
  if (digitalRead(ROLL_PIN) == HIGH) {
    rollStart = micros();
  } else {
    rollPulse = micros() - rollStart;
  }
}

void IRAM_ATTR handlePitch() {
  if (digitalRead(PITCH_PIN) == HIGH) {
    pitchStart = micros();
  } else {
    pitchPulse = micros() - pitchStart;
  }
}

void IRAM_ATTR handleYaw() {
  if (digitalRead(YAW_PIN) == HIGH) {
    yawStart = micros();
  } else {
    yawPulse = micros() - yawStart;
  }
}

static float getNormalizedValue(uint32_t min_range, uint32_t max_range ,uint32_t value){

    // Prevent division by zero if the range is flat
    if (max_range - min_range == 0)
    {
        return 0.0f; 
    }

    // Calculate the normalized percentage ,Cast to float BEFORE dividing to preserve decimal precision
    return ((float)(value - min_range) / (float)(max_range - min_range)) * 100.0f;
}

void rx_setup(){
    // Configure pins as inputs with internal pull-downs (or floating depending on receiver)
  pinMode(THROTTLE_PIN, INPUT_PULLDOWN);
  pinMode(ROLL_PIN, INPUT_PULLDOWN);
  pinMode(PITCH_PIN, INPUT_PULLDOWN);
  pinMode(YAW_PIN, INPUT_PULLDOWN);

  // Attach interrupts for both RISING and FALLING edges
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), handleThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROLL_PIN), handleRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PITCH_PIN), handlePitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YAW_PIN), handleYaw, CHANGE);
}

void updateRCCMD()
{
    float thrN = getNormalizedValue(MINIMUM_PULSE_WIDTH , MAXIMUM_PULSE_WIDTH , throttlePulse);
    float rollN = getNormalizedValue(MINIMUM_PULSE_WIDTH , MAXIMUM_PULSE_WIDTH , rollPulse);
    float pitchN = getNormalizedValue(MINIMUM_PULSE_WIDTH , MAXIMUM_PULSE_WIDTH , pitchPulse);
    float yawN = getNormalizedValue(MINIMUM_PULSE_WIDTH , MAXIMUM_PULSE_WIDTH , yawPulse);

    //update the commanded throttle and angles of roll , pitch , yaw
    cmd.throttle = thrN;
    cmd.roll = (rollN * (float)MAX_ROLL)/100.0f;
    cmd.pitch = (pitchN * (float)MAX_PITCH)/100.0f;
    cmd.yaw = (yawN * (float)MAX_YAW)/100.0f;
}

float getThrottleCMD(){
    // updateRCCMD();
     return cmd.throttle;
}

float getRollCMD(){
   // updateRCCMD();
    return cmd.roll;
}

float getPitchCMD(){
  //  updateRCCMD();
    return cmd.pitch;
}

float getYawCMD(){
  //  updateRCCMD();
    return cmd.yaw;
}
