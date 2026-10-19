#include <Arduino.h>
#include "drivers/mpu6050_driver.h"
#include "drivers/RCreciever_driver.h"
#include "abstraction/angle_estimation.h"
#include "abstraction/esc_output.h"
#include "Application/attitude_control.h"
#include "Application/motor_mixing.h"
#include "drivers/hmc5883l_driver.h"
#include "logging/data_log.h"

unsigned long previousMicros = 0;
const unsigned long intervalMicros = 1000; // 1 millisecond
bool armed = false;

void setup(void) {
  Serial.begin(115200);
  rx_setup();
  MpuInit();
  MpuCalibration();
  HmcInit();
  HmcCalibration();
  setupESC();
  SDcardSetup();
  startLoggingTask(NULL);

}

void loop() {

  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= intervalMicros) {

    previousMicros += intervalMicros;

  void updateRCCMD();

  float throttle = getThrottleCMD();
  float roll     = getRollCMD()*MAX_ROLL; 
  float pitch    = getPitchCMD()*MAX_PITCH;
  float yaw      = getYawCMD()*MAX_YAW;

  if((throttle == 0.0f) && !armed){
    armed = true;
  }

  if(!armed){
    sendESC(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return; // Skip the rest of the loop if not armed
  }

  updateAttitude();

  float rollAngle = getRollAngle();
  float pitchAngle = getPitchAngle();
  float yawAngle = getYawAngle();

  updatePID_roll(roll, rollAngle);
  updatePID_pitch(pitch, pitchAngle);
  updatePID_yaw(yaw, yawAngle);

  float rollThrust = getRollThrust();
  float pitchThrust = getPitchThrust();
 // float thrust = getThrust();
  float yawThrust = getYawThrust();

  update_motor_commands(throttle, rollThrust, pitchThrust, yawThrust);

  float esc1 = getESCm1();
  float esc2 = getESCm2();
  float esc3 = getESCm3();
  float esc4 = getESCm4();
  float esc5 = getESCm5();
  float esc6 = getESCm6();

    if(!armed){
    sendESC(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  // --- Pack and Send Data to Queue Non-Blocking ---
  DataLog_t logItem = {
        .roll = rollAngle,
        .pitch = pitchAngle,
        .yaw = yawAngle,
        .throttle = throttle,
        .esc1 = esc1,
        .esc2 = esc2,
        .esc3 = esc3,
        .esc4 = esc4,
        .esc5 = esc5,
        .esc6 = esc6
    };
    xQueueSend(telemetryQueue, &logItem, 0);
  }
}
