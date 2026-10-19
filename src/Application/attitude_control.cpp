#include "attitude_control.h"

//stores the thrust values to be applied
final_thrust_t thrust_applied ={};

float error_sumRoll = 0;
float error_sumPitch = 0;
float error_sumYaw = 0;
float error_sumAltitude = 0;

float RollAngle_prev = 0;
float PitchAngle_prev = 0;
float YawAngle_prev = 0;
float Altitude_prev = 0;

//DT is the time step for the PID calculations, which is set to 0.001 seconds (1 millisecond).
void updatePID_roll(float setpoint, float currentRollAngle) {
    float error = setpoint - currentRollAngle;
    
    // 1. Proportional Term
    float p_term = (float)KP_ROLL * error;
    
    // 2. Derivative Term 
    float d_term = (float)KD_ROLL * (currentRollAngle - RollAngle_prev) / DT;

    // 3. Calculate tentative output to check for saturation
    float i_term_temp = (float)KI_ROLL * error_sumRoll;
    float thr = p_term + i_term_temp - d_term;

    // 4. Conditional Anti-Windup
    // Only accumulate error if we aren't saturated in the direction of the error
    bool is_saturated_upper = (thr >= MAX_ROLL_THR);
    bool is_saturated_lower = (thr <= -MAX_ROLL_THR);

    if (!((is_saturated_upper && error > 0.0f) || (is_saturated_lower && error < 0.0f))) {
        error_sumRoll += error * DT;
    }

    // 5. Recompute final terms with updated integral sum
    float i_term = (float)KI_ROLL * error_sumRoll;
    thr = p_term + i_term - d_term;

    // 6. Actuator Output Clamping
    if (thr >= MAX_ROLL_THR) {
        thrust_applied.rollThrust = (float)MAX_ROLL_THR;
    } else if (thr <= -MAX_ROLL_THR) {
        thrust_applied.rollThrust = -(float)MAX_ROLL_THR;
    } else {
        thrust_applied.rollThrust = thr;
    }

    // Save state for next cycle
    RollAngle_prev = currentRollAngle;
}

void updatePID_altitude(float setpoint, float currentAltitude){
    
    float error = setpoint - currentAltitude;
    
    // 1. Proportional Term
    float p_term = (float)KP_ALTITUDE * error;
    
    // 2. Derivative Term 
    float d_term = (float)KD_ALTITUDE * (currentAltitude - Altitude_prev) / DT;

    // 3. Calculate tentative output to check for saturation
    float i_term_temp = (float)KI_ALTITUDE * error_sumAltitude;
    float thr = p_term + i_term_temp - d_term;

    // 4. Conditional Anti-Windup
    // Only accumulate error if we aren't saturated in the direction of the error
    bool is_saturated_upper = (thr >= MAX_THRUST);
    bool is_saturated_lower = (thr <= -MAX_THRUST);

    if (!((is_saturated_upper && error > 0.0f) || (is_saturated_lower && error < 0.0f))) {
        error_sumAltitude += error * DT;
    }

    // 5. Recompute final terms with updated integral sum
    float i_term = (float)KI_ALTITUDE * error_sumAltitude;
    thr = p_term + i_term - d_term;

    // 6. Actuator Output Clamping
    if (thr >= MAX_THRUST) {
        thrust_applied.thrust = (float)MAX_THRUST;
    } else if (thr <= -MAX_THRUST) {
        thrust_applied.thrust = -(float)MAX_THRUST;
    } else {
        thrust_applied.thrust = thr;
    }

    // Save state for next cycle
    Altitude_prev = currentAltitude;

}

void updatePID_pitch(float setpoint, float currentPitchAngle){

      float error = setpoint - currentPitchAngle;
    
    // 1. Proportional Term
    float p_term = (float)KP_PITCH * error;
    
    // 2. Derivative Term (Measurement derivative with correct damping sign)
    // Subtracted (-) to act as a damper against rapid angular changes
    float d_term = (float)KD_PITCH * (currentPitchAngle - PitchAngle_prev) / DT;

    // 3. Calculate tentative output to check for saturation
    float i_term_temp = (float)KI_PITCH * error_sumPitch;
    float thr = p_term + i_term_temp - d_term;

    // 4. Conditional Anti-Windup
    // Only accumulate error if we aren't saturated in the direction of the error
    bool is_saturated_upper = (thr >= MAX_PITCH_THR);
    bool is_saturated_lower = (thr <= -MAX_PITCH_THR);

    if (!((is_saturated_upper && error > 0.0f) || (is_saturated_lower && error < 0.0f))) {
        error_sumPitch += error * DT;
    }

    // 5. Recompute final terms with updated integral sum
    float i_term = (float)KI_PITCH * error_sumPitch;
    thr = p_term + i_term - d_term;

    // 6. Actuator Output Clamping
    if (thr >= MAX_PITCH_THR) {
        thrust_applied.pitchThrust = (float)MAX_PITCH_THR;
    } else if (thr <= -MAX_PITCH_THR) {
        thrust_applied.pitchThrust = -(float)MAX_PITCH_THR;
    } else {
        thrust_applied.pitchThrust = thr;
    }

    // Save state for next cycle
    PitchAngle_prev = currentPitchAngle;

}

void updatePID_yaw(float setpoint, float currentYawAngle){

          float error = setpoint - currentYawAngle;
    
    // 1. Proportional Term
    float p_term = (float)KP_YAW * error;
    
    // 2. Derivative Term 
    float d_term = (float)KD_YAW * (currentYawAngle - YawAngle_prev) / DT;

    // 3. Calculate tentative output to check for saturation
    float i_term_temp = (float)KI_YAW * error_sumYaw;
    float thr = p_term + i_term_temp - d_term;

    // 4. Conditional Anti-Windup
    // Only accumulate error if we aren't saturated in the direction of the error
    bool is_saturated_upper = (thr >= MAX_YAW_THR);
    bool is_saturated_lower = (thr <= -MAX_YAW_THR);

    if (!((is_saturated_upper && error > 0.0f) || (is_saturated_lower && error < 0.0f))) {
        error_sumYaw += error * DT;
    }

    // 5. Recompute final terms with updated integral sum
    float i_term = (float)KI_YAW * error_sumYaw;
    thr = p_term + i_term - d_term;

    // 6. Actuator Output Clamping
    if (thr >= MAX_YAW_THR) {
        thrust_applied.yawThrust = (float)MAX_YAW_THR;
    } else if (thr <= -MAX_YAW_THR) {
        thrust_applied.yawThrust = -(float)MAX_YAW_THR;
    } else {
        thrust_applied.yawThrust = thr;
    }

    // Save state for next cycle
    YawAngle_prev = currentYawAngle;

}

// Getter functions to access the thrust values
float getThrust(){
    return thrust_applied.thrust;
}

float getRollThrust(){
    return thrust_applied.rollThrust;
}
float getPitchThrust(){
    return thrust_applied.pitchThrust;

}
float getYawThrust(){
    return thrust_applied.yawThrust;

}
