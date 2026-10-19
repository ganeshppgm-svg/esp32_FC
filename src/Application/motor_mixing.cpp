#include "motor_mixing.h"
#include <float.h>

float esc_commands[6] = {0};

const float mixing_matrix[6][4] = {
    {1 , 0     , 1   , 1},   
    {1 ,-0.866 , 0.5 ,-1},
    {1 ,-0.866 ,-0.5 , 1},
    {1 , 0     ,-1   ,-1},
    {1 , 0.866 ,-0.5 , 1},
    {1 , 0.866 , 0.5 ,-1}
//   T ,   R   ,  P  , Y
};


//update motor commands based on the thrust, roll, pitch and yaw inputs
void update_motor_commands(float thrust ,float roll ,float pitch ,float yaw){

    float max = FLT_MIN;

    for(int i=0;i<6;i++){
        esc_commands[i] = mixing_matrix[i][0]*thrust +
                          mixing_matrix[i][1]*roll +
                          mixing_matrix[i][2]*pitch +
                          mixing_matrix[i][3]*yaw; 
        if(esc_commands[i] > max){
            max = esc_commands[i];
        }                           
    }

    //apply saturation if any motor command exceeds 1
    if(max > 1){
        float sub_offset = max - 1;
        for(int i=0;i<6;i++){
        esc_commands[i]-=sub_offset;                         
    }
    }

}

//getter functions to access the motor commands
float getESCm1(){
    return esc_commands[0];
}

float getESCm2(){
    return esc_commands[1];
}

float getESCm3(){
    return esc_commands[2];
}

float getESCm4(){
    return esc_commands[3];
}

float getESCm5(){
    return esc_commands[4];
}

float getESCm6(){
    return esc_commands[5];
}