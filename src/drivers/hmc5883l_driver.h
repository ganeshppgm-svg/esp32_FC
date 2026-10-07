#ifndef HMC5883L_DRIVER_H
#define HMC5883L_DRIVER_H

#include <Wire.h>
#include "FC_config.h"

#define HMC5883L_ADDR 0x1E
#define REG_CONFIG_A        0x00
#define REG_CONFIG_B        0x01
#define REG_MODE            0x02
#define REG_DATA_X_MSB      0x03

#define GAUSS_SCALE_FACTOR_1_9G  820.0f

typedef struct {
    float magX;
    float magY;
    float magZ;
} MagData_t;

typedef struct {
    float offsetX;
    float offsetY;
    float offsetZ;
    float scaleX;
    float scaleY;
    float scaleZ;
} MagCalibration_t;

void HmcInit();
void updateRawMagData();
void HmcCalibration();
void applyCalibration();
float getHeading();


#endif 