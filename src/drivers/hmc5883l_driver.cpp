#include "hmc5883l_driver.h"
#include <Arduino.h>

MagData_t rawMagData = {};
MagData_t calibratedMagData = {};
MagCalibration_t magCalibration = {};

int16_t x_min = 32767, x_max = -32768;
int16_t y_min = 32767, y_max = -32768;
int16_t z_min = 32767, z_max = -32768;

bool calibrationComplete = false;
bool calibrationStarted = false;
unsigned long calibStartTime = 0;
const unsigned long calibDurationMs = 15000;

void HmcInit() {
    rawMagData = {};
    calibratedMagData = {};
    magCalibration = {};
    magCalibration.scaleX = 1.0f;
    magCalibration.scaleY = 1.0f;
    magCalibration.scaleZ = 1.0f;
    calibrationComplete = false;
    calibrationStarted = false;
    calibStartTime = 0;

    Wire.begin(HMC_I2C_SDA, HMC_I2C_SCL);

    // Configure Configuration Register A: 8 samples averaged, 30 Hz data rate, normal measurement mode
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(REG_CONFIG_A);
    Wire.write(0x74); 
    Wire.endTransmission();

    // Configure Gain Register: +/- 1.9 Gauss (0x40)
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(REG_CONFIG_B);
    Wire.write(0x40);
    Wire.endTransmission();

    //Configure Mode Register: Continuous-Measurement Mode (0x00)
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(REG_MODE);
    Wire.write(0x00);
    Wire.endTransmission();
}

void updateRawMagData(){
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(REG_DATA_X_MSB); // Point to data output X MSB register
    Wire.endTransmission(false);
    Wire.requestFrom(HMC5883L_ADDR, 6);
    
    if (Wire.available() == 6) {
        int16_t raw_x = (Wire.read() << 8) | Wire.read();
        int16_t raw_z = (Wire.read() << 8) | Wire.read(); // Note: HMC5883L register order is X, Z, Y
        int16_t raw_y = (Wire.read() << 8) | Wire.read();
    
        rawMagData.magX = (float)raw_x;
        rawMagData.magY = (float)raw_y;
        rawMagData.magZ = (float)raw_z;
    }
}

void HmcCalibration() {

  Serial.println("=========================================");
  Serial.println("CALIBRATION STARTED: Rotate the sensor");
  Serial.println("smoothly in all directions for 15 seconds...");
  Serial.println("=========================================");

    calibrationStarted = true;
    unsigned long currentMillis = millis();
    calibStartTime = currentMillis;
    while (currentMillis - calibStartTime < calibDurationMs) {
        // Perform calibration updates
        currentMillis = millis();
        updateRawMagData();
        if (rawMagData.magX < x_min) x_min = rawMagData.magX;
        if (rawMagData.magX > x_max) x_max = rawMagData.magX;
        if (rawMagData.magY < y_min) y_min = rawMagData.magY;
        if (rawMagData.magY > y_max) y_max = rawMagData.magY;
        if (rawMagData.magZ < z_min) z_min = rawMagData.magZ;
        if (rawMagData.magZ > z_max) z_max = rawMagData.magZ;
    }
    calibrationStarted = false;
    float span_x = x_max - x_min;
    float span_y = y_max - y_min;
    float span_z = z_max - z_min;
    float avg_span = (span_x + span_y + span_z) / 3.0f;
    magCalibration.offsetX = (x_max + x_min) / 2.0f;
    magCalibration.offsetY = (y_max + y_min) / 2.0f;
    magCalibration.offsetZ = (z_max + z_min) / 2.0f;
    magCalibration.scaleX = avg_span / span_x;
    magCalibration.scaleY = avg_span / span_y;
    magCalibration.scaleZ = avg_span / span_z;

    calibrationComplete = true;

  Serial.println("=========================================");
  Serial.println("CALIBRATION COMPLETE ");
  Serial.println("=========================================");
}

void applyCalibration() {
    calibratedMagData.magX = (rawMagData.magX - magCalibration.offsetX) * magCalibration.scaleX;
    calibratedMagData.magY = (rawMagData.magY - magCalibration.offsetY) * magCalibration.scaleY;
    calibratedMagData.magZ = (rawMagData.magZ - magCalibration.offsetZ) * magCalibration.scaleZ;
}

float getHeading() {
    // Implementation for heading calculation

    updateRawMagData();
    applyCalibration();

    // physical units conversion (Gauss)
    MagData_t p_mag = {};
    p_mag.magX = calibratedMagData.magX/(float)GAUSS_SCALE_FACTOR_1_9G;
    p_mag.magY = calibratedMagData.magY/(float)GAUSS_SCALE_FACTOR_1_9G;
    p_mag.magZ = calibratedMagData.magZ/(float)GAUSS_SCALE_FACTOR_1_9G;

    float heading = atan2(p_mag.magY, p_mag.magX) * (180.0f / PI); // Convert to degrees
    return heading;
}
