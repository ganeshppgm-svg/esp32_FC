#include "mpu6050_driver.h"

AccGyro_t att = {}; 
static AccGyro_t sensor_offsets = {};

Adafruit_MPU6050 mpu;
TwoWire I2C_MPU = TwoWire(0);

void MpuInit(){
     // Initialize I2C with the designated pins and a 100kHz clock speed
  I2C_MPU.begin(I2C_SDA, I2C_SCL, 100000);

  Serial.println("Initializing MPU6050 on ESP32-S3...");

  // Try to initialize the MPU6050 using the custom I2C bus
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("Failed to find MPU6050 chip! Check wiring.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Set gyroscope range 
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  // Set filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void MpuCalibration(){
  const uint16_t calibration_samples = 500;
  const float gravity = 9.80665f;
  AccGyro_t totals = {};

  Serial.println("Calibrating MPU6050; keep the sensor level and still.");

  //collect multiple samples to calculate average offsets for accelerometer and gyroscope
  for (uint16_t sample = 0; sample < calibration_samples; ++sample) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    totals.accelX += a.acceleration.x;
    totals.accelY += a.acceleration.y;
    totals.accelZ += a.acceleration.z;
    totals.gyroX += g.gyro.x;
    totals.gyroY += g.gyro.y;
    totals.gyroZ += g.gyro.z;
    delay(2);
  }

  //update the offsets
  const float sample_count = (float)(calibration_samples);
  sensor_offsets.accelX = totals.accelX / sample_count;
  sensor_offsets.accelY = totals.accelY / sample_count;
  sensor_offsets.accelZ = totals.accelZ / sample_count - gravity;
  sensor_offsets.gyroX = totals.gyroX / sample_count;
  sensor_offsets.gyroY = totals.gyroY / sample_count;
  sensor_offsets.gyroZ = totals.gyroZ / sample_count;

  Serial.println("MPU6050 calibration complete.");
}

void updateSensorData(){
      // Get new sensor events (accelerometer, gyroscope, and temperature)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  att.accelX = a.acceleration.x - sensor_offsets.accelX;
  att.accelY = a.acceleration.y - sensor_offsets.accelY;
  att.accelZ = a.acceleration.z - sensor_offsets.accelZ;
  att.gyroX = g.gyro.x - sensor_offsets.gyroX;
  att.gyroY = g.gyro.y - sensor_offsets.gyroY;
  att.gyroZ = g.gyro.z - sensor_offsets.gyroZ;
}

// Getter function to access the sensor data
AccGyro_t getSensorData(){
    return att;
}

// Function to print the sensor data to Serial Monitor
void printSensorData(){
    // Print Accelerometer data (m/s^2)
  Serial.print("Accel X: "); Serial.print(att.accelX);
  Serial.print(", Y: "); Serial.print(att.accelY);
  Serial.print(", Z: "); Serial.print(att.accelZ);
  Serial.print(" m/s^2 | ");

  // Print Gyroscope data (rad/s)
  Serial.print("Gyro X: "); Serial.print(att.gyroX);
  Serial.print(", Y: "); Serial.print(att.gyroY);
  Serial.print(", Z: "); Serial.print(att.gyroZ);
  Serial.println(" rad/s | ");
}