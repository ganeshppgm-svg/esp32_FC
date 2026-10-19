#include "bmp280_driver.h"

Adafruit_BMP280 bmp; // I2C interface
const float SEALIGHT_PRESSURE_HPA = 1013.25;

void bmpsetup(){
      // Initialize BMP280 with default I2C address 0x76 (or 0x77)
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could a valid BMP280 sensor be found, check wiring!"));
    while (1);
  }

  // Configure sensor settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,    
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,     
                  Adafruit_BMP280::STANDBY_MS_500); 
}

// getter function to read altitude
float getAltitude(){
    return bmp.readAltitude(SEALIGHT_PRESSURE_HPA);
}