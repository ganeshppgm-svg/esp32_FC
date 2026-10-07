#ifndef BMP280_DRIVER_H
#define BMP280_DRIVER_H

#include <Wire.h>
#include <Adafruit_BMP280.h>

void bmpsetup();
float getAltitude();

#endif