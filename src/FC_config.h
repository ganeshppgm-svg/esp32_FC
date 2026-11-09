#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* Pin Configurations */

// Define pins for I2C bus (Shared by MPU6050, BMP280, and HMC5883L)
#define MPU_I2C_SDA       8
#define MPU_I2C_SCL       9

#define BMP_I2C_SDA   8
#define BMP_I2C_SCL   9

#define HMC_I2C_SDA   8
#define HMC_I2C_SCL   9

// Define GPIO pins for RC channels (Receiver inputs)
#define THROTTLE_PIN  1
#define ROLL_PIN      2
#define PITCH_PIN     4
#define YAW_PIN       6

// Pin definitions for SD Card SPI interface (HSPI / SPI2)
#define SD_CS_PIN     10
#define SD_MOSI_PIN   11
#define SD_MISO_PIN   13
#define SD_SCK_PIN    12

/* PWM Channel Settings and Control Ranges */

// RC transmitter PWM receive signal settings (Standard 1000µs - 2000µs, 50Hz)
#define MINIMUM_PULSE_WIDTH 1000U // in micro-seconds
#define MAXIMUM_PULSE_WIDTH 2000U
#define PULSE_PERIOD        20000U // 20 ms period (50Hz)

/* ESC Configuration */

// Minimum and maximum pulse widths PWM signals to ESCs
#define MAX_PULSE_WIDTH_ESC 2000U // in micro-seconds
#define MIN_PULSE_WIDTH_ESC 1000U

// ESC PWM output pins (All 6 pins are unique, safe, non-conflicting GPIOs)
#define ESC1 14U
#define ESC2 15U
#define ESC3 21U
#define ESC4 38U
#define ESC5 39U
#define ESC6 40U

// Control ranges in degrees
#define MAX_ROLL   30U
#define MAX_PITCH  30U
#define MAX_YAW    45U

#endif // FC_CONFIG_H