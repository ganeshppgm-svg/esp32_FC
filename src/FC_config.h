#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/*pin configurations*/
// Define pins for MPU6050 I2C interface
#define I2C_SDA 8
#define I2C_SCL 9

//define pins for BMP280 I2C interface
#define BMP_I2C_SDA 8
#define BMP_I2C_SCL 9

// Define GPIO pins for RC channels
#define THROTTLE_PIN  4
#define ROLL_PIN      16
#define PITCH_PIN     17
#define YAW_PIN       18

//pins for HMC5883L I2C interface
#define HMC_I2C_SDA 8
#define HMC_I2C_SCL 9

//pin for SD card chip select
#define SD_CS_PIN 5
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 12
#define SD_SCK_PIN 13

/*PWM channel settings and control ranges*/
// RC transmitter PWM recieve signal settings
#define MINIMUM_PULSE_WIDTH 0U //in micro-seconds
#define MAXIMUM_PULSE_WIDTH 0U
#define PULSE_PERIOD        0U

/* ESC configuration */
//minimum and maximum pulse widths PWM signals to ESC
#define MAX_PULSE_WIDTH_ESC 0U //in micro-seconds
#define MIN_PULSE_WIDTH_ESC 0U

//esc pins
#define ESC1 0U
#define ESC2 1U
#define ESC3 0U
#define ESC4 1U
#define ESC5 0U
#define ESC6 1U

//control ranges in degrees
#define MAX_ROLL  30U
#define MAX_PITCH 30U
#define MAX_YAW   45U

#endif