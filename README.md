# ESP32 Flight Controller

This project is a lightweight ESP32-based flight controller for a multirotor aircraft. It reads sensor data from an IMU, interprets radio-control commands, estimates the vehicle attitude, and adjusts motor outputs to maintain stable flight.

## What the flight controller does

- Reads motion data from the MPU6050 gyroscope and accelerometer
- Uses the HMC5883L magnetometer for heading reference
- Receives pilot commands from an RC receiver
- Estimates roll, pitch, and yaw angles
- Applies PID-based stabilization to keep the craft level and controllable
- Mixes the stabilized commands into motor outputs for a six-motor configuration
- Sends PWM signals to ESCs for motor control
- Logs flight telemetry for debugging and tuning

## Hardware platform

- ESP32-S3 DevKitC-1
- MPU6050 inertial measurement unit
- HMC5883L compass module
- RC receiver for stick inputs
- Electronic speed controllers (ESCs) connected to six motors

## Build and run

This is a PlatformIO project. Open the project folder in VS Code with the PlatformIO extension and build/upload the firmware for the `esp32-s3-devkitc-1` environment.

```bash
pio run -e esp32-s3-devkitc-1
pio run -e esp32-s3-devkitc-1 -t upload
```

## Project notes

The firmware is organized into driver, abstraction, application, and logging layers to separate sensor handling, attitude estimation, control logic, and output management. It is designed as a practical embedded control system for experimenting with stabilized drone flight on an ESP32 platform.
