# Porting the same core back to ESP32-S3 (PlatformIO)

The SIL node compiles the core source files directly from `lib/flight_core`. PlatformIO discovers the same library through `lib/flight_core/library.json`.

In the ESP32 application, create one persistent `flight_core::FlightController` and call it from the control loop. The hardware layer should only fill `SensorData` and `RcInput`, then translate `Output::motor_throttle[0..5]` to ESC pulse widths.

Illustrative adapter (merge into your existing `src/main.cpp`; do not compile this file as a second `setup()/loop()`):

```cpp
#include <Arduino.h>
#include <flight_core/fc_controller.hpp>

flight_core::FlightController fc;

void runFlightControlFromHardware()
{
  const AccGyro_t imu = getSensorData(); // after updateSensorData()
  flight_core::SensorData s{};
  s.accel_x_mps2 = imu.accelX;
  s.accel_y_mps2 = imu.accelY;
  s.accel_z_mps2 = imu.accelZ;
  s.gyro_x_rps = imu.gyroX;
  s.gyro_y_rps = imu.gyroY;
  s.gyro_z_rps = imu.gyroZ;

  // Populate s.mag_* from your calibrated HMC5883L readout and set
  // s.magnetometer_valid=true only after calibration/data validation.

  flight_core::RcInput rc{};
  rc.throttle = getThrottleCMD() / 100.0f; // current RC driver returns 0..100
  rc.roll = getRollCMD() / MAX_ROLL;
  rc.pitch = getPitchCMD() / MAX_PITCH;
  rc.yaw = getYawCMD() / MAX_YAW;
  rc.armed = armed; // use an explicit arm switch / failsafe state

  static uint32_t previous_us = micros();
  const uint32_t now_us = micros();
  const float dt = static_cast<uint32_t>(now_us - previous_us) * 1.0e-6f;
  previous_us = now_us;

  const flight_core::Output out = fc.update(s, rc, dt);
  const float m[6] = {
    out.motor_throttle[0], out.motor_throttle[1], out.motor_throttle[2],
    out.motor_throttle[3], out.motor_throttle[4], out.motor_throttle[5]
  };
  sendESC(m[0], m[1], m[2], m[3], m[4], m[5]);
}
```

Before using this adapter, fix the existing `void updateRCCMD();` declaration in `main.cpp` to a function call (`updateRCCMD();`), remove the duplicate `MAX_ROLL` / `MAX_PITCH` / `MAX_YAW` scaling in `main.cpp`, and ensure `MINIMUM_PULSE_WIDTH` and `MAXIMUM_PULSE_WIDTH` in `FC_config.h` are real receiver pulse limits. The repository currently has zero placeholders for pulse and ESC limits and zero PID gains, so those values cannot be treated as flight-ready.

To prevent duplicate definitions after migrating the old algorithms, remove or exclude these legacy translation units from the PlatformIO build once the ESP32 `main.cpp` calls `flight_core`:

- `src/Application/attitude_control.cpp`
- `src/Application/motor_mixing.cpp`
- `src/abstraction/angle_estimation.cpp`

Keep hardware drivers and SD logging in the ESP32 build only. Do not include Arduino, Wire, ESP32Servo, FreeRTOS, or GPIO headers in `flight_core`.
