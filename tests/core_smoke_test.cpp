#include <cassert>
#include <cmath>
#include <iostream>

#include "flight_core/fc_controller.hpp"

int main()
{
  flight_core::Parameters params;
  params.roll_pid.kp = 0.01F;
  params.roll_pid.ki = 0.0F;
  params.roll_pid.kd = 0.0F;
  params.pitch_pid = params.roll_pid;
  params.yaw_rate_pid.kp = 0.001F;
  flight_core::FlightController fc(params);

  flight_core::SensorData sensor;
  sensor.accel_z_mps2 = 9.80665F;
  sensor.magnetometer_valid = false;

  flight_core::RcInput rc;
  rc.armed = false;
  rc.throttle = 0.5F;
  auto out = fc.update(sensor, rc, 0.0025F);
  for (float motor : out.motor_throttle) { assert(motor == 0.0F); }

  rc.armed = true;
  rc.throttle = 0.55F;
  out = fc.update(sensor, rc, 0.0025F);
  for (float motor : out.motor_throttle) {
    assert(std::isfinite(motor));
    assert(motor >= 0.0F && motor <= 1.0F);
  }

  rc.roll = 0.4F;
  out = fc.update(sensor, rc, 0.0025F);
  bool motors_differ = false;
  for (std::size_t i = 1; i < out.motor_throttle.size(); ++i) {
    if (std::fabs(out.motor_throttle[i] - out.motor_throttle[0]) > 1.0e-6F) {
      motors_differ = true;
    }
  }
  assert(motors_differ);

  rc.armed = false;
  out = fc.update(sensor, rc, 0.0025F);
  for (float motor : out.motor_throttle) { assert(motor == 0.0F); }

  std::cout << "flight_core smoke test passed\n";
  return 0;
}
