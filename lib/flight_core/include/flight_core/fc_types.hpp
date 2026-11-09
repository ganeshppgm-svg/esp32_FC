#pragma once

#include <array>

namespace flight_core
{

constexpr unsigned int kMotorCount = 6;

// Sensor conventions for both the Gazebo model and the ESP32 adapter:
// body frame: ROS FLU (X forward, Y left, Z up)
// gyro: rad/s; accelerometer: m/s^2; magnetometer: Tesla (direction is what matters).
struct SensorData
{
  float accel_x_mps2{0.0F};
  float accel_y_mps2{0.0F};
  float accel_z_mps2{9.80665F};

  float gyro_x_rps{0.0F};
  float gyro_y_rps{0.0F};
  float gyro_z_rps{0.0F};

  float mag_x_t{0.0F};
  float mag_y_t{0.0F};
  float mag_z_t{0.0F};
  bool magnetometer_valid{false};
};

// RC contract: roll/pitch/yaw in [-1, +1], throttle in [0, 1].
struct RcInput
{
  float throttle{0.0F};
  float roll{0.0F};
  float pitch{0.0F};
  float yaw{0.0F};
  bool armed{false};
};

struct Attitude
{
  float roll_deg{0.0F};
  float pitch_deg{0.0F};
  float yaw_deg{0.0F};
};

struct Output
{
  std::array<float, kMotorCount> motor_throttle{{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
  Attitude attitude{};
  bool armed{false};
};

struct PidGains
{
  float kp{0.0F};
  float ki{0.0F};
  float kd{0.0F};
  float output_limit{0.20F};
  float integral_limit{100.0F};
};

struct Parameters
{
  float max_roll_deg{25.0F};
  float max_pitch_deg{25.0F};
  float max_yaw_rate_deg_s{120.0F};
  float attitude_filter_tau_s{0.50F};
  float motor_idle_fraction{0.04F};
  float throttle_cutoff{0.02F};

  // Conservative starting values for simulation experiments only.
  // They are not validated flight gains and must be tuned for the actual frame.
  PidGains roll_pid{0.012F, 0.00015F, 0.0020F, 0.20F, 100.0F};
  PidGains pitch_pid{0.012F, 0.00015F, 0.0020F, 0.20F, 100.0F};
  PidGains yaw_rate_pid{0.0020F, 0.00005F, 0.0F, 0.15F, 500.0F};
};

}  // namespace flight_core
