#include "flight_core/attitude_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace flight_core
{
namespace
{
constexpr float kRadToDeg = 57.29577951308232F;
constexpr float kGravity = 9.80665F;
float wrap_degrees(float angle)
{
  while (angle > 180.0F) { angle -= 360.0F; }
  while (angle < -180.0F) { angle += 360.0F; }
  return angle;
}

float magnetometer_heading_deg(const SensorData &s, float roll_rad, float pitch_rad)
{
  // Tilt compensation for an FLU body frame. The magnetometer is expected
  // to be expressed in the same sensor/body frame as the IMU.
  const float cr = std::cos(roll_rad);
  const float sr = std::sin(roll_rad);
  const float cp = std::cos(pitch_rad);
  const float sp = std::sin(pitch_rad);
  const float horizontal_x = s.mag_x_t * cp + s.mag_z_t * sp;
  const float horizontal_y = s.mag_x_t * sr * sp + s.mag_y_t * cr - s.mag_z_t * sr * cp;
  return std::atan2(horizontal_y, horizontal_x) * kRadToDeg;
}
}  // namespace

AttitudeEstimator::AttitudeEstimator(float filter_tau_s) : tau_s_(filter_tau_s) {}

void AttitudeEstimator::set_filter_tau(float tau_s)
{
  tau_s_ = std::max(0.001F, tau_s);
}

void AttitudeEstimator::reset()
{
  initialized_ = false;
  mag_reference_initialized_ = false;
  mag_yaw_offset_deg_ = 0.0F;
  attitude_ = Attitude{};
}

Attitude AttitudeEstimator::update(const SensorData &s, float dt_s)
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0F) {
    return attitude_;
  }

  const float accel_norm = std::sqrt(
    s.accel_x_mps2 * s.accel_x_mps2 +
    s.accel_y_mps2 * s.accel_y_mps2 +
    s.accel_z_mps2 * s.accel_z_mps2);
  const float accel_roll = std::atan2(s.accel_y_mps2, s.accel_z_mps2) * kRadToDeg;
  const float accel_pitch = std::atan2(
    -s.accel_x_mps2,
    std::sqrt(s.accel_y_mps2 * s.accel_y_mps2 + s.accel_z_mps2 * s.accel_z_mps2)) * kRadToDeg;

  const float gyro_roll_dps = s.gyro_x_rps * kRadToDeg;
  const float gyro_pitch_dps = s.gyro_y_rps * kRadToDeg;
  const float gyro_yaw_dps = s.gyro_z_rps * kRadToDeg;

  if (!initialized_) {
    attitude_.roll_deg = accel_roll;
    attitude_.pitch_deg = accel_pitch;
    attitude_.yaw_deg = 0.0F;
    initialized_ = true;
  } else {
    const float alpha = tau_s_ / (tau_s_ + dt_s);
    const bool accel_is_plausible = accel_norm > 0.5F * kGravity && accel_norm < 1.5F * kGravity;
    const float roll_gyro = attitude_.roll_deg + gyro_roll_dps * dt_s;
    const float pitch_gyro = attitude_.pitch_deg + gyro_pitch_dps * dt_s;

    attitude_.roll_deg = accel_is_plausible ?
      alpha * roll_gyro + (1.0F - alpha) * accel_roll : roll_gyro;
    attitude_.pitch_deg = accel_is_plausible ?
      alpha * pitch_gyro + (1.0F - alpha) * accel_pitch : pitch_gyro;
    attitude_.yaw_deg = wrap_degrees(attitude_.yaw_deg + gyro_yaw_dps * dt_s);
  }

  const float mag_norm = std::sqrt(
    s.mag_x_t * s.mag_x_t + s.mag_y_t * s.mag_y_t + s.mag_z_t * s.mag_z_t);
  if (s.magnetometer_valid && std::isfinite(mag_norm) && mag_norm > 1.0e-9F) {
    const float roll_rad = attitude_.roll_deg / kRadToDeg;
    const float pitch_rad = attitude_.pitch_deg / kRadToDeg;
    const float heading_deg = magnetometer_heading_deg(s, roll_rad, pitch_rad);

    // Establish a relative yaw zero when the first valid magnetic sample arrives.
    if (!mag_reference_initialized_) {
      mag_yaw_offset_deg_ = wrap_degrees(heading_deg - attitude_.yaw_deg);
      mag_reference_initialized_ = true;
    }

    const float mag_yaw_deg = wrap_degrees(heading_deg - mag_yaw_offset_deg_);
    const float alpha = tau_s_ / (tau_s_ + dt_s);
    attitude_.yaw_deg = wrap_degrees(
      alpha * attitude_.yaw_deg + (1.0F - alpha) *
      (attitude_.yaw_deg + wrap_degrees(mag_yaw_deg - attitude_.yaw_deg)));
  }

  return attitude_;
}

}  // namespace flight_core
