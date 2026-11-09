#include "flight_core/fc_controller.hpp"

#include <algorithm>
#include <cmath>

namespace flight_core
{
namespace
{
constexpr float kRadToDeg = 57.29577951308232F;

float clamp_value(float value, float lower, float upper)
{
  return std::max(lower, std::min(value, upper));
}

// Six-rotor matrix retained from src/Application/motor_mixing.cpp in esp32_FC.
// Columns are collective, roll, pitch, yaw. Rotor positions around the frame:
// 0 front, 1 front-right, 2 rear-right, 3 rear, 4 rear-left, 5 front-left.
const float kMixing[6][4] = {
  {1.0F,  0.000F,  1.0F,  1.0F},
  {1.0F, -0.866F,  0.5F, -1.0F},
  {1.0F, -0.866F, -0.5F,  1.0F},
  {1.0F,  0.000F, -1.0F, -1.0F},
  {1.0F,  0.866F, -0.5F,  1.0F},
  {1.0F,  0.866F,  0.5F, -1.0F}
};

void mix_and_desaturate(
  float collective, float roll, float pitch, float yaw,
  std::array<float, kMotorCount> &motors)
{
  float differential[kMotorCount]{};
  float max_positive = 0.0F;
  float max_negative = 0.0F;

  for (unsigned int i = 0; i < kMotorCount; ++i) {
    differential[i] =
      kMixing[i][1] * roll + kMixing[i][2] * pitch + kMixing[i][3] * yaw;
    max_positive = std::max(max_positive, differential[i]);
    max_negative = std::min(max_negative, differential[i]);
  }

  // Preserve torque ratios and collective while scaling corrections to fit [0, 1].
  float scale = 1.0F;
  if (max_positive > 1.0e-6F) {
    scale = std::min(scale, (1.0F - collective) / max_positive);
  }
  if (max_negative < -1.0e-6F) {
    scale = std::min(scale, collective / (-max_negative));
  }
  scale = clamp_value(scale, 0.0F, 1.0F);

  for (unsigned int i = 0; i < kMotorCount; ++i) {
    motors[i] = clamp_value(collective + scale * differential[i], 0.0F, 1.0F);
  }
}
}  // namespace

FlightController::FlightController(const Parameters &parameters)
: estimator_(parameters.attitude_filter_tau_s),
  roll_pid_(parameters.roll_pid),
  pitch_pid_(parameters.pitch_pid),
  yaw_rate_pid_(parameters.yaw_rate_pid)
{
  set_parameters(parameters);
}

void FlightController::set_parameters(const Parameters &parameters)
{
  parameters_ = parameters;
  estimator_.set_filter_tau(parameters_.attitude_filter_tau_s);
  roll_pid_.set_gains(parameters_.roll_pid);
  pitch_pid_.set_gains(parameters_.pitch_pid);
  yaw_rate_pid_.set_gains(parameters_.yaw_rate_pid);
}

void FlightController::reset()
{
  estimator_.reset();
  roll_pid_.reset();
  pitch_pid_.reset();
  yaw_rate_pid_.reset();
}

Output FlightController::update(const SensorData &sensor, const RcInput &rc, float dt_s)
{
  Output output{};
  if (!std::isfinite(dt_s) || dt_s < 0.0001F || dt_s > 0.05F) {
    reset();
    return output;
  }

  output.attitude = estimator_.update(sensor, dt_s);
  output.armed = rc.armed;
  if (!rc.armed) {
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_rate_pid_.reset();
    return output;
  }

  const float throttle = clamp_value(rc.throttle, 0.0F, 1.0F);
  if (throttle <= parameters_.throttle_cutoff) {
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_rate_pid_.reset();
    return output;
  }

  const float roll_setpoint_deg = clamp_value(rc.roll, -1.0F, 1.0F) * parameters_.max_roll_deg;
  const float pitch_setpoint_deg = clamp_value(rc.pitch, -1.0F, 1.0F) * parameters_.max_pitch_deg;
  const float yaw_rate_setpoint_dps =
    clamp_value(rc.yaw, -1.0F, 1.0F) * parameters_.max_yaw_rate_deg_s;

  const float roll_rate_dps = sensor.gyro_x_rps * kRadToDeg;
  const float pitch_rate_dps = sensor.gyro_y_rps * kRadToDeg;
  const float yaw_rate_dps = sensor.gyro_z_rps * kRadToDeg;

  const float roll_correction = roll_pid_.update(
    roll_setpoint_deg, output.attitude.roll_deg, roll_rate_dps, dt_s);
  const float pitch_correction = pitch_pid_.update(
    pitch_setpoint_deg, output.attitude.pitch_deg, pitch_rate_dps, dt_s);
  const float yaw_correction = yaw_rate_pid_.update(
    yaw_rate_setpoint_dps, yaw_rate_dps, 0.0F, dt_s);

  // Mixer collective is normalized [0,1]. Add a small idle fraction only
  // after the throttle has crossed the explicit cutoff threshold.
  const float collective = parameters_.motor_idle_fraction +
    throttle * (1.0F - parameters_.motor_idle_fraction);

  mix_and_desaturate(
    collective, roll_correction, pitch_correction, yaw_correction,
    output.motor_throttle);
  return output;
}

}  // namespace flight_core
