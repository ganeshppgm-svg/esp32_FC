#include "flight_core/pid.hpp"

#include <algorithm>
#include <cmath>

namespace flight_core
{
namespace
{
float clamp_value(float value, float lower, float upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

PidController::PidController(const PidGains &gains) : gains_(gains) {}

void PidController::set_gains(const PidGains &gains)
{
  gains_ = gains;
  integral_ = clamp_value(integral_, -gains_.integral_limit, gains_.integral_limit);
}

void PidController::reset()
{
  integral_ = 0.0F;
}

float PidController::update(
  float setpoint, float measurement, float measured_rate, float dt_s)
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0F) {
    return 0.0F;
  }

  const float error = setpoint - measurement;
  const float p_term = gains_.kp * error;
  const float d_term = -gains_.kd * measured_rate;
  const float without_i = p_term + d_term;
  const float proposed_i = clamp_value(
    integral_ + error * dt_s, -gains_.integral_limit, gains_.integral_limit);
  const float proposed_output = without_i + gains_.ki * proposed_i;

  // Conditional integration: don't integrate farther into an active limit.
  const bool pushes_high = proposed_output > gains_.output_limit && error > 0.0F;
  const bool pushes_low = proposed_output < -gains_.output_limit && error < 0.0F;
  if (!pushes_high && !pushes_low) {
    integral_ = proposed_i;
  }

  return clamp_value(
    without_i + gains_.ki * integral_, -gains_.output_limit, gains_.output_limit);
}

}  // namespace flight_core
