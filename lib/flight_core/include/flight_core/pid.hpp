#pragma once

#include "flight_core/fc_types.hpp"

namespace flight_core
{

class PidController
{
public:
  explicit PidController(const PidGains &gains = PidGains{});

  void set_gains(const PidGains &gains);
  void reset();

  // Derivative is taken on the measured variable to avoid setpoint kick.
  float update(float setpoint, float measurement, float measured_rate, float dt_s);

private:
  PidGains gains_{};
  float integral_{0.0F};
};

}  // namespace flight_core
