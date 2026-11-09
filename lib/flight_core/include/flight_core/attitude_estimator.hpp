#pragma once

#include "flight_core/fc_types.hpp"

namespace flight_core
{

class AttitudeEstimator
{
public:
  explicit AttitudeEstimator(float filter_tau_s = 0.50F);
  void set_filter_tau(float tau_s);
  void reset();
  Attitude update(const SensorData &sensor, float dt_s);

private:
  float tau_s_{0.50F};
  bool initialized_{false};
  bool mag_reference_initialized_{false};
  float mag_yaw_offset_deg_{0.0F};
  Attitude attitude_{};
};

}  // namespace flight_core
