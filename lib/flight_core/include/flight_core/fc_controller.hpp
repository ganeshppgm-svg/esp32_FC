#pragma once

#include "flight_core/attitude_estimator.hpp"
#include "flight_core/fc_types.hpp"
#include "flight_core/pid.hpp"

namespace flight_core
{

class FlightController
{
public:
  explicit FlightController(const Parameters &parameters = Parameters{});

  void set_parameters(const Parameters &parameters);
  void reset();
  Output update(const SensorData &sensor, const RcInput &rc, float dt_s);

private:
  Parameters parameters_{};
  AttitudeEstimator estimator_{};
  PidController roll_pid_{};
  PidController pitch_pid_{};
  PidController yaw_rate_pid_{};
};

}  // namespace flight_core
