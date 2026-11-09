#include <algorithm>
#include <array>
#include <functional>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuators.hpp"
#include "flight_core/fc_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class FcSilNode final : public rclcpp::Node
{
public:
  FcSilNode() : Node("fc_sil_node")
  {
    flight_core::Parameters parameters;
    parameters.max_roll_deg = declare_parameter<double>("max_roll_deg", 25.0);
    parameters.max_pitch_deg = declare_parameter<double>("max_pitch_deg", 25.0);
    parameters.max_yaw_rate_deg_s = declare_parameter<double>("max_yaw_rate_deg_s", 120.0);
    parameters.attitude_filter_tau_s = declare_parameter<double>("attitude_filter_tau_s", 0.50);
    parameters.motor_idle_fraction = declare_parameter<double>("motor_idle_fraction", 0.04);
    parameters.throttle_cutoff = declare_parameter<double>("throttle_cutoff", 0.02);
    parameters.roll_pid = read_pid_parameters("roll", parameters.roll_pid);
    parameters.pitch_pid = read_pid_parameters("pitch", parameters.pitch_pid);
    parameters.yaw_rate_pid = read_pid_parameters("yaw_rate", parameters.yaw_rate_pid);
    max_motor_speed_rad_s_ = declare_parameter<double>("max_motor_speed_rad_s", 800.0);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.50);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.10);
    mag_timeout_s_ = declare_parameter<double>("mag_timeout_s", 0.25);

    controller_.set_parameters(parameters);

    motor_pub_ = create_publisher<actuator_msgs::msg::Actuators>("/fc/motor_cmd", 10);
    telemetry_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/fc/telemetry", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/fc/imu", rclcpp::SensorDataQoS(),
      std::bind(&FcSilNode::on_imu, this, std::placeholders::_1));
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "/fc/mag", rclcpp::SensorDataQoS(),
      std::bind(&FcSilNode::on_mag, this, std::placeholders::_1));
    rc_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/fc/rc", rclcpp::QoS(10),
      std::bind(&FcSilNode::on_rc, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(50ms, std::bind(&FcSilNode::watchdog, this));

    RCLCPP_INFO(get_logger(),
      "FC SIL ready. RC axes=[roll,pitch,yaw,throttle(-1..+1)], buttons[0]=arm. "
      "Motors: 6, max speed: %.1f rad/s", max_motor_speed_rad_s_);
    publish_zero_motors();
  }

private:
  flight_core::PidGains read_pid_parameters(
    const std::string &prefix, const flight_core::PidGains &defaults)
  {
    flight_core::PidGains gains = defaults;
    gains.kp = declare_parameter<double>(prefix + "_kp", defaults.kp);
    gains.ki = declare_parameter<double>(prefix + "_ki", defaults.ki);
    gains.kd = declare_parameter<double>(prefix + "_kd", defaults.kd);
    gains.output_limit = declare_parameter<double>(prefix + "_output_limit", defaults.output_limit);
    gains.integral_limit = declare_parameter<double>(prefix + "_integral_limit", defaults.integral_limit);
    return gains;
  }

  static float clamp_axis(double value)
  {
    return static_cast<float>(std::max(-1.0, std::min(1.0, value)));
  }

  void on_rc(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 4U || msg->buttons.empty()) {
      rc_.armed = false;
      rc_received_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "RC expects axes [roll,pitch,yaw,throttle] and buttons[0] arm switch");
      return;
    }

    rc_.roll = clamp_axis(msg->axes[0]);
    rc_.pitch = clamp_axis(msg->axes[1]);
    rc_.yaw = clamp_axis(msg->axes[2]);
    // Standard joystick convention [-1,+1] is converted to throttle [0,1].
    rc_.throttle = static_cast<float>((clamp_axis(msg->axes[3]) + 1.0F) * 0.5F);
    rc_.armed = (msg->buttons[0] != 0);
    rc_received_ = true;
    last_rc_wall_ = std::chrono::steady_clock::now();

    if (rc_.armed != last_armed_log_) {
      RCLCPP_INFO(get_logger(), "SIL controller %s", rc_.armed ? "ARMED" : "DISARMED");
      last_armed_log_ = rc_.armed;
    }
  }

  void on_mag(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    sensor_.mag_x_t = static_cast<float>(msg->magnetic_field.x);
    sensor_.mag_y_t = static_cast<float>(msg->magnetic_field.y);
    sensor_.mag_z_t = static_cast<float>(msg->magnetic_field.z);
    const double norm = std::sqrt(
      msg->magnetic_field.x * msg->magnetic_field.x +
      msg->magnetic_field.y * msg->magnetic_field.y +
      msg->magnetic_field.z * msg->magnetic_field.z);
    sensor_.magnetometer_valid = std::isfinite(norm) && norm > 1.0e-9;
    last_mag_wall_ = std::chrono::steady_clock::now();
    mag_received_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    last_imu_wall_ = wall_now;
    imu_received_ = true;

    sensor_.accel_x_mps2 = static_cast<float>(msg->linear_acceleration.x);
    sensor_.accel_y_mps2 = static_cast<float>(msg->linear_acceleration.y);
    sensor_.accel_z_mps2 = static_cast<float>(msg->linear_acceleration.z);
    sensor_.gyro_x_rps = static_cast<float>(msg->angular_velocity.x);
    sensor_.gyro_y_rps = static_cast<float>(msg->angular_velocity.y);
    sensor_.gyro_z_rps = static_cast<float>(msg->angular_velocity.z);

    const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
    const rclcpp::Time effective_stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : stamp;

    if (!have_imu_stamp_) {
      last_imu_stamp_ = effective_stamp;
      have_imu_stamp_ = true;
      publish_zero_motors();
      return;
    }

    const double dt = (effective_stamp - last_imu_stamp_).seconds();
    last_imu_stamp_ = effective_stamp;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05) {
      controller_.reset();
      publish_zero_motors();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "IMU time step invalid (%.6f s); controller reset and motors set to zero", dt);
      return;
    }

    const bool rc_fresh = rc_received_ &&
      std::chrono::duration<double>(wall_now - last_rc_wall_).count() <= rc_timeout_s_;
    const bool mag_fresh = mag_received_ &&
      std::chrono::duration<double>(wall_now - last_mag_wall_).count() <= mag_timeout_s_;
    sensor_.magnetometer_valid = sensor_.magnetometer_valid && mag_fresh;

    flight_core::RcInput safe_rc = rc_;
    safe_rc.armed = safe_rc.armed && rc_fresh;

    const auto result = controller_.update(sensor_, safe_rc, static_cast<float>(dt));
    publish_output(result);
  }

  void watchdog()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    const bool imu_fresh = imu_received_ &&
      std::chrono::duration<double>(wall_now - last_imu_wall_).count() <= imu_timeout_s_;
    const bool rc_fresh = rc_received_ &&
      std::chrono::duration<double>(wall_now - last_rc_wall_).count() <= rc_timeout_s_;
    if (!imu_fresh || !rc_fresh || !rc_.armed) {
      publish_zero_motors();
    }
  }

  void publish_zero_motors()
  {
    actuator_msgs::msg::Actuators command;
    command.header.stamp = now().to_msg();
    command.velocity.assign(flight_core::kMotorCount, 0.0F);
    motor_pub_->publish(command);
  }

  void publish_output(const flight_core::Output &output)
  {
    actuator_msgs::msg::Actuators command;
    command.header.stamp = now().to_msg();
    command.velocity.resize(flight_core::kMotorCount, 0.0F);

    std_msgs::msg::Float32MultiArray telemetry;
    telemetry.data.reserve(9);
    telemetry.data.push_back(output.attitude.roll_deg);
    telemetry.data.push_back(output.attitude.pitch_deg);
    telemetry.data.push_back(output.attitude.yaw_deg);

    for (std::size_t i = 0; i < flight_core::kMotorCount; ++i) {
      const float normalized = output.motor_throttle[i];
      command.velocity[i] = output.armed ?
        normalized * static_cast<float>(max_motor_speed_rad_s_) : 0.0F;
      telemetry.data.push_back(normalized);
    }

    motor_pub_->publish(command);
    telemetry_pub_->publish(telemetry);
  }

  flight_core::FlightController controller_{};
  flight_core::SensorData sensor_{};
  flight_core::RcInput rc_{};

  double max_motor_speed_rad_s_{800.0};
  double rc_timeout_s_{0.50};
  double imu_timeout_s_{0.10};
  double mag_timeout_s_{0.25};

  bool rc_received_{false};
  bool imu_received_{false};
  bool mag_received_{false};
  bool have_imu_stamp_{false};
  bool last_armed_log_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  std::chrono::steady_clock::time_point last_rc_wall_{};
  std::chrono::steady_clock::time_point last_imu_wall_{};
  std::chrono::steady_clock::time_point last_mag_wall_{};

  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcSilNode>());
  rclcpp::shutdown();
  return 0;
}
