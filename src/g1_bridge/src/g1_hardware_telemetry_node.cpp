#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_hg/msg/low_state.hpp"

using namespace std::chrono_literals;

namespace {
diagnostic_msgs::msg::KeyValue diagnostic_value(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}
}  // namespace

class G1HardwareTelemetry final : public rclcpp::Node {
 public:
  G1HardwareTelemetry() : Node("g1_bridge") {
    low_state_topic_ = declare_parameter<std::string>("low_state_topic", "/lowstate");
    joint_state_topic_ =
        declare_parameter<std::string>("joint_state_topic", "/g1/joint_states");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/g1/imu/data");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_in_torso");
    low_state_timeout_s_ = declare_parameter<double>("low_state_timeout_s", 0.5);
    telemetry_rate_hz_ = declare_parameter<double>("telemetry_publish_rate_hz", 50.0);
    joint_indices_ = declare_parameter<std::vector<int64_t>>(
        "joint_indices", std::vector<int64_t>{});
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names", std::vector<std::string>{});

    if (joint_indices_.size() != joint_names_.size() || joint_names_.empty()) {
      throw std::runtime_error("joint_indices and joint_names must be non-empty and equal-sized");
    }
    if (telemetry_rate_hz_ <= 0.0 || low_state_timeout_s_ <= 0.0) {
      throw std::runtime_error("telemetry rate and low-state timeout must be positive");
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/g1/control_enabled", rclcpp::QoS(1).transient_local().reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    low_state_sub_ = create_subscription<unitree_hg::msg::LowState>(
        low_state_topic_, rclcpp::SensorDataQoS(),
        std::bind(&G1HardwareTelemetry::on_low_state, this, std::placeholders::_1));
    diagnostics_timer_ =
        create_wall_timer(1s, std::bind(&G1HardwareTelemetry::publish_diagnostics, this));

    publish_control_disabled();
    RCLCPP_WARN(
        get_logger(),
        "Physical G1 telemetry-only node started; this executable contains no motion command path");
  }

 private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void on_low_state(const unitree_hg::msg::LowState::SharedPtr msg) {
    const auto received_at = std::chrono::steady_clock::now();
    last_low_state_time_ = received_at;
    have_low_state_ = true;

    const double min_period_s = 1.0 / telemetry_rate_hz_;
    const double elapsed =
        std::chrono::duration<double>(received_at - last_telemetry_pub_time_).count();
    if (elapsed < min_period_s) {
      return;
    }
    last_telemetry_pub_time_ = received_at;

    const auto stamp = now();
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = joint_names_;
    joints.position.reserve(joint_indices_.size());
    joints.velocity.reserve(joint_indices_.size());
    joints.effort.reserve(joint_indices_.size());

    for (const auto index : joint_indices_) {
      if (index < 0 || static_cast<std::size_t>(index) >= msg->motor_state.size()) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Configured joint index %ld is outside motor_state", index);
        return;
      }
      const auto &motor = msg->motor_state[static_cast<std::size_t>(index)];
      joints.position.push_back(motor.q);
      joints.velocity.push_back(motor.dq);
      joints.effort.push_back(motor.tau_est);
    }
    joint_pub_->publish(joints);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_id_;
    // Unitree publishes quaternion in [w, x, y, z] order.
    imu.orientation.w = msg->imu_state.quaternion[0];
    imu.orientation.x = msg->imu_state.quaternion[1];
    imu.orientation.y = msg->imu_state.quaternion[2];
    imu.orientation.z = msg->imu_state.quaternion[3];
    imu.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu.angular_velocity.z = msg->imu_state.gyroscope[2];
    imu.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu.linear_acceleration.z = msg->imu_state.accelerometer[2];
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
  }

  bool low_state_is_recent() const {
    if (!have_low_state_) {
      return false;
    }
    const auto age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_low_state_time_)
                         .count();
    return age <= low_state_timeout_s_;
  }

  void publish_control_disabled() {
    std_msgs::msg::Bool msg;
    msg.data = false;
    enabled_pub_->publish(msg);
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Unitree G1 hardware telemetry";
    status.hardware_id = "unitree_g1";
    const bool state_ok = low_state_is_recent();
    status.level = state_ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                            : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = state_ok ? "Receiving low state" : "Low state missing or stale";
    status.values.push_back(diagnostic_value("mode", "telemetry_only"));
    status.values.push_back(diagnostic_value("control_enabled", "false"));
    status.values.push_back(diagnostic_value("motion_command_path", "absent"));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string low_state_topic_;
  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string imu_frame_id_;
  bool have_low_state_{false};
  double low_state_timeout_s_{0.5};
  double telemetry_rate_hz_{50.0};
  std::vector<int64_t> joint_indices_;
  std::vector<std::string> joint_names_;
  SteadyTime last_low_state_time_{};
  SteadyTime last_telemetry_pub_time_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<G1HardwareTelemetry>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_hardware_telemetry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
