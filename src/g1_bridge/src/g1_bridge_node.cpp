#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <functional>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_hg/msg/low_state.hpp"

using namespace std::chrono_literals;

namespace {
constexpr int32_t kSetVelocityApiId = 7105;

double clamp_symmetric(double value, double limit) {
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

diagnostic_msgs::msg::KeyValue diagnostic_value(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}
}  // namespace

class G1Bridge final : public rclcpp::Node {
 public:
  G1Bridge() : Node("g1_bridge") {
    low_state_topic_ = declare_parameter<std::string>("low_state_topic", "lowstate");
    command_request_topic_ =
        declare_parameter<std::string>("command_request_topic", "/api/sport/request");
    joint_state_topic_ =
        declare_parameter<std::string>("joint_state_topic", "/g1/joint_states");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/g1/imu/data");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_in_torso");

    motion_interface_enabled_ =
        declare_parameter<bool>("motion_interface_enabled", true);
    control_enabled_ = declare_parameter<bool>("start_control_enabled", false);
    require_recent_low_state_ = declare_parameter<bool>("require_recent_low_state", true);
    low_state_timeout_s_ = declare_parameter<double>("low_state_timeout_s", 0.5);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.25);
    command_rate_hz_ = declare_parameter<double>("command_publish_rate_hz", 20.0);
    telemetry_rate_hz_ = declare_parameter<double>("telemetry_publish_rate_hz", 50.0);
    command_duration_s_ = declare_parameter<double>("command_duration_s", 0.2);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.5);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.3);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.8);
    joint_indices_ = declare_parameter<std::vector<int64_t>>(
        "joint_indices", std::vector<int64_t>{});
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names", std::vector<std::string>{});

    if (joint_indices_.size() != joint_names_.size() || joint_names_.empty()) {
      throw std::runtime_error("joint_indices and joint_names must be non-empty and equal-sized");
    }
    if (command_rate_hz_ <= 0.0 || telemetry_rate_hz_ <= 0.0 || cmd_timeout_s_ <= 0.0) {
      throw std::runtime_error("rates and timeouts must be positive");
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/g1/control_enabled", rclcpp::QoS(1).transient_local().reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));

    low_state_sub_ = create_subscription<unitree_hg::msg::LowState>(
        low_state_topic_, rclcpp::SensorDataQoS(),
        std::bind(&G1Bridge::on_low_state, this, std::placeholders::_1));
    if (motion_interface_enabled_) {
      command_pub_ =
          create_publisher<unitree_api::msg::Request>(command_request_topic_, 10);
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", rclcpp::QoS(10),
          std::bind(&G1Bridge::on_cmd_vel, this, std::placeholders::_1));
      enable_service_ = create_service<std_srvs::srv::SetBool>(
          "/g1/enable_control",
          std::bind(&G1Bridge::on_enable, this, std::placeholders::_1,
                    std::placeholders::_2));

      const auto command_period = std::chrono::duration<double>(1.0 / command_rate_hz_);
      command_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(command_period),
          std::bind(&G1Bridge::command_tick, this));
    } else {
      control_enabled_ = false;
    }
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&G1Bridge::publish_diagnostics, this));

    last_cmd_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    last_telemetry_pub_time_ = last_cmd_time_;
    publish_control_enabled();
    RCLCPP_WARN(
        get_logger(), "G1 bridge started in %s mode; motion control is %s",
        motion_interface_enabled_ ? "telemetry + motion-interface" : "telemetry-only",
        control_enabled_ ? "ENABLED" : "DISABLED");
  }

  ~G1Bridge() override {
    if (control_enabled_ && rclcpp::ok()) {
      publish_velocity(0.0, 0.0, 0.0);
    }
  }

 private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void on_low_state(const unitree_hg::msg::LowState::SharedPtr msg) {
    last_low_state_time_ = std::chrono::steady_clock::now();
    have_low_state_ = true;

    const double min_period_s = 1.0 / telemetry_rate_hz_;
    const double elapsed = std::chrono::duration<double>(
                               last_low_state_time_ - last_telemetry_pub_time_)
                               .count();
    if (elapsed < min_period_s) {
      return;
    }
    last_telemetry_pub_time_ = last_low_state_time_;

    const auto stamp = now();
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = joint_names_;
    joints.position.reserve(joint_indices_.size());
    joints.velocity.reserve(joint_indices_.size());
    joints.effort.reserve(joint_indices_.size());

    for (const auto index : joint_indices_) {
      if (index < 0 || static_cast<std::size_t>(index) >= msg->motor_state.size()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
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
    // Covariances are not supplied by the G1 low-state message.
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    desired_vx_ = clamp_symmetric(msg->linear.x, max_linear_x_);
    desired_vy_ = clamp_symmetric(msg->linear.y, max_linear_y_);
    desired_wz_ = clamp_symmetric(msg->angular.z, max_angular_z_);
    last_cmd_time_ = std::chrono::steady_clock::now();
    watchdog_stopped_ = false;
  }

  void on_enable(const std_srvs::srv::SetBool::Request::SharedPtr request,
                 std_srvs::srv::SetBool::Response::SharedPtr response) {
    if (request->data && require_recent_low_state_ && !low_state_is_recent()) {
      response->success = false;
      response->message = "Cannot enable: no recent G1 lowstate received";
      return;
    }

    control_enabled_ = request->data;
    desired_vx_ = desired_vy_ = desired_wz_ = 0.0;
    last_cmd_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    watchdog_stopped_ = false;
    publish_velocity(0.0, 0.0, 0.0);
    publish_control_enabled();

    response->success = true;
    response->message = control_enabled_ ? "G1 motion control enabled" : "G1 stopped and control disabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void command_tick() {
    if (!control_enabled_) {
      return;
    }
    if (require_recent_low_state_ && !low_state_is_recent()) {
      if (!watchdog_stopped_) {
        publish_velocity(0.0, 0.0, 0.0);
        watchdog_stopped_ = true;
        RCLCPP_ERROR(get_logger(), "Low-state watchdog stopped G1 commands");
      }
      return;
    }

    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                   last_cmd_time_)
                         .count();
    if (age > cmd_timeout_s_) {
      if (!watchdog_stopped_) {
        publish_velocity(0.0, 0.0, 0.0);
        watchdog_stopped_ = true;
        RCLCPP_WARN(get_logger(), "cmd_vel watchdog sent stop");
      }
      return;
    }
    publish_velocity(desired_vx_, desired_vy_, desired_wz_);
  }

  bool low_state_is_recent() const {
    if (!have_low_state_) {
      return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                   last_low_state_time_)
                         .count();
    return age <= low_state_timeout_s_;
  }

  void publish_velocity(double vx, double vy, double wz) {
    if (!command_pub_) {
      return;
    }
    unitree_api::msg::Request request;
    request.header.identity.api_id = kSetVelocityApiId;
    request.header.identity.id = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    std::ostringstream json;
    json.imbue(std::locale::classic());
    json << std::setprecision(7) << "{\"velocity\":[" << vx << ',' << vy << ',' << wz
         << "],\"duration\":" << command_duration_s_ << '}';
    request.parameter = json.str();
    command_pub_->publish(request);
  }

  void publish_control_enabled() {
    std_msgs::msg::Bool msg;
    msg.data = control_enabled_;
    enabled_pub_->publish(msg);
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Unitree G1 ROS 2 bridge";
    status.hardware_id = "unitree_g1";
    const bool state_ok = low_state_is_recent();
    status.level = state_ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                            : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = state_ok ? "Receiving low state" : "Low state missing or stale";
    status.values.push_back(
        diagnostic_value("control_enabled", control_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
        "motion_interface_enabled", motion_interface_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
        "cmd_watchdog_stopped", watchdog_stopped_ ? "true" : "false"));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string low_state_topic_;
  std::string command_request_topic_;
  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string imu_frame_id_;
  bool motion_interface_enabled_{true};
  bool control_enabled_{false};
  bool require_recent_low_state_{true};
  bool have_low_state_{false};
  bool watchdog_stopped_{false};
  double low_state_timeout_s_{0.5};
  double cmd_timeout_s_{0.25};
  double command_rate_hz_{20.0};
  double telemetry_rate_hz_{50.0};
  double command_duration_s_{0.2};
  double max_linear_x_{0.5};
  double max_linear_y_{0.3};
  double max_angular_z_{0.8};
  double desired_vx_{0.0};
  double desired_vy_{0.0};
  double desired_wz_{0.0};
  std::vector<int64_t> joint_indices_;
  std::vector<std::string> joint_names_;
  SteadyTime last_cmd_time_{};
  SteadyTime last_low_state_time_{};
  SteadyTime last_telemetry_pub_time_{};

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<G1Bridge>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
